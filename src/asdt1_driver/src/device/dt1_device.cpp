/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dt1_device.h"

#include "connect_status.h"
#include "dt1_device_cdc.h"
#include "dt1_device_hist.h"
#include "dt1_modes.h"
#include "logger.h"

#include <cinttypes>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

static constexpr float minimum_supported_version = 1.01;
static constexpr double ACCEL_SCALE = 9.81 / 8192.0;
static constexpr double GYRO_SCALE = (500.0 / 32768.0) * M_PI / 180.0;

static bool is_supported_device(const std::string& version);

/*
 * Find the port for device with serial_number.
 * If serial_number is empty, find any AS-DT1 device.
 *
 * Returns empty string if not found.
 */
static std::string find_port(const std::string& serial_number, const log_callback& _log_cb)
{
    std::string search_str;
    if (serial_number.empty())
    {
        LOG_INFO("Trying to find any AS-DT1 on a serial port...");
        search_str = "AS-DT1";
    }
    else
    {
        LOG_INFO("Trying to find AS-DT1 with serial number %s on a serial port...", serial_number.c_str());
        search_str = serial_number;
    }

    std::vector<serial::PortInfo> ports = serial::list_ports();

    LOG_DEBUG("#######################");
    LOG_DEBUG("Listing available ports");
    LOG_DEBUG("#######################");
    for (size_t i = 0; i < ports.size(); i++)
    {
        LOG_DEBUG("%zu. Port: %s", i + 1, ports[i].port.c_str());
        LOG_DEBUG("%zu. Description: %s", i + 1, ports[i].description.c_str());
        LOG_DEBUG("%zu. Hardware id: %s", i + 1, ports[i].hardware_id.c_str());
    }

    for (const auto& pi : ports)
    {
        if (pi.description.starts_with("Sony") && pi.description.find("AS-DT1") != std::string::npos)
        {
            if (serial_number.empty())
            {
                LOG_INFO("Found AS-DT1 on port %s", pi.port.c_str());
                return std::string(pi.port);
            }

            int len = std::max(7, static_cast<int>(serial_number.size()));
            if (pi.description.substr(5, len) == serial_number || pi.description.substr(12, len) == serial_number)
            {
                LOG_INFO("Found AS-DT1 with serial number %s on port %s", serial_number.c_str(), pi.port.c_str());
                return std::string(pi.port);
            }
        }
    }

    return "";
}

static std::string get_word_from(const std::string& str, size_t pos)
{
    if (pos >= str.size()) return "";

    size_t start = pos;
    while (pos < str.size() && !std::isspace(static_cast<unsigned char>(str[pos])))
    {
        ++pos;
    }

    return str.substr(start, pos - start);
}

static std::string get_hw_id(const std::string& dev_info)
{
    size_t pos = dev_info.find(':');
    while (pos != std::string::npos)
    {
        if (pos + 1 < dev_info.size() && std::isdigit(static_cast<unsigned char>(dev_info[pos + 1])))
        {
            pos++;
            break;
        }
        pos = dev_info.find(':', pos + 1);
    }

    return get_word_from(dev_info, pos);
}

dt1_device::dt1_device(const std::string& serial_number, const std::function<void(connect_status)>& connect_cb,
                       point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb) :
    _serial_number(serial_number), _connect_cb(connect_cb), _point_cloud_cb(std::move(point_cloud_cb)),
    _imu_cb(std::move(imu_cb)), _log_cb(std::move(log_cb))
{
}

std::unique_ptr<dt1_device> dt1_device::create(const std::string& point_cloud_type, const std::string& serial_number,
                                               const std::function<void(connect_status)>& connect_cb,
                                               point_cloud_callback point_cloud_cb, imu_callback imu_cb,
                                               log_callback log_cb)
{
    if (point_cloud_type == "XYZ")
    {
        return std::make_unique<dt1_device_cdc>(serial_number, connect_cb, std::move(point_cloud_cb), std::move(imu_cb),
                                                std::move(log_cb));
    }

    if (point_cloud_type == "XYZITR")
    {
        return std::make_unique<dt1_device_hist>(serial_number, connect_cb, std::move(point_cloud_cb),
                                                 std::move(imu_cb), std::move(log_cb));
    }

    throw std::invalid_argument("Unsupported point_cloud_type: " + point_cloud_type +
                                ". Supported values are XYZ and XYZITR");
}

dt1_device::~dt1_device()
{
    stop();
    if (_serial_thread.joinable()) { _serial_thread.join(); }
}

void dt1_device::start()
{
    if (_running) return;

    if (_serial_thread.joinable()) { _serial_thread.join(); }

    _running = true;
    _latest_imu_sample_ts = 0;
    _serial_thread = std::thread(&dt1_device::run, this);
    start_impl();
}

void dt1_device::stop()
{
    if (!_running) return;

    _running = false;
    stop_impl();
    if (_serial_thread.joinable()) { _serial_thread.join(); }
}

void dt1_device::run()
{
    try
    {
        /*
         * 1, Connect
         */
        while (_running)
        {
            std::string port = find_port(_serial_number, _log_cb);
            if (!port.empty()) { LOG_INFO("Found device on port %s", port.c_str()); }
            else
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            while (_running)
            {
                if (connect(port)) break;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (!_running) break;

            std::this_thread::sleep_for(std::chrono::seconds(1));
            stop_if_streaming();

            if (!verify_device_mode()) { continue; }

            _device_info = get_device_info();

            if (_device_info.mode != _mode)
            {
                LOG_INFO("Setting mode to %s and rebooting...", dt1_modes_list[_mode].mode_str.c_str());
                send_and_receive(std::string("flmode ") + dt1_modes_list[_mode].mode_str);
                send_and_receive("reboot");
                _serial->close();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            break;
        }

        /*
         * 2. Setup and start streaming
         */
        if (_running)
        {
            _connect_cb({true, _device_info});

            if (_sync_mode == sync_mode::master)
            {
                // Extra delay for master to ensure bootup of slaves before start
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            // Set defaults to next startup
            send_and_receive_ack("flformat binary imu");
            send_and_receive_ack("fltrgout disable");
            send_and_receive_ack("fltrgin disable");
            send_and_receive_ack("flfsync 0");
            
            send_and_receive_ack("format binary imu");
            send_and_receive_ack("fltrgout riselast");

            if (_sync_mode == sync_mode::master || _sync_mode == sync_mode::none)
            {
                LOG_INFO("Setting device %s to master", _device_info.hw_id.c_str());
                char cmd[16];
                snprintf(cmd, sizeof(cmd), "fsync %.2f", 1000.0 / _frame_rate_hz);
                send_and_receive_ack("trgin disable");
                send_and_receive_ack(cmd);
            }
            else if (_sync_mode == sync_mode::slave)
            {
                LOG_INFO("Setting device %s to slave", _device_info.hw_id.c_str());
                send_and_receive_ack("trgin rise");
                send_and_receive_ack("fsync 0");
            }

            int skip_frames = _frame_rate_hz;
            while (skip_frames-- > 0)
            {
                read_line_until("END"); // skip the first second of frames for IMU data stability
            }
        }

        /*
         * 3. Main serial data receive loop.
         */
        while (_running)
        {
            process_serial_data();
        }
    }
    catch (unsupported_device_error&)
    {
        LOG_ERROR("Unsupported device version: %s", _device_info.device_version.c_str());
    }
    catch (std::exception& e)
    {
        LOG_ERROR("Exception in device %s: %s", _device_info.hw_id.c_str(), e.what());
    }

    if (!_running && _serial != nullptr) 
    { 
        send_and_receive_ack("fsync 0"); 
        send_and_receive_ack("trgin disable");
    }

    _connect_cb({false, _device_info});

    if (_serial != nullptr) { _serial->close(); }

    LOG_INFO("Closing DT1 provider");

    _running = false;
}

bool dt1_device::verify_device_mode()
{
    send_and_receive("flstart");
    std::string mode_resp = read_line();

    read_to_prompt();

    const char* expected_mode = expected_flstart_mode();
    LOG_INFO("flstart returned %s", mode_resp.c_str());
    if (mode_resp.find(expected_mode) == std::string::npos)
    {
        LOG_ERROR("Device reported wrong flstart mode: %s, expected %s", mode_resp.c_str(), expected_mode);
        LOG_ERROR("Setting %s mode and rebooting...", expected_mode);
        send_and_receive(std::string("flstart ") + expected_mode);
        send_and_receive("reboot");

        _serial->close();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return false;
    }

    return true;
}

void dt1_device::stop_if_streaming()
{
    size_t bytes_available = _serial->available();

    if (bytes_available > 0)
    {
        LOG_WARN("Device may be streaming already, sending fsync 0 to stop it");
        _serial->write("fsync 0\r");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    _serial->flush();
}

bool dt1_device::connect(const std::string& port)
{
    try
    {
        _serial = std::make_unique<serial::Serial>(port, 25E6, serial::Timeout::simpleTimeout(4000));
        _serial->flush();
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Failed opening port %s: %s", port.c_str(), e.what());
        return false;
    }

    return true;
}

device_info dt1_device::get_device_info()
{
    device_info info;
    _serial->flush();
    send_and_receive_ack("\r");
    _serial->flushInput();

    send_and_receive("ver");
    std::string version = read_line();
    std::string settings = read_line();
    std::string dev_info = read_line();
    std::string dummy = read_line();

    std::ostringstream oss;
    oss << "\n-Got-device-info:---\n";
    oss << "Version : " << version << '\n';
    oss << "Settings: " << settings << '\n';
    oss << "Device info: " << dev_info << '\n';
    oss << "Other: " << dummy << '\n';
    oss << "---------------------";

    LOG_INFO("%s", oss.str().c_str());

    info.device_version = version;
    info.hw_id = get_hw_id(dev_info);

    if (!is_supported_device(version))
    {
        LOG_ERROR("Not a supported device version: %s", version.c_str());
        throw unsupported_device_error("");
    }

    read_to_prompt();

    send_and_receive("flmode");
    std::string mode_resp = read_line();

    size_t i;
    for (i = 0; i < dt1_modes_list.size(); i++)
    {
        if (mode_resp.find(dt1_modes_list[i].mode_str) != std::string::npos) break;
    }

    if (i >= dt1_modes_list.size())
    {
        LOG_ERROR("Device reported unknown mode: %s", mode_resp.c_str());
        throw std::runtime_error("Unknown mode");
    }

    LOG_INFO("Device is using mode: %s", dt1_modes_list[i].mode_str.c_str());
    info.mode = i;

    read_to_prompt();

    return info;
}

bool dt1_device::send_and_receive_ack(const std::string& cmd) const
{
    std::string send(cmd);
    LOG_DEBUG("Sending: %s", cmd.c_str());
    send += '\r';
    _serial->write(send);

    return read_to_prompt();
}

void dt1_device::send_and_receive(const std::string& command) const
{
    std::string send(command);
    LOG_DEBUG("Sending: %s", command.c_str());
    send += '\r';
    _serial->write(send);
    read_line_until(command);
}

std::string dt1_device::read_line() const
{
    std::string line;
    _serial->readline(line);
    LOG_DEBUG("Read line: %s", line.c_str());

    auto pos = line.find_last_not_of("\r\n");
    if (pos != std::string::npos)
        line.erase(pos + 1);
    else
        line.clear();

    return line;
}

std::string dt1_device::read_line_until(const std::string& find_str) const
{
    std::string line;
    do
    {
        line.clear();
        _serial->readline(line);
        LOG_DEBUG("Read line until: %s, got:%s", find_str.c_str(), line.c_str());
    } while (line.find(find_str) == std::string::npos && _running);

    return line;
}

bool dt1_device::read_to_prompt() const
{
    std::string resp;
    LOG_DEBUG("Reading ack");

    try
    {
        do
        {
            resp = _serial->read(1);
            if (resp.empty())
            {
                LOG_DEBUG("Got empty response while waiting for prompt");
                continue;
            }
        } while (resp[0] != '>');
    }
    catch (const std::out_of_range& e)
    {
        LOG_ERROR("Did not get prompt: %s", e.what());
        return false;
    }

    LOG_DEBUG("Reading ack done");
    return true;
}

void dt1_device::read_imu()
{
    // Read IMU data
    _imu_data.clear();
    read_line_until("BEGIN IMU");

    while (_running)
    {
        std::array<uint8_t, 16> imu_line;
        int read_count = 0;
        while (read_count < 3)
        {
            read_count += _serial->read(imu_line.data() + read_count, 3 - read_count);
        }
        if (memcmp(imu_line.data(), "END", 3) == 0) break;
        while (read_count < 16)
        {
            read_count += _serial->read(imu_line.data() + read_count, 16 - read_count);
        }

        _imu_data.emplace_back(imu_line);
    }
}

uint64_t dt1_device::read_timestamp()
{
    std::string line = read_line_until("tm=");
    if (line.empty()) return 0;

    uint64_t timestamp = 0;
    sscanf(line.c_str(), "tm=%" SCNu64, &timestamp);
    return timestamp;
}

void dt1_device::unpack_imu_data(uint64_t frame_ts)
{
    _imu_samples.clear();
    bool got_reset = false;

    // read_imu() clears and refills _imu_data every frame before this runs, so
    // there's nothing to erase here - just consume it in place.
    for (const auto& sample : _imu_data)
    {
        uint8_t head = *sample.data();
        int16_t accel_x = static_cast<int16_t>(static_cast<uint16_t>(sample[1]) << 8 | sample[2]);
        int16_t accel_y = static_cast<int16_t>(static_cast<uint16_t>(sample[3]) << 8 | sample[4]);
        int16_t accel_z = static_cast<int16_t>(static_cast<uint16_t>(sample[5]) << 8 | sample[6]);
        int16_t gyro_x = static_cast<int16_t>(static_cast<uint16_t>(sample[7]) << 8 | sample[8]);
        int16_t gyro_y = static_cast<int16_t>(static_cast<uint16_t>(sample[9]) << 8 | sample[10]);
        int16_t gyro_z = static_cast<int16_t>(static_cast<uint16_t>(sample[11]) << 8 | sample[12]);
        uint8_t temp = sample[13];
        uint16_t sample_ts = static_cast<uint16_t>(static_cast<uint16_t>(sample[14]) << 8 | sample[15]);

        if (head == 0x6C && got_reset) break; // workaround for double reset issue

        if (head == 0x6C)
        {
            _latest_imu_sample_ts = frame_ts + sample_ts * 1'000LL;
            got_reset = true;
        }
        else if (_latest_imu_sample_ts > 0) { _latest_imu_sample_ts += 10'000'000LL; }

        if (_latest_imu_sample_ts > 0)
        {
            imu_sample s{_latest_imu_sample_ts + _clock_offset,
                         Vector3f{static_cast<float>(accel_y * ACCEL_SCALE), static_cast<float>(accel_x * ACCEL_SCALE),
                                  static_cast<float>(accel_z * ACCEL_SCALE)},
                         Vector3f{static_cast<float>(gyro_y * GYRO_SCALE), static_cast<float>(gyro_x * GYRO_SCALE),
                                  static_cast<float>(gyro_z * GYRO_SCALE)},
                         temp};

            _imu_samples.emplace_back(s);
        }
    }
}

bool dt1_device::set_mode(int new_mode)
{
    if (new_mode < 0 || new_mode >= static_cast<int>(dt1_modes_list.size()))
    {
        LOG_ERROR("Invalid mode number: %d", new_mode);
        return false;
    }
    _mode = new_mode;

    return true;
}

void dt1_device::set_frame_rate(int frame_rate_hz)
{
    if (frame_rate_hz >= 1 && frame_rate_hz <= 30) { _frame_rate_hz = frame_rate_hz; }
}

int dt1_device::get_frame_rate() const
{
    return _frame_rate_hz;
}

void dt1_device::set_sync_mode(sync_mode sync_mode)
{
    _sync_mode = sync_mode;
}

sync_mode dt1_device::get_sync_mode() const
{
    return _sync_mode;
}

void dt1_device::reboot()
{
    if (_serial == nullptr)
    {
        LOG_WARN("Cannot reboot: device is not connected yet");
        return;
    }

    stop();

    _serial->open();
    _serial->flush();

    LOG_INFO("Rebooting camera...");
    send_and_receive("reboot");

    _serial->close();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    start();
}

static bool is_supported_device(const std::string& version)
{
    float v = 0;

    size_t i = 0;
    while (i < version.size() && !std::isdigit(static_cast<unsigned char>(version[i])))
        i++;

    size_t start = i;
    while (i < version.size() && (std::isdigit(static_cast<unsigned char>(version[i])) || version[i] == '.'))
        i++;

    if (i != start)
    {
        std::string num = version.substr(start, i - start);
        v = std::stof(num);
    }

    return v >= minimum_supported_version;
}
