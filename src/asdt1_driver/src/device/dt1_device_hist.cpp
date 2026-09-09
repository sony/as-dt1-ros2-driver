/*
 * Copyright 2026 Sony Depthsensing Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dt1_device_hist.h"

#include "dt1_modes.h"
#include "logger.h"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr uint8_t bank_to_ring_full[] = {0, 12, 4, 16, 8, 20, 1, 13, 5, 17, 9, 21};
static constexpr uint8_t bank_to_ring_half[] = {0, 6, 2, 8, 4, 10};
static const uint32_t thr_exposure = 2000;

static uint8_t bank_to_ring(uint8_t bank, uint8_t mp_idx, int mode)
{
    uint8_t ring = 0;
    if (mode == cdc_mode_30m30f) { ring = bank_to_ring_half[bank] + mp_idx / 24; }
    else { ring = bank_to_ring_full[bank] + 2 * (mp_idx / 24); }

    return ring;
}

static int xioctl(int fd, unsigned long req, void* arg)
{
    int r;
    do
    {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

dt1_device_hist::dt1_device_hist(const std::string& serial_number,
                                 const std::function<void(connect_status)>& connect_cb,
                                 point_cloud_callback point_cloud_cb, imu_callback imu_cb, log_callback log_cb) :
    dt1_device(serial_number, connect_cb, std::move(point_cloud_cb), std::move(imu_cb), std::move(log_cb))
{
}

dt1_device_hist::~dt1_device_hist()
{
    stop();
}

void dt1_device_hist::start_impl()
{
    if (_v4l2_thread.joinable()) { _v4l2_thread.join(); }

    _v4l2_thread = std::thread(&dt1_device_hist::run_v4l2, this);
}

void dt1_device_hist::stop_impl()
{
    if (_v4l2_thread.joinable()) { _v4l2_thread.join(); }
    if (_fd >= 0)
    {
        v4l2_stream_off();
        _fd = -1;
    }
}

void dt1_device_hist::process_serial_data()
{
    LOG_DEBUG("Read serial frame from camera %s", _device_info.hw_id.c_str());

    if (!_running) return;

    read_imu();

    // Sync clocks (find clock offset)
    uint64_t end_ts = read_timestamp() * 1'000'000LL;

    auto now = std::chrono::high_resolution_clock::now();
    uint64_t receive_ts = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();
    _clock_offset = receive_ts - end_ts;

    // Calculate frame timestamp from end timestamp and exposure time
    // Note: this is an approximation, but we dont get start of exposure timestanmp in hist mode.
    uint64_t frame_ts =
        end_ts - static_cast<uint64_t>(dt1_modes_list[_mode].tot_exposure) * 1'000'000ULL; // subtract total exposure

    unpack_imu_data(frame_ts);

    if (_running && _imu_cb && !_imu_samples.empty()) { _imu_cb(_imu_samples); }
}

void dt1_device_hist::run_v4l2()
{
    while (_running)
    {
        _fd = open_video_device_by_serial(_serial_number);
        if (_fd < 0)
        {
            LOG_INFO("Failed opening V4L2 device. Will retry...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        break;
    }

    if (!_running) { return; }

    bool streaming = v4l2_stream_on();
    if (!streaming)
    {
        LOG_ERROR("Failed starting v4l2 streaming");
        _running = false;
    }

    _points_xyzitr.reserve(dt1_modes_list[_mode].nbr_points_x * dt1_modes_list[_mode].nbr_points_y);
    while (_running)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(_fd, &fds);

        struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
        int r = select(_fd + 1, &fds, NULL, NULL, &tv);
        if (r < 0)
        {
            LOG_ERROR("select failed");
            break;
        }
        if (r == 0)
        {
            LOG_ERROR("select timeout");
            break;
        }

        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(_fd, VIDIOC_DQBUF, &buf) < 0)
        {
            if (errno == EAGAIN) { continue; }
            LOG_ERROR("VIDIOC_DQBUF failed");
            break;
        }

        if (buf.flags & V4L2_BUF_FLAG_ERROR)
        {
            LOG_WARN("Buffer captured with error (USB transfer failure?), skipping");
            xioctl(_fd, VIDIOC_QBUF, &buf);
            continue;
        }

        if (buf.bytesused ==
            static_cast<uint32_t>(dt1_modes_list[_mode].hist_frame_width * dt1_modes_list[_mode].hist_frame_height * 2))
        {
            hist_frame_data* hist_frame = static_cast<hist_frame_data*>(_buffers[buf.index].start);
            unpack_hist_frame(hist_frame);
        }
        else
        {
            LOG_WARN("Did not receive expected number of bytes %d", buf.bytesused);
        }

        if (xioctl(_fd, VIDIOC_QBUF, &buf) < 0)
        {
            LOG_ERROR("VIDIOC_QBUF failed");
            break;
        }
    }

    _running = false;
}

void dt1_device_hist::unpack_hist_frame(const hist_frame_data* hist_frame)
{
    const uint32_t initial_bank_ts = hist_frame->bank[0].ebd.sensor_ts[0] << 24 |
                                     hist_frame->bank[0].ebd.sensor_ts[2] << 16 |
                                     hist_frame->bank[0].ebd.sensor_ts[4] << 8 | hist_frame->bank[0].ebd.sensor_ts[6];
    uint32_t cpu_ts = hist_frame->bank[0].ebd.cpu_ts[3] << 24 | hist_frame->bank[0].ebd.cpu_ts[2] << 16 |
                      hist_frame->bank[0].ebd.cpu_ts[1] << 8 | hist_frame->bank[0].ebd.cpu_ts[0];

    _points_xyzitr.resize(dt1_modes_list[_mode].nbr_points_x * dt1_modes_list[_mode].nbr_points_y);
    int dest_idx = 0;
    int nbr_banks = std::min(dt1_modes_list[_mode].nbr_banks, 12);
    for (int bank_idx = 0; bank_idx < nbr_banks; ++bank_idx) // for every bank
    {
        const bank_data& bank = hist_frame->bank[bank_idx];
        uint32_t sensor_ts = bank.ebd.sensor_ts[0] << 24 | bank.ebd.sensor_ts[2] << 16 | bank.ebd.sensor_ts[4] << 8 |
                             bank.ebd.sensor_ts[6];

        float ts = static_cast<float>(sensor_ts - initial_bank_ts) * 500.0f;

        for (int mp_idx = 0; mp_idx < 48; ++mp_idx) // for every point in bank
        {
            float x_f, y_f, z_f, height_f;
            const peak_data& peak = bank.mp[mp_idx].first;

            float ring = bank_to_ring(bank_idx, mp_idx, _mode);

            if (peak.err == 0)
            {
                uint32_t x = (peak.x[2] << 16 | peak.x[1] << 8 | peak.x[0]);
                uint32_t y = (peak.y[2] << 16 | peak.y[1] << 8 | peak.y[0]);
                uint32_t z = (peak.z[2] << 16 | peak.z[1] << 8 | peak.z[0]);
                uint32_t height = (peak.height[2] << 16 | peak.height[1] << 8 | peak.height[0]);

                if (_mode == cdc_mode_20m)
                {
                    // special treatment for 20m mode.
                    // select between low or high 1st peak
                    const bank_data& bank_high = hist_frame->bank[bank_idx + 12];
                    const peak_data& peak_high = bank_high.mp[mp_idx].first;
                    uint32_t z_high = (peak_high.z[2] << 16 | peak_high.z[1] << 8 | peak_high.z[0]);
                    if ((thr_exposure < z) && !((z != 65536) && (z_high <= thr_exposure)))
                    {
                        x = (peak_high.x[2] << 16 | peak_high.x[1] << 8 | peak_high.x[0]);
                        y = (peak_high.y[2] << 16 | peak_high.y[1] << 8 | peak_high.y[0]);
                        z = z_high;
                        height = (peak_high.height[2] << 16 | peak_high.height[1] << 8 | peak_high.height[0]);
                        uint32_t sensor_ts_high = bank_high.ebd.sensor_ts[0] << 24 | bank_high.ebd.sensor_ts[2] << 16 |
                                                  bank_high.ebd.sensor_ts[4] << 8 | bank_high.ebd.sensor_ts[6];
                        ts = static_cast<float>(sensor_ts_high - initial_bank_ts) * 500.0f;
                    }
                }
                x_f = ((x & 0x200000) == 0) ? static_cast<float>(x) / 4000.0f
                                            : -static_cast<float>(0x400000 - x) / 4000.0f;
                y_f = ((y & 0x200000) == 0) ? static_cast<float>(y) / 4000.0f
                                            : -static_cast<float>(0x400000 - y) / 4000.0f;
                z_f = static_cast<float>(z) / 4000.0f;
                height_f = static_cast<float>(height);
   
                if (z_f > 0.2f)
                {
                    _points_xyzitr[dest_idx++] = Vector6f(-y_f, x_f, z_f, height_f, ts, ring);
                }
            }
            else
            {
                LOG_DEBUG("Found invalid dot in bank=%d ts=%u its=%u mp_idx=%d err=%d", bank_idx, sensor_ts,
                          initial_bank_ts, mp_idx, peak.err);
            }
        }
    }

    if (_running && _point_cloud_cb && _clock_offset > 0)
    {
        _points_xyzitr.resize(dest_idx);
        _point_cloud_cb(static_cast<uint64_t>(cpu_ts) * 1'000'000ULL + _clock_offset, std::cref(_points_xyzitr));
    }
}

bool dt1_device_hist::v4l2_stream_on()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));

    v4l2_capability cap{};
    if (xioctl(_fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        LOG_ERROR("VIDIOC_QUERYCAP failed");
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        LOG_ERROR("Device is not a capture device");
        return false;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        LOG_ERROR("Device does not support streaming I/O");
        return false;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width = dt1_modes_list[_mode].hist_frame_width;
    fmt.fmt.pix.height = dt1_modes_list[_mode].hist_frame_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(_fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        LOG_ERROR("VIDIOC_S_FMT failed");
        return false;
    }

    v4l2_requestbuffers req{};
    req.count = 3;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        LOG_ERROR("VIDIOC_REQBUFS failed");
        return false;
    }

    _buffers.clear();
    _buffers.resize(req.count);

    for (uint32_t i = 0; i < req.count; ++i)
    {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(_fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            LOG_ERROR("VIDIOC_QUERYBUF failed");
            return false;
        }

        _buffers[i].length = buf.length;
        _buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, buf.m.offset);
        if (_buffers[i].start == MAP_FAILED)
        {
            LOG_ERROR("mmap failed");
            return false;
        }
    }

    for (uint32_t i = 0; i < req.count; ++i)
    {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(_fd, VIDIOC_QBUF, &buf) < 0)
        {
            LOG_ERROR("VIDIOC_QBUF failed");
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(_fd, VIDIOC_STREAMON, &type) < 0)
    {
        LOG_ERROR("VIDIOC_STREAMON failed");
        return false;
    }

    return true;
}

void dt1_device_hist::v4l2_stream_off()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(_fd, VIDIOC_STREAMOFF, &type) < 0) { LOG_ERROR("VIDIOC_STREAMOFF failed"); }

    for (const auto& buffer : _buffers)
    {
        if (buffer.start != nullptr && buffer.start != MAP_FAILED) { munmap(buffer.start, buffer.length); }
    }
    _buffers.clear();
    close(_fd);
}

static std::optional<std::string> devnode_from_major_minor(unsigned int major_num, unsigned int minor_num)
{
    for (const auto& entry : fs::directory_iterator("/dev"))
    {
        std::error_code ec;
        entry.symlink_status(ec);
        if (ec) continue;
        if (!fs::is_character_file(entry.path(), ec)) continue;
        if (ec) continue;

        struct stat sb{};
        if (::stat(entry.path().c_str(), &sb) != 0) continue;
        if (!S_ISCHR(sb.st_mode)) continue;

        if (major(sb.st_rdev) == major_num && minor(sb.st_rdev) == minor_num) { return entry.path().string(); }
    }
    return std::nullopt;
}

static std::vector<std::string> list_media_devices()
{
    std::vector<std::string> out;
    for (const auto& entry : fs::directory_iterator("/dev"))
    {
        const std::string name = entry.path().filename().string();
        if (name.rfind("media", 0) == 0) { out.push_back(entry.path().string()); }
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::optional<dt1_device_hist::media_match_result> dt1_device_hist::find_media_by_serial(const std::string& wanted_serial)
{
    auto devices_list = list_media_devices();
    for (const auto& media_dev : devices_list)
    {
        LOG_INFO("Trying to open media device: %s", media_dev.c_str());
        int mfd = ::open(media_dev.c_str(), O_RDONLY);
        if (mfd < 0) { continue; }

        media_device_info mdi{};
        if (xioctl(mfd, MEDIA_IOC_DEVICE_INFO, &mdi) != 0)
        {
            ::close(mfd);
            LOG_INFO("Get media_ioc_device_info failed");
            continue;
        }

        if (wanted_serial.empty() && strcmp(mdi.model, "AS-DT1") != 0)
        {
            ::close(mfd);
            LOG_INFO("Not an AS-DT1 device: %s", mdi.model);
            continue;
        }

        if (!wanted_serial.empty() && wanted_serial != mdi.serial)
        {
            ::close(mfd);
            LOG_INFO("Not correct serial: %s wanted: %s", mdi.serial, wanted_serial.c_str());
            continue;
        }

        // First call: get counts
        media_v2_topology topo{};
        if (xioctl(mfd, MEDIA_IOC_G_TOPOLOGY, &topo) != 0)
        {
            ::close(mfd);
            return std::nullopt;
        }

        std::vector<media_v2_entity> entities(topo.num_entities);
        std::vector<media_v2_interface> interfaces(topo.num_interfaces);
        std::vector<media_v2_pad> pads(topo.num_pads);
        std::vector<media_v2_link> links(topo.num_links);

        topo.ptr_entities = reinterpret_cast<__u64>(entities.data());
        topo.ptr_interfaces = reinterpret_cast<__u64>(interfaces.data());
        topo.ptr_pads = reinterpret_cast<__u64>(pads.data());
        topo.ptr_links = reinterpret_cast<__u64>(links.data());

        if (xioctl(mfd, MEDIA_IOC_G_TOPOLOGY, &topo) != 0)
        {
            ::close(mfd);
            return std::nullopt;
        }

        media_match_result result;
        result.media_dev = media_dev;
        result.mdi = mdi;

        // Build list of V4L interfaces -> /dev/videoX
        for (const auto& intf : interfaces)
        {
            if ((intf.intf_type & MEDIA_INTF_T_V4L_BASE) != MEDIA_INTF_T_V4L_BASE) { continue; }

            auto devnode = devnode_from_major_minor(intf.devnode.major, intf.devnode.minor);
            if (!devnode) { continue; }

            // We only care about /dev/video*
            if (fs::path(*devnode).filename().string().rfind("video", 0) != 0) { continue; }

            video_node_info vinfo;
            vinfo.devnode = *devnode;

            // Find interface link to entity, so we can capture entity name.
            // Links connect interfaces and entities.
            __u32 entity_id = 0;
            for (const auto& link : links)
            {
                if (link.source_id == intf.id)
                {
                    entity_id = link.sink_id;
                    break;
                }
                if (link.sink_id == intf.id)
                {
                    entity_id = link.source_id;
                    break;
                }
            }

            for (const auto& ent : entities)
            {
                if (ent.id == entity_id)
                {
                    vinfo.entity_name = reinterpret_cast<const char*>(ent.name);
                    break;
                }
            }

            // Probe V4L2 capabilities to distinguish video vs metadata nodes.
            int vfd = ::open(vinfo.devnode.c_str(), O_RDWR | O_NONBLOCK);
            if (vfd >= 0)
            {
                v4l2_capability cap{};
                if (xioctl(vfd, VIDIOC_QUERYCAP, &cap) == 0)
                {
                    const __u32 dcaps = cap.device_caps ? cap.device_caps : cap.capabilities;
                    vinfo.is_video_capture =
                        (dcaps & V4L2_CAP_VIDEO_CAPTURE) || (dcaps & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
                    vinfo.is_metadata_capture = (dcaps & V4L2_CAP_META_CAPTURE);
                }
                ::close(vfd);
            }

            result.videos.push_back(std::move(vinfo));
        }

        ::close(mfd);
        return result;
    }

    return std::nullopt;
}

int dt1_device_hist::open_video_device_by_serial(const std::string& serial)
{
    auto match = find_media_by_serial(serial);
    if (!match)
    {
        if (serial.empty()) { LOG_ERROR("No AS-DT1 media device found"); }
        else
        {
            LOG_ERROR("No media device found for serial %s", serial.c_str());
        }
        return -1;
    }

    LOG_INFO("Matched media device: %s", match->media_dev.c_str());
    LOG_INFO("  model   : %s", match->mdi.model);
    LOG_INFO("  serial  : %s", match->mdi.serial);
    LOG_INFO("  bus     : %s", match->mdi.bus_info);

    // Prefer a real video capture node, not metadata.
    for (const auto& v : match->videos)
    {
        if (v.is_video_capture && !v.is_metadata_capture)
        {
            int fd = ::open(v.devnode.c_str(), O_RDWR | O_NONBLOCK);
            if (fd >= 0)
            {
                LOG_INFO("Opening %s", v.devnode.c_str());
                return fd;
            }
        }
    }

    // Fallback: first video capture node
    for (const auto& v : match->videos)
    {
        if (v.is_video_capture)
        {
            int fd = ::open(v.devnode.c_str(), O_RDWR | O_NONBLOCK);
            if (fd >= 0)
            {
                LOG_INFO("Opening fallback %s", v.devnode.c_str());
                return fd;
            }
        }
    }

    LOG_ERROR("No suitable /dev/videoX capture node found for serial %s", serial.c_str());
    return -1;
}
