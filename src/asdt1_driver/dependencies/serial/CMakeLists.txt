cmake_minimum_required(VERSION 3.5)

set(target_name "serial")

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()

set(SRC
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
if(APPLE)
    list(APPEND SRC src/impl/unix.cc)
    list(APPEND SRC src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    list(APPEND SRC src/impl/unix.cc)
    list(APPEND SRC src/impl/list_ports/list_ports_linux.cc)
else()
    list(APPEND SRC src/impl/win.cc)
    list(APPEND SRC src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${target_name} ${SRC})
target_include_directories(${target_name} PUBLIC include)

if(APPLE)
    target_link_libraries(${target_name} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${target_name} rt pthread)
else()
    target_link_libraries( setupapi)
endif()

target_compile_options(${target_name} PRIVATE -Wno-unused-but-set-variable -Wno-sign-compare)

## Uncomment for example
#add_executable(serial_example examples/serial_example.cc)
#add_dependencies(serial_example ${target_name})
#target_link_libraries(serial_example ${target_name})


