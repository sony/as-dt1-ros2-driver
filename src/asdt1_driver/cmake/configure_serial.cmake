
# Build serial as a static library linked into the component; a shared libserial.so is not installed
# and the component would fail to load at runtime when BUILD_SHARED_LIBS is ON
set(_ASDT1_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(${DEPENDENCIES_DIR}/serial)
set(BUILD_SHARED_LIBS ${_ASDT1_BUILD_SHARED_LIBS})
set_target_properties(serial PROPERTIES POSITION_INDEPENDENT_CODE ON)

set_license_info(serial "serial" ${DEPENDENCIES_DIR}/serial/LICENSE)