find_package(Eigen3 REQUIRED NO_MODULE)

message(STATUS "[Eigen] found: ${EIGEN3_INCLUDE_DIRS}")

# Avoid using LGPL licensed parts of Eigen. See
# eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html
add_definitions(-DEIGEN_MPL2_ONLY)

set_license_info(Eigen3::Eigen "Eigen" ${LICENSES_DIR}/MPL-2.0.txt)