cmake_minimum_required(VERSION 3.10)
project(imu_gps_localization)

add_compile_options(-std=c++14)

find_package(Eigen3  REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  third_party/GeographicLib/include
)

add_executable(imu_gps_fusion
    main.cpp
	src/eskf.cpp
    src/initializer.cpp
    src/imu_processor.cpp
    src/gps_processor.cpp
    src/imu_reader.cpp
    src/gps_reader.cpp
)

target_link_libraries(imu_gps_fusion ${EIGEN3_LIBS}  pthread atomic)