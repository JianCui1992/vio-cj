cmake_minimum_required(VERSION 2.8.3)
project(slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV 3.3 REQUIRED)
find_package(Eigen3)

set(LIBS
${OpenCV_LIBS}
${PROJECT_SOURCE_DIR}/../camera_model/lib/libcamera_model.so
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(

  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/../camera_model/include
  ${PROJECT_SOURCE_DIR}/src/common
  ${PROJECT_SOURCE_DIR}/src/feature
  ${PROJECT_SOURCE_DIR}/src/system
  ${PROJECT_SOURCE_DIR}/src/track
  ${PROJECT_SOURCE_DIR}/src/initial
  ${PROJECT_SOURCE_DIR}/src/factor
  ${PROJECT_SOURCE_DIR}/src/utility
)

add_library(${PROJECT_NAME} SHARED
	src/common/common_include.cpp
	src/feature/feature_tracker.cpp
	src/feature/feature_tracker_node.cpp
	src/system/data.cpp
	src/system/system.cpp
	src/track/tracking.cpp
  src/track/estimator.cpp
  src/track/feature_manager.cpp
  src/utility/utility.cpp
  src/initial/initial_alignment.cpp
  src/initial/initial_ex_rotation.cpp
  src/initial/initial_sfm.cpp
  src/initial/solve_5pts.cpp
  src/factor/marginalization_factor.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${LIBS})


add_executable(main
    src/test/main.cpp
)
target_link_libraries(main ${OpenCV_LIBS} ${LIBS} ${PROJECT_NAME})
