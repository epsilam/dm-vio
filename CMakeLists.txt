cmake_minimum_required(VERSION 3.0.2)
project(dmvio_ros)

set(CMAKE_CXX_STANDARD 17)

IF(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()
set(BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  message_generation
  rosbag
  image_transport
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Pangolin 0.6 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(LibZip QUIET)

################



IF(${Boost_VERSION} GREATER_EQUAL 106500)
    message("Building with stacktrace support.")
	set(STACKTRACE_LIBRARIES dl)
	set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
ENDIF()


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE(Mac OS X specific code)
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
    "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
	"-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
	"-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
	"-O3 -DNDEBUG"
)

add_message_files(
  FILES
  DMVIOPoseMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs message_runtime image_transport
)

file(GLOB dso_SOURCE_FILES
  "${PROJECT_SOURCE_DIR}/src/dso/FullSystem/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/dso/OptimizationBackend/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/dso/util/*.cpp"
)

file(GLOB dmvio_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/IMU/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/GTSAMIntegration/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/IMUInitialization/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/util/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/live/*.cpp"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/dso
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

# decide if we have pangolin
if (Pangolin_FOUND)
	message("--- found PANGOLIN, compiling with pangolin library.")
	include_directories( ${Pangolin_INCLUDE_DIRS} )
	set(dso_pangolin_SOURCE_FILES
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/Pangolin/KeyFrameDisplay.cpp
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/Pangolin/PangolinDSOViewer.cpp
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/Pangolin/FollowCamMode.cpp)
	set(HAS_PANGOLIN 1)
else ()
	message("--- could not find PANGOLIN, not compiling with pangolin library.")
	message("    this means there will be no 3D display / GUI available for dmvio_dataset.")
	set(dso_pangolin_SOURCE_FILES )
	set(HAS_PANGOLIN 0)
endif ()

# decide if we have openCV
if (OpenCV_FOUND)
	message("--- found OpenCV, compiling with opencv library.")
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	set(dso_opencv_SOURCE_FILES
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/OpenCV/ImageRW_OpenCV.cpp)
	set(HAS_OPENCV 1)
else ()
	message("--- could not find OpenCV, not compiling with opencv library.")
	message("    this means there will be no image display, and image read / load functionality.")
	set(dso_opencv_SOURCE_FILES
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/ImageDisplay_dummy.cpp
			${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/ImageRW_dummy.cpp)
	set(HAS_OPENCV 0)
endif ()

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
	message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
	add_definitions(-DHAS_ZIPLIB=1)
	include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} )
else()
	message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
	set(LIBZIP_LIBRARY "")
endif()

# compile main library.
include_directories(${YAML_CPP_INCLUDE_DIR})
add_library(dmvio ${dso_SOURCE_FILES} ${dmvio_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()


# build main executable (only if we have both OpenCV and Pangolin)
if (OpenCV_FOUND AND Pangolin_FOUND)
	set(DMVIO_LINKED_LIBRARIES boost_system ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} gtsam ${YAML_CPP_LIBRARIES} ${STACKTRACE_LIBRARIES})

  message("--- compiling dmvio_ros_node")
  add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/src/main_ros.cpp ${PROJECT_SOURCE_DIR}/src/dso/IOWrapper/OutputWrapper/ROSOutputWrapper.cpp)
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
  target_link_libraries(${PROJECT_NAME}_node dmvio ${DMVIO_LINKED_LIBRARIES} ${catkin_LIBRARIES})

else()
	message("--- not building dmvio_ros, since we either don't have openCV or Pangolin.")
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY configs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.txt"
  PATTERN "*.yaml"
)

add_subdirectory(test)
