cmake_minimum_required(VERSION 3.10)

project(tensorrt_yolo)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# ============ ros =============
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        message_generation
)

add_message_files(
        FILES
        InferResult.msg
        KeyPoint.msg
        Results.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport message_runtime std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# ============= cuda ============
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# ============= tensorrt ============
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Embed_platform on")
  include_directories(/usr/include/aarch64-linux-gnu)
  link_directories(/usr/lib/aarch64-linux-gnu)
else()
  message("Embed_platform off")
  include_directories(/usr/include/x86_64-linux-gnu)
  link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# ============ opencv ============
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
add_definitions("-Wall -g")

# ============ yolo infer shared lib ============
include_directories(${PROJECT_SOURCE_DIR}/include)
cuda_add_library(yolo_infer
        src/infer.cpp
        src/calibrator.cpp
        src/postprocess.cu
        src/preprocess.cu
        src/config.cpp
        src/config.cpp
)
target_link_libraries(yolo_infer
        cudart
        nvinfer
        nvonnxparser
        ${OpenCV_LIBS})

# ======== main execute file ========
add_definitions("-Wall -g")

add_executable(camera_infer_node
        src/camera_infer.cpp
        include/camera_infer.h
        include/byte_tracker.h
        include/bytekalman_filter.h
        include/lapjv.h
        include/strack.h
        src/bytekalman_filter.cpp
        src/byte_tracker.cpp
        src/lapjv.cpp
        src/strack.cpp
        include/KeyPoint.h
)
add_definitions("-Wall -g")

target_link_libraries(camera_infer_node
        yolo_infer
        ${catkin_LIBRARIES}
)
add_executable(image_infer_node
        src/image_infer.cpp
        include/image_infer.h
        include/KeyPoint.h
)
target_link_libraries(image_infer_node
        yolo_infer
        ${catkin_LIBRARIES}
)
