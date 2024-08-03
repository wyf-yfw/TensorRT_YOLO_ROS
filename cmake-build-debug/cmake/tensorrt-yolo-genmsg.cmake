# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tensorrt-yolo: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itensorrt-yolo:/home/wyf/catkin_ws/src/tensorrt_yolo/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tensorrt-yolo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_custom_target(_tensorrt-yolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tensorrt-yolo" "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tensorrt-yolo
  "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensorrt-yolo
)

### Generating Services

### Generating Module File
_generate_module_cpp(tensorrt-yolo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensorrt-yolo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tensorrt-yolo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tensorrt-yolo_generate_messages tensorrt-yolo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_dependencies(tensorrt-yolo_generate_messages_cpp _tensorrt-yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensorrt-yolo_gencpp)
add_dependencies(tensorrt-yolo_gencpp tensorrt-yolo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensorrt-yolo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tensorrt-yolo
  "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensorrt-yolo
)

### Generating Services

### Generating Module File
_generate_module_eus(tensorrt-yolo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensorrt-yolo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tensorrt-yolo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tensorrt-yolo_generate_messages tensorrt-yolo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_dependencies(tensorrt-yolo_generate_messages_eus _tensorrt-yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensorrt-yolo_geneus)
add_dependencies(tensorrt-yolo_geneus tensorrt-yolo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensorrt-yolo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tensorrt-yolo
  "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensorrt-yolo
)

### Generating Services

### Generating Module File
_generate_module_lisp(tensorrt-yolo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensorrt-yolo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tensorrt-yolo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tensorrt-yolo_generate_messages tensorrt-yolo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_dependencies(tensorrt-yolo_generate_messages_lisp _tensorrt-yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensorrt-yolo_genlisp)
add_dependencies(tensorrt-yolo_genlisp tensorrt-yolo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensorrt-yolo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tensorrt-yolo
  "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensorrt-yolo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tensorrt-yolo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensorrt-yolo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tensorrt-yolo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tensorrt-yolo_generate_messages tensorrt-yolo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_dependencies(tensorrt-yolo_generate_messages_nodejs _tensorrt-yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensorrt-yolo_gennodejs)
add_dependencies(tensorrt-yolo_gennodejs tensorrt-yolo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensorrt-yolo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tensorrt-yolo
  "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensorrt-yolo
)

### Generating Services

### Generating Module File
_generate_module_py(tensorrt-yolo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensorrt-yolo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tensorrt-yolo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tensorrt-yolo_generate_messages tensorrt-yolo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wyf/catkin_ws/src/tensorrt_yolo/msg/Pixel3dPoint.msg" NAME_WE)
add_dependencies(tensorrt-yolo_generate_messages_py _tensorrt-yolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensorrt-yolo_genpy)
add_dependencies(tensorrt-yolo_genpy tensorrt-yolo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensorrt-yolo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensorrt-yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensorrt-yolo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tensorrt-yolo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensorrt-yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensorrt-yolo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tensorrt-yolo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensorrt-yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensorrt-yolo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tensorrt-yolo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensorrt-yolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensorrt-yolo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tensorrt-yolo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensorrt-yolo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensorrt-yolo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensorrt-yolo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tensorrt-yolo_generate_messages_py std_msgs_generate_messages_py)
endif()
