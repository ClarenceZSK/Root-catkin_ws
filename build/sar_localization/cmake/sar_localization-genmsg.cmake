# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sar_localization: 3 messages, 0 services")

set(MSG_I_FLAGS "-Isar_localization:/root/catkin_ws/src/sar_localization/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sar_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Motor.msg" NAME_WE)
add_custom_target(_sar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sar_localization" "/root/catkin_ws/src/sar_localization/msg/Motor.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Imu.msg" NAME_WE)
add_custom_target(_sar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sar_localization" "/root/catkin_ws/src/sar_localization/msg/Imu.msg" "std_msgs/Header"
)

get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Csi.msg" NAME_WE)
add_custom_target(_sar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sar_localization" "/root/catkin_ws/src/sar_localization/msg/Csi.msg" "std_msgs/Float64MultiArray:std_msgs/Header:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
)
_generate_msg_cpp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Csi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
)
_generate_msg_cpp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_cpp(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sar_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Motor.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_cpp _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Imu.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_cpp _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Csi.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_cpp _sar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sar_localization_gencpp)
add_dependencies(sar_localization_gencpp sar_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
)
_generate_msg_lisp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Csi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
)
_generate_msg_lisp(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_lisp(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sar_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Motor.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_lisp _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Imu.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_lisp _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Csi.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_lisp _sar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sar_localization_genlisp)
add_dependencies(sar_localization_genlisp sar_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
)
_generate_msg_py(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Csi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
)
_generate_msg_py(sar_localization
  "/root/catkin_ws/src/sar_localization/msg/Imu.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
)

### Generating Services

### Generating Module File
_generate_module_py(sar_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sar_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sar_localization_generate_messages sar_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Motor.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_py _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Imu.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_py _sar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/sar_localization/msg/Csi.msg" NAME_WE)
add_dependencies(sar_localization_generate_messages_py _sar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sar_localization_genpy)
add_dependencies(sar_localization_genpy sar_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sar_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sar_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sar_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sar_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(sar_localization_generate_messages_py std_msgs_generate_messages_py)
