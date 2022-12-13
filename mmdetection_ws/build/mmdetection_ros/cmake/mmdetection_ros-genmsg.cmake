# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mmdetection_ros: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Ivision_msgs:/opt/ros/melodic/share/vision_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mmdetection_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_custom_target(_mmdetection_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mmdetection_ros" "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" "sensor_msgs/Image:geometry_msgs/Pose2D:vision_msgs/ObjectHypothesisWithPose:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:vision_msgs/Detection2D:std_msgs/Header:geometry_msgs/Point:vision_msgs/BoundingBox2D:vision_msgs/Detection2DArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(mmdetection_ros
  "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2DArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmdetection_ros
)

### Generating Module File
_generate_module_cpp(mmdetection_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmdetection_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mmdetection_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mmdetection_ros_generate_messages mmdetection_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_dependencies(mmdetection_ros_generate_messages_cpp _mmdetection_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmdetection_ros_gencpp)
add_dependencies(mmdetection_ros_gencpp mmdetection_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmdetection_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(mmdetection_ros
  "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2DArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmdetection_ros
)

### Generating Module File
_generate_module_eus(mmdetection_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmdetection_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mmdetection_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mmdetection_ros_generate_messages mmdetection_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_dependencies(mmdetection_ros_generate_messages_eus _mmdetection_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmdetection_ros_geneus)
add_dependencies(mmdetection_ros_geneus mmdetection_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmdetection_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(mmdetection_ros
  "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2DArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmdetection_ros
)

### Generating Module File
_generate_module_lisp(mmdetection_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmdetection_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mmdetection_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mmdetection_ros_generate_messages mmdetection_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_dependencies(mmdetection_ros_generate_messages_lisp _mmdetection_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmdetection_ros_genlisp)
add_dependencies(mmdetection_ros_genlisp mmdetection_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmdetection_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(mmdetection_ros
  "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2DArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmdetection_ros
)

### Generating Module File
_generate_module_nodejs(mmdetection_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmdetection_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mmdetection_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mmdetection_ros_generate_messages mmdetection_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_dependencies(mmdetection_ros_generate_messages_nodejs _mmdetection_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmdetection_ros_gennodejs)
add_dependencies(mmdetection_ros_gennodejs mmdetection_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmdetection_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(mmdetection_ros
  "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/ObjectHypothesisWithPose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2D.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/melodic/share/vision_msgs/cmake/../msg/Detection2DArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmdetection_ros
)

### Generating Module File
_generate_module_py(mmdetection_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmdetection_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mmdetection_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mmdetection_ros_generate_messages mmdetection_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrol2/mmdetection_ws/src/mmdetection_ros/srv/mmdetSrv.srv" NAME_WE)
add_dependencies(mmdetection_ros_generate_messages_py _mmdetection_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmdetection_ros_genpy)
add_dependencies(mmdetection_ros_genpy mmdetection_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmdetection_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmdetection_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmdetection_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mmdetection_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(mmdetection_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET vision_msgs_generate_messages_cpp)
  add_dependencies(mmdetection_ros_generate_messages_cpp vision_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmdetection_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmdetection_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mmdetection_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(mmdetection_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET vision_msgs_generate_messages_eus)
  add_dependencies(mmdetection_ros_generate_messages_eus vision_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmdetection_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmdetection_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mmdetection_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(mmdetection_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET vision_msgs_generate_messages_lisp)
  add_dependencies(mmdetection_ros_generate_messages_lisp vision_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmdetection_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmdetection_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mmdetection_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(mmdetection_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET vision_msgs_generate_messages_nodejs)
  add_dependencies(mmdetection_ros_generate_messages_nodejs vision_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmdetection_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmdetection_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmdetection_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mmdetection_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(mmdetection_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET vision_msgs_generate_messages_py)
  add_dependencies(mmdetection_ros_generate_messages_py vision_msgs_generate_messages_py)
endif()
