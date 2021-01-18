# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mapping: 0 messages, 1 services")

set(MSG_I_FLAGS "-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mapping_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_custom_target(_mapping_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapping" "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" "nav_msgs/MapMetaData:geometry_msgs/Pose:nav_msgs/OccupancyGrid:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(mapping
  "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping
)

### Generating Module File
_generate_module_cpp(mapping
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mapping_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mapping_generate_messages mapping_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_dependencies(mapping_generate_messages_cpp _mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_gencpp)
add_dependencies(mapping_gencpp mapping_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(mapping
  "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping
)

### Generating Module File
_generate_module_eus(mapping
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mapping_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mapping_generate_messages mapping_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_dependencies(mapping_generate_messages_eus _mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_geneus)
add_dependencies(mapping_geneus mapping_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(mapping
  "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping
)

### Generating Module File
_generate_module_lisp(mapping
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mapping_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mapping_generate_messages mapping_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_dependencies(mapping_generate_messages_lisp _mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_genlisp)
add_dependencies(mapping_genlisp mapping_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(mapping
  "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping
)

### Generating Module File
_generate_module_nodejs(mapping
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mapping_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mapping_generate_messages mapping_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_dependencies(mapping_generate_messages_nodejs _mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_gennodejs)
add_dependencies(mapping_gennodejs mapping_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(mapping
  "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping
)

### Generating Module File
_generate_module_py(mapping
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mapping_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mapping_generate_messages mapping_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/florentin/rob_mob/src/mapping/srv/BinaryMap.srv" NAME_WE)
add_dependencies(mapping_generate_messages_py _mapping_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapping_genpy)
add_dependencies(mapping_genpy mapping_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapping_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapping
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(mapping_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapping
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(mapping_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapping
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(mapping_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapping
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(mapping_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapping
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(mapping_generate_messages_py nav_msgs_generate_messages_py)
endif()
