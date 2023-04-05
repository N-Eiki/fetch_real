# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fetch_images: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fetch_images_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_custom_target(_fetch_images_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetch_images" "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(fetch_images
  "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_images
)

### Generating Module File
_generate_module_cpp(fetch_images
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_images
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fetch_images_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fetch_images_generate_messages fetch_images_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_dependencies(fetch_images_generate_messages_cpp _fetch_images_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_images_gencpp)
add_dependencies(fetch_images_gencpp fetch_images_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_images_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(fetch_images
  "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_images
)

### Generating Module File
_generate_module_eus(fetch_images
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_images
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fetch_images_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fetch_images_generate_messages fetch_images_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_dependencies(fetch_images_generate_messages_eus _fetch_images_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_images_geneus)
add_dependencies(fetch_images_geneus fetch_images_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_images_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(fetch_images
  "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_images
)

### Generating Module File
_generate_module_lisp(fetch_images
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_images
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fetch_images_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fetch_images_generate_messages fetch_images_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_dependencies(fetch_images_generate_messages_lisp _fetch_images_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_images_genlisp)
add_dependencies(fetch_images_genlisp fetch_images_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_images_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(fetch_images
  "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_images
)

### Generating Module File
_generate_module_nodejs(fetch_images
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_images
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fetch_images_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fetch_images_generate_messages fetch_images_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_dependencies(fetch_images_generate_messages_nodejs _fetch_images_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_images_gennodejs)
add_dependencies(fetch_images_gennodejs fetch_images_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_images_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(fetch_images
  "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_images
)

### Generating Module File
_generate_module_py(fetch_images
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_images
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fetch_images_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fetch_images_generate_messages fetch_images_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rl-user/repos/nagata/catkin_ws/src/fetch_images/srv/GetImages.srv" NAME_WE)
add_dependencies(fetch_images_generate_messages_py _fetch_images_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_images_genpy)
add_dependencies(fetch_images_genpy fetch_images_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_images_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_images)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_images
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(fetch_images_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fetch_images_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_images)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_images
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(fetch_images_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fetch_images_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_images)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_images
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(fetch_images_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fetch_images_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_images)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_images
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(fetch_images_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fetch_images_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_images)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_images\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_images
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(fetch_images_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fetch_images_generate_messages_py std_msgs_generate_messages_py)
endif()
