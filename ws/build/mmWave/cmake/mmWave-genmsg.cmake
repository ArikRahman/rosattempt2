# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mmWave: 1 messages, 0 services")

set(MSG_I_FLAGS "-ImmWave:/home/arik/rosattempt2/ws/src/mmWave/msg;-Istd_msgs:/nix/store/j9vvd6ppcwwqyp0rrb58l647l018r1gi-ros-env/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mmWave_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_custom_target(_mmWave_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mmWave" "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mmWave
  "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmWave
)

### Generating Services

### Generating Module File
_generate_module_cpp(mmWave
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmWave
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mmWave_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mmWave_generate_messages mmWave_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_dependencies(mmWave_generate_messages_cpp _mmWave_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmWave_gencpp)
add_dependencies(mmWave_gencpp mmWave_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmWave_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mmWave
  "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmWave
)

### Generating Services

### Generating Module File
_generate_module_eus(mmWave
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmWave
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mmWave_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mmWave_generate_messages mmWave_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_dependencies(mmWave_generate_messages_eus _mmWave_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmWave_geneus)
add_dependencies(mmWave_geneus mmWave_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmWave_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mmWave
  "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmWave
)

### Generating Services

### Generating Module File
_generate_module_lisp(mmWave
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmWave
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mmWave_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mmWave_generate_messages mmWave_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_dependencies(mmWave_generate_messages_lisp _mmWave_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmWave_genlisp)
add_dependencies(mmWave_genlisp mmWave_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmWave_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mmWave
  "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmWave
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mmWave
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmWave
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mmWave_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mmWave_generate_messages mmWave_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_dependencies(mmWave_generate_messages_nodejs _mmWave_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmWave_gennodejs)
add_dependencies(mmWave_gennodejs mmWave_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmWave_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mmWave
  "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmWave
)

### Generating Services

### Generating Module File
_generate_module_py(mmWave
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmWave
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mmWave_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mmWave_generate_messages mmWave_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arik/rosattempt2/ws/src/mmWave/msg/data_frame.msg" NAME_WE)
add_dependencies(mmWave_generate_messages_py _mmWave_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mmWave_genpy)
add_dependencies(mmWave_genpy mmWave_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mmWave_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmWave)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mmWave
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mmWave_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmWave)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mmWave
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mmWave_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmWave)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mmWave
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mmWave_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmWave)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mmWave
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mmWave_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmWave)
  install(CODE "execute_process(COMMAND \"/nix/store/j9vvd6ppcwwqyp0rrb58l647l018r1gi-ros-env/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmWave\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mmWave
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mmWave_generate_messages_py std_msgs_generate_messages_py)
endif()
