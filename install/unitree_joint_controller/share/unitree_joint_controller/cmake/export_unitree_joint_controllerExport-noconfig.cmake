#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "unitree_joint_controller::unitree_joint_controller" for configuration ""
set_property(TARGET unitree_joint_controller::unitree_joint_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(unitree_joint_controller::unitree_joint_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libunitree_joint_controller.so"
  IMPORTED_SONAME_NOCONFIG "libunitree_joint_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS unitree_joint_controller::unitree_joint_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_unitree_joint_controller::unitree_joint_controller "${_IMPORT_PREFIX}/lib/libunitree_joint_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
