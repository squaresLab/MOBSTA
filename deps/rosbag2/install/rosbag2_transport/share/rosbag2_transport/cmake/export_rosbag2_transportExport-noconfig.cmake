#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rosbag2_transport::rosbag2_transport" for configuration ""
set_property(TARGET rosbag2_transport::rosbag2_transport APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rosbag2_transport::rosbag2_transport PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librosbag2_transport.so"
  IMPORTED_SONAME_NOCONFIG "librosbag2_transport.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rosbag2_transport::rosbag2_transport )
list(APPEND _IMPORT_CHECK_FILES_FOR_rosbag2_transport::rosbag2_transport "${_IMPORT_PREFIX}/lib/librosbag2_transport.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
