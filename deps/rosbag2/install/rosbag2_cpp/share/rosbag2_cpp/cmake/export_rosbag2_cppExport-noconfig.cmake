#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rosbag2_cpp::rosbag2_cpp" for configuration ""
set_property(TARGET rosbag2_cpp::rosbag2_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rosbag2_cpp::rosbag2_cpp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librosbag2_cpp.so"
  IMPORTED_SONAME_NOCONFIG "librosbag2_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rosbag2_cpp::rosbag2_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_rosbag2_cpp::rosbag2_cpp "${_IMPORT_PREFIX}/lib/librosbag2_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
