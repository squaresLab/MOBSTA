#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rosbag2_compression_zstd::rosbag2_compression_zstd" for configuration ""
set_property(TARGET rosbag2_compression_zstd::rosbag2_compression_zstd APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rosbag2_compression_zstd::rosbag2_compression_zstd PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librosbag2_compression_zstd.so"
  IMPORTED_SONAME_NOCONFIG "librosbag2_compression_zstd.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rosbag2_compression_zstd::rosbag2_compression_zstd )
list(APPEND _IMPORT_CHECK_FILES_FOR_rosbag2_compression_zstd::rosbag2_compression_zstd "${_IMPORT_PREFIX}/lib/librosbag2_compression_zstd.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
