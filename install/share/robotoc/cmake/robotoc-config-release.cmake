#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robotoc::robotoc" for configuration "Release"
set_property(TARGET robotoc::robotoc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robotoc::robotoc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobotoc.so"
  IMPORTED_SONAME_RELEASE "librobotoc.so"
  )

list(APPEND _cmake_import_check_targets robotoc::robotoc )
list(APPEND _cmake_import_check_files_for_robotoc::robotoc "${_IMPORT_PREFIX}/lib/librobotoc.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
