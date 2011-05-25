include (FindPackageHandleStandardArgs)

find_path (JSON_INCLUDE_DIRS
  "libjson"
  PATHS
    /usr/local/include
    /usr/include
  PATH_SUFFIXES
    libjson
)

find_library (JSON_LIBRARIES
  "json"
  PATHS
    /usr/local/lib
    /usr/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(JSON 
  "libjson cannot be found"
  JSON_LIBRARIES
  JSON_INCLUDE_DIRS
)

MARK_AS_ADVANCED(JSON_INCLUDE_DIRS JSON_LIBRARIES)
