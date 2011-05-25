include (FindPackageHandleStandardArgs)

find_path (FCGIPP_INCLUDE_DIRS
  "fastcgi++"
  PATHS
    /usr/local/include
    /usr/include
  PATH_SUFFIXES
    fastcgi++
)

find_library (FCGIPP_LIBRARIES
  "fastcgipp"
  PATHS
    /usr/local/lib
    /usr/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(FCGIPP 
  "FastCGI++ library cannot be found"
  FCGIPP_LIBRARIES
  FCGIPP_INCLUDE_DIRS
)

MARK_AS_ADVANCED(FCGIPP_INCLUDE_DIRS FCGIPP_LIBRARIES)
