# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_p5_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED p5_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(p5_FOUND FALSE)
  elseif(NOT p5_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(p5_FOUND FALSE)
  endif()
  return()
endif()
set(_p5_CONFIG_INCLUDED TRUE)

# output package information
if(NOT p5_FIND_QUIETLY)
  message(STATUS "Found p5: 0.0.0 (${p5_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'p5' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${p5_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(p5_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${p5_DIR}/${_extra}")
endforeach()
