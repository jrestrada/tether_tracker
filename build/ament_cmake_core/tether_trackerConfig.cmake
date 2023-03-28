# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tether_tracker_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tether_tracker_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tether_tracker_FOUND FALSE)
  elseif(NOT tether_tracker_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tether_tracker_FOUND FALSE)
  endif()
  return()
endif()
set(_tether_tracker_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tether_tracker_FIND_QUIETLY)
  message(STATUS "Found tether_tracker: 0.0.0 (${tether_tracker_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tether_tracker' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tether_tracker_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tether_tracker_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tether_tracker_DIR}/${_extra}")
endforeach()
