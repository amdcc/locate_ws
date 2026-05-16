# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_uart_to_mcu_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED uart_to_mcu_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(uart_to_mcu_FOUND FALSE)
  elseif(NOT uart_to_mcu_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(uart_to_mcu_FOUND FALSE)
  endif()
  return()
endif()
set(_uart_to_mcu_CONFIG_INCLUDED TRUE)

# output package information
if(NOT uart_to_mcu_FIND_QUIETLY)
  message(STATUS "Found uart_to_mcu: 0.0.0 (${uart_to_mcu_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'uart_to_mcu' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${uart_to_mcu_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(uart_to_mcu_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${uart_to_mcu_DIR}/${_extra}")
endforeach()
