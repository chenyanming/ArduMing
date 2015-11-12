# Install script for directory: /Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/build/src/libsbp-static.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp-static.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp-static.a")
    execute_process(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp-static.a")
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/build/src/libsbp.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp.dylib")
    execute_process(COMMAND "/usr/bin/install_name_tool"
      -id "libsbp.dylib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsbp.dylib")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/libsbp" TYPE FILE FILES
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/acquisition.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/bootload.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/common.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/edc.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/ext_events.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/file_io.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/flash.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/logging.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/navigation.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/observation.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/piksi.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/sbp.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/settings.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/system.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/tracking.h"
    "/Users/chandamon/ArduMing/Test/sbp_tutorial/libsbp/c/include/libsbp/version.h"
    )
endif()

