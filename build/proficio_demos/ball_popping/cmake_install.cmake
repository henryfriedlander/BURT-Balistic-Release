# Install script for directory: /home/robot/proficio_toolbox/proficio_demos/ball_popping

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping")
FILE(INSTALL DESTINATION "/home/robot/proficio_toolbox/proficio_demos/ball_popping" TYPE EXECUTABLE FILES "/home/robot/proficio_toolbox/build/proficio_demos/ball_popping/ball_popping")
  IF(EXISTS "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/ball_popping/ball_popping")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND chown -R robot:robot /home/robot/proficio_toolbox/proficio_demos/ball_popping)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
