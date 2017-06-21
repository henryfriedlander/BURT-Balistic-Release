# Install script for directory: /home/robot/proficio_toolbox/proficio_demos/test_prog

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
    SET(CMAKE_INSTALL_CONFIG_NAME "")
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
  IF(EXISTS "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod"
         RPATH "")
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
FILE(INSTALL DESTINATION "/home/robot/proficio_toolbox/proficio_demos/test_prog" TYPE EXECUTABLE FILES "/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
  IF(EXISTS "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/robot/proficio_toolbox/proficio_demos/test_prog/cube_sphere_mod")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND chown -R : /home/robot/proficio_toolbox/proficio_demos/test_prog)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/robot/proficio_toolbox/proficio_demos/test_prog/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/robot/proficio_toolbox/proficio_demos/test_prog/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
