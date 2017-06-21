cmake_minimum_required(VERSION 2.6)

project(proficio_toolbox)
set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 1)
set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}")
set(${PROJECT_NAME}_SOVERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}")

# debug build by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Fail to link if there are undefined symbols in a shared library
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

# Location of custom Finders
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")

### Options
#TODO(ab): Set up and test Debian installation option
#option(CONFIG_DEBIAN "Set to ON to copy standard proficio-config.cmake, required for Debian packaging" OFF)
#TODO(ab): Test all of these options!
option(CONFIG_PACKAGE "Set to ON to set up CPACK variables necessary for packaging" OFF)
option(NON_REALTIME "Set to ON to avoid building code that depends on a real-time operating system" OFF)
option(OPTIMIZE_FOR_PROCESSOR "Set to ON to build binaries that are optimized for this specific computer and can't be copied to other machines" OFF)

if(OPTIMIZE_FOR_PROCESSOR)
  # TODO(dc): Does this turn on sse2 if supported by processor? What about -mfpmath=sse?
  set(NEW_FLAGS "-march=native -mtune=native")
  
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NEW_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NEW_FLAGS}")
  
  message(STATUS "OPTIMIZE_FOR_PROCESSOR: ${NEW_FLAGS}")
  set(NEW_FLAGS)  # unset
endif()

# Record the name of the user executing the cmake command
execute_process(COMMAND whoami OUTPUT_VARIABLE current_user OUTPUT_STRIP_TRAILING_WHITESPACE)
set(OWNER ${current_user} CACHE STRING "Cache the cmake var")
### Dependencies

# Record these for when the library is installed
set(exported_include_dirs "")
set(exported_link_dirs "")
set(exported_definitions "")
set(exported_libraries "")

## Put proficio_toolbox headers in the include path
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_BINARY_DIR}/include")

find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

set(exported_include_dirs ${exported_include_dirs} ${BARRETT_INCLUDE_DIRS})
set(exported_link_dirs ${exported_link_dirs} ${BARRETT_LIBRARY_DIRS})
set(exported_definitions ${exported_definitions} ${BARRETT_DEFINITIONS})

find_package(CGAL REQUIRED COMPONENTS Core)
include(${CGAL_USE_FILE})
include_directories(BEFORE ../include)

set(exported_libraries ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${BARRETT_LIBRARIES})

### Installation
# The trailing slash in "include/" is important
install(DIRECTORY include/
  DESTINATION include
  PATTERN ".git" EXCLUDE
  PATTERN ".h"
  PATTERN ".svn" EXCLUDE
  PATTERN "~" EXCLUDE
)

add_subdirectory(src)

## A config file that other cmake projects can use to build against proficio_toolbox
configure_file(${PROJECT_SOURCE_DIR}/cmake/proficio-config.cmake.in ${PROJECT_SOURCE_DIR}/cmake/proficio-config.cmake
  ESCAPE_QUOTES @ONLY
)
#TODO(ab): Create the standard proficio-config.cmake needed for Debian installation
#if(CONFIG_DEBIAN)  
#message(STATUS "Using standard proficio-config.cmake for Debian installation.")
#  install(FILES ${PROJECT_SOURCE_DIR}/cmake/debian/proficio-config.cmake
#          DESTINATION share/proficio
#  )
#else()
  message(STATUS "Using compile-time generated proficio-config.cmake.")
  install(FILES ${PROJECT_SOURCE_DIR}/cmake/proficio-config.cmake
          DESTINATION share/proficio
  )
#endif()

set(Proficio_DIR ${PROJECT_SOURCE_DIR}/cmake/)
add_subdirectory(configurations)
add_subdirectory(proficio_demos)
add_subdirectory(programs)

### Packaging Options
if(CONFIG_PACKAGE)
  include(InstallRequiredSystemLibraries)

  set(CPACK_SET_DESTDIR "on")
  set(CPACK_SOURCE_GENERATOR "TGZ")
  set(CPACK_GENERATOR "DEB")

  set(CPACK_PACKAGE_NAME "${PROJECT_NAME}-${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  set(CPACK_PACKAGE_DIRECTORY "${PROJECT_SOURCE_DIR}/debian")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Barrett Technology C++ toolbox for Proficio")
  set(CPACK_PACKAGE_DESCRIPTION "${PROJECT_NAME} is an extension of libbarrett that includes tools and demos specific to the Proficio.")
  set(CPACK_PACKAGE_VENDOR "Barrett Technology")
  set(CPACK_PACKAGE_CONTACT "Barrett Support <support@barrett.com>")  

  set(CPACK_PACKAGE_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
  set(CPACK_PACKAGE_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
  set(CPACK_PACKAGE_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")
  set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}_${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README.txt")
  set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/COPYING.txt")  
  set(CPACK_SOURCE_IGNORE_FILES "proficio_toolbox_programming_manual.doc;property_list.py;cmake/barrett-config.cmake$;bt-wam*;proficio_toolbox.so*;Doxyfile$;install_manifest.txt;CMakeCache.txt;CMakeFiles/;cmake_install.cmake;^/CPack;.*/_CPack;.*/CPack;/.*\\\\.tar\\\\.gz;/.*\\\\.deb;.project;.cproject;.pyproject;Makefile;//CVS/;/\\\\.svn/;\\\\.swp$")
  
  set(CPACK_DEBIAN_PACKAGE_NAME "${PROJECT_NAME}-dev")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libconfig-barrett (>=1.4.5), libboost-thread-dev (>= 1.45.0), libboost-python-dev (>=1.45.0), libgsl0-dev (>=1.14), libncurses5-dev, python-dev (>=2.7)")
  if(NOT NON_REALTIME)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libxenomai-dev (>= 2.5.5.2)")
  endif()
  set(CPACK_DEBIAN_PACKAGE_DESCRIPTION_SUMMARY "${CPACK_PACKAGE_DESCRIPTION_SUMMARY}")
  set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "${CPACK_PACKAGE_DESCRIPTION}")
  set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
  set(CPACK_DEBIAN_PACKAGE_SECTION "library")
  set(CPACK_DEBIAN_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/cmake/debian/postinst")
  
  include(CPack)
endif()
