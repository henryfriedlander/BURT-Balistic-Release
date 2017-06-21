cmake_minimum_required(VERSION 2.6)
project(proficio_toolbox_tests)

## proficio
find_package(Proficio REQUIRED)
include_directories(${PROFICIO_INCLUDE_DIRS})
link_directories(${PROFICIO_LIBRARY_DIRS})
add_definitions(${PROFICIO_DEFINITIONS})

set(GCC_COVERAGE_COMPILE_FLAGS "-frounding-math")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
    target_link_libraries(${prog} ${PROFICIO_LIBRARIES})
    install(TARGETS ${prog} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
	endforeach()
endfunction()

add_subdirectory(user_gravity_compensation)
add_subdirectory(safety_systems)
