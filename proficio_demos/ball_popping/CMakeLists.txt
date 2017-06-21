cmake_minimum_required(VERSION 2.6)
project(ball_popping)

## proficio_toolbox
find_package(Proficio REQUIRED)
include_directories(${PROFICIO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)
link_directories(${PROFICIO_LIBRARY_DIRS})
add_definitions(${PROFICIO_DEFINITIONS})

set(GCC_COVERAGE_COMPILE_FLAGS "-frounding-math")

add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

add_executable(${PROJECT_NAME} ball_popping.cpp)
target_link_libraries(${PROJECT_NAME} ${PROFICIO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)

install(CODE "execute_process(COMMAND chown -R ${OWNER}:${OWNER} ${CMAKE_CURRENT_SOURCE_DIR})")
