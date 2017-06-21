cmake_minimum_required(VERSION 2.6)
project(proficio_toolbox_programs)

## proficio_toolbox
find_package(Proficio REQUIRED)
include_directories(${PROFICO_INCLUDE_DIRS})
link_directories(${PROFICIO_LIBRARY_DIRS})
add_definitions(${PROFICIO_DEFINITIONS})

set(programs
  state_toggle
  homecheck
)
foreach (prog ${programs})
  add_executable(${prog} ${prog}.cpp)
  target_link_libraries(${prog} ${PROFICIO_LIBRARIES})

  install(TARGETS ${prog} RUNTIME DESTINATION bin)
endforeach()
