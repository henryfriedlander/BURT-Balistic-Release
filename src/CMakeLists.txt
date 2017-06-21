# Always compile these files
set(proficio_SOURCES
  systems/haptics/cylindrical_assist.cpp
  systems/haptics/shrinking_sphere.cpp
  systems/haptics/haptic_floor.cpp
  systems/haptics/haptic_sphere.cpp
  )

set(libs ${Boost_LIBRARIES} ${GSL_LIBRARIES})  #TODO(dc): libconfig finder?

add_library(proficio ${proficio_SOURCES})
target_link_libraries(proficio ${libs})
set_target_properties(proficio PROPERTIES
  OUTPUT_NAME proficio
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION ${${PROJECT_NAME}_SOVERSION}

  # Allow creating static and shared libs without conflicts
  CLEAN_DIRECT_OUTPUT 1
  )

# Record the libraries that applications should link against if they want to use libbarrett
set(exported_libraries ${exported_libraries} ${libs} proficio PARENT_SCOPE)


install(TARGETS proficio
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(CODE "message(STATUS \"inside ${exported_libraries}...\")")
install(CODE "message(STATUS \"Running ldconfig...\")")
install(CODE "execute_process(COMMAND ldconfig)")
