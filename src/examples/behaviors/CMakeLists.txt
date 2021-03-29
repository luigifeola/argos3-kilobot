if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

  #
  # Lab0: Blinky
  #
  add_executable(blinky blinky.c)
  target_link_libraries(blinky argos3plugin_simulator_kilolib)
 
  #
  # Lab1.2: Simple Movement
  #
  add_executable(simple_movement simple_movement.c)
  target_link_libraries(simple_movement argos3plugin_simulator_kilolib)
  #
  # Checking max_turning_ticks
  #
  add_executable(simple_rotation simple_rotation.c)
  target_link_libraries(simple_rotation argos3plugin_simulator_kilolib)

  #
  # Lab1.3: Non-blocked Movement
  #
  add_executable(nonblocked_movement nonblocked_movement.c)
  target_link_libraries(nonblocked_movement argos3plugin_simulator_kilolib)

  #
  # Lab2.1-2.2: Test Speaker and Test Listener
  #
  add_executable(test_speaker test_speaker.c)
  target_link_libraries(test_speaker argos3plugin_simulator_kilolib)
  add_executable(test_listener test_listener.c)
  target_link_libraries(test_listener argos3plugin_simulator_kilolib)

  #
  # Custom Test to check sending message maximum frequency Speaker-Listener
  #
  add_executable(custom_test_speaker custom_test_speaker.c)
  target_link_libraries(custom_test_speaker argos3plugin_simulator_kilolib)
  add_executable(custom_test_listener custom_test_listener.c)
  target_link_libraries(custom_test_listener argos3plugin_simulator_kilolib)

  #
  # Lab2.3-2.4: Modified Test Speaker and Test Listener
  #
  add_executable(test_speaker_mod test_speaker_mod.c)
  target_link_libraries(test_speaker_mod argos3plugin_simulator_kilolib)
  add_executable(test_listener_mod test_listener_mod.c)
  target_link_libraries(test_listener_mod argos3plugin_simulator_kilolib)

  #
  # Lab3: Disperse
  #
  add_executable(disperse disperse.c)
  target_link_libraries(disperse argos3plugin_simulator_kilolib)

  #
  # Lab4: Orbit
  #
  add_executable(orbit_star orbit_star.c)
  target_link_libraries(orbit_star argos3plugin_simulator_kilolib)
  add_executable(orbit_planet orbit_planet.c)
  target_link_libraries(orbit_planet argos3plugin_simulator_kilolib)

  #
  # Lab5: Move to Light
  #
  add_executable(move_to_light move_to_light.c)
  target_link_libraries(move_to_light argos3plugin_simulator_kilolib)

  #
  # Lab6: Simple Gradient
  #
  add_executable(gradient_simple gradient_simple.c)
  target_link_libraries(gradient_simple argos3plugin_simulator_kilolib)

  #
  # Lab7: Sync
  #
  add_executable(sync sync.c)
  target_link_libraries(sync argos3plugin_simulator_kilolib)


  #
  # ARK loop function: demoC
  #
  add_executable(forager forager.c)  
  target_link_libraries(forager argos3plugin_simulator_kilolib)


  #
  # ARK loop function: navigation_experiment
  #
  add_executable(move_to_position move_to_position.c)  
  target_link_libraries(move_to_position argos3plugin_simulator_kilolib)

  #
  # ARK loop function: clustering
  #
  add_executable(clustering clustering.c)  
  target_link_libraries(clustering argos3plugin_simulator_kilolib)

  #
  # ARK loop function: crwlevy
  #
  add_executable(crwlevy crwlevy.c)  
  target_link_libraries(crwlevy argos3plugin_simulator_kilolib)

  #
  # ARK loop function: crwlevy_with_positioning
  #
  add_executable(crwlevy_with_positioning crwlevy_with_positioning.c)  
  target_link_libraries(crwlevy_with_positioning argos3plugin_simulator_kilolib)

  #
  # My test : pi/2 left rotation
  #
  add_executable(test_rotation test_rotation.c)
  target_link_libraries(test_rotation argos3plugin_simulator_kilolib)

  #
  # Debugging example
  #
  add_executable(test_debug test_debug.h test_debug.c)
  target_link_libraries(test_debug argos3plugin_simulator_kilolib)
  endif(ARGOS_BUILD_FOR_SIMULATOR)


