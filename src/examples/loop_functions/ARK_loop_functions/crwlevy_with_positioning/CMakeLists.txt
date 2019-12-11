include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(ALF_crwlevy_positioning_loop_function
  MODULE
  crwlevy_with_positioning.h
  crwlevy_with_positioning.cpp
)


target_link_libraries(ALF_crwlevy_positioning_loop_function
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)
