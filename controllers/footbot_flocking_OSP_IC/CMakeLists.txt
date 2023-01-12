add_library(footbot_flocking_OSP_IC MODULE
  message_manager.h
  message_manager.cpp
  footbot_flocking_OSP_IC.h
  footbot_flocking_OSP_IC.cpp)
target_link_libraries(footbot_flocking_OSP_IC
  argos3core_simulator
  argos3plugin_simulator_footbotnetworked
  argos3plugin_simulator_genericrobot
  utility)
