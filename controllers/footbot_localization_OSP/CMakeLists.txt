add_library(footbot_localization_OSP MODULE
  message_manager.h
  message_manager.cpp
  footbot_localization_OSP.h
  footbot_localization_OSP.cpp)
target_link_libraries(footbot_localization_OSP
  argos3core_simulator
  argos3plugin_simulator_footbotnetworked
  argos3plugin_simulator_genericrobot
  utility)
