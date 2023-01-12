add_library(footbot_time_sync_OSP MODULE
  message_manager.h
  message_manager.cpp
  footbot_time_sync_OSP.h
  footbot_time_sync_OSP.cpp)
target_link_libraries(footbot_time_sync_OSP
  argos3core_simulator
  argos3plugin_simulator_footbotnetworked
  argos3plugin_simulator_genericrobot
  utility)
