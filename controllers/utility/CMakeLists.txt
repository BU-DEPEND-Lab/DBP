add_library(utility STATIC
  OSP_messages.h)

target_include_directories(utility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utility
  argos3core_simulator
  argos3plugin_simulator_footbotnetworked
  argos3plugin_simulator_genericrobot)
