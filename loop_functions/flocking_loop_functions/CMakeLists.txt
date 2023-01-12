include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(flocking_qtuser_functions MODULE
  flocking_qtuser_functions.h
  flocking_qtuser_functions.cpp)

target_link_libraries(flocking_qtuser_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
