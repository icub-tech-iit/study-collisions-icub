################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Install models
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV2_5_collisions DESTINATION share/gazebo)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV3_collisions DESTINATION share/gazebo)

# Install worlds
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/world DESTINATION share/gazebo)