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

# Install meshes - THIS WILL OVERWRITE THE MESHES FROM ROBOTOLOGY
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/simmechanics DESTINATION share/iCub/meshes)