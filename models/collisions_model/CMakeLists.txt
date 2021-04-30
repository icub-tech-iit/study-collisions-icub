################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Install models
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV2_5_collisions/collisions_model_iCubGazeboV2_5.sdf ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV2_5_collisions/model.config DESTINATION share/gazebo/iCubGazeboV2_5_collisions)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV3_collisions/model.urdf ${CMAKE_CURRENT_SOURCE_DIR}/iCubGazeboV3_collisions/model.config DESTINATION share/gazebo/iCubGazeboV3_collisions)

# Install worlds
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/world DESTINATION share/gazebo)

# Install meshes - THIS WILL OVERWRITE THE MESHES FROM ROBOTOLOGY
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/simmechanics DESTINATION share/iCub/meshes)