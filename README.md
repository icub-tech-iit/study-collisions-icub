study-collisions-icub
=====================

This repository contains the necessary code to install and run the collision detector for both iCub3 and iCub2.5.
It studies the collision between the upper arms and the torso of the robots.

## Installation
### Dependencies

The depencies are:
- [CMake](https://cmake.org/install/) (minimum version 3.12)
- [Boost](https://www.boost.org/)
- [Gazebo](http://gazebosim.org/tutorials?cat=install)
- [Robotology-superbuild](https://github.com/robotology/robotology-superbuild#source-installation) (tested on version v2021.02)
  - ROBOTOLOGY\_USES\_GAZEBO=ON
- [matio-cpp](https://github.com/dic-iit/matio-cpp#installation)


### Linux
```
git clone https://github.com/icub-tech-iit/study-collisions-icub
cd study-collisions-icub
mkdir build && cd build
cmake ../
make
[sudo] make install
```
Notice: sudo is not necessary if you specify the CMAKE_INSTALL_PREFIX to point to the robotology-superbuild install folder (usually `robotology-superbuild/build/install`).

## Usage

The study is performed by running two modules: a Gazebo plugin (`collisionDetector`) that detects the collisions in gazebo, and a module (`jointSpaceIterator`) that goes through all joint position combinations, checks the collisions, and saves the data into a .mat file.

Before executing any modules, make sure a `yarpserver` is running in the network.

The following steps should be executed in a sequence:

- on a terminal, run `yarpserver` (or have a server already running in the network);
- on a different terminal, run `gazebo` with the corresponding world file (for iCub2.5 replace `iCubGazeboV3` for `iCubGazeboV2_5` on the file name):
```
gazebo world/collision_world_iCubGazeboV3.world 
```
- on gazebo GUI, activate `view->contacts`;
- on another terminal, run the joint space iterator module (for icub 2.5 replace `iCubV3` for `iCubV2_5`):
```
jointSpaceIterator --robot iCubSim --robotVersion iCubV3
```
- finally, connect the collision ports:
```
yarp connect /collision:o /JSI/collision:i
```

After connecting the ports, the module will automatically start moving the robot and collecting data. This process can take a few hours depending on the angle interval size (default 3 degrees, ~1 hour).

### Changing the angle interval size

The angle interval size for the `jointSpaceIterator` module is defined in [jointSpaceIterator.hpp#L29](https://github.com/icub-tech-iit/study-collisions-icub/blob/075ca65098aa4c7afa9f9d46a28285e4040d1c4e/modules/jointSpaceIterator/include/jointSpaceIterator.hpp#L29). You can change this value to the desired interval size, recompile, and run. Bear in mind that very small intervals will result in extended runtimes! 
