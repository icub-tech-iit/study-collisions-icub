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
- on another terminal, run the joint space iterator module (for icub 2.5 replace `iCubV3` for `iCubV2_5`). Specify also which arm to run the test on (left/right):
```
jointSpaceIterator --robot icubSim --robotVersion iCubV3 --arm right
```
- finally, connect the collision ports:
```
yarp connect /collision:o /JSI/collision:i
```

After connecting the ports, the module will automatically start moving the robot and collecting data. This process can take a few hours depending on the angle interval size (default 3 degrees, ~1 hour).

### Options and heuristics for the test

You can change some of the options and heuristics to make the test run faster. These `jointSpaceIterator` options are defined in [jointSpaceIterator.hpp#L29-L35](https://github.com/icub-tech-iit/study-collisions-icub/blob/642811f815947548bf612542f479a0070db88d40/modules/jointSpaceIterator/include/jointSpaceIterator.hpp#L29-L35). You can specify the interval size for each angle individually (The critical angle in this study was the shoulder roll, so we used a much finer interval size for that angle, and larger ones for the remaining two). Bear in mind that smaller interval sizes means longer running times, or even the module being unable to run due to a too-large results matrix.

You can also activate a particular heuristic to speed up the study by focusing on a particular window for the roll angle. This is defined in [jointSpaceIterator.hpp#L33-L35](https://github.com/icub-tech-iit/study-collisions-icub/blob/642811f815947548bf612542f479a0070db88d40/modules/jointSpaceIterator/include/jointSpaceIterator.hpp#L33-L35). This can be particularly useful if you already have a rough idea of where the collisions happen, or following a run with larger step sizes, in order to more accurately pinpoint the collisions.


## Common issues

### Running jointSpaceIterator module

A common issue arises when the step intervals chosen for the test are set too small. This leads to an exponential increase in the number of intervals the module has to go through, which in turn means the results matrix has to be initialized with too many entries, leading to a module kill. If this happens, try increasing the interval size for the angles. A good policy is to only have an interval size <1 for one single angle.

### iCub3 Gazebo model issues

The iCub3 URDF model used for this study was tweaked to allow the study of the collisions for the upper arms only. As a consequences, collisions in the shoulder and elbow joints and on the forearms have been disabled. If you are using your own model to run the collision test and are detecting too many collisions, double-check the URDF file in the sections corresponding to these joints/links, and remove the `<collision>` segment.

If, on the other hand, you are not detecting any collisions while running the test, the issue could be two-fold:
1. Double check that the `view->contacts` option is enabled on your gazebo GUI;
2. Check your model URDF file and confirm that there is a `<collision>` section for the upperarm link. Additionally, make sure you have included a contact sensor for that link. for more information on this, check the model instructions here [iCubV3](https://github.com/icub-tech-iit/study-collisions-icub/tree/master/models/collisions_model/iCubGazeboV3_collisions) or here [iCubV2_5](https://github.com/icub-tech-iit/study-collisions-icub/tree/master/models/collisions_model/iCubGazeboV2_5_collisions).
