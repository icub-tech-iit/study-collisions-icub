#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

#include <yarp/sig/all.h>

#include <yarp/os/Bottle.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/RpcClient.h>

// gazebo stuff
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>





#define INTERVAL_SIZE_DEGREES 1.0

class jointSpaceIterator {

    public:
        yarp::os::ResourceFinder rf;

        std::string robotName;
        std::string context;
        std::string arm_ini_file;

        yarp::os::Bottle jointLimitsBottle;

        std::vector<std::vector<std::vector<int> > > collision_map;

        yarp::os::Property options;
        yarp::dev::PolyDriver* robotDevice;

        yarp::dev::IPositionControl *pos;
        yarp::dev::IEncoders *encs;

        yarp::sig::Vector encoders, command;

        gazebo::physics::Model *robotModel;

        std::vector<std::string> sensorVector;
        
        // set ports to receive the collision data


        bool init();
        bool getJointLimits();
        bool iterate();
        bool computeCollision(int joint_0, int joint_1, int joint_2);

        jointSpaceIterator(yarp::os::ResourceFinder rf, std::string &robotName);

        ~jointSpaceIterator();

};