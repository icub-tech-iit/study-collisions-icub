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

// for storing the results
#include <matioCpp/matioCpp.h>

#define INTERVAL_SIZE_DEGREES_PITCH 3.0
#define INTERVAL_SIZE_DEGREES_ROLL 0.1
#define INTERVAL_SIZE_DEGREES_YAW 1.0

#define USE_HEURISTIC true
#define JOINT_ROLL_MAX 17.5
#define JOINT_ROLL_MIN 15.0

class jointSpaceIterator {

    public:
        yarp::os::ResourceFinder rf;

        std::string robotName;
        std::string robotVersion;
        std::string context;
        std::string arm_ini_file;
        std::string arm;

        yarp::os::Bottle jointLimitsBottle;

        std::vector<std::vector<std::vector<int> > > collision_map;

        yarp::os::Property options;
        yarp::dev::PolyDriver* robotDevice;

        yarp::dev::IPositionControl *pos;
        yarp::dev::IEncoders *encs;

        yarp::sig::Vector encoders, command;

        gazebo::physics::Model *robotModel;

        std::vector<std::string> sensorVector;

        std::vector<int> shoulderJointIntervals { 0, 0, 0 }; //stores how many intervals we have per joint

        std::vector<std::vector<double> > jointLimitsVect {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

        
        // set ports to receive the collision data
        yarp::os::BufferedPort<yarp::os::Bottle> collisions_port_bool;
        yarp::os::Bottle *collisionsBottle;


        bool init();
        bool openPorts();
        bool getJointLimits();
        bool iterate();
        int computeCollision(int joint_0, int joint_1, int joint_2);
        bool saveData();

        jointSpaceIterator(yarp::os::ResourceFinder rf, std::string &robotName, std::string &robotVersion, std::string &arm);

        ~jointSpaceIterator();

};