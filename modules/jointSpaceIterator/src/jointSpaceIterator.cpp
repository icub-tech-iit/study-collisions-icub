#include "jointSpaceIterator.hpp"


jointSpaceIterator::jointSpaceIterator(yarp::os::ResourceFinder _rf, std::string &_robotName) : rf(_rf), robotName(_robotName) {

}

bool jointSpaceIterator::init()
{

    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/right_arm";

    std::string localPorts="/test/client";
    
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    robotDevice = new yarp::dev::PolyDriver(options);
    if (!robotDevice->isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", yarp::dev::Drivers::factory().toString().c_str());
        return false;
    }

    bool ok;
    ok = robotDevice->view(pos);
    ok = ok && robotDevice->view(encs);
 
    if (!ok) {
        yError() << "Problems acquiring interfaces\n";
        return false;
    }

    int nj=0;
    pos->getAxes(&nj);
    encoders;
    command;
    encoders.resize(nj);
    command.resize(nj);

    return true;
}

jointSpaceIterator::~jointSpaceIterator()
{
    robotDevice->close();
}

bool jointSpaceIterator::getJointLimits() {
    jointLimitsBottle = rf.findGroup("LIMITS");
    if(!jointLimitsBottle.isNull())
    {
        yInfo() << jointLimitsBottle.toString();
        yInfo() << jointLimitsBottle.get(1).asList()->toString();
        yInfo() << jointLimitsBottle.get(2).asList()->toString();
        return true;
    }
    return false;
}

bool jointSpaceIterator::iterate()
{
    std::vector<int> shoulderJointIntervals { 0, 0, 0 }; //stores how many intervals we have per joint
    for(int i=1; i<4; i++)
    {
        double jointRange = jointLimitsBottle.get(1).asList()->get(i).asDouble() - jointLimitsBottle.get(2).asList()->get(i).asDouble();
        yInfo() << "joint range value is: " << jointRange;
        shoulderJointIntervals[i-1] = int ( jointRange/INTERVAL_SIZE_DEGREES );
    }

    // initialize the 3D vector of collisions now that we have the size
    std::vector< std::vector< std::vector<int> > > temp_vect(shoulderJointIntervals[0],
                                       std::vector< std::vector<int> >(shoulderJointIntervals[1], 
                                                      std::vector<int>(shoulderJointIntervals[2])));
    collision_map = temp_vect;

    for(int int_joint_0=0; int_joint_0<shoulderJointIntervals[0]; int_joint_0++)
    {
        for(int int_joint_1=0; int_joint_1<shoulderJointIntervals[1]; int_joint_1++)
        {
            for(int int_joint_2=0; int_joint_2<shoulderJointIntervals[2]; int_joint_2++)
            {
                yInfo() << "computing collision";
                if(!computeCollision(int_joint_0, int_joint_1, int_joint_2))
                {
                    yError() << "failed to compute collisions on gazebo, stopping...";
                    return false;
                }
            }
        }
    }
    yInfo() << "simulation completed successfully";
    return true;
}

bool jointSpaceIterator::computeCollision(int joint_0, int joint_1, int joint_2)
{
    bool collision = false;
    bool done = false;

    // degrees
    double angle_joint_0 = joint_0 * INTERVAL_SIZE_DEGREES + jointLimitsBottle.get(2).asList()->get(1).asDouble();
    yInfo() << "angle is: " << angle_joint_0;
    double angle_joint_1 = joint_1 * INTERVAL_SIZE_DEGREES + jointLimitsBottle.get(2).asList()->get(2).asDouble();
    yInfo() << "angle is: " << angle_joint_1;
    double angle_joint_2 = joint_2 * INTERVAL_SIZE_DEGREES + jointLimitsBottle.get(2).asList()->get(3).asDouble();
    yInfo() << "angle is: " << angle_joint_2;
    // first we move the robot in gazebo
    // we need to access the controlboards for this movement and query
    //printf("waiting for encoders");
    while(!encs->getEncoders(encoders.data()))
    {
        yarp::os::Time::delay(0.1);
    }

    yInfo() << "got encoders";

    command=encoders;
    //now set the shoulder to some value
    command[0]=angle_joint_0;
    command[1]=angle_joint_1;
    command[2]=angle_joint_2;
    pos->positionMove(command.data()); 

    yInfo() << "movement started";

    // wait for movement to finish
    while(!done)
    {
        pos->checkMotionDone(&done);
        yarp::os::Time::delay(0.1);
    }

    // then we query for collisions

    if(collision)
    {
        collision_map[joint_0][joint_1][joint_2] = 1; //1 for collision
    }
    else
    {
        collision_map[joint_0][joint_1][joint_2] = 0; //0 for no collision
    }
    return true;
}

int main(int argc, char *argv[]) 
{
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;

    rf.setDefaultConfigFile("/home/alexandre/robotology-superbuild/src/robots-configuration/iCubTemplates/iCubTemplateV2_0/conf/icub_left_arm.ini");
    rf.configure(argc, argv);

    yarp::os::Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return 1;
    }
    std::string robotName=params.find("robot").asString().c_str();

    jointSpaceIterator JSI_module(rf, robotName);

    // run the module

    bool ok = JSI_module.init();
    if(!ok)
    {
        yError() << "something failed in connecting to the robot boards";
        return 1;
    }
    ok = ok && JSI_module.getJointLimits();
    if(!ok)
    {
        yError() << "failed to obtain the joint limits";
        return 1;
    }
    yInfo() << "running module";
    ok = ok && JSI_module.iterate();
    if(!ok)
    {
        yError() << "failed to run the module";
        return 1;
    }

    return 0;
}
