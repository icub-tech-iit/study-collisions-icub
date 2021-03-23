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

    if(!openPorts())
    {
        yError("Failed to open collisions port");
        robotDevice->close();
        return false;
    }
    
    while(collisions_port_bool.getInputCount() <= 0)
    {
        yarp::os::Time::delay(0.2); //wait until the port is connected
    }


    return true;
}

bool jointSpaceIterator::openPorts()
{
    bool ok = collisions_port_bool.open("/JSI/collision:i");
    return ok;
}

jointSpaceIterator::~jointSpaceIterator()
{
    robotDevice->close();
    collisions_port_bool.close();
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
    //bool collision = false;
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
    yInfo() << "checking the collision";
    collisionsBottle = collisions_port_bool.read(true); // we want blocking - we need to make sure we have the output of the simulator
    if (collisionsBottle != NULL)
    {
        yInfo() << "bottle was " << collisionsBottle->toString();
        collision_map[joint_0][joint_1][joint_2] = collisionsBottle->get(0).asInt();
        yInfo() << "we got " << collision_map[joint_0][joint_1][joint_2] << "collisions";
    }

    /*if(collision)
    {
        collision_map[joint_0][joint_1][joint_2] = 1; //1 for collision
    }
    else
    {
        collision_map[joint_0][joint_1][joint_2] = 0; //0 for no collision
    }*/
    return true;
}

bool jointSpaceIterator::saveData()
{
    std::vector<int> linear_matrix;
    // we linearize the 3D matrix before proceding with matio conversion
    for (auto& _collisions_joint0 : collision_map)
    {
        for (auto& _collisions_joint1 : _collisions_joint0)
        {
            for (auto& _collisions_joint2 : _collisions_joint1)
            {
                linear_matrix.push_back(_collisions_joint2);
            }
        }
    }

    // now we start the matioCpp convertion process
    // first create timestamps vector

    // and the structures for the actual data too
    std::vector<matioCpp::Variable> test_data;

    // now we populate the matioCpp matrix
    matioCpp::MultiDimensionalArray<int> out("data", {shoulderJointIntervals[2] , shoulderJointIntervals[1], shoulderJointIntervals[0] }, linear_matrix.data());
    test_data.emplace_back(out); // Data

    matioCpp::Struct collisions_matrix("collisions_matrix", test_data);
    // and finally we write the file
    // since we might save several files, we need to index them
    matioCpp::File file = matioCpp::File::Create("collisions_matrix.mat");
    return file.write(collisions_matrix);
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
    return JSI_module.saveData();
}
