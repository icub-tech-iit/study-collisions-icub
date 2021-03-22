#include "collisionPlugin.hh"


GZ_REGISTER_MODEL_PLUGIN(gazebo::collisionDetectorPlugin)

namespace gazebo
{

    collisionDetectorPlugin::collisionDetectorPlugin() : ModelPlugin()
    {

    }

    collisionDetectorPlugin::~collisionDetectorPlugin()
    {
        closePorts();
        yarp::os::Network::fini();
    }

    void collisionDetectorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Get  contact manager
        contactManager = _parent->GetWorld()->Physics()->GetContactManager();

        yInfo() << "name of the model is " << _parent->GetName();
        yInfo() << "scoped name is " << _parent->GetScopedName();


        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "collisionDetectorPlugin::Load error: yarp network does not seem to be available, is the yarp server running?";
            return;
        }

        if (!_parent)
        {
            yError() << "collisionDetectorPlugin plugin requires a parent \n";
            return;
        }

        GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));
        robotName = _parent->GetScopedName();

        yInfo() << "got robot name: " << robotName;

        
        // Getting .ini configuration file parameters from sdf
        bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, config);

        if (!configuration_loaded) {
            yError() << "collisionDetectorPlugin : File .ini not found, load failed." ;
            return;
        }
        contacts = contactManager->GetContacts();
        yInfo() << "we got " << contacts.size() << " contacts.";

        // Try to open the YARP ports
        if (!this->openPorts()) {
            yError() << "unable to open the port";
            return;
        }
        yInfo() << "ports opened!";

        yarp::os::Time::delay(10);

        // Create callback
        updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&collisionDetectorPlugin::onUpdate, this, _1));
        
    }

    bool collisionDetectorPlugin::openPorts()
    {
        yInfo() << "trying to open the port";
        output_port_bool = new yarp::os::BufferedPort<yarp::os::Bottle>();
        bool ok = output_port_bool->open("/collision:o");
        yInfo() << "status was" << ok;
        if (ok)
        {
            yInfo() << "Port /collision:o opened successfully";
            return true;
        }
        yInfo() << "port not opened...";
        return false;
    }

    bool collisionDetectorPlugin::closePorts()
    {
        output_port_bool->close();
        return true;
    }

    void collisionDetectorPlugin::onUpdate(const gazebo::common::UpdateInfo& info)
    {
        yarp::os::Bottle msg;
        msg.addString("I'm alive!");
        output_port_bool->prepare() = msg;


        ignition::math::Vector3d force = ignition::math::Vector3d::Zero;
        ignition::math::Vector3d torque = ignition::math::Vector3d::Zero;
        for(auto&& contact : contacts)
        {
            force = contact->wrench->body1Force;
            torque = contact->wrench->body1Torque;
            yInfo() << "force: " << force.X();
            msg.addDouble(force.X());
            yInfo() << "torque" << torque.X();
            msg.addDouble(torque.X());
        }
        output_port_bool->write();
        return;
    }

}
