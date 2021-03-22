/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_YARP_CONTACTFORCETORQUES_HH
#define GAZEBO_YARP_CONTACTFORCETORQUES_HH

#include <string>
#include <vector>


#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Pose3.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/sig/Vector.h>

namespace gazebo
{

    class collisionDetectorPlugin : public ModelPlugin
    {
    public:
        collisionDetectorPlugin();
        virtual ~collisionDetectorPlugin();

        /**
         * Loads robot model, reads configuration, 
         * opens network wrapper device and opens device driver
         */
        void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);

        /**
         * Callback for the WorldUpdateBegin Gazebo event.
         */
        void onUpdate(const gazebo::common::UpdateInfo&);

    private:
        gazebo::physics::ContactManager* contactManager {nullptr};
        std::string robotName;

        yarp::os::Property config;

        std::vector< gazebo::physics::Contact* > contacts {nullptr};

        gazebo::event::ConnectionPtr updateConnection;
        gazebo::event::ConnectionPtr resetConnection;

        yarp::os::BufferedPort<yarp::os::Bottle>* output_port_bool;


        bool openPorts();

        bool closePorts();

    };
}

#endif
