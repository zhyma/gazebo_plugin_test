#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

// #include <functional>
// #include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Vector3.hh>

#include <iostream>

#include "ros/ros.h"
#include <env_ctrl/CylinderProperties.h>
#include "std_msgs/String.h"
// #include <std_msgs/Float64MultiArray.h>

// #include <boost/bind.hpp>
// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>

namespace gazebo
{
  class EnvMod : public WorldPlugin
  {
    public:
      EnvMod() : WorldPlugin()
      {
      }

      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
      {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

        ROS_INFO("Loading\n");

        // init ROS node
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "Rod_properties_controller");
        // publisher: pub rod position, orientation, and dimension
        properties_pub = n.advertise<env_ctrl::CylinderProperties>("get_rod_properties", 1000);
        properties_sub = n.subscribe("set_rod_properties", 1000, &EnvMod::properties_callback, this);

        // loading models
        this->world = _world;
        this->add_entity_connection = event::Events::ConnectAddEntity(
                                      std::bind(&EnvMod::addEntityEventCallback, 
                                      this, std::placeholders::_1));
        _world->InsertModelFile("model://rod");
        // _world->InsertModelFile("model://cable");

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&EnvMod::OnUpdate, this));
      }
      void OnUpdate()
      {
        if (this->rod == NULL && rod_ready==true)
        {
          this->rod = this->world->ModelByName("rod");
          if (this->rod != NULL)
            std::cout << "Get model: " << rod->GetName() << std::endl;
        }
        if (this->rod != NULL)
        {
          // update model property if needed.
          if (new_properties)
          {
            ignition::math::Pose3d model_pose = rod->WorldPose();
            ignition::math::Vector3d new_vec(properties[0], properties[1], properties[2]);
            ignition::math::Quaterniond rot(model_pose.Rot());
            ignition::math::Pose3d new_pose(new_vec, rot);
            // model_pose.Pos().X() = properties[0];
            // model_pose.Pos().Y() = properties[1];
            // model_pose.Pos().Z() = properties[2];
            this->rod->SetWorldPose(new_pose);
            new_properties = false;
          }
          
          // Get link "target_rod"
          // publish link pose and states
          physics::LinkPtr link = this->rod->GetLink("target_rod");
          env_ctrl::CylinderProperties msg;
          ignition::math::Pose3d pose = link->WorldPose();
          ignition::math::Vector3<double> position = pose.Pos();

          physics::CollisionPtr collision = link->GetCollision("target_rod_collision");
          physics::ShapePtr shape = collision->GetShape();
          int shape_type = collision->GetShapeType();
          boost::shared_ptr<physics::CylinderShape> cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);

          // x, y, z, radius, length
          msg.x = position.X();
          msg.y = position.Y();
          msg.z = position.Z();
          msg.r = cylinder->GetRadius();
          msg.l = cylinder->GetLength();
          properties_pub.publish(msg);
        }
      }

    private:
      // ROS node part
      ros::NodeHandle n;
      ros::Publisher properties_pub;
      ros::Subscriber properties_sub;

      // Gazebo part
      physics::WorldPtr world;
      // Pointer to the update event connection
      physics::ModelPtr rod = NULL;
      bool rod_ready = false;
      bool cabel_ready = false;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
      event::ConnectionPtr add_entity_connection;

      bool new_properties;
      double properties[5];

      void addEntityEventCallback(const std::string &name)
      {
        // Check entity name...
        // Trigger initialization...
        std::cout << "Callback get:" << name << std::endl;
        if (name=="rod")
          rod_ready = true;
      }

      void properties_callback(const env_ctrl::CylinderProperties::ConstPtr& msg)
      {
        ROS_INFO("sub callback");
        this->properties[0] = msg->x;
        this->properties[1] = msg->y;
        this->properties[2] = msg->z;
        this->properties[3] = msg->r;
        this->properties[4] = msg->l;
        new_properties = true;
        std::cout << properties[0] << ", " << properties[1] << ", " << properties[2];
        std::cout << ", " << properties[3] << ", " << properties[4] << ", " <<std::endl;
      }

  };
  GZ_REGISTER_WORLD_PLUGIN(EnvMod)
}
