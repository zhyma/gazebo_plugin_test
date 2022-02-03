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
// #include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>

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
        ros::init(argc, argv, "Rod_state_controller");
        // publisher: pub rod position, orientation, and dimension
        state_pub = n.advertise<std_msgs::Float64MultiArray>("get_rod_state", 1000);

        // loading models
        this->world = _world;
        this->add_entity_connection = event::Events::ConnectAddEntity(
                                      std::bind(&EnvMod::addEntityEventCallback, 
                                      this, std::placeholders::_1));
        _world->InsertModelFile("model://rod");
        _world->InsertModelFile("model://cable");

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
          // Get link "target_rod"
          physics::LinkPtr link = this->rod->GetLink("target_rod");
          
          std_msgs::Float64MultiArray msg;
          ignition::math::Pose3d pose = link->WorldPose();
          ignition::math::Vector3<double> position = pose.Pos();

          physics::CollisionPtr collision = link->GetCollision("target_rod_collision");
          physics::ShapePtr shape = collision->GetShape();
          int shape_type = collision->GetShapeType();
          boost::shared_ptr<physics::CylinderShape> cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);

          // x, y, z, radius, length
          msg.data.push_back(position.X());
          msg.data.push_back(position.Y());
          msg.data.push_back(position.Z());
          msg.data.push_back(cylinder->GetRadius());
          msg.data.push_back(cylinder->GetLength());
          state_pub.publish(msg);
        }
      }

    private:
      // ROS node part
      ros::NodeHandle n;
      ros::Publisher state_pub;

      // Gazebo part
      physics::WorldPtr world;
      // Pointer to the update event connection
      physics::ModelPtr rod = NULL;
      bool rod_ready = false;
      bool cabel_ready = false;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
      event::ConnectionPtr add_entity_connection;

      void addEntityEventCallback(const std::string &name)
      {
        // Check entity name...
        // Trigger initialization...
        std::cout << "Callback get:" << name << std::endl;
        if (name=="rod")
          rod_ready = true;
      }

  };
  GZ_REGISTER_WORLD_PLUGIN(EnvMod)
}
