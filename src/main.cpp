#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>

// #include <functional>
// #include <gazebo/gazebo.hh>
// #include <gazebo/common/common.hh>
// #include <ignition/math/Vector3.hh>

#include <iostream>


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
        ROS_INFO("OnUpdate()\n");
        // // ROS_INFO("test\n");
        if (this->rod == NULL && rodReady==true)
        {
          this->rod = this->world->ModelByName("rod");
          if (this->rod!= NULL)
            std::cout << "Get model: " << rod->GetName() << std::endl;
        }
      }

    private:
      physics::WorldPtr world;
      // Pointer to the update event connection
      physics::ModelPtr rod = NULL;
      bool rodReady = false;
      bool cabelReady = false;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
      event::ConnectionPtr add_entity_connection;

      void addEntityEventCallback(const std::string &name)
      {
        // Check entity name...
        // Trigger initialization...
        std::cout << "Callback get:" << name << std::endl;
        if (name=="rod")
          rodReady = true;
      }

  };
  GZ_REGISTER_WORLD_PLUGIN(EnvMod)
}
