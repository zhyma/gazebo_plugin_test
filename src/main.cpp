#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


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

        ROS_INFO("Hello World!\n");
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
        // ROS_INFO("test\n");
        if (this->rod == NULL && rodReady==true)
        {
          this->rod = this->world->ModelByName("rod");
          printf("ModelName: %s \n", rod->GetName().c_str());
        }
        // if (this->rodPtr != NULL)
        //   ROS_INFO("rod found\n");
        // else
        //   ROS_INFO("NOT found\n");
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
        // ros::Rate rate(10);
        // ROS_INFO("searching...\n");
        // while (this->rodPtr == NULL)
        // {
        //   ROS_INFO("NOT found\n");
        //   rate.sleep();
        //   this->rodPtr = this->world->ModelByName("rod");
        // }
        // ROS_INFO("done...\n");
        // this->rodPtr = this->world->ModelByName("rod");
        printf("get: %s \n", name.c_str());
        if (name=="rod")
        {rodReady = true;}

        // printf("ModelCount: %d \n", this->world->ModelCount());
        // physics::Model_V list = this->world->Models();
        // for (int i = 0; i < this->world->ModelCount(); i++)
        //   printf("ModelName: %s \n", list[i]->GetName().c_str());
      }

  };
  GZ_REGISTER_WORLD_PLUGIN(EnvMod)
}
