#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

// #include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
// #include <ignition/math/Pose3.hh>
// #include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>

#include <iostream>

#include "ros/ros.h"
#include <env_ctrl/CylinderProperties.h>
#include "std_msgs/String.h"

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
        sdf = _sdf;
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
        // ROS publisher: pub rod position, orientation, and dimension
        properties_pub = n.advertise<env_ctrl::CylinderProperties>("get_rod_properties", 1000);
        properties_sub = n.subscribe("set_rod_properties", 1000, &EnvMod::properties_callback, this);

        this->node = transport::NodePtr(new gazebo::transport::Node());
        this->node->Init(_world->Name());
        visPub = this->node->Advertise<msgs::Visual>("~/visual");
        visPub->WaitForConnection();

        // // Create a publisher on the ~/factory topic
        factPub = node->Advertise<msgs::Factory>("~/factory");

        // loading models
        this->world = _world;
        this->add_entity_connection = event::Events::ConnectAddEntity(
                                      std::bind(&EnvMod::addEntityEventCallback, 
                                      this, std::placeholders::_1));

        // Create the message
        msgs::Factory msg;

        // load rod
        msg.set_sdf_filename("model://rod");

        // Pose to initialize the model to
        msgs::Set(msg.mutable_pose(),
            ignition::math::Pose3d(
              ignition::math::Vector3d(0.2, 0.0, 0.2),
              ignition::math::Quaterniond(0, 0, 0)));

        // Send the message
        factPub->Publish(msg);

        // load cable
        msg.set_sdf_filename("model://cable");

        // Pose to initialize the model to
        msgs::Set(msg.mutable_pose(),
            ignition::math::Pose3d(
              ignition::math::Vector3d(0.2-0.6, 0, 0.2+0.1),
              ignition::math::Quaterniond(0, 0, 0)));

        // Send the message
        factPub->Publish(msg);

        // _world->InsertModelFile("model://rod");
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
        if (this->cable == NULL && cable_ready==true)
        {
          physics::Model_V list = this->world->Models();

          for (int i=0; i < this->world->ModelCount(); i++)
          {
            std::string name = list[i]->GetName();
            std::cout << name << std::endl;
          //   if (name.find("cable") != std::string::npos)
          //   {
          //       this->cable = this->world->ModelByName(name);
          //   }
          }
          this->cable = this->world->ModelByName("cable");
          if (this->cable != NULL)
            std::cout << "Get model: " << cable->GetName() << std::endl;

          cable_ready==false;
        }
        if (this->rod != NULL && this->cable != NULL)
        {
          // Get link "target_rod"
          physics::LinkPtr link = this->rod->GetLink("target_rod");
          // Get link collision property
          physics::CollisionPtr collision = link->GetCollision("target_rod_collision");
          physics::ShapePtr shape = collision->GetShape();
          int shape_type = collision->GetShapeType();
          boost::shared_ptr<physics::CylinderShape> cylinder = boost::dynamic_pointer_cast<physics::CylinderShape>(shape);

          // Get link visual property
          msgs::Visual visualMsg = link->GetVisualMessage("target_rod_vis");

          // update model property if needed.
          if (new_properties)
          {
            world->RemoveModel(this->cable->GetName());
            std::cout<< "cable destoried: " << cable->GetName() << std::endl;
            
            // update rod's visual
            // prepare visual message
            visualMsg.set_name(link->GetScopedName());
            visualMsg.set_parent_name(rod->GetScopedName());

            // update shape
            msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
            geomMsg->set_type(msgs::Geometry::CYLINDER);
            geomMsg->mutable_cylinder()->set_radius(properties[3]);
            geomMsg->mutable_cylinder()->set_length(properties[4]);

            // get color
            visualMsg.mutable_material()->mutable_script()->set_name("Gazebo/Red");

            visPub->Publish(visualMsg);

            // update rod's physical
            ignition::math::Pose3d rod_pose = this->rod->WorldPose();
            ignition::math::Vector3d pos_vec(properties[0], properties[1], properties[2]);
            ignition::math::Quaterniond rod_rot(rod_pose.Rot());
            ignition::math::Pose3d rod_new_pose(pos_vec, rod_rot);
            this->rod->SetWorldPose(rod_new_pose);

            cylinder->SetSize(properties[3], properties[4]);

            std::cout << "new cable" << std::endl;

            msgs::Factory msg;
            msg.set_sdf_filename("model://cable");

            // Pose to initialize the model to
            msgs::Set(msg.mutable_pose(),
                ignition::math::Pose3d(
                  ignition::math::Vector3d(properties[0]-0.6, properties[1], properties[2]+properties[3]+0.1),
                  ignition::math::Quaterniond(0, 0, 0)));
            factPub->Publish(msg);

            new_properties = false;
            std::cout << "new set up done" << std::endl;
          }
          
          // publish link pose and states
          env_ctrl::CylinderProperties msg;
          ignition::math::Pose3d pose = link->WorldPose();
          ignition::math::Vector3<double> position = pose.Pos();

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
      sdf::ElementPtr sdf;

      physics::WorldPtr world;
      // Pointer to the update event connection
      physics::ModelPtr rod = NULL;
      physics::ModelPtr cable = NULL;
      bool rod_ready = false;
      bool cable_ready = false;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
      event::ConnectionPtr add_entity_connection;

      // To update the visual
      transport::NodePtr node;
      transport::PublisherPtr visPub;
      transport::PublisherPtr factPub;

      bool new_properties;
      double properties[5];

      void addEntityEventCallback(const std::string &name)
      {
        // Check entity name...
        // Trigger initialization...
        std::cout << "Callback get:" << name << std::endl;
        if (name=="rod")
          rod_ready = true;
        std::cout << "Callback get:" << name << std::endl;
        if (name=="cable")
          cable_ready = true;
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
