/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#ifndef GAZEBO_ROS_LINK_ATTACHER_HH
#define GAZEBO_ROS_LINK_ATTACHER_HH

#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"
#include <ros/node_handle.h>
#include <sdf/sdf.hh>
#include <vector>

namespace gazebo
{

class GazeboRosLinkAttacher : public WorldPlugin
{
 public:
   /// \brief Constructor
   GazeboRosLinkAttacher();

   /// \brief Destructor
   virtual ~GazeboRosLinkAttacher();

   /// \brief Load the controller
   virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

 protected:
   /// \brief Internal representation of a fixed joint
   struct Request
   {
      enum Type
      {
         ATTACH,
         DETACH,
      };

      enum Status
      {
         PENDING,
         DONE,
         FAILED,
      };

      Type type;
      Status status;
      gazebo_ros_link_attacher::AttachRequest data;
   };

   typedef boost::shared_ptr<Request> RequestPtr;

   boost::shared_ptr<ros::NodeHandle> nodeHandle;
   ros::ServiceServer attachServiceServer;
   ros::ServiceServer detachServiceServer;

   bool OnAttachServiceCall(gazebo_ros_link_attacher::Attach::Request &_request,
                            gazebo_ros_link_attacher::Attach::Response &_response);

   bool OnDetachServiceCall(gazebo_ros_link_attacher::Attach::Request &_request,
                            gazebo_ros_link_attacher::Attach::Response &_response);

   bool HandleServiceCall(Request::Type _requestType,
                          gazebo_ros_link_attacher::Attach::Request &_request);

   /// \brief Attach with a revolute joint
   bool AttachLinks(const gazebo_ros_link_attacher::AttachRequest &_request);

   /// \brief Detach
   bool DetachLinks(const gazebo_ros_link_attacher::AttachRequest &_request);

   physics::LinkPtr FindLink(const std::string &_modelName, const std::string &_linkName);

   std::string MakeJointName(const gazebo_ros_link_attacher::AttachRequest &_request);

   void OnWorldUpdate();

   boost::mutex requestsMutex;
   boost::condition_variable requestsCondition;
   std::vector<RequestPtr> requests;

   /// \brief Pointer to the world.
   physics::WorldPtr world;
   event::ConnectionPtr worldUpdateConnection;
};

} // namespace gazebo

#endif
