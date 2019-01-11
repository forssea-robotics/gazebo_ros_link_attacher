#include <gazebo/common/Plugin.hh>
#include "gazebo_ros_link_attacher.h"
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

// Constructor
GazeboRosLinkAttacher::GazeboRosLinkAttacher() {
}

// Destructor
GazeboRosLinkAttacher::~GazeboRosLinkAttacher() {
}

void GazeboRosLinkAttacher::Load(const physics::WorldPtr _world, const sdf::ElementPtr _sdf) {
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->nodeHandle.reset(new ros::NodeHandle(_sdf->GetAttribute("name")->GetAsString()));

  this->world = _world;
  this->attachServiceServer = this->nodeHandle->advertiseService(
                                "attach", &GazeboRosLinkAttacher::OnAttachServiceCall, this);
  ROS_INFO_STREAM("Attach service at: " << this->nodeHandle->resolveName("attach"));
  this->detachServiceServer = this->nodeHandle->advertiseService(
                                "detach", &GazeboRosLinkAttacher::OnDetachServiceCall, this);
  ROS_INFO_STREAM("Detach service at: " << this->nodeHandle->resolveName("detach"));
  this->worldUpdateConnection = event::Events::ConnectWorldUpdateBegin(
                                  boost::bind(&GazeboRosLinkAttacher::OnWorldUpdate, this));
  ROS_INFO("Link attacher plugin initialized.");
}

bool GazeboRosLinkAttacher::AttachLinks(const gazebo_ros_link_attacher::AttachRequest &_request) {
  const physics::LinkPtr link_1 = FindLink(_request.model_name_1, _request.link_name_1);
  const physics::LinkPtr link_2 = FindLink(_request.model_name_2, _request.link_name_2);
  if (link_1 && link_2) {
    ROS_DEBUG_STREAM("Moving <" << _request.model_name_2 << "> towards <" << _request.model_name_1 << ">");
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d new_pos((link_1->WorldPose().Pos()) + link_1->WorldPose().Rot().RotateVector(
                                     ignition::math::Vector3d(_request.pos.x, _request.pos.y, _request.pos.z) - link_2->RelativePose().Pos()),
                                   link_1->WorldPose().Rot() * link_2->WorldPose().Rot());
    link_2->GetModel()->SetInitialRelativePose(new_pos);
    link_2->GetModel()->Reset();
    link_2->SetWorldTwist(link_1->RelativeLinearVel(), link_1->RelativeAngularVel(), true);
#else
    math::Pose new_pos((link_1->GetWorldPose().pos) + link_1->GetWorldPose().rot.RotateVector(
                           math::Vector3(_request.pos.x, _request.pos.y, _request.pos.z)),
                         link_1->GetWorldPose().rot * link_2->GetWorldPose().rot);
    link_2->GetModel()->SetInitialRelativePose(new_pos);
    link_2->GetModel()->Reset();
    link_2->SetWorldTwist(link_1->GetRelativeLinearVel(), link_1->GetRelativeAngularVel(), true);
#endif
    ROS_DEBUG_STREAM("Creating joint on model <" << _request.model_name_1 << ">");
    const physics::JointPtr joint = link_1->GetModel()->CreateJoint(
                                      MakeJointName(_request), "ball", link_1, link_2);

    if (joint) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      joint->Init();
#else
      joint->Init();
#endif
      return true;
    }
  }
  return false;
}

bool GazeboRosLinkAttacher::DetachLinks(const gazebo_ros_link_attacher::AttachRequest &_request) {
#if (GAZEBO_MAJOR_VERSION >= 8)
  const physics::ModelPtr model_1 = this->world->ModelByName(_request.model_name_1);
  const physics::ModelPtr model_2 = this->world->ModelByName(_request.model_name_2);
#else
  const physics::ModelPtr model_1 = this->world->GetModel(_request.model_name_1);
  const physics::ModelPtr model_2 = this->world->GetModel(_request.model_name_2);
#endif
  if (model_1 && model_2) {
    if (model_1->RemoveJoint(MakeJointName(_request))) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      model_1->SetWorldPose(model_1->WorldPose());
      model_2->SetWorldPose(model_2->WorldPose());
#else
      model_1->SetWorldPose(model_1->GetWorldPose());
      model_2->SetWorldPose(model_2->GetWorldPose());
#endif
      return true;
    }
  } else if (!model_1) {
    ROS_ERROR_STREAM("No model named '" << _request.model_name_1 << "'");
  } else if (!model_2) {
    ROS_ERROR_STREAM("No model named '" << _request.model_name_2 << "'");
  }
  return false;
}

physics::LinkPtr GazeboRosLinkAttacher::FindLink(const std::string &_modelName, const std::string &_linkName) {
  physics::LinkPtr link;
#if (GAZEBO_MAJOR_VERSION >= 8)
  const physics::ModelPtr model = this->world->ModelByName(_modelName);
#else
  const physics::ModelPtr model = this->world->GetModel(_modelName);
#endif
  if (model) {
    link = model->GetLink(_linkName);
    if (!link) {
      ROS_ERROR_STREAM("No link named '" << _linkName << "' in model '" << _modelName << "'");
    } else if (!link->GetInertial()) {
      ROS_WARN_STREAM("Link <" << _modelName << "::" << _linkName << "> has no inertial information");
    }
  } else {
    ROS_ERROR_STREAM("No model named '" << _modelName << "'");
  }

  return link;
}

bool GazeboRosLinkAttacher::OnAttachServiceCall(gazebo_ros_link_attacher::Attach::Request &_request,
    gazebo_ros_link_attacher::Attach::Response &_response) {
  _response.ok = this->HandleServiceCall(Request::ATTACH, _request);
  if (_response.ok) {
    ROS_INFO_STREAM("Successfully attached " << this->MakeJointName(_request));
  } else {
    ROS_ERROR_STREAM("Failed to attach " << this->MakeJointName(_request));
  }
  return true;
}

bool GazeboRosLinkAttacher::OnDetachServiceCall(gazebo_ros_link_attacher::Attach::Request &_request,
    gazebo_ros_link_attacher::Attach::Response &_response) {
  _response.ok = this->HandleServiceCall(Request::DETACH, _request);
  if (_response.ok) {
    ROS_INFO_STREAM("Successfully detached " << this->MakeJointName(_request));
  } else {
    ROS_ERROR_STREAM("Failed to detach " << this->MakeJointName(_request));
  }
  return true;
}

bool GazeboRosLinkAttacher::HandleServiceCall(
  Request::Type _requestType,
  gazebo_ros_link_attacher::Attach::Request &_request) {
  RequestPtr request(new Request);
  request->type = _requestType;
  request->status = Request::PENDING;
  request->data = _request;

  boost::mutex::scoped_lock lock(this->requestsMutex);
  this->requests.push_back(request);
  while (request->status == Request::PENDING) {
    this->requestsCondition.wait(lock);
  }
  return request->status == Request::DONE;
}

std::string GazeboRosLinkAttacher::MakeJointName(const gazebo_ros_link_attacher::AttachRequest &_request) {
  std::ostringstream s;
  s << "<" << _request.model_name_1 << "::" << _request.link_name_1 << ">-"
    << "<" << _request.model_name_2 << "::" << _request.link_name_2 << ">";
  return s.str();
}

void GazeboRosLinkAttacher::OnWorldUpdate() {
  boost::mutex::scoped_lock lock(this->requestsMutex);
  for (const RequestPtr &request : this->requests) {
    if (request->status == Request::PENDING) {
      if (request->type == Request::ATTACH) {
        request->status = this->AttachLinks(request->data) ? Request::DONE : Request::FAILED;
      } else if (request->type == Request::DETACH) {
        request->status = this->DetachLinks(request->data) ? Request::DONE : Request::FAILED;
      }
    }
  }
  this->requests.clear();
  this->requestsCondition.notify_all();
}
} // namespace gazebo
