/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_uav_managers/tracker.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <dynamic_reconfigure/server.h>
#include <trackers_brubotics/bypass_trackerConfig.h>

// added by us: 
#include <mrs_lib/profiler.h>
#include <mrs_msgs/FuturePoint.h>

//}

namespace trackers_brubotics
{

namespace bypass_tracker
{

/* //{ class BypassTracker */

class BypassTracker : public mrs_uav_managers::Tracker {
public:
  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  std::tuple<bool, std::string> activate(const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<mrs_msgs::TrackerCommand>   update(const mrs_msgs::UavState &uav_state, const mrs_uav_managers::Controller::ControlOutput &last_control_output);
  const mrs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr   setVelocityReference(const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;

  bool callbacks_enabled_ = true;

  std::string _uav_name_;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false;
  std::mutex         mutex_uav_state_;

  double uav_x_; // now added by bryan
  double uav_y_; // now added by bryan
  double uav_z_; // now added by bryan

  // | ------------------ dynamics constriants ------------------ |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | ----------------------- goal state ----------------------- |

  double goal_x_       = 0;
  double goal_y_       = 0;
  double goal_z_       = 0;
  double goal_heading_ = 0;
  // added by bryan: -----
  bool     have_goal_ = false;
  std::mutex mutex_goal_;
  //----------

  // | ---------------- the tracker's inner state --------------- |

  std::atomic<bool> is_initialized_ = false;
  std::atomic<bool> is_active_      = false;

  double pos_x_   = 0;
  double pos_y_   = 0;
  double pos_z_   = 0;
  double heading_ = 0;

  // | ------------------- for calculating dt ------------------- |

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                mutex_drs_;
  typedef trackers_brubotics::bypass_trackerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>      Drs_t;
  boost::shared_ptr<Drs_t>                              drs_;
  void                                                  callbackDrs(trackers_brubotics::bypass_trackerConfig &config, uint32_t level);
  DrsConfig_t                                           drs_params_;
  std::mutex                                            mutex_drs_params_;


  // added by Bryan: ---------
  mrs_lib::Profiler profiler_;
  bool hover_          = false;
  bool starting_bool=true;
  ros::Publisher pub_goal_pose_;
  //-----------
};

//}

// | -------------- tracker's interface routines -------------- |

/* //{ initialize() */

bool BypassTracker::initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {
  
  ROS_INFO("[BypassTracker]: start of initialize");

  this->common_handlers_  = common_handlers;
  this->private_handlers_ = private_handlers;

  _uav_name_ = common_handlers->uav_name;

  nh_ = nh; // ros::NodeHandle nh_(parent_nh, "bypass_tracker");

  ros::Time::waitForValid();

  last_update_time_ = ros::Time(0);

  // --------------------------------------------------------------
  // |                     loading parameters                     |
  // --------------------------------------------------------------

  // | -------------------- load param files -------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("trackers_brubotics") + "/config/bypass_tracker.yaml");

  if (!success) {
    return false;
  }

  // | ---------------- load plugin's parameters ---------------- |

  mrs_lib::ParamLoader param_loader(nh_, "BypassTracker");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("trackers_brubotics") + "/config/bypass_tracker.yaml");

  // param_loader.loadParam("some_parameter", drs_params_.some_parameter);

  ROS_INFO("[BypassTracker]: no parameters to be loaded");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BypassTracker]: could not load all parameters!");
    return false;
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&BypassTracker::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // create publishers
  pub_goal_pose_ = nh_.advertise<mrs_msgs::ReferenceStamped>("goal_pose", 10);


  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[BypassTracker]: initialized");

  return true;
}

//}

/* //{ activate() */

std::tuple<bool, std::string> BypassTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd) {

  if (last_tracker_cmd) {

    // actually, you should actually check if these entries are filled in
    pos_x_   = last_tracker_cmd->position.x;
    pos_y_   = last_tracker_cmd->position.y;
    pos_z_   = last_tracker_cmd->position.z;
    heading_ = last_tracker_cmd->heading;

    goal_x_       = pos_x_;
    goal_y_       = pos_y_;
    goal_z_       = pos_z_;
    goal_heading_ = heading_;

  } else {

    auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

    pos_x_ = uav_state.pose.position.x;
    pos_y_ = uav_state.pose.position.y;
    pos_z_ = uav_state.pose.position.z;

    try {
      heading_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
    }
    catch (...) {
      heading_ = 0;
    }
  }

  std::stringstream ss;
  ss << "activated";
  ROS_INFO_STREAM("[BypassTracker]: " << ss.str());

  is_active_ = true;

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void BypassTracker::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[BypassTracker]: deactivated");
}

//}

/* //{ update() */

std::optional<mrs_msgs::TrackerCommand> BypassTracker::update(const mrs_msgs::UavState &                                          uav_state,
                                                               [[maybe_unused]] const mrs_uav_managers::Controller::ControlOutput &last_control_output) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;

    uav_x_     = uav_state_.pose.position.x;
    uav_y_     = uav_state_.pose.position.y;
    uav_z_     = uav_state_.pose.position.z;

    got_uav_state_ = true;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[BypassTracker]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return {};
  }

  // | ------------------- fill in the result ------------------- |

  mrs_msgs::TrackerCommand tracker_cmd;

  tracker_cmd.header.stamp    = uav_state.header.stamp;//ros::Time::now();
  tracker_cmd.header.frame_id = uav_state.header.frame_id;

  if (starting_bool) {

    goal_x_= uav_state.pose.position.x;
    goal_y_= uav_state.pose.position.y;
    goal_z_= uav_state.pose.position.z;
    // set heading based on current odom
    try {
      goal_heading_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
      tracker_cmd.use_heading = 1;
    }
    catch (...) {
      tracker_cmd.use_heading = 0;
      ROS_ERROR_THROTTLE(1.0, "[BypassTracker]: could not calculate the current UAV heading");
    }

    tracker_cmd.position.x = goal_x_;
    tracker_cmd.position.y = goal_y_;
    tracker_cmd.position.z = goal_z_;
    tracker_cmd.heading    = goal_heading_;

    tracker_cmd.use_heading             = 1;
    tracker_cmd.use_position_vertical   = 1;
    tracker_cmd.use_position_horizontal = 1;
    tracker_cmd.use_velocity_vertical   = 1;
    tracker_cmd.use_velocity_horizontal = 1;
    tracker_cmd.use_acceleration        = 0;
    tracker_cmd.use_jerk                = 0;
    tracker_cmd.use_heading             = 1;
    tracker_cmd.use_heading_rate        = 1;

    starting_bool=false;

    ROS_INFO("[Bypass tracker - odom]: [goal_x_=%.2f],[goal_y_=%.2f],[goal_z_=%.2f, goal_heading_=%.2f]",goal_x_,goal_y_,goal_z_,goal_heading_);
  

    return {tracker_cmd};
  }

  // set the desired states from the input of the goto function

  tracker_cmd.position.x     = goal_x_;
  tracker_cmd.position.y     = goal_y_;
  tracker_cmd.position.z     = goal_z_;
  tracker_cmd.heading        = goal_heading_;

  tracker_cmd.use_position_vertical   = 1;
  tracker_cmd.use_position_horizontal = 1;
  tracker_cmd.use_velocity_vertical   = 1;
  tracker_cmd.use_velocity_horizontal = 1;
  tracker_cmd.use_heading             = 1;
  tracker_cmd.use_heading_rate        = 1;

  tracker_cmd.use_acceleration        = 0;
  tracker_cmd.use_jerk                = 0;

  // set the header
  tracker_cmd.header.stamp    = uav_state.header.stamp;
  tracker_cmd.header.frame_id = uav_state.header.frame_id;

  // publish the goal pose to a custom topic
  mrs_msgs::ReferenceStamped goal_pose;
  // goal_pose.header.stamp          = ros::Time::now();
  // goal_pose.header.frame_id  = "fcu_untilted";
  goal_pose.header.stamp    = uav_state.header.stamp;
  goal_pose.header.frame_id = uav_state.header.frame_id;
  goal_pose.reference.position.x  = goal_x_;
  goal_pose.reference.position.y  = goal_y_;
  goal_pose.reference.position.z  = goal_z_;
  goal_pose.reference.heading = goal_heading_;
  pub_goal_pose_.publish(goal_pose);

  return {tracker_cmd};
}

//}

/* //{ resetStatic() */

bool BypassTracker::resetStatic(void) {
  ROS_INFO("[BypassTracker]: no states to reset");
  return false;
}

//}

/* //{ getStatus() */

const mrs_msgs::TrackerStatus BypassTracker::getStatus() {

  mrs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr BypassTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;
  std::stringstream         ss;

  if (cmd->data != callbacks_enabled_) {

    callbacks_enabled_ = cmd->data;

    ss << "callbacks " << (callbacks_enabled_ ? "enabled" : "disabled");
    ROS_INFO_STREAM_THROTTLE(1.0, "[BypassTracker]: " << ss.str());

  } else {

    ss << "callbacks were already " << (callbacks_enabled_ ? "enabled" : "disabled");
    ROS_WARN_STREAM_THROTTLE(1.0, "[BypassTracker]: " << ss.str());
  }

  res.message = ss.str();
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}

//}

/* switchOdometrySource() //{ */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState &new_uav_state) {
 std_srvs::TriggerResponse res;

  res.message = "Switching not implemented";
  res.success = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
  //return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  std_srvs::TriggerResponse res;
  res.message = "hover initiated";
  res.success = true;

  hover_ = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));

  //return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr BypassTracker::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  {
    std::scoped_lock lock(mutex_constraints_);

    constraints_ = cmd->constraints;
  }

  mrs_msgs::DynamicsConstraintsSrvResponse res;

  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

/* //{ setReference() */

const mrs_msgs::ReferenceSrvResponse::ConstPtr BypassTracker::setReference([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  { 
    std::scoped_lock lock(mutex_goal_);
    goal_x_       = cmd->reference.position.x;
    goal_y_       = cmd->reference.position.y;
    goal_z_       = cmd->reference.position.z;
    goal_heading_ = cmd->reference.heading;
  }

  hover_ = false;

  mrs_msgs::ReferenceSrvResponse response;

  response.message = "reference set";
  response.success = true;

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(response));
}

//}

/* //{ setVelocityReference() */

const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr BypassTracker::setVelocityReference([
    [maybe_unused]] const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr BypassTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void BypassTracker::callbackDrs(trackers_brubotics::bypass_trackerConfig &config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[BypassTracker]: dynamic reconfigure params updated");
}

//}

}  // namespace bypass_tracker

}  // namespace trackers_brubotics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trackers_brubotics::bypass_tracker::BypassTracker, mrs_uav_managers::Tracker)