#define VERSION "0.0.5.1"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/tracker.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/utils.h>

//}

#define STOP_THR 1e-3

namespace mrs_uav_trackers
{

namespace bypass_tracker
{

/* //{ class BypassTracker */

typedef enum
{

  IDLE_STATE,
  STOP_MOTION_STATE,
  ACCELERATING_STATE,
  DECELERATING_STATE,
  STOPPING_STATE,

} States_t;

class BypassTracker : public mrs_uav_managers::Tracker {
public:
  void                          initialize(const ros::NodeHandle &parent_nh, const std::string uav_name, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  std::tuple<bool, std::string> activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  const mrs_msgs::PositionCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  const mrs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  bool callbacks_enabled_ = true;

  std::string _version_;
  std::string _uav_name_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false;
  std::mutex         mutex_uav_state_;

  double uav_x_;
  double uav_y_;
  double uav_z_;

  // tracker's inner states
  double _tracker_loop_rate_;
  double _tracker_dt_;
  bool   is_initialized_ = false;
  bool   is_active_      = false;
  bool   first_iter_     = false;

  // | ------------------ dynamics constraints ------------------ |

  double     _horizontal_speed_;
  double     _vertical_speed_;
  double     _horizontal_acceleration_;
  double     _vertical_acceleration_;
  double     _heading_rate_;
  double     _heading_gain_;
  std::mutex mutex_constraints_;

  // | ---------------------- desired goal ---------------------- |

  double     goal_x_;
  double     goal_y_;
  double     goal_z_;
  double     goal_heading_;
  double     have_goal_ = false;
  std::mutex mutex_goal_;
  double global_goal_x = 0;
  double global_goal_y = 0;
  double global_goal_z = 2;
  double global_goal_heading = 0;

  // | ------------------- the state variables ------------------ |
  double state_x_;
  double state_y_;
  double state_z_;
  double state_heading_;

  double speed_x_;
  double speed_y_;
  double speed_heading_;

  double current_heading_;
  double current_vertical_direction_;

  double current_vertical_speed_;
  double current_horizontal_speed_;

  double current_horizontal_acceleration_;
  double current_vertical_acceleration_;

  std::mutex mutex_state_;

  bool hovering;

  // | ------------------------ profiler ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// | -------------- tracker's interface routines -------------- |

/* //{ initialize() */

void BypassTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  _uav_name_             = uav_name;
  this->common_handlers_ = common_handlers;

  ros::NodeHandle nh_(parent_nh, "bypass_tracker");

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                       load parameters                      |
  // --------------------------------------------------------------


  mrs_lib::ParamLoader param_loader(nh_, "BypassTracker");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[BypassTracker]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("horizontal_tracker/horizontal_speed", _horizontal_speed_);
  param_loader.loadParam("horizontal_tracker/horizontal_acceleration", _horizontal_acceleration_);

  param_loader.loadParam("vertical_tracker/vertical_speed", _vertical_speed_);
  param_loader.loadParam("vertical_tracker/vertical_acceleration", _vertical_acceleration_);

  param_loader.loadParam("heading_tracker/heading_rate", _heading_rate_);
  param_loader.loadParam("heading_tracker/heading_gain", _heading_gain_);

  param_loader.loadParam("tracker_loop_rate", _tracker_loop_rate_);

  _tracker_dt_ = 1.0 / double(_tracker_loop_rate_);

  ROS_INFO("[BypassTracker]: tracker_dt: %.2f", _tracker_dt_);

  state_x_       = 0;
  state_y_       = 0;
  state_z_       = 0;
  state_heading_ = 0;

  speed_x_       = 0;
  speed_y_       = 0;
  speed_heading_ = 0;

  current_horizontal_speed_ = 0;
  current_vertical_speed_   = 0;

  current_horizontal_acceleration_ = 0;
  current_vertical_acceleration_   = 0;

  current_vertical_direction_ = 0;

  // --------------------------------------------------------------
  // |                          profiler_                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "BypassTracker", _profiler_enabled_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BypassTracker]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[BypassTracker]: initialized, version %s", VERSION);
}

//}

/* //{ activate() */

std::tuple<bool, std::string> BypassTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) {

  std::stringstream ss;

  if (!got_uav_state_) {

    ss << "odometry not set";
    ROS_ERROR_STREAM("[BypassTracker]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  // copy member variables
  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double uav_heading;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ss << "could not calculate the UAV heading";
    return std::tuple(false, ss.str());
  }

  {
    std::scoped_lock lock(mutex_goal_, mutex_state_);

    if (mrs_msgs::PositionCommand::Ptr() != last_position_cmd) {

      // the last command is usable
      if (last_position_cmd->use_position_horizontal) {
        state_x_ = last_position_cmd->position.x;
        state_y_ = last_position_cmd->position.y;
      } else {
        state_x_ = uav_state.pose.position.x;
        state_y_ = uav_state.pose.position.y;
      }

      if (last_position_cmd->use_position_vertical) {
        state_z_ = last_position_cmd->position.z;
      } else {
        state_z_ = uav_state.pose.position.z;
      }

      if (last_position_cmd->use_heading) {
        state_heading_ = last_position_cmd->heading;
      } else if (last_position_cmd->use_orientation) {
        try {
          state_heading_ = mrs_lib::AttitudeConverter(last_position_cmd->orientation).getHeading();
        }
        catch (...) {
          state_heading_ = uav_heading;
        }
      } else {
        state_heading_ = uav_heading;
      }

      if (last_position_cmd->use_velocity_horizontal) {
        speed_x_ = last_position_cmd->velocity.x;
        speed_y_ = last_position_cmd->velocity.y;
      } else {
        speed_x_ = uav_state.velocity.linear.x;
        speed_y_ = uav_state.velocity.linear.y;
      }

      current_heading_          = atan2(speed_y_, speed_x_);
      current_horizontal_speed_ = sqrt(pow(speed_x_, 2) + pow(speed_y_, 2));

      current_vertical_speed_     = fabs(last_position_cmd->velocity.z);
      current_vertical_direction_ = last_position_cmd->velocity.z > 0 ? +1 : -1;

      current_horizontal_acceleration_ = 0;
      current_vertical_acceleration_   = 0;

      goal_heading_ = last_position_cmd->heading;

      ROS_INFO("[BypassTracker]: initial condition: x=%.2f, y=%.2f, z=%.2f, heading=%.2f", last_position_cmd->position.x, last_position_cmd->position.y,
               last_position_cmd->position.z, last_position_cmd->heading);
      ROS_INFO("[BypassTracker]: initial condition: x_rate=%.2f, y_rate=%.2f, z_rate=%.2f", speed_x_, speed_y_, current_vertical_speed_);

    } else {

      state_x_       = uav_state.pose.position.x;
      state_y_       = uav_state.pose.position.y;
      state_z_       = uav_state.pose.position.z;
      state_heading_ = uav_heading;

      speed_x_                  = uav_state.velocity.linear.x;
      speed_y_                  = uav_state.velocity.linear.y;
      current_heading_          = atan2(speed_y_, speed_x_);
      current_horizontal_speed_ = sqrt(pow(speed_x_, 2) + pow(speed_y_, 2));

      current_vertical_speed_     = fabs(uav_state.velocity.linear.z);
      current_vertical_direction_ = uav_state.velocity.linear.z > 0 ? +1 : -1;

      current_horizontal_acceleration_ = 0;
      current_vertical_acceleration_   = 0;

      goal_heading_ = uav_heading;

      ROS_WARN("[BypassTracker]: the previous command is not usable for activation, using Odometry instead");
    }
  }

  // --------------------------------------------------------------
  // |          horizontal initial conditions prediction          |
  // --------------------------------------------------------------

  double horizontal_t_stop, horizontal_stop_dist, stop_dist_x, stop_dist_y;

  {
    std::scoped_lock lock(mutex_state_);

    horizontal_t_stop    = current_horizontal_speed_ / _horizontal_acceleration_;
    horizontal_stop_dist = (horizontal_t_stop * current_horizontal_speed_) / 2.0;
    stop_dist_x          = cos(current_heading_) * horizontal_stop_dist;
    stop_dist_y          = sin(current_heading_) * horizontal_stop_dist;
  }

  // --------------------------------------------------------------
  // |           vertical initial conditions prediction           |
  // --------------------------------------------------------------

  double vertical_t_stop, vertical_stop_dist;

  {
    std::scoped_lock lock(mutex_state_);

    vertical_t_stop    = current_vertical_speed_ / _vertical_acceleration_;
    vertical_stop_dist = current_vertical_direction_ * (vertical_t_stop * current_vertical_speed_) / 2.0;
  }

  // --------------------------------------------------------------
  // |              heading initial condition  prediction             |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_goal_, mutex_state_);

    goal_x_ = state_x_ + stop_dist_x;
    goal_y_ = state_y_ + stop_dist_y;
    goal_z_ = state_z_ + vertical_stop_dist;

    ROS_INFO("[BypassTracker]: setting z goal to %.2f", goal_z_);
  }

  is_active_ = true;

  ss << "activated";
  ROS_INFO_STREAM("[BypassTracker]: " << ss.str());

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void BypassTracker::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[BypassTracker]: deactivated");
}

//}

/* //{ resetStatic() */

bool BypassTracker::resetStatic(void) {

  if (!is_initialized_) {
    ROS_ERROR("[BypassTracker]: can not reset, not initialized");
    return false;
  }

  if (!is_active_) {
    ROS_ERROR("[BypassTracker]: can not reset, not active");
    return false;
  }

  ROS_INFO("[BypassTracker]: reseting with no dynamics");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double uav_heading;
  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[BypassTracker]: could not calculate the UAV heading");
    return false;
  }

  {
    std::scoped_lock lock(mutex_goal_, mutex_state_, mutex_uav_state_);

    state_x_       = uav_state_.pose.position.x;
    state_y_       = uav_state_.pose.position.y;
    state_z_       = uav_state_.pose.position.z;
    state_heading_ = uav_heading;

    speed_x_                  = 0;
    speed_y_                  = 0;
    current_heading_          = 0;
    current_horizontal_speed_ = 0;

    current_vertical_speed_     = 0;
    current_vertical_direction_ = 0;

    current_horizontal_acceleration_ = 0;
    current_vertical_acceleration_   = 0;

    goal_heading_ = uav_heading;
  }

  return true;
}

//}

/* //{ update() */

const mrs_msgs::PositionCommand::ConstPtr BypassTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
                                                              [[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");


  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
    uav_x_     = uav_state_.pose.position.x;
    uav_y_     = uav_state_.pose.position.y;
    uav_z_     = uav_state_.pose.position.z;

    got_uav_state_ = true;
  }

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return mrs_msgs::PositionCommand::Ptr();
  }
  mrs_msgs::PositionCommand position_cmd;

  position_cmd.header.stamp    = ros::Time::now();
  position_cmd.header.frame_id = uav_state->header.frame_id;
  if (!hovering)
  {
    std::scoped_lock lock(mutex_state_);

    position_cmd.position.x = global_goal_x;
    position_cmd.position.y = global_goal_y;
    position_cmd.position.z = global_goal_z;
    position_cmd.heading    = global_goal_heading;

    position_cmd.velocity.x   = 0;
    position_cmd.velocity.y   = 0;
    position_cmd.velocity.z   = 0;
    position_cmd.heading_rate = 0;

    position_cmd.acceleration.x = 0;
    position_cmd.acceleration.y = 0;
    position_cmd.acceleration.z = 0;

    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_heading             = 0;
    position_cmd.use_heading_rate        = 0;
    position_cmd.use_velocity_vertical   = 0;
    position_cmd.use_velocity_horizontal = 0;
    position_cmd.use_acceleration        = 0;
  }
  else
  {
    std::scoped_lock lock(mutex_state_);

    position_cmd.position.x = uav_x_;
    position_cmd.position.y = uav_y_;
    position_cmd.position.z = uav_z_;
    position_cmd.heading    = 0;

    position_cmd.velocity.x   = 0;
    position_cmd.velocity.y   = 0;
    position_cmd.velocity.z   = 0;
    position_cmd.heading_rate = 0;

    position_cmd.acceleration.x = 0;
    position_cmd.acceleration.y = 0;
    position_cmd.acceleration.z = 0;

    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_heading             = 0;
    position_cmd.use_heading_rate        = 0;
    position_cmd.use_velocity_vertical   = 0;
    position_cmd.use_velocity_horizontal = 0;
    position_cmd.use_acceleration        = 0;
  }

  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}

//}

/* //{ getStatus() */

const mrs_msgs::TrackerStatus BypassTracker::getStatus() {

  mrs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  tracker_status.tracking_trajectory = false;

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

const std_srvs::TriggerResponse::ConstPtr BypassTracker::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) {

  std::scoped_lock lock(mutex_goal_, mutex_state_);

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  double old_heading, new_heading;
  bool   got_headings = true;

  try {
    old_heading = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[BypassTracker]: could not calculate the old UAV heading");
    got_headings = false;
  }

  try {
    new_heading = mrs_lib::AttitudeConverter(new_uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[BypassTracker]: could not calculate the new UAV heading");
    got_headings = false;
  }

  std_srvs::TriggerResponse res;

  if (!got_headings) {
    res.message = "could not calculate the heading difference";
    res.success = false;

    return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
  }

  // | --------- recalculate the goal to new coordinates -------- |

  double dx       = new_uav_state->pose.position.x - uav_state.pose.position.x;
  double dy       = new_uav_state->pose.position.y - uav_state.pose.position.y;
  double dz       = new_uav_state->pose.position.z - uav_state.pose.position.z;
  double dheading = new_heading - old_heading;

  goal_x_ += dx;
  goal_y_ += dy;
  goal_z_ += dz;
  goal_heading_ += dheading;

  // | -------------------- update the state -------------------- |

  state_x_ += dx;
  state_y_ += dy;
  state_z_ += dz;
  state_heading_ += dheading;

  current_heading_ = atan2(goal_y_ - state_y_, goal_x_ - state_x_);

  res.message = "odometry source switched";
  res.success = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}

//}

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  std_srvs::TriggerResponse res;
  res.message = "hover initiated";
  res.success = true;

  hovering = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hovering = false;
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hovering = true;
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hovering = false;
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr BypassTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr BypassTracker::setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd) {

  mrs_msgs::TrackerConstraintsSrvResponse res;

  // this is the place to copy the constraints
  {
    std::scoped_lock lock(mutex_constraints_);

    _horizontal_speed_        = cmd->constraints.horizontal_speed;
    _horizontal_acceleration_ = cmd->constraints.horizontal_acceleration;

    _vertical_speed_        = cmd->constraints.vertical_ascending_speed;
    _vertical_acceleration_ = cmd->constraints.vertical_ascending_acceleration;

    _heading_rate_ = cmd->constraints.heading_speed;
  }

  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr(new mrs_msgs::TrackerConstraintsSrvResponse(res));
}

//}

/* //{ setReference() */

const mrs_msgs::ReferenceSrvResponse::ConstPtr BypassTracker::setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  mrs_msgs::ReferenceSrvResponse res;

  {
    std::scoped_lock lock(mutex_goal_);
    global_goal_x = cmd->reference.position.x;
    global_goal_y = cmd->reference.position.y;
    global_goal_z = cmd->reference.position.z;
    goal_heading_ = mrs_lib::wrapAngle(cmd->reference.heading);
    global_goal_heading = mrs_lib::wrapAngle(cmd->reference.heading);

    ROS_INFO("[BypassTracker]: received new setpoint %.2f, %.2f, %.2f, %.2f", goal_x_, goal_y_, goal_z_, goal_heading_);

    have_goal_ = true;
  }

  hovering = false;

  res.success = true;
  res.message = "reference set";

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(res));
}

//}

/* //{ setTrajectoryReference() */

const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr BypassTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}


//}

}  // namespace bypass_tracker

}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::bypass_tracker::BypassTracker, mrs_uav_managers::Tracker)
