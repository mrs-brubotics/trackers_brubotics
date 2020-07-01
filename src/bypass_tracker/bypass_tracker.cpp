#define VERSION "0.0.0.0"

#include <ros/ros.h>
#include <mrs_uav_managers/tracker.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>

namespace mrs_uav_trackers
{

namespace bypass_tracker
{

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
  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;
  std::mutex         mutex_state_;
  std::mutex         mutex_goal_;

  mrs_lib::Profiler profiler_;

  bool is_initialized_ = false;
  bool is_active_      = false;
  bool hover_          = false;

  double uav_x_ = 0;
  double uav_y_ = 0;
  double uav_z_ = 2;

  double global_goal_x = 0;
  double global_goal_y = 0;
  double global_goal_z = 2;
  double global_goal_heading = 0;

};

void BypassTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {
  ros::Time::waitForValid();
  is_initialized_ = true;

  ROS_INFO("[BypassTracker]: initialized");
}


std::tuple<bool, std::string> BypassTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) {
  std::stringstream ss;
  ss << "Activated";
  is_active_ = true;
  ROS_INFO("[BypassTracker]: activated");
  return std::tuple(true, ss.str());
}


void BypassTracker::deactivate(void) {
  is_active_ = false;
  ROS_INFO("[BypassTracker]: deactivated");
}


bool BypassTracker::resetStatic(void) {
  return true;
}


const mrs_msgs::PositionCommand::ConstPtr BypassTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
                                                              [[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");


  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
    uav_x_     = uav_state_.pose.position.x;
    uav_y_     = uav_state_.pose.position.y;
    uav_z_     = uav_state_.pose.position.z;
  }

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return mrs_msgs::PositionCommand::Ptr();
  }
  mrs_msgs::PositionCommand position_cmd;

  position_cmd.header.stamp    = ros::Time::now();
  position_cmd.header.frame_id = uav_state->header.frame_id;
  if (!hover_)
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


const mrs_msgs::TrackerStatus BypassTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;
  tracker_status.active = is_active_;
  return tracker_status;
}


const std_srvs::SetBoolResponse::ConstPtr BypassTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  res.message = "Callbacks not implemented";
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) {

  std_srvs::TriggerResponse res;

  res.message = "Switching not implemented";
  res.success = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  std_srvs::TriggerResponse res;
  res.message = "hover initiated";
  res.success = true;

  hover_ = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = false;
  return std_srvs::TriggerResponse::Ptr();
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = true;
  return std_srvs::TriggerResponse::Ptr();
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = false;
  return std_srvs::TriggerResponse::Ptr();
}


const std_srvs::TriggerResponse::ConstPtr BypassTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}


const mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr BypassTracker::setConstraints(const mrs_msgs::TrackerConstraintsSrvRequest::ConstPtr &cmd) {

  mrs_msgs::TrackerConstraintsSrvResponse res;

  res.success = true;
  res.message = "No constraints to update";

  return mrs_msgs::TrackerConstraintsSrvResponse::ConstPtr(new mrs_msgs::TrackerConstraintsSrvResponse(res));
}


const mrs_msgs::ReferenceSrvResponse::ConstPtr BypassTracker::setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  mrs_msgs::ReferenceSrvResponse res;

  {
    std::scoped_lock lock(mutex_goal_);
    global_goal_x = cmd->reference.position.x;
    global_goal_y = cmd->reference.position.y;
    global_goal_z = cmd->reference.position.z;
    global_goal_heading = mrs_lib::wrapAngle(cmd->reference.heading);

    ROS_INFO("[BypassTracker]: received new setpoint %.2f, %.2f, %.2f, %.2f", global_goal_x, global_goal_y, global_goal_z, global_goal_heading);
  }

  hover_ = false;

  res.success = true;
  res.message = "reference set";

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(res));
}


const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr BypassTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

}  // namespace bypass_tracker
}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::bypass_tracker::BypassTracker, mrs_uav_managers::Tracker)
