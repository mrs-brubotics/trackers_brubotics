#define VERSION "0.0.0.0"

/*includes//{*/
#include <ros/ros.h>

#include <mrs_uav_managers/tracker.h>

#include <Eigen/Eigen>

#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>

#include <mrs_msgs/FuturePoint.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/OdometryDiag.h>


//}

/*defines//{*/

#define STRING_EQUAL 0
//}

using namespace Eigen;

namespace mrs_uav_trackers
{

namespace derg_tracker
{

/*class DergTracker//{*/
class DergTracker : public mrs_uav_managers::Tracker {
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

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);


  // D erg
  mrs_msgs::FutureTrajectory uav_applied_ref_out;
  mrs_msgs::FuturePoint custom_new_point;

  std::map<std::string, mrs_msgs::FutureTrajectory> other_drones_applied_references;
  std::vector<ros::Subscriber> other_uav_applied_ref_subscriber;

  ros::Publisher uav_applied_ref_message_publisher;

  void callbackOtherUavAppliedRef(const mrs_msgs::FutureTrajectoryConstPtr& msg);
  void DERG_computation();

private:
  std::mutex         mutex_goal_;

  mrs_lib::Profiler profiler_;

  bool is_initialized_ = false;
  bool is_active_      = false;
  bool hover_          = false;

  bool starting_bool=true;

  float goto_ref_x = 0;
  float goto_ref_y = 0;
  float goto_ref_z = 0;
  float goto_ref_heading = 0;
  
  // applied reference
  float applied_ref_x=0;
  float applied_ref_y=0;
  float applied_ref_z=0;
 
  // initial conditions
  MatrixXd init_pos = MatrixXd::Zero(3, 1);
  
  // ERG parameters
  MatrixXd NF_total=MatrixXd::Zero(3, 1);
  float DSM_total;
  MatrixXd NF_att=MatrixXd::Zero(3, 1); // attraction Navigation field
  
  // Wall all_constraints
  MatrixXd NF_w = MatrixXd::Zero(3, 1); // Wall repulsion navigation field

  //D erg
  ros::NodeHandle nh_;
  MatrixXd NF_o = MatrixXd::Zero(3, 1); // obstacle repulsion navigation field
  
  // agent collision avoidance_active_uavs
  MatrixXd NF_a = MatrixXd::Zero(3, 1); // agent repulsion navigation field
  MatrixXd NF_a_co = MatrixXd::Zero(3, 1); // conservative part
  MatrixXd NF_a_nco = MatrixXd::Zero(3, 1); // non conservative part
  
  float pos_error_x;
  float pos_error_y;
  float pos_error;
  float kappa_a=30;
  float DSM_a;

  float other_uav_ref_x;
  float other_uav_ref_y;
  float dist_between_ref_x;
  float dist_between_ref_y;
  float dist_between_ref;
  float max_repulsion_other_uav;
  float sigma_a=1;
  float delta_a=0.01;
  float Ra=0.4;
  float Sa=2;
  float alpha_a=0.1;

  std::string uav_name_;
  std::vector<std::string> other_drone_names_;
  
  std::vector<ros::Subscriber> other_uav_subscribers;

  int my_uav_number;
  int my_uav_priority;

  bool collision_avoidance_enabled_ = false;

  mrs_msgs::OdometryDiag odometry_diagnostics;
  mrs_msgs::FutureTrajectory future_trajectory_out;


};
//}

/*initialize()//{*/
void DergTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  uav_name_ = uav_name;
  
  ros::NodeHandle nh_(parent_nh, "derg_tracker");
  mrs_lib::ParamLoader param_loader(nh_, "DergTracker");

  ros::Time::waitForValid();
  
  future_trajectory_out.stamp    = ros::Time::now();
  future_trajectory_out.uav_name = uav_name_;
  future_trajectory_out.priority = my_uav_priority;
  future_trajectory_out.collision_avoidance = collision_avoidance_enabled_ && ((odometry_diagnostics.estimator_type.name.compare(std::string("GPS")) == STRING_EQUAL) || odometry_diagnostics.estimator_type.name.compare(std::string("RTK")) == STRING_EQUAL);

  uav_applied_ref_out=future_trajectory_out; // initialize the message

  // extract the numerical name
  sscanf(uav_name_.c_str(), "uav%d", &my_uav_number);
  ROS_INFO("[DergTracker]: Numerical ID of this UAV is %d", my_uav_number);
  my_uav_priority = my_uav_number;

  param_loader.loadParam("network/robot_names", other_drone_names_);

  // exclude this drone from the list
  std::vector<std::string>::iterator it = other_drone_names_.begin();
  while (it != other_drone_names_.end()) {

    std::string temp_str = *it;

    int other_uav_priority;
    sscanf(temp_str.c_str(), "uav%d", &other_uav_priority);

    if (other_uav_priority == my_uav_number) {

      other_drone_names_.erase(it);
      continue;
    }

    it++;
  }

  uav_applied_ref_message_publisher = nh_.advertise<mrs_msgs::FutureTrajectory>("uav_applied_ref", 1); // publish the reference with the name of the uav


  // subscribe to other UAV's applied reference
  for (unsigned long i = 0; i < other_drone_names_.size(); i++) {

    std::string applied_ref_topic_name = std::string("/") + other_drone_names_[i] + std::string("/") + std::string("control_manager/derg_tracker/uav_applied_ref");

    ROS_INFO("[DergTracker]: subscribing to %s", applied_ref_topic_name.c_str());

    other_uav_subscribers.push_back(nh_.subscribe(applied_ref_topic_name, 1, &DergTracker::callbackOtherUavAppliedRef, this, ros::TransportHints().tcpNoDelay()));

  }

  is_initialized_ = true;

  ROS_INFO("[DergTracker]: initialized");

}
//}

/*activate()//{*/
std::tuple<bool, std::string> DergTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) {
  std::stringstream ss;
  ss << "Activated";
  is_active_ = true;
  ROS_INFO("[DergTracker]: activated");
  return std::tuple(true, ss.str());
}
//}

/*deactivate()//{*/
void DergTracker::deactivate(void) {
  is_active_ = false;
  ROS_INFO("[DergTracker]: deactivated");
}
//}

/*resetStatic()//{*/
bool DergTracker::resetStatic(void) {
  return true;
}
//}

/*update()//{*/
const mrs_msgs::PositionCommand::ConstPtr DergTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
                                                              [[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");


  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return mrs_msgs::PositionCommand::Ptr();
  }
  mrs_msgs::PositionCommand position_cmd;

  position_cmd.header.stamp    = uav_state->header.stamp;
  position_cmd.header.frame_id = uav_state->header.frame_id;
    if (starting_bool) {

    goto_ref_x= uav_state->pose.position.x;
    goto_ref_y= uav_state->pose.position.y;
    goto_ref_z= uav_state->pose.position.z;
    // set heading based on current odom
    try {
      goto_ref_heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR_THROTTLE(1.0, "[DergTracker]: could not calculate the current UAV heading");
    }

    position_cmd.position.x     = goto_ref_x;
    position_cmd.position.y     = goto_ref_y;
    position_cmd.position.z     = goto_ref_z;
    position_cmd.heading        = goto_ref_heading;

    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_velocity_vertical   = 1;
    position_cmd.use_velocity_horizontal = 1;
    position_cmd.use_acceleration        = 0;
    position_cmd.use_jerk                = 0;
    position_cmd.use_heading             = 1;
    position_cmd.use_heading_rate        = 1;

    starting_bool=false;

    ROS_INFO(
        "[Bypass tracker - odom]: [goto_ref_x=%.2f],[goto_ref_y=%.2f],[goto_ref_z=%.2f, goto_ref_heading=%.2f]",goto_ref_x,goto_ref_y,goto_ref_z,goto_ref_heading);
 
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}

 // set the desired states from the input of the goto function

  position_cmd.position.x     = goto_ref_x;
  position_cmd.position.y     = goto_ref_y;
  position_cmd.position.z     = goto_ref_z;
  position_cmd.heading        = goto_ref_heading;

  position_cmd.use_position_vertical   = 1;
  position_cmd.use_position_horizontal = 1;
  position_cmd.use_velocity_vertical   = 1;
  position_cmd.use_velocity_horizontal = 1;
  position_cmd.use_acceleration        = 0;
  position_cmd.use_jerk                = 0;
  position_cmd.use_heading             = 1;
  position_cmd.use_heading_rate        = 1;

  // set the header
  position_cmd.header.stamp    = uav_state->header.stamp;
  position_cmd.header.frame_id = uav_state->header.frame_id;

  // u have to return a position command
  // can set the jerk to 0
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}
//}

/*DERG_computation()//{*/
  void DergTracker::DERG_computation(){

  custom_new_point.x = applied_ref_x;
  custom_new_point.y = applied_ref_y;
  custom_new_point.z = applied_ref_z;

  uav_applied_ref_out.points.push_back(custom_new_point);
  uav_applied_ref_message_publisher.publish(uav_applied_ref_out);
  uav_applied_ref_out.points.clear();


  /////////////////////////////////////////////////////////////
  //// computation of the agent repulsion navigation field ////
  /////////////////////////////////////////////////////////////
  
  pos_error_x= applied_ref_x - init_pos(0,0);
  pos_error_y= applied_ref_y - init_pos(1,0);
  pos_error= sqrt(pos_error_x*pos_error_x + pos_error_y*pos_error_y);

  DSM_a=kappa_a*(Sa-pos_error);
  
  NF_a_co(0,0)=0;
  NF_a_co(1,0)=0;
  NF_a_co(2,0)=0;

  NF_a_nco(0,0)=0;
  NF_a_nco(1,0)=0;
  NF_a_nco(2,0)=0;
  
  std::map<std::string, mrs_msgs::FutureTrajectory>::iterator u = other_drones_applied_references.begin();

  while (u != other_drones_applied_references.end()) {
    other_uav_ref_x = u->second.points[0].x;
    other_uav_ref_y = u->second.points[0].y;
    dist_between_ref_x = other_uav_ref_x - applied_ref_x;
    dist_between_ref_y = other_uav_ref_y - applied_ref_y;
    dist_between_ref= sqrt(dist_between_ref_x*dist_between_ref_x+dist_between_ref_y*dist_between_ref_y);
    //ROS_INFO("[DergTracker]: dist_between_ref_x: %f", dist_between_ref_x);

    // Conservative part
    max_repulsion_other_uav=(sigma_a-(dist_between_ref-2*Ra-2*Sa))/(sigma_a-delta_a);
    if (0>max_repulsion_other_uav) {
      max_repulsion_other_uav=0;
    }

    NF_a_co(0,0)=NF_a_co(0,0)-max_repulsion_other_uav*(dist_between_ref_x/dist_between_ref);
    NF_a_co(1,0)=NF_a_co(1,0)-max_repulsion_other_uav*(dist_between_ref_y/dist_between_ref);

    // Non conservative part
    if (sigma_a >= dist_between_ref-2*Ra-2*Sa) {
      NF_a_nco(0,0)=NF_a_nco(0,0) + alpha_a*dist_between_ref_y;
      NF_a_nco(1,0)=NF_a_nco(1,0) -alpha_a*dist_between_ref_x;
    }

    u++;
  }
 
  NF_a(0,0)=NF_a_co(0,0) + NF_a_nco(0,0);
  NF_a(1,0)=NF_a_co(1,0) + NF_a_nco(1,0);
  NF_a(2,0)=NF_a_co(2,0) + NF_a_nco(2,0);
  

  ////////////////////////////////////////////////////////////////////
  //// computation of v_dot //////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////

  // total navigation field
  NF_total(0,0)=NF_att(0,0) + NF_w(0,0) + NF_o(0,0) + NF_a(0,0);
  NF_total(1,0)=NF_att(1,0) + NF_w(1,0) + NF_o(1,0) + NF_a(1,0);
  NF_total(2,0)=NF_att(2,0) + NF_w(2,0) + NF_o(2,0) + NF_a(2,0);
  
  if (DSM_a<DSM_total){
    DSM_total=DSM_a;
  }

  //DSM_total=10; // for constant DSM

  }
//}

/*getStatus()//{*/
const mrs_msgs::TrackerStatus DergTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;
  tracker_status.active = is_active_;
  return tracker_status;
}
//}

/*enableCallbacks()//{*/
const std_srvs::SetBoolResponse::ConstPtr DergTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  res.message = "Callbacks not implemented";
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}
//}

/*switchOdometrySource()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) {

  std_srvs::TriggerResponse res;

  res.message = "Switching not implemented";
  res.success = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*hover()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  std_srvs::TriggerResponse res;
  res.message = "hover initiated";
  res.success = true;

  hover_ = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*startTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = false;
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*stopTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = true;
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*resumeTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  hover_ = false;
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*gotoTrajectoryStart()//{*/
const std_srvs::TriggerResponse::ConstPtr DergTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*setConstraints()//{*/
const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr DergTracker::setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  mrs_msgs::DynamicsConstraintsSrvResponse res;

  res.success = true;
  res.message = "No constraints to update";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}
//}

/*setReference()//{*/
const mrs_msgs::ReferenceSrvResponse::ConstPtr DergTracker::setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  mrs_msgs::ReferenceSrvResponse res;

  {
    std::scoped_lock lock(mutex_goal_);

  goto_ref_x=cmd->reference.position.x;
  goto_ref_y=cmd->reference.position.y;
  goto_ref_z=cmd->reference.position.z;
  goto_ref_heading=cmd->reference.heading;

  }

  hover_ = false;

  res.success = true;
  res.message = "reference set";

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(res));
}
//}

/*setTrajectoryReference()//{*/
const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr DergTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}
//}

/*callbackOtherUAVAppliedRef()//{*/
void DergTracker::callbackOtherUavAppliedRef(const mrs_msgs::FutureTrajectoryConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOtherUavAppliedRef");

  mrs_msgs::FutureTrajectory temp_pose= *msg;
  other_drones_applied_references[msg->uav_name] = temp_pose;

  //ROS_INFO("[MpcTracker]: Other UAV reference x: %f", other_drones_applied_references[msg->uav_name].points[0].x);
}
//}

}  // namespace derg_tracker
}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::derg_tracker::DergTracker, mrs_uav_managers::Tracker)
