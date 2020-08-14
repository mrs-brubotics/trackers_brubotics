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

#include <geometry_msgs/PoseArray.h>

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

  void trajectory_prediction(void);
  void DERG_computation();
  void callbackOtherUavAppliedRef(const mrs_msgs::FutureTrajectoryConstPtr& msg);

private:
  mrs_msgs::UavState uav_state_;
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
  
  // gain parameters

  float kpxy = 15;
  float kpz= 15;
  float kvxy=8;
  float kvz=8;

  // initial conditions
  MatrixXd init_pos = MatrixXd::Zero(3, 1);
  MatrixXd init_vel = MatrixXd::Zero(3, 1);


  // prediction Parameters
  float g=9.8066;
  float mass=2.0;

  float C1_x;
  float C2_x;
  float C1_y;
  float C2_y;
  float C1_z;
  float C2_z;

  float delta_xy=kvxy*kvxy-4*kpxy;
  float delta_z=kvz*kvz-4*kpz;

  float lambda1_xy=(-kvxy-sqrt(delta_xy))/2;
  float lambda2_xy=(-kvxy+sqrt(delta_xy))/2;
  float lambda1_z=(-kvz-sqrt(delta_z))/2;
  float lambda2_z=(-kvz+sqrt(delta_z))/2;


  float custom_dt=0.002; // sampling time (in seconds)
  float custom_hor=0.4; // prediction horizon (in seconds)
  float sample_hor=custom_hor/custom_dt; // prediction horion ( in samples)

  geometry_msgs::PoseArray predicted_thrust_out; // array of thrust predictions
  geometry_msgs::PoseArray custom_trajectory_out;

  ros::Publisher custom_predicted_traj_publisher;
  ros::Publisher custom_predicted_thrust_publisher;
  ros::Publisher custom_predicted_velocity_publisher;
  
  // applied reference
  float applied_ref_x=0;
  float applied_ref_y=0;
  float applied_ref_z=0;
 
  // ERG parameters
  MatrixXd v_dot=MatrixXd::Zero(4, 1);// derivative of the applied reference
  MatrixXd NF=MatrixXd::Zero(4, 1); // Navigation field
  MatrixXd ref_dist=MatrixXd::Zero(4, 1); // difference between reference r and applied reference v
  float ref_dist_norm; // norm of res_dist
  float max_dist; // maximum between ref_dist_norm and eta
  float eta; // smoothing parameter
  float DSM; // Dynamic Safety Margin
  float sampling_time=0.002; // sampling frequency 500 Hz
  
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
  mrs_msgs::FutureTrajectory uav_applied_ref_out;
  mrs_msgs::FuturePoint custom_new_point;

  std::map<std::string, mrs_msgs::FutureTrajectory> other_drones_applied_references;
  std::vector<ros::Subscriber> other_uav_applied_ref_subscriber;

  ros::Publisher uav_applied_ref_message_publisher;


};
//}

/*initialize()//{*/
void DergTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  uav_name_ = uav_name;
  
  ros::NodeHandle nh_(parent_nh, "derg_tracker");
  mrs_lib::ParamLoader param_loader(nh_, "DergTracker");

  ros::Time::waitForValid();
  
  custom_predicted_traj_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_traj", 1);
  custom_predicted_thrust_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_thrust", 1);
  custom_predicted_velocity_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_vel", 1); 
  
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

  {
    uav_state_ = *uav_state;
    // up to this part the update() method is evaluated even when the tracker is not active
    if (!is_active_) {
      return mrs_msgs::PositionCommand::Ptr();
    }
  }
  mrs_msgs::PositionCommand position_cmd;
  
  init_pos(0,0)=uav_state->pose.position.x;
  init_pos(1,0)=uav_state->pose.position.y;
  init_pos(2,0)=uav_state->pose.position.z;

  init_vel(0,0)=uav_state->velocity.linear.x;
  init_vel(1,0)=uav_state->velocity.linear.y;
  init_vel(2,0)=uav_state->velocity.linear.z;

  trajectory_prediction();
  
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
    
    applied_ref_x=goto_ref_x;
    applied_ref_y=goto_ref_y;
    applied_ref_z=goto_ref_z;
    
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
        "[Derg tracker - odom]: [goto_ref_x=%.2f],[goto_ref_y=%.2f],[goto_ref_z=%.2f, goto_ref_heading=%.2f]",goto_ref_x,goto_ref_y,goto_ref_z,goto_ref_heading);
 
    return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
  }
  
  DERG_computation(); // modifies the applied reference
  /*
  // in case the DERG isn't used
  applied_ref_x=goto_ref_x;
  applied_ref_y=goto_ref_y;
  applied_ref_z=goto_ref_z;
  */
 
  // set the desired states from the input of the goto function

  position_cmd.position.x     = applied_ref_x;
  position_cmd.position.y     = applied_ref_y;
  position_cmd.position.z     = applied_ref_z;
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

/*trajectory_prediction()//{*/
void DergTracker::trajectory_prediction(){

  // compute the C2 parameter for each coordinate
  C2_x= (-(init_vel(0,0)/lambda1_xy)-goto_ref_x+init_pos(0,0))/(1-(lambda2_xy/lambda1_xy));
  C2_y= (-(init_vel(1,0)/lambda1_xy)-goto_ref_y+init_pos(1,0))/(1-(lambda2_xy/lambda1_xy));
  C2_z= (-(init_vel(2,0)/lambda1_z)-goto_ref_z+init_pos(2,0))/(1-(lambda2_z/lambda1_z));

  C1_x = (-lambda2_xy*C2_x+init_vel(0,0))/lambda1_xy;
  C1_y = (-lambda2_xy*C2_y+init_vel(1,0))/lambda1_xy;
  C1_z = (-lambda2_z*C2_z+init_vel(2,0))/lambda1_z;


  custom_trajectory_out.header.stamp    = ros::Time::now();
  custom_trajectory_out.header.frame_id = uav_state_.header.frame_id;


  predicted_thrust_out.header.stamp    = ros::Time::now();
  predicted_thrust_out.header.frame_id = uav_state_.header.frame_id;

  /*
  geometry_msgs::PoseArray custom_vel_out;
  custom_vel_out.header.stamp    = ros::Time::now();
  custom_vel_out.header.frame_id = uav_state_.header.frame_id;
  */
  float t;

  {
    for (size_t i = 0; i < sample_hor; i++) {
      t=i*custom_dt;
      geometry_msgs::Pose custom_pose;
      geometry_msgs::Pose custom_vel;
      geometry_msgs::Pose predicted_thrust; // not physically correct of course. We just had problems publishing other types of arrays that weren't custom
      geometry_msgs::Pose predicted_thrust_norm;  // predicted thrust norm

      custom_pose.position.x = C1_x*exp(lambda1_xy*t)+C2_x*exp(lambda2_xy*t)+goto_ref_x;
      custom_pose.position.y = C1_y*exp(lambda1_xy*t)+C2_y*exp(lambda2_xy*t)+goto_ref_y;
      custom_pose.position.z = C1_z*exp(lambda1_z*t)+C2_z*exp(lambda2_z*t)+goto_ref_z;
      //ROS_INFO("[DergTracker- prediction]: %.2f", custom_pose.position.x);
      /*
      custom_vel.position.x = lambda1_xy*C1_x*exp(lambda1_xy*t)+lambda2_xy*C2_x*exp(lambda2_xy*t);
      custom_vel.position.y = lambda1_xy*C1_y*exp(lambda1_xy*t)+lambda2_xy*C2_y*exp(lambda2_xy*t);
      custom_vel.position.z = lambda1_z*C1_z*exp(lambda1_z*t)+lambda2_z*C2_z*exp(lambda2_z*t);
      */

      predicted_thrust.position.x = mass*(lambda1_xy*lambda1_xy*C1_x*exp(lambda1_xy*t)+lambda2_xy*lambda2_xy*C2_x*exp(lambda2_xy*t)); //mass*(kpxy*(applied_ref_x-custom_pose.position.x)-kvxy*custom_vel.position.x);//  // thrust x
      predicted_thrust.position.y = mass*(lambda1_xy*lambda1_xy*C1_y*exp(lambda1_xy*t)+lambda2_xy*lambda2_xy*C2_y*exp(lambda2_xy*t));//mass*(kpxy*(applied_ref_y-custom_pose.position.y)-kvxy*custom_vel.position.y);  // thrust y
      predicted_thrust.position.z = mass*(lambda1_z*lambda1_z*C1_z*exp(lambda1_z*t)+lambda2_z*lambda2_z*C2_z*exp(lambda2_z*t)+g);//mass*(kpz*(applied_ref_z-custom_pose.position.z)-kvz*custom_vel.position.z) + mass*g;  // thrust z

      predicted_thrust_norm.position.x= sqrt(predicted_thrust.position.x*predicted_thrust.position.x+predicted_thrust.position.y*predicted_thrust.position.y+predicted_thrust.position.z*predicted_thrust.position.z);

      custom_trajectory_out.poses.push_back(custom_pose);
      //ROS_INFO("[DergTracker]: custom pose x =  [%.2f]",custom_pose.position.x);
      //custom_vel_out.poses.push_back(custom_vel);
      predicted_thrust_out.poses.push_back(predicted_thrust_norm);
    }
  }

  try {
    custom_predicted_traj_publisher.publish(custom_trajectory_out);
    custom_predicted_thrust_publisher.publish(predicted_thrust_out);
    //custom_predicted_velocity_publisher.publish(custom_vel_out);
  }
  catch (...) {
    ROS_ERROR("[DergTracker]: Exception caught during publishing topic %s.", custom_predicted_traj_publisher.getTopic().c_str());
  }

  custom_trajectory_out.poses.clear();
  predicted_thrust_out.poses.clear();

}
//}

/*DERG_computation()//{*/
  void DergTracker::DERG_computation(){

  DSM=10;

  // computation of the Dynamic Safety Margin

  //kappa_s=1;
  //T_max= 119.340267; // in Newtons
  //T_min=0;

  // computation of the navigation field
  ref_dist(0,0)= goto_ref_x-applied_ref_x;
  ref_dist(1,0)= goto_ref_y-applied_ref_y;
  ref_dist(2,0)= goto_ref_z-applied_ref_z;
  ref_dist_norm= sqrt(pow(ref_dist(0,0), 2) + pow(ref_dist(1,0), 2)+ pow(ref_dist(2,0), 2));
  eta=0.05;
  max_dist=eta;
  if (ref_dist_norm>eta){
    max_dist=ref_dist_norm;
  }
  NF(0,0)=ref_dist(0,0)/max_dist;
  NF(1,0)=ref_dist(1,0)/max_dist;
  NF(2,0)=ref_dist(2,0)/max_dist;

  v_dot(0,0)=DSM*NF(0,0);
  v_dot(1,0)=DSM*NF(1,0);
  v_dot(2,0)=DSM*NF(2,0);

  applied_ref_x=v_dot(0,0)*sampling_time + applied_ref_x;
  applied_ref_y=v_dot(1,0)*sampling_time + applied_ref_y;
  applied_ref_z=v_dot(2,0)*sampling_time + applied_ref_z;

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
