#define VERSION "0.0.0.0"

/*includes//{*/
#include <ros/ros.h>

#include <mrs_uav_managers/tracker.h>

#include <geometry_msgs/PoseArray.h>

#include <mrs_msgs/FuturePoint.h>
#include <mrs_msgs/FutureTrajectory.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
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
private:
  ros::NodeHandle                                    nh_;
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

  bool starting_bool=true;

  float goto_ref_x=0;
  float goto_ref_y=0;
  float goto_ref_z=0;

   // applied reference
  float applied_ref_x=0;
  float applied_ref_y=0;
  float applied_ref_z=0;

  // ERG parameters
  MatrixXd v_dot=MatrixXd::Zero(3, 1);// derivative of the applied reference
  MatrixXd NF_total=MatrixXd::Zero(3, 1);
  float DSM_total;
  MatrixXd NF_att=MatrixXd::Zero(3, 1); // attraction Navigation field
  MatrixXd ref_dist=MatrixXd::Zero(3, 1); // difference between reference r and applied reference v
  float ref_dist_norm; // norm of res_dist
  float max_dist; // maximum between ref_dist_norm and eta
  float eta; // smoothing parameter

  float sampling_time=0.002; // sampling frequency 500 Hz

  // thrust constraints
  float DSM_s; // Dynamic Safety Margin for thrust saturation
  float kappa_s=1; // kappa parameter of the DSM_s
  float T_max= 119.340267; // maximum thrust (in Newtons)
  float T_min=0; // minimum thrust (in Newtons)

  float limit_thrust_diff; // min difference between the thrust limits and the predicted thrust
  float diff_Tmax; // difference between Tmax and T
  float diff_Tmin; // difference between T and Tmin

  // Wall all_constraints
  float kappa_w=10;
  float Nw=2; // number of walls
  float DSM_w; // Dynamic Safety Margin for wall constraints
  float arm_radius=0.2; // radius of the quadrotor
  float min_wall_distance; // minimum distance with a wall

  // walls 1 ( y=10 ) 
  MatrixXd d_w=MatrixXd::Zero(3, 1); // d_w matrix of constraints
  MatrixXd c_w=MatrixXd::Zero(3, 3); // c_w matrix of constraints

  MatrixXd NF_w = MatrixXd::Zero(3, 1); // Wall repulsion navigation field
  float sigma_w=2;
  float delta_w=0.02;
  float max_repulsion_wall1;
  
  // gain parameters
  float kpxy = 15;
  float kpz = 15;
  float kvxy = 8;
  float kvz = 8;

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
  float custom_hor=0.4; // prediction horizon (in samples)
  float sample_hor=custom_hor/custom_dt; // prediction horion ( in samples)

  geometry_msgs::PoseArray predicted_thrust_out; // array of thrust predictions
  geometry_msgs::PoseArray custom_trajectory_out;

  ros::Publisher custom_predicted_traj_publisher;
  ros::Publisher custom_predicted_thrust_publisher;
  ros::Publisher custom_predicted_velocity_publisher;

  ros::Publisher applied_ref_publisher;

  //use parameter
  bool use_derg_= true;
  bool use_wall_constraints_ = true;
  bool use_cylindrical_constraints_ = false;
  bool use_agents_avoidance_ = false;

};
//}


/*initialize()//{*/
void DergTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {
  ros::Time::waitForValid();
  is_initialized_ = true;

  ros::NodeHandle nh_(parent_nh, "derg_tracker");
  mrs_lib::ParamLoader param_loader(nh_, "DergTracker");

  param_loader.loadParam("use_derg", use_derg_);
  param_loader.loadParam("use_wall_constraints", use_wall_constraints_);
  param_loader.loadParam("use_cylindrical_constraints", use_cylindrical_constraints_);
  param_loader.loadParam("use_agents_avoidance", use_agents_avoidance_);

  custom_predicted_traj_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_traj", 1);
  custom_predicted_thrust_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_thrust", 1);
  custom_predicted_velocity_publisher = nh_.advertise<geometry_msgs::PoseArray>("custom_predicted_vel", 1);
  ROS_INFO("[DergTracker]: initialized");
   applied_ref_publisher = nh_.advertise<geometry_msgs::Pose>("applied_ref", 1);
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

/*resetStatic//{*/
bool DergTracker::resetStatic(void) {
  return true;
}
//}

/*DERG_computation()//{*/
void DergTracker::DERG_computation(){
  
  //////////////////////////////////////////////////////////////////////////////
  ///////////// Computation of the Saturation Dynamic Safety Margin ////////////
  //////////////////////////////////////////////////////////////////////////////
  
  //DSM_s=10; // for constant DSM_s testing

  limit_thrust_diff=T_max; // initialization at a high value
  for (size_t i = 0; i < sample_hor; i++) {
    diff_Tmax=T_max-predicted_thrust_out.poses[i].position.x;
    //ROS_WARN_THROTTLE(1.0, "[DergTracker] diff_Tmax: %f", diff_Tmax);
    diff_Tmin=predicted_thrust_out.poses[i].position.x-T_min;
    //ROS_WARN_THROTTLE(1.0, "[DergTracker] diff_Tmin: %f", diff_Tmin);
    if (diff_Tmax<limit_thrust_diff) {
      limit_thrust_diff=diff_Tmax;
    }
    if (diff_Tmin < limit_thrust_diff) {
      limit_thrust_diff= diff_Tmin;
    }
    //ROS_WARN_THROTTLE(1.0, "[DergTracker]: %f", limit_thrust_diff);
  }
  DSM_s=kappa_s*limit_thrust_diff;

  predicted_thrust_out.poses.clear(); // empty the array of thrust prediction once used


  ////////////////////////////////////////////////////////////////////////
  ///////////// Computation of the wall Dynamic Safety Margin ////////////
  ////////////////////////////////////////////////////////////////////////
  
  // walls 1 ( y=10 )
  if (use_wall_constraints_){
    DSM_w=0.01;

    d_w(0,0) = 0;
    d_w(1,0) = 10;

    c_w(0,0)=1;
    c_w(1,0)=0;
    c_w(2,0)=0;
    c_w(0,1)=0;
    c_w(1,1)=1;
    c_w(2,1)=0;

    min_wall_distance= d_w(0,0) -custom_trajectory_out.poses[0].position.x;
    for (size_t i = 0; i < sample_hor; i++) {
      if (d_w(0,0) -custom_trajectory_out.poses[i].position.x < min_wall_distance){
        min_wall_distance=d_w(0,0) -custom_trajectory_out.poses[i].position.x;
      }
      if (d_w(1,0) -custom_trajectory_out.poses[i].position.y < min_wall_distance){
        min_wall_distance=d_w(1,0) -custom_trajectory_out.poses[i].position.y;
      }
    }
    //DSM_w=kappa_w*min_wall_distance;
  }

  //////////////////////////////////////////////////////////
  // Computation of the attraction navigation field ///////
  /////////////////////////////////////////////////////////
 
  ref_dist(0,0)= goto_ref_x-applied_ref_x;
  ref_dist(1,0)= goto_ref_y-applied_ref_y;
  ref_dist(2,0)= goto_ref_z-applied_ref_z;
  ref_dist_norm= sqrt(pow(ref_dist(0,0), 2) + pow(ref_dist(1,0), 2)+ pow(ref_dist(2,0), 2));
  eta=0.05;
  max_dist=eta;
  if (ref_dist_norm>eta){
    max_dist=ref_dist_norm;
  }
  NF_att(0,0)=ref_dist(0,0)/max_dist;
  NF_att(1,0)=ref_dist(1,0)/max_dist;
  NF_att(2,0)=ref_dist(2,0)/max_dist;


  /////////////////////////////////////////////////////////////
  // Computation of the wall repulsion navigation field ///////
  /////////////////////////////////////////////////////////////

  if (use_wall_constraints_){
    // wall 1
    max_repulsion_wall1= (sigma_w-(d_w(1,0)-applied_ref_x))/(sigma_w-delta_w);
    if (0 > max_repulsion_wall1){
      max_repulsion_wall1=0;
    }

    NF_w(0,0)=0;
    NF_w(1,0)=-max_repulsion_wall1;
    NF_w(2,0)=0;
  }

  ////////////////////////////////////////////////////////////////////
  ///////////////////// computation of v_dot /////////////////////////
  ///////////////////////////////////////////////////////////////////

  // total navigation field
  NF_total(0,0)=NF_att(0,0) + NF_w(0,0);
  NF_total(1,0)=NF_att(1,0) + NF_w(1,0);
  NF_total(2,0)=NF_att(2,0) + NF_w(2,0);

  // total DSM
  DSM_total=DSM_s;
  if (DSM_w<DSM_total){
    DSM_total=DSM_w;
  }

  v_dot(0,0)=DSM_s*NF_total(0,0);
  v_dot(1,0)=DSM_s*NF_total(1,0);
  v_dot(2,0)=DSM_s*NF_total(2,0);

  applied_ref_x=v_dot(0,0)*sampling_time + applied_ref_x;
  applied_ref_y=v_dot(1,0)*sampling_time + applied_ref_y;
  applied_ref_z=v_dot(2,0)*sampling_time + applied_ref_z;

  geometry_msgs::Pose applied_ref_vec; // applied reference vector

  applied_ref_vec.position.x=applied_ref_x;
  applied_ref_vec.position.y=applied_ref_y;
  applied_ref_vec.position.z=applied_ref_z;

  applied_ref_publisher.publish(applied_ref_vec);

  custom_trajectory_out.poses.clear();
}
//}

/*trajectory_prediction()//{*/
void DergTracker::trajectory_prediction(){

  // compute the C2 parameter for each coordinate
  C2_x= (-(init_vel(0,0)/lambda1_xy)-applied_ref_x+init_pos(0,0))/(1-(lambda2_xy/lambda1_xy));
  C2_y= (-(init_vel(1,0)/lambda1_xy)-applied_ref_y+init_pos(1,0))/(1-(lambda2_xy/lambda1_xy));
  C2_z= (-(init_vel(2,0)/lambda1_z)-applied_ref_z+init_pos(2,0))/(1-(lambda2_z/lambda1_z));

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

/*update()//{*/
const mrs_msgs::PositionCommand::ConstPtr DergTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
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


  init_pos(0,0)=uav_state->pose.position.x;
  init_pos(1,0)=uav_state->pose.position.y;
  init_pos(2,0)=uav_state->pose.position.z;

  init_vel(0,0)=uav_state->velocity.linear.x;
  init_vel(1,0)=uav_state->velocity.linear.y;
  init_vel(2,0)=uav_state->velocity.linear.z;

  trajectory_prediction();

  position_cmd.header.stamp    = ros::Time::now();
  position_cmd.header.frame_id = uav_state->header.frame_id;
    if (starting_bool) {

    goto_ref_x= uav_state->pose.position.x;
    goto_ref_y= uav_state->pose.position.y;
    goto_ref_z= uav_state->pose.position.z;

    applied_ref_x=goto_ref_x;
    applied_ref_y=goto_ref_y;
    applied_ref_z=goto_ref_z;

    position_cmd.position.x     = goto_ref_x;
    position_cmd.position.y     = goto_ref_y;
    position_cmd.position.z     = goto_ref_z;

    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_velocity_vertical   = 1;
    position_cmd.use_velocity_horizontal = 1;
    position_cmd.use_acceleration        = 0;
    position_cmd.use_jerk                = 0;
    position_cmd.heading                 = 1;
    position_cmd.heading_rate            = 1;

    starting_bool=false;

    ROS_INFO(
        "[Derg tracker - odom]: [goto_ref_x=%.2f],[goto_ref_y=%.2f],[goto_ref_z=%.2f]",goto_ref_x,goto_ref_y,goto_ref_z);

  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}

  std::scoped_lock lock(mutex_uav_state_);
  
  if (use_derg_){
    DERG_computation(); // modifies the applied reference
  }
  else{
  // in case the DERG isn't used
  applied_ref_x=goto_ref_x;
  applied_ref_y=goto_ref_y;
  applied_ref_z=goto_ref_z;
  }

 // set the desired states from the input of the goto function

  position_cmd.position.x     = applied_ref_x;
  position_cmd.position.y     = applied_ref_y;
  position_cmd.position.z     = applied_ref_z;

  position_cmd.use_position_vertical   = 1;
  position_cmd.use_position_horizontal = 1;
  position_cmd.use_velocity_vertical   = 1;
  position_cmd.use_velocity_horizontal = 1;
  position_cmd.use_acceleration        = 0;
  position_cmd.use_jerk                = 0;
  position_cmd.heading                 = 1;
  position_cmd.heading_rate            = 1;

  // set the header
  position_cmd.header.stamp    = uav_state->header.stamp;
  position_cmd.header.frame_id = uav_state->header.frame_id;

  // u have to return a position command
  // can set the jerk to 0
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}
//}

/*getStatus()//{*/
const mrs_msgs::TrackerStatus DergTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;
  tracker_status.active = is_active_;
  return tracker_status;
}
//}

/*enableCallBacks()//{*/
const std_srvs::SetBoolResponse::ConstPtr DergTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  res.message = "Callbacks not implemented";
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}
//}

/*switchOdometruSource()//{*/
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

}  // namespace derg_tracker
}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::derg_tracker::DergTracker, mrs_uav_managers::Tracker)
