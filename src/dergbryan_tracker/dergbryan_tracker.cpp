#define VERSION "0.0.5.0"

/*includes//{*/
#include <ros/ros.h>
#include <mrs_uav_managers/tracker.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
//#include <stack>
// #include <iostream>
/*begin includes added by bryan:*/
#include <mrs_lib/attitude_converter.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/FuturePoint.h>
#include <ros/console.h>
#include <trackers_brubotics/DSM.h> // custom ROS message
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <trackers_brubotics/FutureTrajectoryTube.h> // custom ROS message
#include <trackers_brubotics/DistanceBetweenUavs.h> // custom ROS message
#include <trackers_brubotics/TrajectoryTracking.h> // custom ROS message
#include <geometry_msgs/Point.h>
#include <chrono> // computational time calculations
#include <ctime>  // computational time calculations
#include <bits/stdc++.h> // computational time calculations: last method of https://www.geeksforgeeks.org/measure-execution-time-with-high-precision-in-c-c/
#include <trackers_brubotics/ComputationalTime.h> // custom ROS message
/*end includes added by bryan:*/

/*begin includes added by Titouan and Jonathan:*/
#include <std_msgs/Int32.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
/*end included added by Titouan and Jonathan*/

// | ----------------- LOAD---------------- |
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>   // for the position
#include <geometry_msgs/Twist.h> //for the velocity
#include <math.h>  
#include <mrs_msgs/BacaProtocol.h>
#include <std_msgs/UInt8.h>
#include <stdlib.h> 
// | ----------------2UAV LOAD model----------------- |
#include <Eigen/Geometry>



// #include <mrs_lib/utils.h>
// #include <mrs_lib/geometry/cyclic.h>
// #include <mrs_lib/geometry/misc.h>

//}
#define OUTPUT_ATTITUDE_RATE 0
#define OUTPUT_ATTITUDE_QUATERNION 1


using namespace Eigen; 
using vec3_t = mrs_lib::geometry::vec_t<3>;
using sradians = mrs_lib::geometry::sradians;



namespace mrs_uav_trackers
{

namespace dergbryan_tracker
{

/*class DergbryanTracker//{*/
class DergbryanTracker : public mrs_uav_managers::Tracker {
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
  const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr setVelocityReference(const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);
  // other functions:
  // TODO: document these function I/O
  void trajectory_prediction_general(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void trajectory_prediction_general_load_2UAV(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void computeERG();
  // Eigen::Matrix<double, 2, 2>
  Eigen::Vector3d calcCirculationField(std::string type, double dist_x, double dist_y, double dist_z, double dist);
  double getLambda(Eigen::Vector3d &point_link_0, Eigen::Vector3d &point_link_1, Eigen::Vector3d &point_sphere);
  std::tuple< Eigen::Vector3d, Eigen::Vector3d> getMinDistDirLineSegments(Eigen::Vector3d &point0_link0, Eigen::Vector3d &point1_link0, Eigen::Vector3d &point0_link1, Eigen::Vector3d &point1_link1);
  std::tuple< double, Eigen::Vector3d, double>  ComputeSe3CopyController(const mrs_msgs::UavState uavi_state, Eigen::Matrix3d uavi_R, Eigen::Vector3d Payloadposition_vector, Eigen::Vector3d Payloadvelocity_vector,mrs_msgs::PositionCommand uavi_position_cmd ,Eigen::Vector3d uavi_Rp,Eigen::Vector3d uavi_Rv,Eigen::Vector3d uavi_Rpl,Eigen::Vector3d uavi_Ra);
  void publishFollowerDataForLeaderIn2uavs_payload(mrs_msgs::PositionCommand position_cmd);
  void computePSCTrajectoryPredictions(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd); //mother function that calls the correct prediction function depending on the cases.
  double computeDSM_sT_trajpred(geometry_msgs::PoseArray predicted_thrust);
  double computeDSM_sw_trajpred(geometry_msgs::PoseArray predicted_attrate);
  double computeDSM_swing_trajpred(geometry_msgs::PoseArray predicted_swing_angle);
  double computeDSM_Tc_trajpred(geometry_msgs::PoseArray predicted_cable_tension);
  void clearMsgsAfterUpdate();
private:
  ros::NodeHandle                                     nh_;
  ros::NodeHandle                                     nh2_;
  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;
  std::string _uav_name_;
 
  // | ------------------- declaring env (session/bashrc) parameters ------------------- |
  std::string _run_type_;       // set to "simulation" (for Gazebo simulation) OR "uav" (for hardware testing) defined in bashrc or session.yaml. Used for payload transport as payload position comes from two different callbacks depending on how the test is ran (in sim or on real UAV).
  std::string _type_of_system_; // defines the dynamic system model to simulate in the prediction using the related controller: can be 1uav_no_payload, 1uav_payload or 2uavs_payload. Set in session.yaml file.
  double _cable_length_;        // length of the cable between payload COM / anchoring point and COM of the UAV
  double _load_mass_;           // feedforward load mass per uav  defined in the session.yaml of every test file (session variable also used by the xacro for Gazebo simulation)
  double _load_length_;         // length of bar load transported by 2uavs
  // | ------------------- declaring .yaml parameters ------------------- |
  // Se3CopyController:
  std::string _version_;
  bool   _profiler_enabled_ = false;
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kplxy_;      // load position xy gain
  double kvlxy_;      // load velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kaz_;        // acceleration z gain (feed forward, =1)
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain
  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;
  double _thrust_saturation_; // total thrust limit in [0,1]
  int    output_mode_;  // attitude_rate / quaternion
  // DergBryanTracker:
  // TODO: bryan clean ERG paramters and code
  bool _use_derg_;
  std::vector<std::string> _avoidance_other_uav_names_;
  double _pred_horizon_; // prediction horizon (in seconds). Set in config file of the Tracker, in the testing folder.
  double _prediction_dt_; // controller sampling time (in seconds) used in prediction. Set in config file of the Tracker, in the testing folder.
  bool _predicitons_use_body_inertia_;
  int _DSM_type_; // 1: level set based, 2: trajectory based
  int _Lyapunov_type_; // 1: traditional, 2: optimally aligned
  double _epsilon_;
  double _kappa_sT_; // kappa parameter of the DSM_s
  double _kappa_sw_;
  double _kappa_a_;
  double _kappa_w_;  // kappa parameter of the DSM_w
  double _kappa_o_;  // kappa parameter of the DSM_o
  double _kappa_swing_c_; //kappa parameter of the DSM_swing_c_;
  double _kappa_Tc_; //kappa parameter of the DSM_swing_c_;

  double _constant_dsm_;
  bool _enable_dsm_sT_;
  bool _enable_dsm_sw_;
  bool _enable_dsm_a_;
  bool _enable_dsm_w_;
  bool _enable_dsm_o_;
  bool _enable_dsm_swing_c_;
  bool _enable_dsm_Tc_;
  double _T_min_; // lower saturation limit of total thrust
  double _extra_thrust_saturation_ratio_;
  double _constraint_swing_c_; // rad, swing angle constaint payload

  int _DERG_strategy_id_;
  bool _use_distance_xy_;
  // TODO bryan: clean comments below
  // int DERG_strategy_id_ = 3; //0 / 1 / 2 / 3 /4
  // COMPARE AS
  // original strategy: id = 0
  // double Sa = 1.5; 
  // // tube strategy: id = 1
  // double Sa_perp = 0.20; //0.10
  // double Sa_long = Sa-Sa_perp;// see drawing//1.0;//1.5;//2.5;
  // double kappa_a = 20;//10;//50;// 100 for id = 1 //10; for id = 0 // decrease if problems persist
  // OR with kappa scaling
  //double kappa_a = 20*Sa/Sa_perp;
  // OR
  // COMPARE AS
  // original strategy: id = 0
  // double Sa = 0.20; 
  // // tube strategy: id = 1
  // double Sa_perp = Sa; //0.10
  // double Sa_long = 1.3;// see drawing//1.0;//1.5;//2.5;
  // double kappa_a = 20;
  // strategy 2 / 3:
  //double kappa_a = 0.3*20;
  // double kappa_a = 0.3*20*Sa/Sa_perp; //with kappa scaling
  // strategy 4:
  // double Sa = 1.5;
  // double Sa_perp = 0.20; //0.10
  // double Sa_long = Sa-Sa_perp;// see drawing//1.0;//1.5;//2.5;
  // double kappa_a = 0.10*20*Sa/Sa_perp;
  double _Sa_max_;
  double _Sa_perp_max_;
  double _Sa_long_max_;
  double _eta_; // smoothing factor attraction field
  bool _enable_repulsion_a_;
  bool _use_tube_;
  double _zeta_a_;
  double _delta_a_;
  double _alpha_a_;
  std::string _circ_type_; // Default circulation for agent
  bool _enable_repulsion_w_;
  double _zeta_w_;
  double _delta_w_;
  double wall_p_x; // Wall position (considered infinitly long) //Frank TODO: 
  bool _enable_repulsion_o_;
  double _zeta_o_;
  double _delta_o_;
  double _alpha_o_;
  std::string _circ_type_o; //circulation for static obstacles
  Eigen::Vector3d obs_position = Eigen::Vector3d::Zero(3);
  double R_o_; 
  bool _enable_visualization_;
  bool _enable_diagnostics_pub_;
  bool _enable_trajectory_pub_;

  // ---------------
  // ROS Publishers:
  // ---------------
  //|-----------------------------EXAMPLE--------------------------------|//
  ros::Publisher chatter_publisher_; // just an example to follow
  //|-----------------------------UAV--------------------------------|//
  ros::Publisher goal_pose_publisher_;  // reference pose input to tracker (i.e., desired user command or from tmux window or loaded trajectory)
  ros::Publisher applied_ref_pose_publisher_; // reference output of tracker (ensured to be safe by D-ERG if it has been enabled)
  ros::Publisher uav_state_publisher_; // actual uav_state used in tracker
  // TODO add also pos here logged with same header stamp
  //  - ERG trajectory predictions: 
  ros::Publisher predicted_pose_publisher_;         // predicted UAV pose
  ros::Publisher predicted_vel_publisher_;          // predicted UAV velocity
  ros::Publisher predicted_acc_publisher_;          // predicted UAV acceleration
  ros::Publisher predicted_thrust_publisher_;       // predicted UAV total thrust
  ros::Publisher predicted_attrate_publisher_;      // predicted UAV attitude rate
  ros::Publisher predicted_des_attrate_publisher_;  // predicted UAV desired attitude rate
  ros::Publisher predicted_tiltangle_publisher_;    // predicted UAV tilt angle
  //|-----------------------------LOAD--------------------------------|//
  //  - ERG trajectory predictions: 
  ros::Publisher predicted_load_pose_publisher_;             // predicted LOAD pose
  ros::Publisher predicted_load_vel_publisher_;              // predicted LOAD velocity
  ros::Publisher predicted_load_acc_publisher_;              // predicted LOAD acceleration
  ros::Publisher predicted_phi_publisher_;                   // predicted LOAD absolute cable-world phi angle (not encoder angle)
  ros::Publisher predicted_theta_publisher_;                 // predicted LOAD absolute cable-world theta angle (not encoder angle)
  ros::Publisher predicted_phi_dot_publisher_;               // predicted LOAD absolute cable-world phi angular velocity (not encoder velocity)
  ros::Publisher predicted_theta_dot_publisher_;             // predicted LOAD absolute cable-world theta angular velocity (not encoder velocity)
  ros::Publisher predicted_phi_dot_dot_publisher_;           // predicted LOAD absolute cable-world phi angular acceleration (not encoder acceleration)
  ros::Publisher predicted_theta_dot_dot_publisher_;         // predicted LOAD absolute cable-world theta angular acceleration (not encoder acceleration)
  ros::Publisher predicted_load_position_errors_publisher_;  // predicted LOAD position errors
  ros::Publisher predicted_swing_angle_publisher_;           // predicted LOAD swing angles relative to body frame (between cable -mu and -z_B )
  ros::Publisher predicted_Tc_publisher_;                    // predicted LOAD tension in the cable, alongside mu
  //|------------------2UAVsLOAD------------------------|//
  // For communication between uav1 (leader) and uav2 (follower):
  ros::Publisher uav_state_follower_for_leader_pub_;
  ros::Publisher anchoring_point_follower_for_leader_pub_;
  ros::Publisher position_cmd_follower_for_leader_pub_;
  ros::Publisher goal_position_cmd_follower_for_leader_pub_;
  ros::Publisher position_cmd_follower_from_leader_pub_;
  ros::Publisher goal_position_cmd_follower_from_leader_pub_;
  //  - ERG trajectory predictions: 
  ros::Publisher predicted_uav1_poses_publisher_;
  ros::Publisher predicted_uav2_poses_publisher_;
  ros::Publisher predicted_uav1_vel_publisher_;
  ros::Publisher predicted_uav2_vel_publisher_;
  ros::Publisher predicted_uav1_anchoring_point_pose_publisher_;
  ros::Publisher predicted_uav2_anchoring_point_pose_publisher_;
  ros::Publisher predicted_uav1_anchoring_point_vel_publisher_;
  ros::Publisher predicted_uav2_anchoring_point_vel_publisher_;
  ros::Publisher predicted_uav1_thrust_publisher_;
  ros::Publisher predicted_uav2_thrust_publisher_; 
  ros::Publisher predicted_uav1_attitude_rate_publisher_;   
  ros::Publisher predicted_uav2_attitude_rate_publisher_;  
  ros::Publisher predicted_uav1_swing_angle_publisher_; 
  ros::Publisher predicted_uav2_swing_angle_publisher_; 
  ros::Publisher predicted_uav1_tension_force_publisher_; 
  ros::Publisher predicted_uav2_tension_force_publisher_; 

  //|-----------------------------D-ERG--------------------------------|//
  // TODO: bryan clean when improving ERG
  //    - Multi-uav collision avoidance:
  ros::Publisher DSM_publisher_;
  ros::Publisher DSM_uav1_publisher_;
  ros::Publisher DSM_uav2_publisher_;
  ros::Publisher DistanceBetweenUavs_publisher_;
  ros::Publisher avoidance_trajectory_publisher_;   
  ros::Publisher avoidance_applied_ref_publisher_;   
  ros::Publisher avoidance_pos_publisher_;    
  ros::Publisher tube_min_radius_publisher_; // intermdiate step as example
  ros::Publisher future_tube_publisher_; // inspired from FutureTrajectory.msg of ctu mrs
  //  - Data analysis:
  ros::Publisher TrajectoryTracking_publisher_; // for trajectory diagnostics
  ros::Publisher ComputationalTime_publisher_; // ComputationalTime
  //  - RVIZ only:
  ros::Publisher derg_strategy_id_publisher_;
  ros::Publisher point_link_star_publisher_;
  ros::Publisher sa_max_publisher_;
  ros::Publisher sa_perp_max_publisher_;

  // -------------
  // ROS Messages and related global:
  // -------------
  //|-----------------------------UAV--------------------------------|//
  mrs_msgs::UavState uav_state_;       // array of UAV state
  mrs_msgs::UavState uav_state_prev_;  // array of UAV state at previous samle time
  //  - ERG trajectory predictions: 
  geometry_msgs::PoseArray predicted_poses_out_;             // array of predicted UAV poses
  geometry_msgs::PoseArray predicted_velocities_out_;        // array of predicted UAV velocities
  geometry_msgs::PoseArray predicted_accelerations_out_;     // array of predicted UAV accelerations
  geometry_msgs::PoseArray predicted_thrust_out_;            // array of predicted UAV total thrusts
  geometry_msgs::PoseArray predicted_attituderate_out_;      // array of predicted UAV attitude rates
  geometry_msgs::PoseArray predicted_des_attituderate_out_;  // array of predicted UAV desired attitude rates
  geometry_msgs::PoseArray predicted_tiltangle_out_;         // array of predicted UAV tilt angles
  //|-----------------------------LOAD------------------------|//
  
  //geometry_msgs::Vector3 load_pose_error; // TODO: needed?
  //geometry_msgs::Vector3 load_velocity_error; // TODO: needed?
  //  - ERG trajectory predictions: 
  geometry_msgs::PoseArray predicted_load_poses_out_;           // array of predicted LOAD poses
  geometry_msgs::PoseArray predicted_load_velocities_out_;      // array of predicted LOAD velocities
  geometry_msgs::PoseArray predicted_load_accelerations_out_;   // array of predicted LOAD accelerations
  geometry_msgs::PoseArray predicted_phi_out_;                  // array of predicted LOAD absolute cable-world phi angles (not encoder angles)
  geometry_msgs::PoseArray predicted_theta_out_;                // array of predicted LOAD absolute cable-world theta angles (not encoder angles)
  geometry_msgs::PoseArray predicted_phi_dot_out_;              // array of predicted LOAD absolute cable-world phi angular velocities (not encoder velocities)
  geometry_msgs::PoseArray predicted_theta_dot_out_;            // array of predicted LOAD absolute cable-world theta angular velocities (not encoder velocities)
  geometry_msgs::PoseArray predicted_phi_dot_dot_out_;          // array of predicted LOAD absolute cable-world phi angular accelerations (not encoder accelerations)
  geometry_msgs::PoseArray predicted_theta_dot_dot_out_;        // array of predicted LOAD absolute cable-world theta angular accelerations (not encoder accelerations)
  geometry_msgs::PoseArray predicted_load_position_errors_out_; // array of predicted LOAD position errors
  geometry_msgs::PoseArray predicted_swing_angle_out_;          // array of predicted LOAD swing angles relative to body frame (between cable -mu and -z_B )
  geometry_msgs::PoseArray predicted_Tc_out_;                   // array of predicted LOAD tension of the cable; alongside mu. If positive => tension, if negative => compression (to be avoided with ERG)

  //|-----------------------------2UAVsLOAD----------------------|//
  // transfer information of UAV2 to UAV1
  mrs_msgs::UavState anchoring_point_follower_for_leader_msg_;
  //Store and publish the predictions (over whole horizon).
  geometry_msgs::PoseArray predicted_uav1_poses_out_;
  geometry_msgs::PoseArray predicted_uav1_vel_out_;
  geometry_msgs::PoseArray predicted_uav1_acc_out_;
  geometry_msgs::PoseArray predicted_uav2_poses_out_;
  geometry_msgs::PoseArray predicted_uav2_vel_out_;
  geometry_msgs::PoseArray predicted_uav2_acc_out_;
  geometry_msgs::PoseArray predicted_uav1_anchoring_point_pose_out_; 
  geometry_msgs::PoseArray predicted_uav1_anchoring_point_vel_out_;
  geometry_msgs::PoseArray predicted_uav1_anchoring_point_acc_out_;
  geometry_msgs::PoseArray predicted_uav2_anchoring_point_pose_out_;
  geometry_msgs::PoseArray predicted_uav2_anchoring_point_vel_out_;
  geometry_msgs::PoseArray predicted_uav2_anchoring_point_acc_out_;
  geometry_msgs::PoseArray predicted_uav1_thrust_out_;
  geometry_msgs::PoseArray predicted_uav2_thrust_out_;
  geometry_msgs::PoseArray predicted_uav1_attitude_rate_out_;
  geometry_msgs::PoseArray predicted_uav2_attitude_rate_out_;
  geometry_msgs::PoseArray predicted_payload_position_out_;
  geometry_msgs::PoseArray predicted_payload_vel_out_;
  geometry_msgs::PoseArray predicted_payload_acc_out_;
  geometry_msgs::PoseArray predicted_nl_out_;
  geometry_msgs::PoseArray predicted_wl_out_;
  geometry_msgs::PoseArray predicted_dotnl_out_;
  geometry_msgs::PoseArray predicted_dotwl_out_;
  geometry_msgs::PoseArray predicted_uav1_swing_angle_out_;
  geometry_msgs::PoseArray predicted_uav2_swing_angle_out_;
  geometry_msgs::PoseArray predicted_uav1_tension_force_out_;
  geometry_msgs::PoseArray predicted_uav2_tension_force_out_;
  //|-----------------------------D-ERG--------------------------------|//
  //    - Multi-uav collision avoidance:
  mrs_msgs::FutureTrajectory future_trajectory_out_;
  mrs_msgs::FutureTrajectory uav_applied_ref_out_;
  mrs_msgs::FutureTrajectory uav_position_out_;

  // --------------------------------------------------------------
  // ROS subscribers, their callbacks and updated global variables:
  // --------------------------------------------------------------
  //|-----------------------------EXAMPLE--------------------------------|//
  std::vector<mrs_lib::SubscribeHandler<std_msgs::String>> other_uav_subscribers2_;
  void chatterCallback(mrs_lib::SubscribeHandler<std_msgs::String>& sh_ptr);
  //|------------------------LOAD-----------------------------|//
  // TODO: the subscriber names below (load_state_sub_ & data_payload_sub_) are not chosen well. Why not using a common name and add simulation and uav?
  ros::Subscriber load_state_sub_;
  void GazeboLoadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg); // TODO: document
  bool payload_spawned_ = false;  // TODO: document
  bool callback_data_follower_no_delay_ = false; // true if all the data that is published by the follower and subscribed on by leader is not too much delayed
  bool callback_data_leader_no_delay_ = false;   // true if all the data that is published by the leader and subscribed on by follower is not too much delayed
  double _max_time_delay_on_callback_data_follower_; //= 0.100;// = 10*dt_;
  double _max_time_delay_on_callback_data_leader_;// = 0.200;// = 20*dt_;
  Eigen::Vector3d anchoring_pt_pose_position_ = Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  geometry_msgs::Twist anchoring_pt_velocity_;               // anchoring point/payload velocity 
  Eigen::Vector3d anchoring_pt_lin_vel_ = Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  ros::Subscriber data_payload_sub_;
  void BacaLoadStatesCallback(const mrs_msgs::BacaProtocolConstPtr& msg);            // TODO: document
  // TODO: unclear names used for these global variables, change names and add documentation
  float encoder_angle_1_;
  float encoder_angle_2_;
  float encoder_velocity_1_;
  float encoder_velocity_2_;
  //|------------------2UAVsLOAD------------------------|//
  // TODO: check after 1 UAV works
  //for receiving the UAV2 inforlations, when 2UAV+BEAM payload model is simulated.
  ros::Subscriber uav_state_follower_for_leader_sub_;
  void uav_state_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg);
  mrs_msgs::UavState uav_state_follower_for_leader_;
  Eigen::Vector3d uav_position_follower_=Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  Eigen::Vector3d uav_velocity_follower_=Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  
  ros::Subscriber position_cmd_follower_for_leader_sub_; 
  void position_cmd_follower_for_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg);
  mrs_msgs::PositionCommand position_cmd_follower_for_leader_;

  ros::Subscriber goal_position_cmd_follower_for_leader_sub_; 
  void goal_position_cmd_follower_for_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg);
  mrs_msgs::PositionCommand goal_position_cmd_follower_for_leader_;
  
  ros::Subscriber anchoring_point_follower_for_leader_sub_;
  void anchoring_point_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg);
  mrs_msgs::UavState anchoring_point_follower_for_leader_;
  Eigen::Vector3d anchoring_point_follower_position_=Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  Eigen::Vector3d anchoring_point_follower_velocity_=Eigen::Vector3d::Zero(3); // TODO why must it be inititliazed to zero here?
  
  ros::Subscriber position_cmd_follower_from_leader_sub_;
  void position_cmd_follower_from_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg);
  mrs_msgs::PositionCommand position_cmd_follower_from_leader_;

  ros::Subscriber goal_position_cmd_follower_from_leader_sub_;
  void goal_position_cmd_follower_from_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg);
  mrs_msgs::PositionCommand goal_position_cmd_follower_from_leader_;



  //|-----------------------------D-ERG--------------------------------|//
  //    - Multi-uav collision avoidance:
  // TODO: also use these other_uav_subscribers for cooperative load transport
  std::vector<ros::Subscriber>  other_uav_subscribers_; // used for collision avoidance
  void callbackOtherUavAppliedRef(const mrs_msgs::FutureTrajectoryConstPtr& msg);
  std::map<std::string, mrs_msgs::FutureTrajectory> other_uavs_applied_references_;
  void callbackOtherUavPosition(const mrs_msgs::FutureTrajectoryConstPtr& msg);
  std::map<std::string, mrs_msgs::FutureTrajectory> other_uavs_positions_;
  void callbackOtherUavTrajectory(const mrs_msgs::FutureTrajectoryConstPtr& msg);
  std::map<std::string, mrs_msgs::FutureTrajectory> other_uav_avoidance_trajectories_;
  std::vector<mrs_lib::SubscribeHandler<std_msgs::Float32>> other_uav_subscribers3_;
  void callbackOtherUavTubeMinRadius(mrs_lib::SubscribeHandler<std_msgs::Float32>& sh_ptr);
  std::vector<mrs_lib::SubscribeHandler<trackers_brubotics::FutureTrajectoryTube>> other_uav_subscribers4_;
  void callbackOtherUavFutureTrajectoryTube(mrs_lib::SubscribeHandler<trackers_brubotics::FutureTrajectoryTube>& sh_ptr);
  std::map<std::string, trackers_brubotics::FutureTrajectoryTube> other_uav_tube_;
  // std::map<std::string, double> other_uav_tube_min_radius_; // TODO: not used anymore
  // TODO: from ctu MPC tracker
  // void callbackOtherMavTrajectory(mrs_lib::SubscribeHandler<mrs_msgs::FutureTrajectory>& sh_ptr);
  // std::vector<mrs_lib::SubscribeHandler<mrs_msgs::FutureTrajectory>> other_uav_trajectory_subscribers_;
  // std::map<std::string, mrs_msgs::FutureTrajectory> other_uav_avoidance_trajectories_;
  // std::mutex mutex_other_uav_avoidance_trajectories_;
  //|-----------------------------------------------------|//
  
  // ---------------
  // Other variables and functions:
  // ---------------
  // TODO bryan: clean all below
  // | ------------------------ uav state ----------------------- |//

  bool               got_uav_state_ = false; // now added by bryan
  std::mutex         mutex_uav_state_; // now added by bryan
  double uav_heading_; //
  double total_mass_;
  float arm_radius=0.5; //Frank

  double applied_ref_x_;
  double applied_ref_y_;
  double applied_ref_z_;

  bool erg_predictions_theta_trusted_; // init
  double thrust_saturation_physical_;

  double Sa_;

  std_msgs::Float32 Sa_perp_;
  
  double Sa_long_;

  double DSM_total_ = 100000;
  double DSM_total_uav1_ = 100000;
  double DSM_total_uav2_ = 100000;
  // thrust constraints
  double DSM_sT_ = 100000; // Dynamic Safety Margin for total thrust saturation
  double DSM_sT_uav1_ = 100000;
  double DSM_sT_uav2_ = 100000;

  double DSM_sw_ = 100000; // for the (desired or actual) angular body rates 
  double DSM_sw_uav1_ = 100000;;
  double DSM_sw_uav2_ = 100000;;
  // finish added by bryan
  double DSM_swing_c_ = 100000; //Dynamic Safety Margin for swing angle.
  double DSM_swing_c_uav1_ = 100000;
  double DSM_swing_c_uav2_ = 100000;

  double DSM_Tc_ = 100000; //Dynamic Safety Margin for tension in the cable.
  double DSM_Tc_uav1_ = 100000;
  double DSM_Tc_uav2_ = 100000;

  // Static obstacle avoidance 
  double DSM_o_ = 100000; // Dynamic Safety Margin for static obstacle avoidance
  // wall avoidance 
  double DSM_w_ = 100000; // Dynamic Safety Margin for static obstacle avoidance
 
  double min_wall_distance;
  Eigen::Vector3d c_w = Eigen::Vector3d::Zero(3); //wall normal vector 
  Eigen::Vector3d _d_w_ = Eigen::Vector3d::Zero(3); // Wall position (considered infinitly long) //Frank
  // collision avoidance
  int avoidance_this_uav_number_;
  int avoidance_this_uav_priority_;
  
  // added by Titouan and Jonathan
  std_msgs::Int32 DERG_strategy_id;
  std_msgs::Int32 Sa_max_;
  std_msgs::Int32 Sa_perp_max;
  geometry_msgs::Pose point_link_star_;

  double Ra_ = 0.35; // TODO should be taken from urdf, how to do so?? is there  way to subscribe to uav type and then ahrdcode in if cases here?
 
  //Frank : add new DSM_w, DSM_o DONE 
  double DSM_a_ = 100000;

  //|------------------------LOAD-----------------------------|//
  Eigen::Vector3d rel_load_pose_position = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Difference_load_drone_position = Eigen::Vector3d::Zero(3);

  //bool remove_offset_ = true; // As the payload never spawn exactly below the COM of the UAV. This offset is taken out based on the error on the first instant of the controller. TODORAPHAEL : Needed in tracker ? OR only in controller???
  //Eigen::Vector3d load_pose_position_offset_ = Eigen::Vector3d::Zero(3);// The value of the offset is stored in this global var.

  //|-----------------UAV2 informations received to do predictions of 2UAV+beam system inside the tracker of UAV1--------|//
  trackers_brubotics::DSM DSM_msg_;
  trackers_brubotics::DSM DSM_uav1_msg_;
  trackers_brubotics::DSM DSM_uav2_msg_;
  trackers_brubotics::DistanceBetweenUavs DistanceBetweenUavs_msg_;
  // // trajectory loader (mpc_tracker):
  std::tuple<bool, std::string, bool> loadTrajectory(const mrs_msgs::TrajectoryReference msg);
  double _dt1_;

  // the whole trajectory reference split per axis
  std::shared_ptr<VectorXd> des_x_whole_trajectory_;
  std::shared_ptr<VectorXd> des_y_whole_trajectory_;
  std::shared_ptr<VectorXd> des_z_whole_trajectory_;
  std::shared_ptr<VectorXd> des_heading_whole_trajectory_;
  std::mutex                mutex_des_whole_trajectory_;

  // // the reference over the prediction horizon per axis
  // // MatrixXd   des_x_trajectory_;
  // // MatrixXd   des_y_trajectory_;
  // // MatrixXd   des_z_trajectory_;
  // // MatrixXd   des_heading_trajectory_;
  std::mutex mutex_des_trajectory_;


  // trajectory tracking
  std::atomic<bool>       trajectory_tracking_in_progress_ = false;
  // // int        trajectory_tracking_sub_idx_     = 0;  // increases with every iteration of the simulated model
  int        trajectory_tracking_idx_         = 0;  // while tracking, this is the current index in the des_*_whole trajectory
  std::mutex mutex_trajectory_tracking_states_;


  // // current state of the dynamical system
  // //MatrixXd   mpc_x_;          // current state of the uav
  // MatrixXd   mpc_x_heading_;  // current heading of the uav
  // std::mutex mutex_mpc_x_;

  // | ------------------- trajectory tracking ------------------ |

  std::tuple<bool, std::string> resumeTrajectoryTrackingImpl(void);
  std::tuple<bool, std::string> startTrajectoryTrackingImpl(void);
  std::tuple<bool, std::string> stopTrajectoryTrackingImpl(void);
  std::tuple<bool, std::string> gotoTrajectoryStartImpl(void);

  // params of the loaded trajectory
  int    trajectory_size_ = 0;
  double trajectory_dt_;
  bool   trajectory_track_heading_ = false;
  bool   trajectory_tracking_loop_ = false;
  bool   trajectory_set_           = false;
  int    trajectory_count_         = 0;  // counts how many trajectories we have received

  // trajectory diagnostic parameters:
  double arrival_norm_pos_error_treshold_ = 1.0; // 0.50 is too short as long as SS errors take long to converge since this leads to overestimated setling time; // [m]
  double arrival_period_treshold_ = 5.0; // [s]
  bool arrived_at_traj_end_point_ = false; // false by default
  geometry_msgs::Point traj_start_point_;
  geometry_msgs::Point traj_end_point_;
  trackers_brubotics::TrajectoryTracking TrajectoryTracking_msg_;
  bool flag_running_timer_at_traj_end_point_ = false;
  ros::Time time_started_timer_at_traj_end_point_;
  double time_at_start_point_ = 0.0; //default = 0.0
  double time_at_end_point_ = 0.0; //default = 0.0
  // | ------------------- trajectory tracking ------------------ |

  ros::Timer timer_trajectory_tracking_;
  void       timerTrajectoryTracking(const ros::TimerEvent& event);
  // | ------------------------ hovering ------------------------ |
  // ros::Timer timer_hover_;
  // void       timerHover(const ros::TimerEvent& event);
  std::atomic<bool> hover_timer_runnning_ = false;
  std::atomic<bool> hovering_in_progress_ = false;
  void              toggleHover(bool in);
  void setGoal(const double pos_x, const double pos_y, const double pos_z, const double heading, const bool use_heading);

  bool consider_projection_NF_on_max_NF_a_co_ = false;
  double max_repulsion_other_uav_;
  Eigen::Vector3d max_NF_a_co_;

    // | --------------- dynamic reconfigure server --------------- |
  /* TODO: make it compatible with DRS */
  // boost::recursive_mutex                            mutex_drs_;
  // typedef mrs_uav_controllers::se3_controllerConfig DrsConfig_t;
  // typedef dynamic_reconfigure::Server<DrsConfig_t>  Drs_t;
  // boost::shared_ptr<Drs_t>                          drs_;
  // void                                              callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, uint32_t level);
  // DrsConfig_t                                       drs_params_;

  // // | ------------------- the state variables ------------------ |
  std::mutex mutex_state_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

  // | ---------- thrust generation and mass estimation --------- |
  double                        uav_mass_;  // Estimated mass of the UAV (always without payload mass) computed as a vertical integral action in the controller
  
  // mrs_uav_managers::MotorParams _motor_params_;

  std::mutex mutex_gains_;       // locks the gains the are used and filtere

  std::mutex mutex_output_mode_;
  mrs_lib::Profiler profiler;


  // | ---------------------- desired goal ---------------------- |
  double     goal_x_;
  double     goal_y_;
  double     goal_z_;
  double     goal_heading_;
  bool     have_goal_ = false;
  std::mutex mutex_goal_;
  // |-------------------| //

  bool is_initialized_ = false;
  bool is_active_      = false;
  bool starting_bool_=true;
  double dt_ = 0.010; // DO NOT CHANGE! Hardcoded ERG sample time = controller sample time TODO: obtain via loop rate, see MpcTracker
  
  int _num_pred_samples_; // number of tracker's trajectory prediction samples, computed based on prediction dt and predictin horizon.

  // method Kelly:
  /* chrono */  
  std::chrono::time_point<std::chrono::system_clock> start_ERG_, end_ERG_;
  std::chrono::time_point<std::chrono::system_clock> start_NF_, end_NF_;
  std::chrono::time_point<std::chrono::system_clock> start_DSM_, end_DSM_;
  std::chrono::time_point<std::chrono::system_clock> start_pred_, end_pred_;
  std::chrono::duration<double> ComputationalTime_ERG_, ComputationalTime_NF_, ComputationalTime_DSM_, ComputationalTime_pred_;
  trackers_brubotics::ComputationalTime ComputationalTime_msg_;
  // method Zakaria & Frank
  std::stack<clock_t> tictoc_stack; 
};
//}

//WRITE THE FUNCTIONS HERE.


/*initialize()//{*/
void DergbryanTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {
  ROS_INFO("[DergbryanTracker]: start of initialize");
  common_handlers_ = common_handlers;                          
  ros::NodeHandle nh_(parent_nh, "se3_copy_controller"); // NodeHandle 1 for Se3CopyController, used to load controller params
  ros::NodeHandle nh2_(parent_nh, "dergbryan_tracker"); // NodeHandle 2 for DergbryanTracker, used to load tracker params
  _uav_name_       = uav_name; // this UAV
  ros::Time::waitForValid();

  // | ------------------- loading env (session/bashrc) parameters ------------------- |
  ROS_INFO("[DergbryanTracker]: start loading envirenment (session/bashrc) parameters");
  _run_type_          = getenv("RUN_TYPE");
  _type_of_system_    = getenv("TYPE_OF_SYSTEM"); 
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){ // load the required load transportation paramters only if the test is configured for it
    _cable_length_      = std::stod(getenv("CABLE_LENGTH")); 
    if (_type_of_system_=="1uav_payload"){
      _load_mass_         = std::stod(getenv("LOAD_MASS")); // LOAD_MASS is the total load mass of the to be transported object
    }
    else if (_type_of_system_=="2uavs_payload"){ 
      _load_mass_ = 0.50 * std::stod(getenv("LOAD_MASS")); // in case of 2uavs, each uav takes only half of the total load
      _load_length_ = std::stod(getenv("LOAD_LENGTH"));
    }
    // Sanity checks:
    if(_cable_length_ <=0){
      ROS_ERROR("[DergbryanTracker]: _cable_length_ <=0, use a value > 0!");
      ros::requestShutdown();
    }
    if(_load_mass_<=0){
      ROS_ERROR("[DergbryanTracker]: _load_mass_ <=0, use a value > 0!");
      ros::requestShutdown();
    }
  }
  ROS_INFO("[DergbryanTracker]: finished loading environment (session/bashrc) parameters");

  // | ------------------- loading .yaml parameters ------------------- |
  mrs_lib::ParamLoader param_loader(nh_, "Se3CopyController");
  param_loader.loadParam("version", _version_);
  /*TODO: this block triggers the error always, anyone knows solution to this? -> the version of the controller should be that of the tracker. Think how to smooth this.*/
  // if (_version_ != VERSION) {
  //   ROS_ERROR("[DergbryanTracker]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
  //   ros::requestShutdown();
  // }
  
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  // lateral gains and limits:
  param_loader.loadParam("default_gains/horizontal/kp", kpxy_);
  param_loader.loadParam("default_gains/horizontal/kv", kvxy_);
  param_loader.loadParam("default_gains/horizontal/ka", kaxy_);
  param_loader.loadParam("default_gains/horizontal/kiw", kiwxy_);
  param_loader.loadParam("default_gains/horizontal/kib", kibxy_);
  param_loader.loadParam("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("default_gains/horizontal/kib_lim", kibxy_lim_);
  // lateral gains for load damping part of the controller:
  param_loader.loadParam("default_gains/horizontal/kpl", kplxy_);
  param_loader.loadParam("default_gains/horizontal/kvl", kvlxy_);
  // vertical gains:
  param_loader.loadParam("default_gains/vertical/kp", kpz_);
  param_loader.loadParam("default_gains/vertical/kv", kvz_);
  param_loader.loadParam("default_gains/vertical/ka", kaz_);
  // mass estimator (vertical) gains and limits:
  param_loader.loadParam("default_gains/mass_estimator/km", km_);
  param_loader.loadParam("default_gains/mass_estimator/km_lim", km_lim_);
  // attitude gains:
  param_loader.loadParam("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("default_gains/vertical/attitude/kq", kqz_);
  // constraints:
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[DergbryanTracker]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::requestShutdown();
  }
  param_loader.loadParam("constraints/thrust_saturation", _thrust_saturation_); // is further reduced by _thrust_saturation_ratio_
  // output mode:
  param_loader.loadParam("output_mode", output_mode_);
  // TODO: any other Se3CopyController params to load?
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[DergbryanTracker]: could not load all Se3CopyController parameters!");
    ros::requestShutdown();
  } 
  else{
    ROS_INFO("[DergbryanTracker]: correctly loaded all Se3CopyController parameters!");
  }

  mrs_lib::ParamLoader param_loader2(nh2_, "DergbryanTracker");
  param_loader2.loadParam("use_derg", _use_derg_);
  param_loader2.loadParam("network/robot_names", _avoidance_other_uav_names_);
  param_loader2.loadParam("prediction/horizon", _pred_horizon_);
  param_loader2.loadParam("prediction/time_step",_prediction_dt_);
  param_loader2.loadParam("prediction/use_body_inertia",_predicitons_use_body_inertia_);
  param_loader2.loadParam("dynamic_safety_margin/type_id", _DSM_type_);
  param_loader2.loadParam("dynamic_safety_margin/lyapunov/type_id", _Lyapunov_type_);
  param_loader2.loadParam("dynamic_safety_margin/lyapunov/epsilon", _epsilon_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/sT", _kappa_sT_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/sw", _kappa_sw_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/a", _kappa_a_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/w", _kappa_w_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/o", _kappa_o_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/swing_c", _kappa_swing_c_);
  param_loader2.loadParam("dynamic_safety_margin/kappa/Tc", _kappa_Tc_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/constant_dsm", _constant_dsm_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/sT", _enable_dsm_sT_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/sw", _enable_dsm_sw_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/a", _enable_dsm_a_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/w", _enable_dsm_w_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/o", _enable_dsm_o_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/swing_c", _enable_dsm_swing_c_);
  param_loader2.loadParam("dynamic_safety_margin/enable_dsm/Tc", _enable_dsm_Tc_);
  param_loader2.loadParam("constraints/total_thrust/min", _T_min_);
  param_loader2.loadParam("constraints/total_thrust/extra_saturation_ratio", _extra_thrust_saturation_ratio_);
  param_loader2.loadParam("constraints/payload/swing", _constraint_swing_c_);
  param_loader2.loadParam("strategy_id", _DERG_strategy_id_);
  param_loader2.loadParam("use_distance_xy", _use_distance_xy_);
  param_loader2.loadParam("agent_collision_volumes/sphere/radius", _Sa_max_);
  param_loader2.loadParam("agent_collision_volumes/tube/radius/lateral", _Sa_perp_max_);
  param_loader2.loadParam("agent_collision_volumes/tube/radius/longitudinal", _Sa_long_max_);
  param_loader2.loadParam("navigation_field/attraction/smoothing_ratio", _eta_);
  param_loader2.loadParam("navigation_field/repulsion/agents/enabled", _enable_repulsion_a_);
  param_loader2.loadParam("navigation_field/repulsion/agents/use_tube", _use_tube_);
  param_loader2.loadParam("navigation_field/repulsion/agents/influence_margin", _zeta_a_);
  param_loader2.loadParam("navigation_field/repulsion/agents/static_safety_margin", _delta_a_);
  param_loader2.loadParam("navigation_field/repulsion/agents/circulation_gain", _alpha_a_);
  param_loader2.loadParam("navigation_field/repulsion/agents/circulation_type", _circ_type_);
  param_loader2.loadParam("navigation_field/repulsion/wall/enabled", _enable_repulsion_w_); // TODO
  param_loader2.loadParam("navigation_field/repulsion/wall/influence_margin", _zeta_w_); // TODO
  param_loader2.loadParam("navigation_field/repulsion/wall/static_safety_margin", _delta_w_); // TODO
  param_loader2.loadParam("navigation_field/repulsion/wall/wall_position_x", wall_p_x); //TODO Frank : NOT FINAL, need to take account of N_w -> make it a matrix of dim N_w*3 with the position of each wall
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/enabled", _enable_repulsion_o_);
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/influence_margin", _zeta_o_);
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/static_safety_margin", _delta_o_);
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/circulation_gain", _alpha_o_);
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/circulation_type", _circ_type_o);
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/obstacle_position_x", obs_position(0,0));
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/obstacle_position_y", obs_position(1,0));
  param_loader2.loadParam("navigation_field/repulsion/static_obstacle/radius", R_o_);
  param_loader2.loadParam("two_uavs_payload/callback_data_max_time_delay/follower", _max_time_delay_on_callback_data_follower_);
  param_loader2.loadParam("two_uavs_payload/callback_data_max_time_delay/leader", _max_time_delay_on_callback_data_leader_);
  
  // Visualization (rviz):
  param_loader2.loadParam("enable_visualization", _enable_visualization_);
  param_loader2.loadParam("enable_diagnostics_pub", _enable_diagnostics_pub_);
  param_loader2.loadParam("enable_trajectory_pub", _enable_trajectory_pub_);
  // TODO: any other DergbryanTracker params to load?
  // TODO Frank: Add same parameters for the navigation field of the wall and obstable - DONE HALF
  // TODO: Reactivate this parameters NO - They are remplaced by enable_dsm_x
  // param_loader2.loadParam("use_wall_constraints", use_wall_constraints_);
  // param_loader2.loadParam("use_cylindrical_constraints", use_cylindrical_constraints_);
  // param_loader2.loadParam("use_agents_avoidance", use_agents_avoidance_);
  // Frank : Add _kappa_o, _kappa_w - DONE 
  // Frank : Add _zeta_w, _delta_w, _sigma_o, _delta_o, R_o_j (this last one must come from the world file where the obstacle is defined)
  // Frank : Add _enable_dsm_w, _enable_dsm_o - DONE 
  // Frank : Add _d_w  for walls (this last one must come from the world file where the obstacle is defined) - DONE
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[DergbryanTracker]: could not load all DergbryanTracker parameters!");
    ros::requestShutdown();
  } 
  else{
    ROS_INFO("[DergbryanTracker]: correctly loaded all DergbryanTracker parameters!");
  }

  // | ------------------- this uav and other uavs ------------------- |
  // extract the numerical uav name:
  sscanf(_uav_name_.c_str(), "uav%d", &avoidance_this_uav_number_);
  ROS_INFO("[DergbryanTracker]: Numerical ID of this UAV is %d", avoidance_this_uav_number_);
  avoidance_this_uav_priority_ = avoidance_this_uav_number_;
  // exclude this uav from the list:
  std::vector<std::string>::iterator it = _avoidance_other_uav_names_.begin();
  while (it != _avoidance_other_uav_names_.end()) {
    std::string temp_str = *it;
    int other_uav_priority;
    sscanf(temp_str.c_str(), "uav%d", &other_uav_priority);
    if (other_uav_priority == avoidance_this_uav_number_) {
      _avoidance_other_uav_names_.erase(it);
      continue;
    }
    it++;
  }
  ROS_INFO("[DergbryanTracker]: _avoidance_other_uav_names_ contains IDs of all other UAVs.");

  // | ------------------- create publishers ------------------- |
  // TODO bryan: change below to correct msg types (e.g., do not use PoseArray for a thrust or angle)
  // EXAMPLE:
  chatter_publisher_ = nh2_.advertise<std_msgs::String>("chatter", 1);
  // UAV:
  goal_pose_publisher_ = nh2_.advertise<mrs_msgs::ReferenceStamped>("goal_pose", 1);
  applied_ref_pose_publisher_ = nh2_.advertise<mrs_msgs::ReferenceStamped>("applied_ref_pose", 1);
  uav_state_publisher_ = nh2_.advertise<mrs_msgs::UavState>("uav_state", 1);
  predicted_pose_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_poses", 1);
  predicted_vel_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_vels", 1);
  predicted_acc_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_accs", 1);
  predicted_thrust_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_thrust", 1);
  predicted_attrate_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_attrate", 1);
  predicted_des_attrate_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_des_predicted_attrate", 1);
  predicted_tiltangle_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_tiltangle", 1);
  // TODO: create topic similar to predicted_load_position_errors_publisher_
  
  // 1 UAV LOAD:
  if (_type_of_system_=="1uav_payload"){
    predicted_load_pose_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_load_poses", 1);
    predicted_load_vel_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_load_vels", 1);
    predicted_load_acc_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_load_accs", 1); 
    //  phi and theta are used to monitor the world angles (i.e., the ones predicted in the model, not the ones of the encoder). 
    predicted_phi_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_phi", 1);
    predicted_theta_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_theta", 1);
    predicted_phi_dot_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_phi_dot", 1);
    predicted_theta_dot_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_theta_dot", 1);
    predicted_phi_dot_dot_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_phi_dot_dot", 1);
    predicted_theta_dot_dot_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_theta_dot_dot", 1);
    predicted_load_position_errors_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_load_position_errors", 1); // The error vector of the load position, predicted.
    predicted_swing_angle_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_swing_angle", 1);
    predicted_Tc_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_Tc", 1);
  }

  // 2UAVsLOAD:
  // TODO: 
  // generlize the code below based on leader and follower comments above on publishers and not just hardcoded for uav 1 or uav 2. So that code works for any id.
  // See below for subscribers on _avoidance_other_uav_names_ to embed the subcriber part in an automated way to subscibe on info of other uavs. 
  // Make sure the code checks that if a sessions is loaded for , that _avoidance_other_uav_names_ entered via session.yaml contains exactly 2 ids where the lowest id has the highest prioirty and hence is the leader. Esle return an error.
  // So also split the code in a part for publishing and part for subscribing and add it at the correct place.
  // TODO: it is not good to explicitely use a uav id  (e.g., uav 1 , uav2) in the names as with the hardware (nuc and nimbro) you won't be able to choose the uav id (these are hardcoded in the nuc and must not be changed). There is just one "leader" (what you call uav1) and one "follower" (what you call uav2). I would advice to always use the highest priority uav as the leader. So do a check on _avoidance_other_uav_names_ defined above to see which uavs are defined and add an extra requirement that there may be at most 2 uavs specified in the list, else return a ROS_ERROR.
  if (_type_of_system_ == "2uavs_payload"){
    predicted_uav1_poses_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_pose", 1);
    predicted_uav2_poses_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_pose", 1);
    predicted_uav1_vel_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_vel", 1);
    predicted_uav2_vel_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_vel", 1);
    predicted_uav1_anchoring_point_pose_publisher_= nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_anchoring_point_pose", 1);
    predicted_uav2_anchoring_point_pose_publisher_= nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_anchoring_point_pose", 1);
    predicted_uav1_anchoring_point_vel_publisher_= nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_anchoring_point_vel", 1);
    predicted_uav2_anchoring_point_vel_publisher_= nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_anchoring_point_vel", 1);
    predicted_uav1_thrust_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_thrust", 1);
    predicted_uav2_thrust_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_thrust", 1);
    predicted_uav1_attitude_rate_publisher_ =  nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_attitude_rate", 1);
    predicted_uav2_attitude_rate_publisher_ =  nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_attitude_rate", 1);
    predicted_uav1_swing_angle_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_swing_angle", 1);
    predicted_uav2_swing_angle_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_swing_angle", 1);
    predicted_uav1_tension_force_publisher_ = nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav1_tension_force", 1);
    predicted_uav2_tension_force_publisher_= nh2_.advertise<geometry_msgs::PoseArray>("custom_predicted_uav2_tension_force", 1);
    if (_uav_name_ == "uav1"){  // leader
      DSM_uav1_publisher_ = nh2_.advertise<trackers_brubotics::DSM>("DSM_leader", 1);
      DSM_uav2_publisher_ = nh2_.advertise<trackers_brubotics::DSM>("DSM_follower", 1);
    }
    // for communication between the leader and follower uavs: follower publishes state feedback to leader and leader commands the reference to the follower
    if (_uav_name_ == "uav1"){  // leader
      position_cmd_follower_from_leader_pub_ = nh2_.advertise<mrs_msgs::PositionCommand>("position_cmd_follower_from_leader", 1); 
      goal_position_cmd_follower_from_leader_pub_ = nh2_.advertise<mrs_msgs::PositionCommand>("goal_position_cmd_follower_from_leader", 1); 
    }
    else if (_uav_name_ == "uav2"){ // follower
      uav_state_follower_for_leader_pub_ = nh2_.advertise<mrs_msgs::UavState>("uav_state_follower_for_leader", 1);
      anchoring_point_follower_for_leader_pub_ = nh2_.advertise<mrs_msgs::UavState>("anchoring_point_follower_for_leader", 1);
      position_cmd_follower_for_leader_pub_ = nh2_.advertise<mrs_msgs::PositionCommand>("position_cmd_follower_for_leader", 1); 
      goal_position_cmd_follower_for_leader_pub_ = nh2_.advertise<mrs_msgs::PositionCommand>("goal_position_cmd_follower_for_leader", 1); 
    }
  }

  // general:
  DSM_publisher_ = nh2_.advertise<trackers_brubotics::DSM>("DSM", 1);
  DistanceBetweenUavs_publisher_ = nh2_.advertise<trackers_brubotics::DistanceBetweenUavs>("DistanceBetweenUavs", 1);
  TrajectoryTracking_publisher_ = nh2_.advertise<trackers_brubotics::TrajectoryTracking>("TrajectoryTracking", 1);
  ComputationalTime_publisher_ = nh2_.advertise<trackers_brubotics::ComputationalTime>("ComputationalTime", 1);
  // DERG - multi-uav collision avoidance:
  tube_min_radius_publisher_ = nh2_.advertise<std_msgs::Float32>("tube_min_radius", 1);
  future_tube_publisher_ = nh2_.advertise<trackers_brubotics::FutureTrajectoryTube>("future_trajectory_tube", 1);
  avoidance_applied_ref_publisher_ = nh2_.advertise<mrs_msgs::FutureTrajectory>("uav_applied_ref", 1);
  avoidance_pos_publisher_ = nh2_.advertise<mrs_msgs::FutureTrajectory>("uav_position", 1);
  avoidance_trajectory_publisher_= nh2_.advertise<mrs_msgs::FutureTrajectory>("predicted_trajectory", 1);
  // visualization:
  if(_enable_visualization_){
    derg_strategy_id_publisher_ = nh2_.advertise<std_msgs::Int32>("derg_strategy_id", 1);
    point_link_star_publisher_ = nh2_.advertise<geometry_msgs::Pose>("point_link_star", 1);
    sa_max_publisher_ = nh2_.advertise<std_msgs::Int32>("sa_max", 1);
    sa_perp_max_publisher_ = nh2_.advertise<std_msgs::Int32>("sa_perp_max", 1);
  }
  // TODO: see mpc_tracker for other topics 
  ROS_INFO("[DergbryanTracker]: advertised all publishers.");

  // | ------------------- create subscribers ------------------- |
  // this uav subscribes to own (i.e., of this uav) load states:
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){
    if (_run_type_ == "simulation"){ // subscriber of the gazebo simulation
      load_state_sub_ =  nh_.subscribe("/gazebo/link_states", 1, &DergbryanTracker::GazeboLoadStatesCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else if (_run_type_ == "uav"){ // subscriber of the hardware encoders
      std::string slash = "/";
      data_payload_sub_ = nh_.subscribe(slash.append(_uav_name_.append("/serial/received_message")), 1, &DergbryanTracker::BacaLoadStatesCallback, this, ros::TransportHints().tcpNoDelay()); // TODO: explain how this is used for 2 uav hardware
    }
    else{ // undefined
      ROS_ERROR("[DergbryanTracker]: undefined _run_type_ used for uav with payload!");
      ros::requestShutdown();
    }
    if (_type_of_system_ == "2uavs_payload"){
      // for communication between the leader and follower uavs: leader subscribes to state feedback of follower and follower to the reference the leader commanded
      if (_uav_name_ == "uav1"){  // leader
        uav_state_follower_for_leader_sub_ = nh2_.subscribe("/uav2/control_manager/dergbryan_tracker/uav_state_follower_for_leader", 1, &DergbryanTracker::uav_state_follower_for_leader_callback, this, ros::TransportHints().tcpNoDelay());
        anchoring_point_follower_for_leader_sub_ = nh2_.subscribe("/uav2/control_manager/dergbryan_tracker/anchoring_point_follower_for_leader", 1, &DergbryanTracker::anchoring_point_follower_for_leader_callback, this, ros::TransportHints().tcpNoDelay());
        position_cmd_follower_for_leader_sub_ = nh2_.subscribe("/uav2/control_manager/dergbryan_tracker/position_cmd_follower_for_leader", 1, &DergbryanTracker::position_cmd_follower_for_leader_callback, this, ros::TransportHints().tcpNoDelay());
        goal_position_cmd_follower_for_leader_sub_ = nh2_.subscribe("/uav2/control_manager/dergbryan_tracker/goal_position_cmd_follower_for_leader", 1, &DergbryanTracker::goal_position_cmd_follower_for_leader_callback, this, ros::TransportHints().tcpNoDelay());
      }
      else if (_uav_name_ == "uav2"){ // follower
        position_cmd_follower_from_leader_sub_ = nh2_.subscribe("/uav1/control_manager/dergbryan_tracker/position_cmd_follower_from_leader", 1, &DergbryanTracker::position_cmd_follower_from_leader_callback, this, ros::TransportHints().tcpNoDelay());
        goal_position_cmd_follower_from_leader_sub_ = nh2_.subscribe("/uav1/control_manager/dergbryan_tracker/goal_position_cmd_follower_from_leader", 1, &DergbryanTracker::goal_position_cmd_follower_from_leader_callback, this, ros::TransportHints().tcpNoDelay());
      }
    }
  }

  // this uav subscribes to these topics of other uavs:
  mrs_lib::SubscribeHandlerOptions shopts;
   // TODO !!! see mpc tracker for other options
  shopts.nh                 = nh_; // TODO: shouldn't this be nh2 as this is related to tracker and not to controller?
  shopts.node_name          = "DergbryanTracker";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();
  for (int i = 0; i < int(_avoidance_other_uav_names_.size()); i++) {
    // just an example:
    std::string chatter_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/chatter");
    ROS_INFO("[DergbryanTracker]: subscribing to %s", chatter_topic_name.c_str());
    other_uav_subscribers2_.push_back(mrs_lib::SubscribeHandler<std_msgs::String>(shopts, chatter_topic_name, &DergbryanTracker::chatterCallback, this));

    // subscribe to other uav topics for e.g., collision avoidance and cooperative load transport:
    std::string applied_ref_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/uav_applied_ref");  
    ROS_INFO("[DergbryanTracker]: subscribing to %s", applied_ref_topic_name.c_str());
    other_uav_subscribers_.push_back(nh_.subscribe(applied_ref_topic_name, 1, &DergbryanTracker::callbackOtherUavAppliedRef, this, ros::TransportHints().tcpNoDelay()));

    std::string position_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/uav_position"); 
    ROS_INFO("[DergbryanTracker]: subscribing to %s", position_topic_name.c_str());
    other_uav_subscribers_.push_back(nh_.subscribe(position_topic_name, 1, &DergbryanTracker::callbackOtherUavPosition, this, ros::TransportHints().tcpNoDelay()));

    std::string trajectory_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/predicted_trajectory");
    ROS_INFO("[DergbryanTracker]: subscribing to %s", trajectory_topic_name.c_str());
    other_uav_subscribers_.push_back(nh_.subscribe(trajectory_topic_name, 1, &DergbryanTracker::callbackOtherUavTrajectory, this, ros::TransportHints().tcpNoDelay()));

    std::string tube_min_radius_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/tube_min_radius");
    ROS_INFO("[DergbryanTracker]: subscribing to %s", tube_min_radius_topic_name.c_str());
    other_uav_subscribers3_.push_back(mrs_lib::SubscribeHandler<std_msgs::Float32>(shopts, tube_min_radius_topic_name, &DergbryanTracker::callbackOtherUavTubeMinRadius, this));

    std::string future_trajectory_tube_topic_name = std::string("/") + _avoidance_other_uav_names_[i] + std::string("/") + std::string("control_manager/dergbryan_tracker/future_trajectory_tube");
    ROS_INFO("[DergbryanTracker]: subscribing to %s", future_trajectory_tube_topic_name.c_str());
    other_uav_subscribers4_.push_back(mrs_lib::SubscribeHandler<trackers_brubotics::FutureTrajectoryTube>(shopts, future_trajectory_tube_topic_name, &DergbryanTracker::callbackOtherUavFutureTrajectoryTube, this));
  }
  ROS_INFO("[DergbryanTracker]: linked all subscribers to their callbacks.");

  // | ---------------- prepare stuff from params --------------- |
  _num_pred_samples_ = (int)(_pred_horizon_/_prediction_dt_); // double converted to int
  _thrust_saturation_ = _extra_thrust_saturation_ratio_*_thrust_saturation_; // as to not trigger the emergency landing too quickly

  // ensure output_mode_ is defined well:
  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[DergbryanTracker]: output mode has to be {0, 1}!");
    ros::requestShutdown();
  }

  // TODO: check unit _tilt_angle_failsafe_ and if used
  // convert to radians
  //_tilt_angle_failsafe_ = (_tilt_angle_failsafe_ / 180.0) * M_PI;
  
  // profiler:
  profiler = mrs_lib::Profiler(nh2_, "DergbryanTracker", _profiler_enabled_);// TODO: why do we use a controller param to enable the tracker profiler?
  // setTrajectory functions similar to mpc_tracker
  // TODO: do we use it? How to choose update rate of tracker?
  _dt1_ = dt_; //1.0 / _mpc_rate_;
  // mpc_x_heading_ = MatrixXd::Zero(_mpc_n_states_heading_, 1); // TODO: used?

  // timers:
  timer_trajectory_tracking_  = nh_.createTimer(ros::Rate(1.0), &DergbryanTracker::timerTrajectoryTracking, this, false, false);
  // timer_hover_                = nh_.createTimer(ros::Rate(10.0), &DergbryanTracker::timerHover, this, false, false); // TODO: do we need it?
  
  // initialization completed:
  is_initialized_ = true;
  ROS_INFO("[DergbryanTracker]: initialized");
}


/*activate()//{*/
std::tuple<bool, std::string> DergbryanTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) {
  
  std::stringstream ss;

  if (!got_constraints_) {
    ss << "can not activate, missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!got_uav_state_) {
    ss << "odometry not set";
    ROS_ERROR_STREAM("[DergbryanTracker]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  trajectory_tracking_in_progress_ = false;

  toggleHover(true);

  ss << "Activated";
  is_active_ = true;
  ROS_INFO("[DergbryanTracker]: activated");
  return std::tuple(true, ss.str());
}

/*deactivate()//{*/
void DergbryanTracker::deactivate(void) {

  toggleHover(false);

  is_active_ = false;
  trajectory_tracking_in_progress_ = false;

  timer_trajectory_tracking_.stop();
  {
    std::scoped_lock lock(mutex_trajectory_tracking_states_);

    trajectory_tracking_idx_     = 0;
    //trajectory_tracking_sub_idx_ = 0;
  }


  ROS_INFO("[DergbryanTracker]: deactivated");
  //publishDiagnostics();
}
//}

/*resetStatic()//{*/
bool DergbryanTracker::resetStatic(void) {
  ROS_INFO("[DergbryanTracker]: no states to reset");


  // {
  // std::scoped_lock lock(mutex_mpc_x_);
    trajectory_tracking_in_progress_ = false;
  // }

  return true;
}
//}

/*update()//{*/
const mrs_msgs::PositionCommand::ConstPtr DergbryanTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
                                                              [[maybe_unused]] const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {
  // ROS_INFO("[DergbryanTracker]: updating the DergbryanTracker");
  mrs_lib::Routine profiler_routine = profiler.createRoutine("update");
  {
    std::scoped_lock lock(mutex_uav_state_);
    uav_state_ = *uav_state;
    got_uav_state_ = true;
  }
  /* TODO: make it compatible with DRS */
  //auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return mrs_msgs::PositionCommand::Ptr();
  }

  uav_mass_        = common_handlers_->getMass(); //update the estimated mass, stored in global variable as all part of the code will need it. Don't vary over an update, but changes between the updates.
  //ROS_INFO_STREAM("uav_mass_"<< uav_mass_);

  mrs_msgs::PositionCommand position_cmd;
  // set the header
  position_cmd.header.stamp    = uav_state_.header.stamp;
  position_cmd.header.frame_id = uav_state_.header.frame_id;
  if (starting_bool_) {
    // stay in place
    goal_x_= uav_state_.pose.position.x;
    goal_y_= uav_state_.pose.position.y;
    goal_z_= uav_state_.pose.position.z;
    // set heading based on current odom
    try {
      goal_heading_ = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();
      position_cmd.use_heading = 1;
    }
    catch (...) {
      position_cmd.use_heading = 0;
      ROS_ERROR_THROTTLE(1.0, "[DergbryanTracker]: could not calculate the current UAV heading");
    }

    position_cmd.position.x     = goal_x_;
    position_cmd.position.y     = goal_y_;
    position_cmd.position.z     = goal_z_;
    position_cmd.heading        = goal_heading_;

    // Currently we hardcode as follows:
    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_velocity_vertical   = 1;
    position_cmd.use_velocity_horizontal = 1;
    position_cmd.use_acceleration        = 0;
    position_cmd.use_jerk                = 0;
    position_cmd.use_heading             = 1;
    position_cmd.use_heading_rate        = 1;
  
    ROS_INFO("[Dergbryan tracker - odom]: [goal_x_=%.2f],[goal_y_=%.2f],[goal_z_=%.2f, goal_heading_=%.2f]",goal_x_,goal_y_,goal_z_,goal_heading_);

    applied_ref_x_ = position_cmd.position.x;
    applied_ref_y_ = position_cmd.position.y;
    applied_ref_z_ = position_cmd.position.z;
    //add heading applied ref

    uav_state_prev_ = uav_state_; // required for computing derrivative of angular rate

    starting_bool_=false;
    return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
  }
  /* begin copy of se3controller*/ // TO KEEP FOR BRYAN
  // | -------------------- calculate the dt -------------------- |

  // double dt;
  // dt = 0.010;
  /* TODO: this if condition is commented since we will only allow this tracker to be activated if before the se3 was activated. 
  so will need to see how to use const mrs_msgs::ControllerStatus Se3Controller::getStatus() { of the controller in here to check if it is active*/

  // if (first_iteration_) {

  //   last_update_time_ = uav_state->header.stamp;

  //   first_iteration_ = false;

  //   ROS_INFO("[DergbryanTracker]: first iteration");

  //   return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));

  // } else {

  /* NOTE: assume a fixed sampling time used in the controller for this DergTracker*/
  /* TODO: load dt (se3 control sample time) as inverse of control sampling frequency defined somewhere in mrs tracker.yaml files (e.g. see mpc, line trackers) and set to 100Hz for controller (*/
  // dt = 0.010; 
  // dt                = (uav_state->header.stamp - last_update_time_).toSec();
  // last_update_time_ = uav_state->header.stamp;
  // }

  /* NOTE: commented this section bacause we assume the controller has a fixed sample time of dt*/
  // if (fabs(dt) <= 0.001) {

  //   ROS_DEBUG("[DergbryanTracker]: the last odometry message came too close (%.2f s)!", dt);

  //   if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

  //     return last_attitude_cmd_;

  //   } else {

  //     return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
  //   }
  // }

  //|----------------------------------------------------- |//


  // | ----------------- get the current heading ---------------- |

  double uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[DergbryanTracker]: could not calculate the UAV heading");
  }

  uav_heading_ = uav_heading;

  // Currently we hardcode as follows:
  position_cmd.use_position_vertical   = 1;
  position_cmd.use_position_horizontal = 1;
  position_cmd.use_velocity_vertical   = 1;
  position_cmd.use_velocity_horizontal = 1;
  position_cmd.use_acceleration        = 0;
  position_cmd.use_jerk                = 0;
  position_cmd.use_heading             = 1;
  position_cmd.use_heading_rate        = 1;
  // initially applied_ref_x_, applied_ref_y_, applied_ref_z_ is defined as stay where you are when starting_bool_ = 1;
  position_cmd.position.x     = applied_ref_x_;
  position_cmd.position.y     = applied_ref_y_;
  position_cmd.position.z     = applied_ref_z_;
  position_cmd.heading        = goal_heading_;

  // publishFollowerDataForLeaderIn2uavs_payload:
  /* 
     For the 2UAVs+payload case we must first ensure the required information of 
     the follower UAV is published so that leader UAV can perform the predictions.
  */
  if(_type_of_system_=="2uavs_payload"){
      publishFollowerDataForLeaderIn2uavs_payload(position_cmd); // published by follower uav
      if(_uav_name_ == "uav1"){
        // check the accompanying callbacks of the leader uav if the msgs of the follower are received from timestamps which are not delayed too much wrt the current timestamp of the uav_state_.
        double time_delay_1 = std::abs(uav_state_follower_for_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
        double time_delay_2 = std::abs(anchoring_point_follower_for_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
        double time_delay_3 = std::abs(position_cmd_follower_for_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
        double time_delay_4 = std::abs(goal_position_cmd_follower_for_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
        // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_1 = " << time_delay_1);
        // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_2 = " << time_delay_2);
        // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_3 = " << time_delay_3);
        // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_4 = " << time_delay_4);
        if (time_delay_1<_max_time_delay_on_callback_data_follower_ && payload_spawned_ && time_delay_2<_max_time_delay_on_callback_data_follower_ && time_delay_3<_max_time_delay_on_callback_data_follower_ && time_delay_4<_max_time_delay_on_callback_data_follower_){
          callback_data_follower_no_delay_ = true;
        } else {
          callback_data_follower_no_delay_ = false;
          ROS_WARN_STREAM("[DergbryanTracker]: follower data is delayed too much and/or payload_not yet spawned!");
        }
      }
  }

  if (_use_derg_){
    //tictoc_stack.push(clock()); // TODO: clean if not used
    computePSCTrajectoryPredictions(position_cmd, uav_heading, last_attitude_cmd);
    //ROS_INFO_STREAM("Prediction calculation took = \n "<< (double)(clock()- tictoc_stack.top())/CLOCKS_PER_SEC << "seconds.");
    //tictoc_stack.pop(); // TODO: clean if not used
    if(_type_of_system_=="1uav_no_payload" || (_type_of_system_=="1uav_payload" && payload_spawned_)){
      computeERG(); // computes the applied_ref_x_, applied_ref_y_, applied_ref_z_
    } 
    else if((_type_of_system_=="2uavs_payload" && _uav_name_ == "uav1" && payload_spawned_ && callback_data_follower_no_delay_)){
      computeERG(); // computes the applied_ref_x_, applied_ref_y_, applied_ref_z_, position_cmd_follower_from_leader_, goal_position_cmd_follower_from_leader_
    }
    else if ((_type_of_system_=="2uavs_payload" || _type_of_system_=="1uav_payload") && !payload_spawned_){
      // allows the UAV to be repositioned before spawning and attaching the payload
      // but without ERG, so only safe for small steps!
      applied_ref_x_ = goal_x_;
      applied_ref_y_ = goal_y_;
      applied_ref_z_ = goal_z_;
    }
    else if (_type_of_system_=="2uavs_payload" && _uav_name_ == "uav2" && payload_spawned_){
      // check the callback of the follower uav if the msgs of the leader is received from a timestamp which is not delayed too much wrt the current timestamp of the uav_state_.
      double time_delay_1 = std::abs(position_cmd_follower_from_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
      double time_delay_2 = std::abs(goal_position_cmd_follower_from_leader_.header.stamp.toSec() - uav_state_.header.stamp.toSec());
      // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_1 = " << time_delay_1);
      // ROS_INFO_STREAM("[DergbryanTracker]: time_delay_2 = " << time_delay_2);
      if (time_delay_1 < _max_time_delay_on_callback_data_leader_ && time_delay_2 < _max_time_delay_on_callback_data_leader_){
        callback_data_leader_no_delay_ = true;
        // update the follower's applied ref:
        applied_ref_x_ = position_cmd_follower_from_leader_.position.x;
        applied_ref_y_ = position_cmd_follower_from_leader_.position.y;
        applied_ref_z_ = position_cmd_follower_from_leader_.position.z;
        // update the follower's target ref:
        /* We do not update the goal_ vars below as this would conflict with the follower uav being commanded other target 
           references via setGoal and timerTrajectoryTracking. So this topic can be interpreted as the actual target reference
           for the follower uav and commanded by the leader uav. It ensures that an infeasible target request, which violates 
           the bar length constraint, is converted in a feasible target command for the follower. 
           goal_x_ = goal_position_cmd_follower_from_leader_.position.x;
           goal_y_ = goal_position_cmd_follower_from_leader_.position.y;
           goal_z_ = goal_position_cmd_follower_from_leader_.position.z;
        */
      } else {
        callback_data_leader_no_delay_ = false;
        // don't update the follower's applied_ref_x_, applied_ref_y_, applied_ref_z_
        ROS_WARN_STREAM("[DergbryanTracker]: leader data is delayed too much!");
      }
    } else{
      ROS_ERROR_STREAM("[DergbryanTracker]: the case variables for the update() routine are not well configured!!!");
    }
    // prepare function output:
    position_cmd.position.x     = applied_ref_x_; 
    position_cmd.position.y     = applied_ref_y_;
    position_cmd.position.z     = applied_ref_z_;
    position_cmd.heading        = goal_heading_;
  } 
  else{ // bypass the ERG (i.e., potentially UNSAFE!):
     
    ROS_INFO_STREAM("we are inside else");
    // set applied ref = desired goal ref (bypass tracker)
    applied_ref_x_ = goal_x_;
    applied_ref_y_ = goal_y_;
    applied_ref_z_ = goal_z_;
    // prepare function output:
    position_cmd.position.x     = applied_ref_x_;
    position_cmd.position.y     = applied_ref_y_;
    position_cmd.position.z     = applied_ref_z_;
    position_cmd.heading        = goal_heading_;
    // but still compute the predictions
    computePSCTrajectoryPredictions(position_cmd, uav_heading, last_attitude_cmd);
  }

  // ROS_INFO_STREAM("uav_state_.pose.position.x = " << uav_state_.pose.position.x);
  // ROS_INFO_STREAM("uav_state_.pose.position.y = " << uav_state_.pose.position.y);
  // ROS_INFO_STREAM("uav_state_.pose.position.z = " << uav_state_.pose.position.z);
  // ROS_INFO_STREAM("uav_heading_ = " << uav_heading_);
  // ROS_INFO_STREAM("goal_x_ = " << goal_x_);
  // ROS_INFO_STREAM("goal_y_ = " << goal_y_);
  // ROS_INFO_STREAM("goal_z_ = " << goal_z_);
  // ROS_INFO_STREAM("goal_heading_ = " << goal_heading_);
  // ROS_INFO_STREAM("DSM_total_ = " << DSM_total_);
  // ROS_INFO_STREAM("position_cmd.position.x = " << position_cmd.position.x);
  // ROS_INFO_STREAM("position_cmd.position.y = " << position_cmd.position.y);
  // ROS_INFO_STREAM("position_cmd.position.z = " << position_cmd.position.z);
  // ROS_INFO_STREAM("position_cmd.heading = " << position_cmd.heading);


  // Prepare the applied_ref_pose_msg:
  mrs_msgs::ReferenceStamped applied_ref_pose_msg;
  applied_ref_pose_msg.header.stamp    = uav_state_.header.stamp;
  applied_ref_pose_msg.header.frame_id = uav_state_.header.frame_id;
  applied_ref_pose_msg.reference.position.x  = applied_ref_x_;
  applied_ref_pose_msg.reference.position.y  = applied_ref_y_;
  applied_ref_pose_msg.reference.position.z  = applied_ref_z_;
  applied_ref_pose_msg.reference.heading = goal_heading_; // currently no ERG on heading/yaw TODO: add one

  // Prepare the goal_pose_msg:
  mrs_msgs::ReferenceStamped goal_pose_msg;
  goal_pose_msg.header.stamp    = uav_state_.header.stamp;
  goal_pose_msg.header.frame_id = uav_state_.header.frame_id;
  goal_pose_msg.reference.position.x  = goal_x_;
  goal_pose_msg.reference.position.y  = goal_y_;
  goal_pose_msg.reference.position.z  = goal_z_;
  goal_pose_msg.reference.heading = goal_heading_;

  // Publishers:
  // chatter_publisher_: example (disabled by default):
  if (false) {
    std_msgs::String msg;
    msg.data = "Some chatter in the update routine";
    try {
      chatter_publisher_.publish(msg);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", chatter_publisher_.getTopic().c_str());
    }
  }
  
  // uav_state_publisher_:
  try {
    uav_state_publisher_.publish(uav_state_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", uav_state_publisher_.getTopic().c_str());
  }

  // applied_ref_pose_publisher_:
  // if(_enable_visualization_){ // curently only used for RVIZ
  try {
    applied_ref_pose_publisher_.publish(applied_ref_pose_msg);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", applied_ref_pose_publisher_.getTopic().c_str());
  }
  // }  

  // goal_pose_publisher_:
  try {
    goal_pose_publisher_.publish(goal_pose_msg);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", goal_pose_publisher_.getTopic().c_str());
  }
  
  // ComputationalTime_publisher_:
  ComputationalTime_msg_.stamp = uav_state_.header.stamp;
  try {
    ComputationalTime_publisher_.publish(ComputationalTime_msg_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", ComputationalTime_publisher_.getTopic().c_str());
  }


  clearMsgsAfterUpdate();
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}

void DergbryanTracker::clearMsgsAfterUpdate(){
  // Clear (i.e., empty) ROS msgs

  // Note: under communication delay the arrays of the predicitons will be empty and not equal to the last know uav position!
  // TODO maybe then better to place these in the try catch statement right after they are published.
  
  //  - Predictions computed in trajectory_prediction_general() and used in computeERG():
  predicted_poses_out_.poses.clear();
  predicted_velocities_out_.poses.clear();
  predicted_accelerations_out_.poses.clear();
  predicted_thrust_out_.poses.clear();
  predicted_attituderate_out_.poses.clear();
  predicted_des_attituderate_out_.poses.clear();
  predicted_tiltangle_out_.poses.clear();
  
    //-------------LOAD-------------//
  predicted_load_poses_out_.poses.clear();
  predicted_load_velocities_out_.poses.clear();
  predicted_load_accelerations_out_.poses.clear();

  predicted_phi_out_.poses.clear();
  predicted_theta_out_.poses.clear();
  predicted_phi_dot_out_.poses.clear();
  predicted_theta_dot_out_.poses.clear();
  predicted_phi_dot_dot_out_.poses.clear();
  predicted_theta_dot_dot_out_.poses.clear();
  predicted_load_position_errors_out_.poses.clear();
  predicted_swing_angle_out_.poses.clear();
  predicted_Tc_out_.poses.clear();
  //-----------2UAV pred-----------//
  predicted_uav1_poses_out_.poses.clear();
  predicted_uav2_poses_out_.poses.clear();
  predicted_uav1_vel_out_.poses.clear();
  predicted_uav2_vel_out_.poses.clear();
  predicted_uav1_anchoring_point_pose_out_.poses.clear();
  predicted_uav2_anchoring_point_pose_out_.poses.clear();
  predicted_uav1_anchoring_point_vel_out_.poses.clear();
  predicted_uav2_anchoring_point_vel_out_.poses.clear();
  predicted_uav1_thrust_out_.poses.clear();
  predicted_uav2_thrust_out_.poses.clear();
  predicted_uav1_attitude_rate_out_.poses.clear();
  predicted_uav2_attitude_rate_out_.poses.clear();
  predicted_uav1_swing_angle_out_.poses.clear();
  predicted_uav2_swing_angle_out_.poses.clear();
  predicted_uav1_tension_force_out_.poses.clear();
  predicted_uav2_tension_force_out_.poses.clear();
  //  - DERG - collision avoidance:
  uav_applied_ref_out_.points.clear();
  uav_position_out_.points.clear();
  future_trajectory_out_.points.clear();
  // Compute time:
  ComputationalTime_msg_.parts.clear();
  ComputationalTime_msg_.name_parts.clear();
}

void DergbryanTracker::computePSCTrajectoryPredictions(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd){ 
  /* Mother function that computes the trajectory predictions of the PSC which will be used by the ERG. 
     It contains all the different cases that depend on the _type_of_system_ of which we want to predict the behavior.
  */
  erg_predictions_theta_trusted_ = true; // initialize

  if(_type_of_system_=="2uavs_payload" && _uav_name_=="uav1" && callback_data_follower_no_delay_){
    trajectory_prediction_general_load_2UAV(position_cmd, uav_heading, last_attitude_cmd); // TODO: check 2 uav case later  
  }
  else if(_type_of_system_=="1uav_no_payload" || (_type_of_system_=="1uav_payload" && payload_spawned_)){
    trajectory_prediction_general(position_cmd, uav_heading, last_attitude_cmd);  
  }  
  else{
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: the case variables for the computePSCTrajectoryPredictions() routine did not allow to compute the trajectory predictions");
  }
}

void DergbryanTracker::publishFollowerDataForLeaderIn2uavs_payload(mrs_msgs::PositionCommand position_cmd){
// TODO: add a case for hardware
/*
This function ensures that the data of the follower UAV (i.e., state of the uav, its anchoring point, and the position cmd) are published.
The publishing of the payload state is only allowed from when the payload has spawned as to pevent NaN from being used in the trajectory predictions on the leader uav. 
*/
  if (_run_type_ == "simulation"){
    if (_uav_name_ == "uav2"){  // only publish info of the follower UAV, not of the leader
      // 1) uav_state_follower_for_leader_pub_:
      try {
        uav_state_follower_for_leader_pub_.publish(uav_state_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", uav_state_follower_for_leader_pub_.getTopic().c_str());
      }

      // 2) position_cmd_follower_for_leader_pub_:
      position_cmd_follower_for_leader_ = position_cmd;
      try {
        position_cmd_follower_for_leader_pub_.publish(position_cmd_follower_for_leader_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", position_cmd_follower_for_leader_pub_.getTopic().c_str());
      }

      // 3) anchoring_point_follower_for_leader_pub_:
      if(payload_spawned_){ // only if the payload has spawned, start to publish its position and velocity  
        anchoring_point_follower_for_leader_msg_.header.stamp = uav_state_.header.stamp;
        anchoring_point_follower_for_leader_msg_.header.frame_id = uav_state_.header.frame_id;
        
        anchoring_point_follower_for_leader_msg_.pose.position.x = anchoring_pt_pose_position_[0];
        anchoring_point_follower_for_leader_msg_.pose.position.y = anchoring_pt_pose_position_[1];
        anchoring_point_follower_for_leader_msg_.pose.position.z = anchoring_pt_pose_position_[2];

        anchoring_point_follower_for_leader_msg_.velocity.linear.x = anchoring_pt_lin_vel_[0];
        anchoring_point_follower_for_leader_msg_.velocity.linear.y = anchoring_pt_lin_vel_[1];
        anchoring_point_follower_for_leader_msg_.velocity.linear.z = anchoring_pt_lin_vel_[2];

        try {
          anchoring_point_follower_for_leader_pub_.publish(anchoring_point_follower_for_leader_msg_);
        }
        catch (...) {
          ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", anchoring_point_follower_for_leader_pub_.getTopic().c_str());
        }
      }

      // 4) goal_position_cmd_follower_for_leader_pub_:
      goal_position_cmd_follower_for_leader_.header.stamp    = uav_state_.header.stamp;
      goal_position_cmd_follower_for_leader_.header.frame_id = uav_state_.header.frame_id;
      goal_position_cmd_follower_for_leader_.position.x = goal_x_;
      goal_position_cmd_follower_for_leader_.position.y = goal_y_;
      goal_position_cmd_follower_for_leader_.position.z = goal_z_;
      goal_position_cmd_follower_for_leader_.heading = goal_heading_;
      try {
        goal_position_cmd_follower_for_leader_pub_.publish(goal_position_cmd_follower_for_leader_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", goal_position_cmd_follower_for_leader_pub_.getTopic().c_str());
      }
    }
  }
  //TODO : FK of hardware case to put here.
}


/*getStatus()//{*/
const mrs_msgs::TrackerStatus DergbryanTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;
  tracker_status.active = is_active_;
  tracker_status.tracking_trajectory = trajectory_tracking_in_progress_;


    if (trajectory_tracking_in_progress_) {

    //auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

    std::scoped_lock lock(mutex_des_whole_trajectory_);
    }

  return tracker_status;
}
//}

/*enableCallbacks()//{*/
const std_srvs::SetBoolResponse::ConstPtr DergbryanTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  res.message = "Callbacks not implemented";
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}
//}

/*switchOdometrySource()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) {

  std_srvs::TriggerResponse res;

  res.message = "Switching not implemented";
  res.success = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*hover()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {

  toggleHover(true);

  std::stringstream ss;
  ss << "initiating hover";

  std_srvs::TriggerResponse res;
  res.success = true;
  res.message = ss.str();

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*startTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  std::stringstream ss;

  auto [success, message] = startTrajectoryTrackingImpl();

  std_srvs::TriggerResponse res;
  res.success = success;
  res.message = message;
  
  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*stopTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  auto [success, message] = stopTrajectoryTrackingImpl();

  std_srvs::TriggerResponse res;
  res.success = success;
  res.message = message;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*resumeTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  auto [success, message] = resumeTrajectoryTrackingImpl();

  std_srvs::TriggerResponse res;
  res.success = success;
  res.message = message;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr& cmd) {

  
  auto [success, message] = gotoTrajectoryStartImpl();

  std_srvs::TriggerResponse res;
  res.success = success;
  res.message = message;

  if(_enable_diagnostics_pub_){ 
    // the global variables used in the TrajectoryTracking_msg_ computations will be reset accordingly
    time_at_start_point_ = 0.0;
    time_at_end_point_ = 0.0;
    flag_running_timer_at_traj_end_point_ = false;
    arrived_at_traj_end_point_ = false;
  }

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}

/*setConstraints()//{*/
const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr DergbryanTracker::setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  got_constraints_ = true;

  

   ROS_INFO("[DergbryanTracker]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}
//}

/*setReference()//{*/
const mrs_msgs::ReferenceSrvResponse::ConstPtr DergbryanTracker::setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  toggleHover(false);

  setGoal(cmd->reference.position.x, cmd->reference.position.y, cmd->reference.position.z, cmd->reference.heading, true);

  mrs_msgs::ReferenceSrvResponse res;
  res.success = true;
  res.message = "reference set";

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(res));
}
//}

/* //{ setGoal() */

//set absolute goal
void DergbryanTracker::setGoal(const double pos_x, const double pos_y, const double pos_z, const double heading, const bool use_heading) {

  double desired_heading = sradians::wrap(heading);
  {
  std::scoped_lock lock(mutex_goal_);

  goal_x_ = pos_x;
  goal_y_ = pos_y;
  goal_z_= pos_z;
  goal_heading_ = desired_heading;

  }
  // double desired_heading = sradians::wrap(heading);

  // auto mpc_x_heading = mrs_lib::get_mutexed(mutex_mpc_x_, mpc_x_heading_);

  // if (!use_heading) {
  //   desired_heading = mpc_x_heading(0, 0);
  // }

  trajectory_tracking_in_progress_ = false;
  timer_trajectory_tracking_.stop();

  // setSinglePointReference(pos_x, pos_y, pos_z, desired_heading);

  // publishDiagnostics();
}

//}

/* //{ setVelocityReference() */

const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr DergbryanTracker::setVelocityReference([
    [maybe_unused]] const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/*setTrajectoryReference()//{*/
const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr DergbryanTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  
  std::stringstream ss;

  auto [success, message, modified] = loadTrajectory(cmd->trajectory);

  //ROS_INFO_STREAM("setTrajectoryReference cmd->trajectory = \n" << cmd->trajectory);
  mrs_msgs::TrajectoryReferenceSrvResponse response;
  response.success  = success;
  response.message  = message;
  response.modified = modified;

  return mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr(new mrs_msgs::TrajectoryReferenceSrvResponse(response));
}
//}

//--------------------------------------------------------------------------//
void DergbryanTracker::trajectory_prediction_general(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd){
  
  // TODO: as this function is running the controller updated to a pre-stabilized system model, it would make sense to put these into a library so that ti can be used in both controller and tracker. Merge this with simulate se3 controller

  // TODO: check what is needed for computational time logging
  // Computational time:
  // // method Kelly:
  // start_pred_ = std::chrono::system_clock::now(); 
  // // method group A:
  // tictoc_stack.push(clock()); 
  // // last method of https://www.geeksforgeeks.org/measure-execution-time-with-high-precision-in-c-c/
  // auto start = std::chrono::high_resolution_clock::now(); 
  // // unsync the I/O of C and C++.
  // std::ios_base::sync_with_stdio(false);
  // // method https://answers.ros.org/question/166286/measure-codenode-running-time/: 
  // ros::WallTime WallTime_start, WallTime_end;
  // WallTime_start = ros::WallTime::now();
  // method: clock() CPU time: https://stackoverflow.com/questions/20167685/measuring-cpu-time-in-c/43800564
  std::clock_t c_start = std::clock();


  // | --------------------- initialize the state of the UAV (+PAYLOAD) --------------------- |
  mrs_msgs::UavState uav_state = uav_state_; // uav_state represents the locally defined predicted state
  // Opl - position load in global frame
  // Ovl - velocity load in global frame
  Eigen::Vector3d Opl = anchoring_pt_pose_position_;
  Eigen::Vector3d Ovl = anchoring_pt_lin_vel_;

  // --------------------------------------------------------------
  // |          load the control reference and estimates          | --> the reference is assumed constant over the prediction horizon
  // --------------------------------------------------------------
  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference
  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3);
  /* NOTE: replaced control_reference -> (input of controller) by position_cmd.use_ defined above in the update function*/
  if (position_cmd.use_position_vertical || position_cmd.use_position_horizontal) {
    // ROS_INFO_STREAM("position_cmd.use_position_horizontal = \n" << position_cmd.use_position_horizontal);
    // std::cout << "with std: "<< position_cmd.use_position_horizontal << std::endl;
    if (position_cmd.use_position_horizontal) { 
      Rp[0] = position_cmd.position.x;
      Rp[1] = position_cmd.position.y;
    } else {
      Rv[0] = 0;
      Rv[1] = 0;
    }

    if (position_cmd.use_position_vertical) {
      Rp[2] = position_cmd.position.z;
    } else {
      Rv[2] = 0;
    }
  }

  if (position_cmd.use_velocity_horizontal) {
    Rv[0] = position_cmd.velocity.x;
    Rv[1] = position_cmd.velocity.y;
  } else {
    Rv[0] = 0;
    Rv[1] = 0;
  }

  if (position_cmd.use_velocity_vertical) {
    Rv[2] = position_cmd.velocity.z;
  } else {
    Rv[2] = 0;
  }

  if (position_cmd.use_acceleration) {
    Ra << position_cmd.acceleration.x, position_cmd.acceleration.y, position_cmd.acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }
  /* test streaming the references Rp, Rv, Ra when the uav is moving.*/
  // ROS_INFO_STREAM("Rp = \n" << Rp);
  // ROS_INFO_STREAM("Rv = \n" << Rv);
  // ROS_INFO_STREAM("Ra = \n" << Ra);
 
  /*TODO: where to print Rw? When it is set*/
  // ROS_INFO_STREAM("Rw = \n" << Rw);


  // | --------------------------LOAD--------------------------|
  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------
  // TODO: Rpl and Rvl not defined, nor used. Should be the uav ref cable length down. Not used in actual error of Pandolfo.
  // Rpl - position reference load in global frame
  // Rvl - velocity reference load in global frame
  Eigen::Vector3d Rpl = Eigen::Vector3d::Zero(3); // TODO: bryan currently not used as we send reference of UAV
    
  // TODO Raphael : For me it's useless we can delete this as no integral action is predicted. I would have deleted it but it seems you added it for a reason ???
  // | --------------------- initialize body and world integrals --------------------- |
  /* NOTE: this part is added by Bryan, was originally in the controllers activate function */ 
  // Ib_b_[0] = -last_attitude_cmd->disturbance_bx_b;
  // Ib_b_[1] = -last_attitude_cmd->disturbance_by_b;

  // Iw_w_[0] = -last_attitude_cmd->disturbance_wx_w;
  // Iw_w_[1] = -last_attitude_cmd->disturbance_wy_w;

    // ROS_INFO(
    //     "[DergbryanTracker]: setting the mass difference and integrals from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: "
    //     "%.2f, %.2f N",
    //     uav_mass_difference_, Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
  /* NOTE: TILL HERE*/


  // ----------------------------------------------------
  // |          prepare variables for publishers        |
  // ----------------------------------------------------
  predicted_poses_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_poses_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_velocities_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_velocities_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_accelerations_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_accelerations_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_thrust_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_thrust_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_attituderate_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_attituderate_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_des_attituderate_out_.header.stamp = uav_state_.header.stamp; //ros::Time::now();
  predicted_des_attituderate_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_tiltangle_out_.header.stamp = uav_state_.header.stamp;//ros::Time::now();
  predicted_tiltangle_out_.header.frame_id = uav_state_.header.frame_id;
  // | -------------------1UAV+PAYLOAD----------------- |
  predicted_load_poses_out_.header.stamp = uav_state_.header.stamp; 
  predicted_load_poses_out_.header.frame_id = uav_state_.header.frame_id; 
  predicted_load_velocities_out_.header.stamp = uav_state_.header.stamp; 
  predicted_load_velocities_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_load_accelerations_out_.header.stamp = uav_state_.header.stamp;
  predicted_load_accelerations_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_phi_out_.header.stamp = uav_state_.header.stamp; 
  predicted_theta_out_.header.stamp = uav_state_.header.stamp;
  predicted_phi_dot_out_.header.stamp = uav_state_.header.stamp; 
  predicted_theta_dot_out_.header.stamp = uav_state_.header.stamp; 
  predicted_phi_dot_dot_out_.header.stamp = uav_state_.header.stamp;
  predicted_theta_dot_dot_out_.header.stamp = uav_state_.header.stamp;
  predicted_load_position_errors_out_.header.stamp=uav_state.header.stamp;
  predicted_swing_angle_out_.header.stamp = uav_state_.header.stamp;
  predicted_swing_angle_out_.header.frame_id = uav_state_.header.frame_id;
  predicted_Tc_out_.header.stamp = uav_state_.header.stamp;
  predicted_Tc_out_.header.frame_id = uav_state_.header.frame_id;
  geometry_msgs::Pose predicted_pose;
  geometry_msgs::Pose predicted_vel;
  geometry_msgs::Pose predicted_acc;
  geometry_msgs::Pose predicted_thrust; 
  geometry_msgs::Pose predicted_thrust_norm; 
  geometry_msgs::Pose predicted_attituderate;
  geometry_msgs::Pose predicted_des_attituderate;
  geometry_msgs::Pose predicted_tilt_angle; 
  // | -------------------1UAV+PAYLOAD----------------- |
  geometry_msgs::Pose predicted_load_pose;
  geometry_msgs::Pose predicted_load_vel;
  geometry_msgs::Pose predicted_load_acc;
  geometry_msgs::Pose predicted_load_theta;
  geometry_msgs::Pose predicted_load_phi;
  geometry_msgs::Pose predicted_load_dot_theta;
  geometry_msgs::Pose predicted_load_dot_phi;
  geometry_msgs::Pose predicted_load_dot_dot_theta;
  geometry_msgs::Pose predicted_load_dot_dot_phi;
  geometry_msgs::Pose predicted_q_state_dot_dot_uav;
  geometry_msgs::Pose predicted_q_state_dot_dot_load;

  //geometry_msgs::Pose force_to_publish;
  //geometry_msgs::Pose predicted_load_position_errors_to_publish;
  geometry_msgs::Pose predicted_swing_angle;
  geometry_msgs::Pose predicted_Tc;

  // ----------------------------------------------------
  // |          prepare variables for predictions        |
  // ----------------------------------------------------
  double theta_load_cable;
  double phi_load_cable;  
  double theta_dot_load_cable;
  double phi_dot_load_cable;  
  double theta_dot_dot_load_cable;
  double phi_dot_dot_load_cable;
  double swing_angle;
  double Tc; //Tension in the cable
  Eigen::Matrix3d R;
  //Eigen::Matrix3d Rdot;
  Eigen::Matrix3d skew_Ow;
  Eigen::Matrix3d skew_Ow_des;
  Eigen::Vector3d attitude_rate_pred;
  Eigen::Vector3d attitude_rate_pred_prev;
  Eigen::Vector3d desired_attitude_rate_pred;
  Eigen::Vector3d torque_pred;
  Eigen::Vector3d attitude_acceleration_pred;

  // ----------------------------------------------------
  // |          start trajectory prediction loop        |
  // ----------------------------------------------------
  for (int i = 0; i < _num_pred_samples_; i++) {
    if(!erg_predictions_theta_trusted_){
      ROS_INFO_STREAM("we break since erg_predictions_theta_trusted_ = false \n");
      break;
    }
    if(i == 0){  // first iteration
      // TODO: generalize prediction function such that we can choose in which initital state we start to predict (for discrete time ERG)
      // TODO: why have some defined Eigen variables and some ROS? Clean it.
      // Use initial conditions for first iteration:
      // Pose:
      // Position:
      predicted_pose.position.x = uav_state.pose.position.x;
      predicted_pose.position.y = uav_state.pose.position.y;
      predicted_pose.position.z = uav_state.pose.position.z;
      // R - current uav attitude
      R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);
      //R = Eigen::Matrix3d::Identity(3, 3); // fake attitude // TODO: needed?
      // Velocity:
      // Linear:
      predicted_vel.position.x = uav_state.velocity.linear.x;
      predicted_vel.position.y = uav_state.velocity.linear.y;
      predicted_vel.position.z = uav_state.velocity.linear.z;
      // Ow - UAV angular rate:
      Eigen::Vector3d attitude_rate_pred(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z); // actual predicted attitude rate
      if (_predicitons_use_body_inertia_){
        Eigen::Vector3d attitude_rate_pred_prev(uav_state_prev_.velocity.angular.x, uav_state_prev_.velocity.angular.y, uav_state_prev_.velocity.angular.z);
      }
      // LOAD
      if(_type_of_system_=="1uav_payload" && payload_spawned_){
        // Encode the state of the load into the custom quantities, to be published as pose array.
        // Position:
        predicted_load_pose.position.x = Opl[0];
        predicted_load_pose.position.y = Opl[1];
        predicted_load_pose.position.z = Opl[2];
        // Velocity:
        predicted_load_vel.position.x = Ovl[0];
        predicted_load_vel.position.y = Ovl[1];
        predicted_load_vel.position.z = Ovl[2];
        // Corrected IK to get angles phi and theta used in prediction model thesis Raphael:
        // Cable angles:
        theta_load_cable = asin((Opl[0] - uav_state.pose.position.x)/_cable_length_); 
        phi_load_cable = asin((Opl[1] - uav_state.pose.position.y)/(_cable_length_*cos(theta_load_cable)));
        // Cable angular rates:
        theta_dot_load_cable = pow(1.0 - pow((Opl[0] - uav_state.pose.position.x)/_cable_length_,2.0),-1.0/2.0) 
        * ((Ovl[0] - uav_state.velocity.linear.x)/_cable_length_);
        phi_dot_load_cable = ((Ovl[1] - uav_state.velocity.linear.y)*cos(theta_load_cable) + (Opl[1] - uav_state.pose.position.y)*theta_dot_load_cable*sin(theta_load_cable))/(_cable_length_*pow(cos(theta_load_cable),2.0)*pow(1.0-pow((Opl[1] - uav_state.pose.position.y)/(_cable_length_*cos(theta_load_cable)),2),0.5));

        //Custom quantity to publish. TODO change it to a double instead of a pose.
        predicted_load_theta.position.x = theta_load_cable;
        predicted_load_phi.position.x = phi_load_cable;
        predicted_load_dot_theta.position.x = theta_dot_load_cable;
        predicted_load_dot_phi.position.x = phi_dot_load_cable;
      }
    } 
    else{ // after first iteration
      // Discrete trajectory prediction using the Euler formula's
      // TODO: in control predictions define predicted_acc also via uav_state
      // TODO: why are we suing two variables to update the states? Make it simpler.
      uav_state.velocity.linear.x = uav_state.velocity.linear.x + predicted_acc.position.x*_prediction_dt_;
      predicted_vel.position.x = uav_state.velocity.linear.x;

      uav_state.velocity.linear.y = uav_state.velocity.linear.y + predicted_acc.position.y*_prediction_dt_;
      predicted_vel.position.y = uav_state.velocity.linear.y;

      uav_state.velocity.linear.z = uav_state.velocity.linear.z + predicted_acc.position.z*_prediction_dt_;
      predicted_vel.position.z = uav_state.velocity.linear.z;

      uav_state.pose.position.x = uav_state.pose.position.x + uav_state.velocity.linear.x*_prediction_dt_;
      predicted_pose.position.x = uav_state.pose.position.x;

      uav_state.pose.position.y = uav_state.pose.position.y + uav_state.velocity.linear.y*_prediction_dt_;
      predicted_pose.position.y = uav_state.pose.position.y;

      uav_state.pose.position.z = uav_state.pose.position.z + uav_state.velocity.linear.z*_prediction_dt_;
      predicted_pose.position.z = uav_state.pose.position.z;

      // LOAD:
      if(_type_of_system_=="1uav_payload" && payload_spawned_){
        _predicitons_use_body_inertia_=0; //For now don't use this with the payload. TODO: should not be initialized here but in init function.
        // Integrate the acceleration angles of the model, to find the velocity and position of these. 
        theta_dot_load_cable = theta_dot_load_cable + theta_dot_dot_load_cable*_prediction_dt_;
        phi_dot_load_cable = phi_dot_load_cable + phi_dot_dot_load_cable*_prediction_dt_;
        theta_load_cable = theta_load_cable + theta_dot_load_cable*_prediction_dt_;
        phi_load_cable = phi_load_cable + phi_dot_load_cable*_prediction_dt_;
        // ROS_INFO_STREAM("theta_load_cable  = \n" << theta_load_cable);
        // ROS_INFO_STREAM("phi_load_cable  = \n" << phi_load_cable);
        // ROS_INFO_STREAM("theta_dot_load_cable  = \n" << theta_dot_load_cable);
        // ROS_INFO_STREAM("phi_dot_load_cable  = \n" << phi_dot_load_cable);
        // ROS_INFO_STREAM("theta_dot_dot_load_cable  = \n" << theta_dot_dot_load_cable);
        // ROS_INFO_STREAM("phi_dot_dot_load_cable  = \n" << phi_dot_dot_load_cable);
        predicted_load_theta.position.x = theta_load_cable;
        predicted_load_phi.position.x = phi_load_cable;
        predicted_load_dot_theta.position.x = theta_dot_load_cable;
        predicted_load_dot_phi.position.x = phi_dot_load_cable;
        predicted_load_dot_dot_theta.position.x = theta_dot_dot_load_cable;
        predicted_load_dot_dot_phi.position.x = phi_dot_dot_load_cable;
     
        // FK, to get the position of the payload in Cartesian coordinates instead of the minimal angles phi and theta, see Raphael thesis:
        Opl[0] = uav_state.pose.position.x + sin(theta_load_cable)*_cable_length_; 
        Opl[1] = uav_state.pose.position.y + sin(phi_load_cable)*cos(theta_load_cable)*_cable_length_; 
        Opl[2] = uav_state.pose.position.z - _cable_length_*cos(phi_load_cable)*cos(theta_load_cable);

        predicted_load_pose.position.x = Opl[0];
        predicted_load_pose.position.y = Opl[1];
        predicted_load_pose.position.z = Opl[2];

        Ovl[0] = cos(theta_load_cable)*_cable_length_*theta_dot_load_cable + uav_state.velocity.linear.x;  
        Ovl[1] = cos(phi_load_cable)*cos(theta_load_cable)*_cable_length_*phi_dot_load_cable -_cable_length_*sin(phi_load_cable)*sin(theta_load_cable) * theta_dot_load_cable
        + uav_state.velocity.linear.y; 
        Ovl[2] = uav_state.velocity.linear.z + _cable_length_*(sin(phi_load_cable)*phi_dot_load_cable*cos(theta_load_cable) 
        + cos(phi_load_cable)*sin(theta_load_cable)*theta_dot_load_cable);
        
        predicted_load_vel.position.x = Ovl[0];
        predicted_load_vel.position.y = Ovl[1];
        predicted_load_vel.position.z = Ovl[2];
        // Acceleration is not computed as not needed. If want it to monitor its values for debuging reasons, must be put here : 

        //compute swing angle prediction // TODO: why not done?

      }

      // UAV Attitude:
      if (!_predicitons_use_body_inertia_){
        // if we ignore the body rate inertial dynamics:
        // assumption made (inner loop is infinitely fast): desired_attitude_rate_pred = attitude_rate_pred
        skew_Ow_des << 0.0     , -desired_attitude_rate_pred(2), desired_attitude_rate_pred(1),
                  desired_attitude_rate_pred(2) , 0.0,       -desired_attitude_rate_pred(0),
                  -desired_attitude_rate_pred(1), desired_attitude_rate_pred(0),  0.0;
        
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
        R = (I + skew_Ow_des*_prediction_dt_ + 1.0/2.0*skew_Ow_des*_prediction_dt_*skew_Ow_des*_prediction_dt_ + 1.0/6.0*skew_Ow_des*_prediction_dt_*skew_Ow_des*_prediction_dt_*skew_Ow_des*_prediction_dt_)*R;
      }
      else{
        // if we include the body rate inertial dynamics:
        attitude_rate_pred = attitude_rate_pred + attitude_acceleration_pred*_prediction_dt_;
        skew_Ow << 0.0     , -attitude_rate_pred(2), attitude_rate_pred(1),
                  attitude_rate_pred(2) , 0.0,       -attitude_rate_pred(0),
                  -attitude_rate_pred(1), attitude_rate_pred(0),  0.0;
        // if (i==5){
        //   ROS_INFO_STREAM("skew_Ow (i=5) = \n" << skew_Ow);
        // }
        // ensure Ow is updated correctly in predictions....
        //; // or add - (equivalent to transpose?) UNUSED
        //skew_Ow = -skew_Ow;
        // R = R + Rdot*dt; with Rdot = skew_Ow*R THIS IS WRONG 
        // --> CORRECT to use exponential map like below (= Taylor series)
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
        R = (I + skew_Ow*_prediction_dt_ + 1.0/2.0*skew_Ow*_prediction_dt_*skew_Ow*_prediction_dt_ + 1.0/6.0*skew_Ow*_prediction_dt_*skew_Ow*_prediction_dt_*skew_Ow*_prediction_dt_)*R;
        // for publishing:
        predicted_attituderate.position.x = attitude_rate_pred(0,0);
        predicted_attituderate.position.y = attitude_rate_pred(1,0);
        predicted_attituderate.position.z = attitude_rate_pred(2,0);
        predicted_attituderate_out_.poses.push_back(predicted_attituderate);
      }
      // debug UAV attitude:
      //ROS_INFO_STREAM("R(2) = \n" << R.col(2));
      Eigen::Vector3d temp = R.col(2);
      double normR2 = temp(0)*temp(0) + temp(1)*temp(1) + temp(2)*temp(2);
      if ((normR2 >=1.02) || (normR2 <=0.98))
      {
        ROS_INFO_STREAM("something is wrong: normR2 too far from 1= " << normR2);
      }
      
      // Update uav_state.pose.orientation from the current rotation matrix (used later for parasitic heading rate compensation)
      uav_state.pose.orientation = mrs_lib::AttitudeConverter(R);
      // if (i == 5 || i == 50) {
      //   ROS_INFO_STREAM("uav_state.pose.orientation = " << uav_state.pose.orientation); // a quaternion
      // }
    } 


    predicted_poses_out_.poses.push_back(predicted_pose);
    predicted_velocities_out_.poses.push_back(predicted_vel);
    
    if(_type_of_system_=="1uav_payload"){
      predicted_load_poses_out_.poses.push_back(predicted_load_pose);
      predicted_load_velocities_out_.poses.push_back(predicted_load_vel);
      predicted_phi_out_.poses.push_back(predicted_load_phi);
      predicted_theta_out_.poses.push_back(predicted_load_theta);
      predicted_phi_dot_out_.poses.push_back(predicted_load_dot_phi);
      predicted_theta_dot_out_.poses.push_back(predicted_load_dot_theta);
      predicted_phi_dot_dot_out_.poses.push_back(predicted_load_dot_dot_phi);
      predicted_theta_dot_dot_out_.poses.push_back(predicted_load_dot_dot_theta);
      predicted_load_accelerations_out_.poses.push_back(predicted_load_acc);
    }

    // |---------------------- Controller part ---------------------------|
    double thrust_force; // The projection of the total thrust vector
    double theta;
    if(_type_of_system_=="1uav_payload" && payload_spawned_){
      // _predicitons_use_body_inertia_=0; // For now don't predict the attitude dynamic. => w_pred=wd_pref
      Eigen::Vector3d Payloadposition_vector; // as the ComputeSe3CopyController function needs a Vector3d for the load pose (and not a pose type).
      Payloadposition_vector[0]=Opl[0];
      Payloadposition_vector[1]=Opl[1];
      Payloadposition_vector[2]=Opl[2];
      std::tie(thrust_force,desired_attitude_rate_pred,theta)=ComputeSe3CopyController(uav_state, R, Payloadposition_vector , Ovl ,position_cmd , Rp , Rv , Rpl , Ra );
    } 
    else{ // As there is no load, the arguments related to the payload, passed to the ComputeSe3CopyController function are null vector. This is to keep the same number of argument for both cases, and having one function only. 
      std::tie(thrust_force,desired_attitude_rate_pred,theta)=ComputeSe3CopyController(uav_state, R, Eigen::Vector3d::Zero(3) , Eigen::Vector3d::Zero(3) ,position_cmd , Rp , Rv , Eigen::Vector3d::Zero(3) , Eigen::Vector3d::Zero(3) );
    }
      // Publish predicted tilt angle,hrust and attitude rate that were returned by the controller function//
      predicted_tilt_angle.position.x = theta; // change later to a non vec type
      predicted_tiltangle_out_.poses.push_back(predicted_tilt_angle);

      predicted_thrust_norm.position.x = thrust_force; // change later to a non vec type; Why not f?
      predicted_thrust_out_.poses.push_back(predicted_thrust_norm);

      predicted_des_attituderate.position.x = desired_attitude_rate_pred(0,0);
      predicted_des_attituderate.position.y = desired_attitude_rate_pred(1,0);
      predicted_des_attituderate.position.z = desired_attitude_rate_pred(2,0);
      predicted_des_attituderate_out_.poses.push_back(predicted_des_attituderate);

    //------------------------Equations of motion part----------------------//
    if(_type_of_system_=="1uav_payload" && payload_spawned_){ //1UAV and payload
      double total_mass = uav_mass_ + _load_mass_;
      // ROS_INFO_STREAM("total_mass in prediction loop \n"<<total_mass);

      // Mass matrix:
      Eigen::MatrixXd M_matrix = Eigen::MatrixXd(5, 5);
      M_matrix(0,0) = total_mass; M_matrix(0,1) = 0.0; M_matrix(0,2) = 0.0; 
      M_matrix(0,3) = 0.0; M_matrix(0,4) = _cable_length_*_load_mass_*cos(theta_load_cable);
      
      M_matrix(1,0) = 0.0; M_matrix(1,1) = total_mass; M_matrix(1,2) = 0.0; 
      M_matrix(1,3) = _cable_length_*_load_mass_*cos(phi_load_cable)*cos(theta_load_cable); 
      M_matrix(1,4) = -_cable_length_*_load_mass_*sin(phi_load_cable)*sin(theta_load_cable);
      
      M_matrix(2,0) = 0.0; M_matrix(2,1) = 0.0; M_matrix(2,2) = total_mass; 
      M_matrix(2,3) = _cable_length_*_load_mass_*cos(theta_load_cable)*sin(phi_load_cable); 
      M_matrix(2,4) = _cable_length_*_load_mass_*cos(phi_load_cable)*sin(theta_load_cable);
      
      M_matrix(3,0) = 0.0; M_matrix(3,1) = _cable_length_*_load_mass_*cos(phi_load_cable)*cos(theta_load_cable); 
      M_matrix(3,2) = _cable_length_*_load_mass_*sin(phi_load_cable)*cos(theta_load_cable); 
      M_matrix(3,3) = pow(_cable_length_,2.0)*_load_mass_*(cos(theta_load_cable))*(cos(theta_load_cable)); M_matrix(3,4) = 0.0;
      
      M_matrix(4,0) = _cable_length_*_load_mass_*cos(theta_load_cable); 
      M_matrix(4,1) = -_cable_length_*_load_mass_*sin(phi_load_cable)*sin(theta_load_cable); 
      M_matrix(4,2) = _cable_length_*_load_mass_*cos(phi_load_cable)*sin(theta_load_cable); 
      M_matrix(4,3) = 0.0; M_matrix(4,4) = pow(_cable_length_,2.0)*_load_mass_;
      // ROS_INFO_STREAM("M_matrix = \n" << M_matrix);
      // ROS_INFO_STREAM("M_matrix inverse = \n" << M_matrix.inverse());

      // Coriolis matrix:
      Eigen::MatrixXd C_matrix = Eigen::MatrixXd(5, 5);
      C_matrix(0,0) = 0.0; C_matrix(0,1) = 0.0; C_matrix(0,2) = 0.0; C_matrix(0,3) = 0.0; 
      C_matrix(0,4) = -_cable_length_*theta_dot_load_cable*_load_mass_*sin(theta_load_cable);

      C_matrix(1,0) = 0.0; C_matrix(1,1) = 0.0; C_matrix(1,2) = 0.0; 
      C_matrix(1,3) = -_cable_length_*_load_mass_*(phi_dot_load_cable*sin(phi_load_cable)*cos(theta_load_cable) 
      + theta_dot_load_cable*cos(phi_load_cable)*sin(theta_load_cable)); 
      C_matrix(1,4) = -_cable_length_*_load_mass_*(phi_dot_load_cable*cos(phi_load_cable)*sin(theta_load_cable) 
      + theta_dot_load_cable*sin(phi_load_cable)*cos(theta_load_cable));

      C_matrix(2,0) = 0.0; C_matrix(2,1) = 0.0; C_matrix(2,2) = 0.0; 
      C_matrix(2,3) = _cable_length_*_load_mass_*(phi_dot_load_cable*cos(phi_load_cable)*cos(theta_load_cable) 
      - theta_dot_load_cable*sin(phi_load_cable)*sin(theta_load_cable)); 
      C_matrix(2,4) = _cable_length_*_load_mass_*(theta_dot_load_cable*cos(phi_load_cable)*cos(theta_load_cable) 
      - phi_dot_load_cable*sin(phi_load_cable)*sin(theta_load_cable));

      C_matrix(3,0) = 0.0; C_matrix(3,1) = 0.0; C_matrix(3,2) = 0.0; 
      C_matrix(3,3) = (-1.0/2.0)*pow(_cable_length_,2.0)*theta_dot_load_cable*_load_mass_*sin(2.0*theta_load_cable); 
      C_matrix(3,4) = (-1.0/2.0)*pow(_cable_length_,2.0)*phi_dot_load_cable*_load_mass_*sin(2.0*theta_load_cable);
      
      C_matrix(4,0) = 0.0; C_matrix(4,1) = 0.0; C_matrix(4,2) = 0.0; 
      C_matrix(4,3) = (1.0/2.0)*pow(_cable_length_,2.0)*phi_dot_load_cable*_load_mass_*sin(2.0*theta_load_cable); C_matrix(4,4) = 0.0;
      // ROS_INFO_STREAM("C_matrix = \n" << C_matrix);

      // Gravity vector:
      Eigen::MatrixXd G_vector = Eigen::MatrixXd(5, 1);
      G_vector(0,0) = 0.0; G_vector(1,0) = 0.0; G_vector(2,0) = (total_mass)*common_handlers_->g; // g here is positif
      G_vector(3,0) = _cable_length_*common_handlers_->g*_load_mass_*cos(theta_load_cable)*sin(phi_load_cable); 
      G_vector(4,0) = _cable_length_*common_handlers_->g*_load_mass_*cos(phi_load_cable)*sin(theta_load_cable);
      // ROS_INFO_STREAM("G_vector = \n" << G_vector);

      // Damping (natural) matrix
      // TODO: currently we assume no natural damping
      Eigen::MatrixXd D_matrix = Eigen::MatrixXd::Zero(5, 5); // Adding damping force created by air and non perfect joint.
      D_matrix(3,3)=0.0;//0.15;
      D_matrix(4,4)=0.0;//0.15;
      //ROS_INFO_STREAM("D_matrix = \n" << D_matrix);

      // Generalized force vector:
      Eigen::MatrixXd u_vector = Eigen::MatrixXd(5, 1);
      Eigen::Vector3d f = thrust_force*R.col(2); // applied thrust is the thrust vector projected on z_B. (= vertical up axis of the uav)
      u_vector(0,0) = f[0]; u_vector(1,0) = f[1];
      u_vector(2,0) = f[2]; u_vector(3,0) = 0.0; u_vector(4,0) = 0.0;
      // ROS_INFO_STREAM("u = \n" << u_vector);

      // Generalized coordinates vector:
      Eigen::MatrixXd q_state = Eigen::MatrixXd(5, 1);
      Eigen::MatrixXd q_state_dot = Eigen::MatrixXd(5, 1);
      Eigen::MatrixXd q_state_dot_dot = Eigen::MatrixXd(5, 1);
      q_state(0,0) = uav_state.pose.position.x; q_state(1,0) = uav_state.pose.position.y; 
      q_state(2,0) = uav_state.pose.position.z; q_state(3,0) = phi_load_cable; q_state(4,0) = theta_load_cable;
      // ROS_INFO_STREAM("q_state = \n" << q_state);
      q_state_dot(0,0) = uav_state.velocity.linear.x; q_state_dot(1,0) = uav_state.velocity.linear.y; 
      q_state_dot(2,0) = uav_state.velocity.linear.z; q_state_dot(3,0) = phi_dot_load_cable; 
      q_state_dot(4,0) = theta_dot_load_cable;
      // ROS_INFO_STREAM("q_state_dot = \n" << q_state_dot);

      // Model update:
      q_state_dot_dot = (M_matrix.inverse())*(u_vector - (C_matrix + D_matrix)*q_state_dot - G_vector);
      // ROS_INFO_STREAM("q_state_dot_dot = \n" << q_state_dot_dot);

      Eigen::Vector3d acceleration_uav;

      acceleration_uav[0] = q_state_dot_dot(0,0);
      acceleration_uav[1] = q_state_dot_dot(1,0);
      acceleration_uav[2] = q_state_dot_dot(2,0);
      predicted_acc.position.x = acceleration_uav[0];
      predicted_acc.position.y = acceleration_uav[1];
      predicted_acc.position.z = acceleration_uav[2];

      phi_dot_dot_load_cable = q_state_dot_dot(3,0);
      theta_dot_dot_load_cable= q_state_dot_dot(4,0);

      predicted_q_state_dot_dot_uav.position.x = acceleration_uav[0];
      predicted_q_state_dot_dot_uav.position.y = acceleration_uav[1];
      predicted_q_state_dot_dot_uav.position.z = acceleration_uav[2];
      predicted_q_state_dot_dot_load.position.x = phi_dot_dot_load_cable;
      predicted_q_state_dot_dot_load.position.y = theta_dot_dot_load_cable;

      // Compute swing angle predictions:
      Eigen::Vector3d uav_position(uav_state.pose.position.x,uav_state.pose.position.y,uav_state.pose.position.z); //get a vector of the UAV position to ease the following computations.
      Eigen::Vector3d mu; //Unit vector indicating cable orientation.
      Eigen::Vector3d zB; //unit vector z_B of the UAV body frame
      mu = (uav_position-Opl).normalized();
      // ROS_INFO_STREAM("mu after norm"<<mu);
      zB = R.col(2);
      // ROS_INFO_STREAM("zB"<<zB);
      swing_angle = acos((mu.dot(zB))/(mu.norm()*zB.norm())); //Compute this swing angle (positive only, between 0 and pi)
      // ROS_INFO_STREAM("swing_angle"<<swing_angle);
      predicted_swing_angle.position.x = swing_angle;
      predicted_swing_angle_out_.poses.push_back(predicted_swing_angle); 

      // Compute cable tension predictions:
      double mq = uav_mass_; // Mass of the UAV
      Tc = (-mq*acceleration_uav + mq*(Eigen::Vector3d(0, 0, -common_handlers_->g)) + f).dot(mu.transpose());
      predicted_Tc.position.x = Tc;
      predicted_Tc_out_.poses.push_back(predicted_Tc); 
    }
    else{ // 1UAV no payload equations
      // TODO: the comme,nt below seems not correct as actually Raphael did same as Bryan: proejct total thrust force on zB.
      // Rapahel: Bryan used thrust_force in eom instead of f as I did. what does it changes??? Thrust force takes attitude into action when my predictions assume that the attitude has been controller perfectly.
      double total_mass = uav_mass_;
      Eigen::Vector3d acceleration_uav = 1.0/total_mass * (thrust_force*R.col(2) + total_mass * (Eigen::Vector3d(0, 0, -common_handlers_->g)));
      // Eigen::Vector3d acceleration_uav = 1.0/total_mass * (thrust_force*Rd.col(2) + total_mass * (Eigen::Vector3d(0, 0, -common_handlers_->g))); --> do not use desired force
      predicted_acc.position.x = acceleration_uav[0];
      predicted_acc.position.y = acceleration_uav[1];
      predicted_acc.position.z = acceleration_uav[2];
      predicted_accelerations_out_.poses.push_back(predicted_acc);
    }

    // Attitude dynamics:
    if (!_predicitons_use_body_inertia_)// if we ignore the body rate inertial dynamics:
    { 
      // do nothing
    }
    else { // if we include the body rate inertial dynamics:
      // Compute the predicted torque:
      // See https://docs.px4.io/master/en/flight_stack/controller_diagrams.html, https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html#rate-controller
      // P-action:
      Eigen::Array3d  Kp_tau = Eigen::Array3d::Zero(3);
      double kp_tau = 0.15; //0.175; //0.15;//3.50; //1.50;//0.15 (good);//0.10; (way too much second peak in predicted thrust)//0.50 (less good); //0.20 (ok)
      Kp_tau[0] = kp_tau;
      Kp_tau[1] = kp_tau;
      Kp_tau[2] = kp_tau/1.0;
      Eigen::Vector3d tau_P_error = desired_attitude_rate_pred - attitude_rate_pred;
      // D-action:
      Eigen::Array3d  Kd_tau = Eigen::Array3d::Zero(3);
      double kd_tau = 0.0; //0.01; //0.00001; //0.01;
      Kd_tau[0] = kd_tau;
      Kd_tau[1] = kd_tau;
      Kd_tau[2] = kd_tau/10.0;
      Eigen::Vector3d tau_D_error = -(attitude_rate_pred - attitude_rate_pred_prev)/_prediction_dt_; // see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
      attitude_rate_pred_prev = attitude_rate_pred; // update prev for use in next prediction iteration
    
      // P+D-action:
      torque_pred = Kp_tau * tau_P_error.array() + Kd_tau * tau_D_error.array(); //TODO add derivative action
      // if (i==0 || i==1){
      //   //ROS_INFO_STREAM("term = \n" << torque_pred);
      // }
      
      Eigen::Matrix3d J = Eigen::Matrix3d::Zero(3,3); // body inertia UAV
      // values taken from ~/mrs_workspace/src/simulation/ros_packages/mrs_simulation/models/mrs_robots_description/urdf/f450.xacro
      double inertia_body_radius = 0.20; // [m]
      double inertia_body_height = 0.05; // [m]
      double Jxx = uav_mass_ * (3.0 * inertia_body_radius * inertia_body_radius + inertia_body_height * inertia_body_height) / 12.0;
      double Jyy = uav_mass_ * (3.0 * inertia_body_radius * inertia_body_radius + inertia_body_height * inertia_body_height) / 12.0;
      double Jzz = uav_mass_ * inertia_body_radius * inertia_body_radius / 2.0;
      double scale_inertia = 1.0; //1.0 for no scaling
      J(0,0) = Jxx*scale_inertia;
      J(1,1) = Jyy*scale_inertia;
      J(2,2) = Jzz*scale_inertia;
      Eigen::Matrix3d skew_attitude_rate_pred; 
      skew_attitude_rate_pred << 0.0    , -attitude_rate_pred(2), attitude_rate_pred(1),
                                attitude_rate_pred(2) , 0.0,     -attitude_rate_pred(0),
                                -attitude_rate_pred(1),  attitude_rate_pred(0),     0.0;
      attitude_acceleration_pred = J.inverse()*(-skew_attitude_rate_pred*J*attitude_rate_pred + torque_pred);
    }
  } // end for loop prediction

  // Computational time:
  // // method Kelly:
  // end_pred_ = std::chrono::system_clock::now();
  // ComputationalTime_pred_ = end_pred_ - start_pred_;
  // //ComputationalTime_msg_.trajectory_predictions = ComputationalTime_pred_.count(); 
  // // method group A:
  // //ComputationalTime_msg_.trajectory_predictions = (double)(clock()- tictoc_stack.top())/CLOCKS_PER_SEC; 
  // //ROS_INFO_STREAM("Prediction calculation took = \n "<< (double)(clock()- tictoc_stack.top())/CLOCKS_PER_SEC << "seconds.");
  // tictoc_stack.pop();
  // // last method of: https://www.geeksforgeeks.org/measure-execution-time-with-high-precision-in-c-c/:
  // auto end = std::chrono::high_resolution_clock::now();
  // double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  // time_taken *= 1e-9;
  // //ComputationalTime_msg_.trajectory_predictions = time_taken;
  // // method: https://answers.ros.org/question/166286/measure-codenode-running-time/
  // WallTime_end = ros::WallTime::now();
  // //ComputationalTime_msg_.trajectory_predictions = (WallTime_end - WallTime_start).toNSec() * 1e-9;
  // // method: clock() CPU time, https://stackoverflow.com/questions/20167685/measuring-cpu-time-in-c/43800564:
  // std::clock_t c_end = std::clock();
  // ComputationalTime_msg_.trajectory_predictions = (c_end-c_start) / (double)CLOCKS_PER_SEC;
  std::clock_t c_end = std::clock();
  //std::setprecision(2);
  double part = 1000.0*(c_end-c_start) / (double)CLOCKS_PER_SEC;
  ComputationalTime_msg_.parts.push_back(part);
  std::string name_part = "trajectory prediction";
  ComputationalTime_msg_.name_parts.push_back(name_part);

  // Publishers:
  // avoid publishing all trajectory predictions since not required for control, only useful for post-analysis:

  if(_enable_trajectory_pub_ || _enable_visualization_){// TODO change visualization with real topics communicated by uavs, not this topic
    try {
        predicted_pose_publisher_.publish(predicted_poses_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_pose_publisher_.getTopic().c_str());
    }
  }
  
  // UAV
  if (_enable_trajectory_pub_) {
    try {
      predicted_vel_publisher_.publish(predicted_velocities_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_vel_publisher_.getTopic().c_str());
    }
    try {
      predicted_acc_publisher_.publish(predicted_accelerations_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_acc_publisher_.getTopic().c_str());
    }
    try {
      predicted_thrust_publisher_.publish(predicted_thrust_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_thrust_publisher_.getTopic().c_str());
    }
    try {
      predicted_attrate_publisher_.publish(predicted_attituderate_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_attrate_publisher_.getTopic().c_str());
    }
    try {
      predicted_des_attrate_publisher_.publish(predicted_des_attituderate_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_des_attrate_publisher_.getTopic().c_str());
    }
    try {
      predicted_tiltangle_publisher_.publish(predicted_tiltangle_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_tiltangle_publisher_.getTopic().c_str());
    }
  }

  // -------------1UAV+payload-----------------------//
  if(_type_of_system_=="1uav_payload"&& payload_spawned_){
    if (_enable_trajectory_pub_) {
      try {
        predicted_load_pose_publisher_.publish(predicted_load_poses_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_load_pose_publisher_.getTopic().c_str());
      }
      try {
        predicted_load_vel_publisher_.publish(predicted_load_velocities_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_load_vel_publisher_.getTopic().c_str());
      }
      try {
        predicted_load_acc_publisher_.publish(predicted_load_accelerations_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_load_acc_publisher_.getTopic().c_str());
      }
      try {
        predicted_swing_angle_publisher_.publish(predicted_swing_angle_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_swing_angle_publisher_.getTopic().c_str());
      }
      try {
        predicted_Tc_publisher_.publish(predicted_Tc_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_Tc_publisher_.getTopic().c_str());
      }
      try {
        predicted_phi_publisher_.publish(predicted_phi_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_phi_publisher_.getTopic().c_str());
      }
      try {
        predicted_theta_publisher_.publish(predicted_theta_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_theta_publisher_.getTopic().c_str());
      }
      try {
        predicted_phi_dot_publisher_.publish(predicted_phi_dot_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_phi_dot_publisher_.getTopic().c_str());
      }
      try {
        predicted_theta_dot_publisher_.publish(predicted_theta_dot_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_theta_dot_publisher_.getTopic().c_str());
      }
      try {
        predicted_phi_dot_dot_publisher_.publish(predicted_phi_dot_dot_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_phi_dot_dot_publisher_.getTopic().c_str());
      }
      try {
        predicted_theta_dot_dot_publisher_.publish(predicted_theta_dot_dot_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_theta_dot_dot_publisher_.getTopic().c_str());
      }
      try {
        predicted_load_position_errors_publisher_.publish(predicted_load_position_errors_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_load_position_errors_publisher_.getTopic().c_str());
      }
    }
  }
} // end of the prediction_general function.

//---------------------------------LOAD 2UAV---------------------------------//
void DergbryanTracker::trajectory_prediction_general_load_2UAV(mrs_msgs::PositionCommand position_cmd, double uav_heading, const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd){
    // --------------------------------------------------------------
    // |          load the control reference and estimates          | --> the reference is assumed constant over the prediction

    // Arguments : position.cmd, uav_heading and last attitude_cmd are for UAV1
    //UAV2 informations are received in callbacks and stored into global variables, that can be accessed here without being passed as arguments.
    //position_cmd_follower_for_leader_ is the command for uav2, both are typed mrs_msgs::PositionCommand. 
    //uav_state_follower_for_leader_ is the state of the UAV
    //anchoring_point_follower_position_ and velocities_ are the linear position and velocities

    
    // ROS_INFO_STREAM("_load_mass_= \n" <<_load_mass_);
    // --------------------------------------------------------------
    // Rp - position reference in global frame
    // Rv - velocity reference in global frame
    // Ra - velocity reference in global frame
    // Rw - angular velocity reference
    //Every values in here that are for UAV1 are shown with a 1 at the end, same for the UAV2 variables.
    // uav1
    Eigen::Vector3d Rp1 = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d Rv1 = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d Ra1 = Eigen::Vector3d::Zero(3);
    // Eigen::Vector3d Rw1 = Eigen::Vector3d::Zero(3);
    /* NOTE: replaced control_reference -> (input of controller) by position_cmd.use_ defined above in the update function*/
    if (position_cmd.use_position_vertical || position_cmd.use_position_horizontal) {
      if (position_cmd.use_position_horizontal) { 
        Rp1[0] = position_cmd.position.x;
        Rp1[1] = position_cmd.position.y;
      } else {
        Rv1[0] = 0;
        Rv1[1] = 0;
      }

      if (position_cmd.use_position_vertical) {
        Rp1[2] = position_cmd.position.z;
      } else {
        Rv1[2] = 0;
      }
    }

    if (position_cmd.use_velocity_horizontal) {
      Rv1[0] = position_cmd.velocity.x;
      Rv1[1] = position_cmd.velocity.y;
    } else {
      Rv1[0] = 0;
      Rv1[1] = 0;
    }

    if (position_cmd.use_velocity_vertical) {
      Rv1[2] = position_cmd.velocity.z;
    } else {
      Rv1[2] = 0;
    }

    if (position_cmd.use_acceleration) {
      Ra1 << position_cmd.acceleration.x, position_cmd.acceleration.y, position_cmd.acceleration.z;
    } else {
      Ra1 << 0, 0, 0;
    }
    
    /*VALIDATE: test streaming the references Rp, Rv, Ra when the drone is moving.*/
    // ROS_INFO_STREAM("Rp = \n" << Rp);
    // ROS_INFO_STREAM("Rv = \n" << Rv);
    // ROS_INFO_STREAM("Ra = \n" << Ra);
  
    /*QUESTION: where to print Rw? When it is set*/
    //ROS_INFO_STREAM("DERG = \n" << _use_derg_);

    //uav2 
    Eigen::Vector3d Rp2 = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d Rv2 = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d Ra2 = Eigen::Vector3d::Zero(3);
    // Eigen::Vector3d Rw2 = Eigen::Vector3d::Zero(3); //already defined in the function as always 0 out of the function.
    /* NOTE: replaced control_reference -> (input of controller) by position_cmd.use_ defined above in the update function*/
    if (position_cmd_follower_for_leader_.use_position_vertical || position_cmd_follower_for_leader_.use_position_horizontal) {
      if (position_cmd_follower_for_leader_.use_position_horizontal) { 
        Rp2[0] = position_cmd_follower_for_leader_.position.x;
        Rp2[1] = position_cmd_follower_for_leader_.position.y;
      } else {
        Rv2[0] = 0;
        Rv2[1] = 0;
      }

      if (position_cmd_follower_for_leader_.use_position_vertical) {
        Rp2[2] = position_cmd_follower_for_leader_.position.z;
      } else {
        Rv2[2] = 0;
      }
    }

    if (position_cmd_follower_for_leader_.use_velocity_horizontal) {
      Rv2[0] = position_cmd_follower_for_leader_.velocity.x;
      Rv2[1] = position_cmd_follower_for_leader_.velocity.y;
    } else {
      Rv2[0] = 0;
      Rv2[1] = 0;
    }

    if (position_cmd_follower_for_leader_.use_velocity_vertical) {
      Rv2[2] = position_cmd_follower_for_leader_.velocity.z;
    } else {
      Rv2[2] = 0;
    }

    if (position_cmd_follower_for_leader_.use_acceleration) {
      Ra2 << position_cmd_follower_for_leader_.acceleration.x, position_cmd_follower_for_leader_.acceleration.y, position_cmd_follower_for_leader_.acceleration.z;
    } else {
      Ra2 << 0, 0, 0;
    }

    // --------------------------------------------------------------
    // |          load the control reference of the load             | --> the reference is assumed constant over the prediction
    // --------------------------------------------------------------
    // Rpl1,2 - position reference of the anchoring point 1,2 in global frame
    // Rvl1,2 - velocity reference of the anchoring point 1,2 load in global frame

    
    Eigen::Vector3d Rpl1 = Eigen::Vector3d::Zero(3); // reference for 1st anchoring point
    Eigen::Vector3d Rpl2 = Eigen::Vector3d::Zero(3);// reference for the 2nd anchoring point

    if (position_cmd.use_position_vertical || position_cmd.use_position_horizontal) {

      if (position_cmd.use_position_horizontal) {
        Rpl1[0] = position_cmd.position.x;
        Rpl1[1] = position_cmd.position.y;
      } 

      if (position_cmd.use_position_vertical) {
        Rpl1[2] = position_cmd.position.z - _cable_length_;
      } 
    }
    if (position_cmd_follower_for_leader_.use_position_vertical || position_cmd_follower_for_leader_.use_position_horizontal) {

      if (position_cmd_follower_for_leader_.use_position_horizontal) {
        Rpl2[0] = position_cmd_follower_for_leader_.position.x;
        Rpl2[1] = position_cmd_follower_for_leader_.position.y;
      } 

      if (position_cmd_follower_for_leader_.use_position_vertical) {
        Rpl2[2] = position_cmd_follower_for_leader_.position.z - _cable_length_;
      } 
    }

    // | --------------------------------- |
   //We need vectors of the different states so it can be used in the matrix/vectorial expressions later. => get Eigen::Vector3d version of each variable 
    // | --------------------- initialize the state of the UAV1 --------------------- |

    mrs_msgs::UavState uav1_state = uav_state_; // uav_state represents the locally defined predicted state
    // geometry_msgs::Vector3 load_pose_position = anchoring_pt_pose_position_ ;// anchoring point 1, pain to use it as geometry msg, better to use vector.

    Eigen::Vector3d uav1_anchoring_point_position = anchoring_pt_pose_position_; //init local variable of anchoring point position of uav1

    Eigen::Vector3d uav1_anchoring_point_velocity = anchoring_pt_lin_vel_;
    // Eigen::Vector3d pred_old_uav1_anchoring_point_velo

    Eigen::Vector3d uav1_position;
    uav1_position[0]=uav1_state.pose.position.x;
    uav1_position[1]=uav1_state.pose.position.y;
    uav1_position[2]=uav1_state.pose.position.z; 

    Eigen::Vector3d uav1_velocity;
    uav1_velocity[0]=uav1_state.velocity.linear.x;
    uav1_velocity[1]=uav1_state.velocity.linear.y;
    uav1_velocity[2]=uav1_state.velocity.linear.z;

    Eigen::Vector3d uav1_acc;
    // | --------------------- initialize the state of the UAV2 --------------------- |

    mrs_msgs::UavState uav2_state = uav_state_follower_for_leader_; // uav2_state represents the locally defined predicted state
    Eigen::Vector3d uav2_position=uav_position_follower_; // uav2 states in vector forms
    Eigen::Vector3d uav2_velocity=uav_velocity_follower_;

    Eigen::Vector3d uav2_anchoring_point_position = anchoring_point_follower_position_; //anchoring point 2
    Eigen::Vector3d uav2_anchoring_point_velocity = anchoring_point_follower_velocity_;

    Eigen::Vector3d uav2_acc;
    // Beam payload position velocity (i.e. Center of mass of the beam. p_l,dot(p_l) in my thesis)
    Eigen::Vector3d Payload_position ;
    Eigen::Vector3d Payload_velocity ;
    Eigen::Vector3d Payload_acc;  
    // Beam payload orientation unit vector and angular velocity 
    Eigen::Vector3d nl ;
    Eigen::Vector3d dotnl;
    Eigen::Vector3d wl ;
    Eigen::Vector3d dotwl ;
    //
    double J_l=(1.0/2.0)*(_load_mass_*2.0)*pow(1.5,2.0); //Inertia of the load. _load_mass_ is the load lifted by one of the 2UAV, so it needs to be *2 to have the full mass of the beam payload.


  predicted_uav1_poses_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_poses_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_poses_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_poses_out_.header.frame_id = uav2_state.header.frame_id;
  
  predicted_uav1_vel_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_vel_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_vel_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_vel_out_.header.frame_id = uav2_state.header.frame_id;

  predicted_uav1_acc_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_acc_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_acc_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_acc_out_.header.frame_id = uav2_state.header.frame_id;

  predicted_uav1_thrust_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_thrust_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_thrust_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_thrust_out_.header.frame_id = uav2_state.header.frame_id;
  
  predicted_uav1_attitude_rate_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_attitude_rate_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_attitude_rate_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_attitude_rate_out_.header.frame_id = uav2_state.header.frame_id;

  // TODO: Don't we need uav1/2-specific similar to predicted_des_attituderate_out_?
  // TODO: Don't we need uav1/2-specific similar to predicted_tiltangle_out_?

  predicted_uav1_anchoring_point_pose_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_anchoring_point_pose_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_anchoring_point_pose_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_anchoring_point_pose_out_.header.frame_id = uav2_state.header.frame_id;
  
  predicted_uav1_anchoring_point_vel_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_anchoring_point_vel_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_anchoring_point_vel_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_anchoring_point_vel_out_.header.frame_id = uav2_state.header.frame_id;

  predicted_uav1_anchoring_point_acc_out_.header.stamp=uav1_state.header.stamp;
  predicted_uav1_anchoring_point_acc_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_anchoring_point_acc_out_.header.stamp=uav2_state.header.stamp;
  predicted_uav2_anchoring_point_acc_out_.header.frame_id = uav2_state.header.frame_id;

  predicted_payload_position_out_.header.stamp=uav1_state.header.stamp;
  predicted_payload_position_out_.header.frame_id = uav1_state.header.frame_id;
  
  predicted_payload_vel_out_.header.stamp=uav1_state.header.stamp;
  predicted_payload_vel_out_.header.frame_id = uav1_state.header.frame_id;
  
  predicted_payload_acc_out_.header.stamp=uav1_state.header.stamp;
  predicted_payload_acc_out_.header.frame_id = uav1_state.header.frame_id;

  predicted_nl_out_.header.stamp=uav1_state.header.stamp;
  predicted_nl_out_.header.frame_id = uav1_state.header.frame_id;

  predicted_wl_out_.header.stamp=uav1_state.header.stamp;
  predicted_wl_out_.header.frame_id = uav1_state.header.frame_id;

  predicted_dotnl_out_.header.stamp=uav1_state.header.stamp;
  predicted_dotnl_out_.header.frame_id = uav1_state.header.frame_id;

  predicted_dotwl_out_.header.stamp=uav1_state.header.stamp;
  predicted_dotwl_out_.header.frame_id = uav1_state.header.frame_id;

  predicted_uav1_swing_angle_out_.header.stamp = uav1_state.header.stamp;
  predicted_uav1_swing_angle_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_swing_angle_out_.header.stamp = uav2_state.header.stamp;
  predicted_uav2_swing_angle_out_.header.frame_id = uav2_state.header.frame_id;
  
  
  predicted_uav1_tension_force_out_.header.stamp = uav1_state.header.stamp;
  predicted_uav1_tension_force_out_.header.frame_id = uav1_state.header.frame_id;
  predicted_uav2_tension_force_out_.header.stamp = uav2_state.header.stamp;
  predicted_uav2_tension_force_out_.header.frame_id = uav2_state.header.frame_id;

  // TODO: Don't we need uav1/2-specific similar to predicted_phi_out_, predicted_theta_out_, predicted_phi_dot_out_, predicted_theta_dot_out_, predicted_phi_dot_dot_out_, predicted_theta_dot_dot_out_?
  // TODO: Don't we need uav1/2-specific similar to predicted_load_position_errors_out_?

  // Definition of custom quantities, used as "pose" typed version of the other quantities. See these as "to publish" variables.
  //same names used as before 
  //uav1
  geometry_msgs::Pose custom_uav1_pose;
  geometry_msgs::Pose custom_uav1_vel;
  geometry_msgs::Pose custom_uav1_acceleration;
  //uav2
  geometry_msgs::Pose custom_uav2_pose;
  geometry_msgs::Pose custom_uav2_vel;
  geometry_msgs::Pose custom_uav2_acceleration;
  
  // AnchoringPoint of uav1
  geometry_msgs::Pose custom_uav1_anchoring_point_pose;
  geometry_msgs::Pose custom_uav1_anchoring_point_vel;
  geometry_msgs::Pose custom_uav1_anchoring_point_acceleration;
  
  // AnchoringPoint of uav2
  geometry_msgs::Pose custom_uav2_anchoring_point_pose;
  geometry_msgs::Pose custom_uav2_anchoring_point_vel;
  geometry_msgs::Pose custom_uav2_anchoring_point_acceleration;

  // Control Inputs uav1
  geometry_msgs::Pose predicted_uav1_thrust; 
  geometry_msgs::Pose predicted_uav1_thrust_norm; 
  geometry_msgs::Pose predicted_uav1_attituderate;
  
  // Control Inputs uav2
  geometry_msgs::Pose predicted_uav2_thrust; 
  geometry_msgs::Pose predicted_uav2_thrust_norm; 
  geometry_msgs::Pose predicted_uav2_attituderate;

  // Beam payload position velocity to publish
  geometry_msgs::Pose custom_Payload_position ;
  geometry_msgs::Pose custom_Payload_velocity ;
  geometry_msgs::Pose custom_Payload_acc;

  // Swing angle uav1 and uav2
  geometry_msgs::Pose predicted_uav1_swing_angle;
  geometry_msgs::Pose predicted_uav2_swing_angle;

  // Tension force  uav1 and uav2
  geometry_msgs::Pose custom_uav1_tension_force;
  geometry_msgs::Pose custom_uav2_tension_force;

  // Beam payload orientation unit vector and angular velocity to publish
  geometry_msgs::Pose custom_nl ;
  geometry_msgs::Pose custom_wl ;
  geometry_msgs::Pose custom_dotnl;
  geometry_msgs::Pose custom_dotwl;
  //to delete--
  geometry_msgs::Pose predicted_load_theta;
  geometry_msgs::Pose predicted_load_phi;
  geometry_msgs::Pose predicted_load_dot_theta;
  geometry_msgs::Pose predicted_load_dot_phi;
  geometry_msgs::Pose predicted_q_state_dot_dot_uav;
  geometry_msgs::Pose predicted_q_state_dot_dot_load;
  

  geometry_msgs::Pose predicted_load_dot_dot_theta;
  geometry_msgs::Pose predicted_load_dot_dot_phi;
  
  geometry_msgs::Pose predicted_load_position_errors_to_publish;

  // Thesis b: Test 06/08

  double theta_load_cable;
  double phi_load_cable;  
  double theta_dot_load_cable;
  double phi_dot_load_cable;  
  double theta_dot_dot_load_cable;
  double phi_dot_dot_load_cable;
  

  std_msgs::Float32 pos_difference_x;
  std_msgs::Float32 pos_difference_y;
  //--
  Eigen::Matrix3d uav1_R;
  Eigen::Matrix3d uav1_Rdot;
  Eigen::Matrix3d uav1_skew_Ow;
  Eigen::Vector3d uav1_attitude_rate_pred;

  Eigen::Matrix3d uav2_R;
  Eigen::Matrix3d uav2_Rdot;
  Eigen::Matrix3d uav2_skew_Ow;
  Eigen::Vector3d uav2_attitude_rate_pred;

  // // R - current uav attitude
  // uav1_R = mrs_lib::AttitudeConverter(uav1_state.pose.orientation);
  // uav2_R = mrs_lib::AttitudeConverter(uav2_state.pose.orientation);

  double m1=uav_mass_;//estimated mass of the UAV1, updated in the update function. 
  double m2=uav_mass_;
  double ml= _load_mass_*2.0; //As the _load_mass_ is half of the payload. As this is the value used in controller to control the anchoring point position.
  double d1=_load_length_/2.0;
  double d2=-_load_length_/2.0;


  // Eigen::MatrixXd x_full_state = Eigen::MatrixXd(5, 5);

  // Prediction loop (over 3sec horizon)
  for (int i = 0; i < _num_pred_samples_; i++) {
    if(!erg_predictions_theta_trusted_){
      ROS_INFO_STREAM("we break since erg_predictions_theta_trusted_ = false \n");
      break;
    }
    if(i==0){
      //Initial conditions for first iteration + Fill in custom poses

      //uav1
      custom_uav1_pose.position.x=uav1_state.pose.position.x; //init_pos(0,0)
      custom_uav1_pose.position.y=uav1_state.pose.position.y;
      custom_uav1_pose.position.z=uav1_state.pose.position.z;
       
      custom_uav1_vel.position.x=uav1_state.velocity.linear.x;//init_vel(0,0);
      custom_uav1_vel.position.y=uav1_state.velocity.linear.y;
      custom_uav1_vel.position.z=uav1_state.velocity.linear.z;
      //uav2
      custom_uav2_pose.position.x=uav2_state.pose.position.x;//init_pos(0,0)
      custom_uav2_pose.position.y=uav2_state.pose.position.y;
      custom_uav2_pose.position.z=uav2_state.pose.position.z;

      custom_uav2_vel.position.x=uav2_state.velocity.linear.x;//init_vel(0,0);
      custom_uav2_vel.position.y=uav2_state.velocity.linear.y;
      custom_uav2_vel.position.z=uav2_state.velocity.linear.z;

      // R - current uav attitude
      uav1_R = mrs_lib::AttitudeConverter(uav1_state.pose.orientation);
      uav2_R = mrs_lib::AttitudeConverter(uav2_state.pose.orientation);
      // // Ow - UAV angular rate
      Eigen::Vector3d Ow1(uav1_state.velocity.angular.x, uav1_state.velocity.angular.y, uav1_state.velocity.angular.z);
      Eigen::Vector3d Ow2(uav2_state.velocity.angular.x, uav2_state.velocity.angular.y, uav2_state.velocity.angular.z);
      uav1_attitude_rate_pred = Ow1;
      uav2_attitude_rate_pred = Ow2;
       

      // Thesis B: Step 2: initialize the custom load predictions for the first iteration

      custom_uav1_anchoring_point_pose.position.x=uav1_anchoring_point_position[0];
      custom_uav1_anchoring_point_pose.position.y=uav1_anchoring_point_position[1];
      custom_uav1_anchoring_point_pose.position.z=uav1_anchoring_point_position[2];

      custom_uav2_anchoring_point_pose.position.x=uav2_anchoring_point_position[0];
      custom_uav2_anchoring_point_pose.position.y=uav2_anchoring_point_position[1];
      custom_uav2_anchoring_point_pose.position.z=uav2_anchoring_point_position[2];

      custom_uav1_anchoring_point_vel.position.x=uav1_anchoring_point_velocity[0];
      custom_uav1_anchoring_point_vel.position.y=uav1_anchoring_point_velocity[1];
      custom_uav1_anchoring_point_vel.position.z=uav1_anchoring_point_velocity[2];

      custom_uav2_anchoring_point_vel.position.x=uav2_anchoring_point_velocity[0];
      custom_uav2_anchoring_point_vel.position.y=uav2_anchoring_point_velocity[1];
      custom_uav2_anchoring_point_vel.position.z=uav2_anchoring_point_velocity[2];
      
      // custom_Payload_position.position.x = load_pose_position.x;
      // predicted_load_pose.position.y = load_pose_position.y;
      // predicted_load_pose.position.z = load_pose_position.z;

      // custom_Payload_velocity.position.x = load_lin_vel[0];
      // predicted_load_vel.position.y = load_lin_vel[1];
      // predicted_load_vel.position.z = load_lin_vel[2];

      // FK : Beam payload center of mass position and velocity - Raphael
      Payload_position=(uav1_anchoring_point_position+uav2_anchoring_point_position)/2.0; // vector 3d.
      Payload_velocity=(uav1_anchoring_point_velocity+uav2_anchoring_point_velocity)/2.0;

      custom_Payload_position.position.x=Payload_position[0];// poses to publish
      custom_Payload_position.position.y=Payload_position[1];
      custom_Payload_position.position.z=Payload_position[2];

      custom_Payload_velocity.position.x=Payload_velocity[0];
      custom_Payload_velocity.position.y=Payload_velocity[1];
      custom_Payload_velocity.position.z=Payload_velocity[2];

      //FK : Beam orientation vector nl and angular velocity wl - Raphael
      nl=(uav1_anchoring_point_position-uav2_anchoring_point_position).normalized();//normalized to get unit vector
      // nl=nl.normalized(); 
      // to publish
      custom_nl.position.x=nl[0];
      custom_nl.position.y=nl[1];
      custom_nl.position.z=nl[2];

      // use the following shorter names for each element of the anchoring points ,
      //For readability in ndot formula, otherwise it would be too long and hard to read.

      double xa1=uav1_anchoring_point_position[0];
      double ya1=uav1_anchoring_point_position[1];
      double za1=uav1_anchoring_point_position[2];

      double xa2=uav2_anchoring_point_position[0];
      double ya2=uav2_anchoring_point_position[1];
      double za2=uav2_anchoring_point_position[2];

      double xa1D=uav1_anchoring_point_velocity[0];
      double ya1D=uav1_anchoring_point_velocity[1];
      double za1D=uav1_anchoring_point_velocity[2];

      double xa2D=uav2_anchoring_point_velocity[0];
      double ya2D=uav2_anchoring_point_velocity[1];
      double za2D=uav2_anchoring_point_velocity[2];

      // compute each element of dot(n_l)
     
      double xnlD = (xa1D*pow(ya1,2.0) + xa1D*pow(ya2,2.0) + xa1D*pow(za1,2.0) + xa1D*pow(za2,2.0) - xa2D*pow((ya1 - ya2),2.0) - xa2D*pow((za1 - za2),2.0) 
      - xa1*ya1*ya1D + xa1*ya1*ya2D + xa1*ya2*ya1D + xa2*ya1*ya1D - 2.0*xa1D*ya1*ya2 - xa1*ya2*ya2D - xa2*ya1*ya2D - xa2*ya2*ya1D 
      + xa2*ya2*ya2D - xa1*za1*za1D + xa1*za1*za2D + xa1*za2*za1D + xa2*za1*za1D - 2.0*xa1D*za1*za2 - xa1*za2*za2D - xa2*za1*za2D 
      - xa2*za2*za1D + xa2*za2*za2D)/pow((pow((xa1 - xa2),2.0) + pow((ya1 - ya2),2.0) + pow((za1 - za2),2.0)),(3.0/2.0)) ;

      double ynlD = (pow(xa1,2.0)*ya1D + pow(xa2,2.0)*ya1D + ya1D*pow(za1,2.0) + ya1D*pow(za2,2.0) - ya2D*pow((xa1 - xa2),2.0) - ya2D*pow((za1 - za2),2.0) 
      - xa1*xa1D*ya1 - 2.0*xa1*xa2*ya1D + xa1*xa1D*ya2 + xa1*xa2D*ya1 + xa2*xa1D*ya1 - xa1*xa2D*ya2 - xa2*xa1D*ya2 - xa2*xa2D*ya1 
      + xa2*xa2D*ya2 - ya1*za1*za1D + ya1*za1*za2D + ya1*za2*za1D + ya2*za1*za1D - 2.0*ya1D*za1*za2 - ya1*za2*za2D - ya2*za1*za2D 
      - ya2*za2*za1D + ya2*za2*za2D)/pow((pow((xa1 - xa2),2.0) + pow((ya1 - ya2),2.0) + pow((za1 - za2),2.0)),(3.0/2.0)) ;


      double znlD =(pow(xa1,2.0)*za1D + pow(xa2,2.0)*za1D + pow(ya1,2.0)*za1D + pow(ya2,2.0)*za1D - za2D*pow((xa1 - xa2),2.0) - za2D*pow((ya1 - ya2),2.0) 
      - xa1*xa1D*za1 - 2.0*xa1*xa2*za1D + xa1*xa1D*za2 + xa1*xa2D*za1 + xa2*xa1D*za1 - xa1*xa2D*za2 - xa2*xa1D*za2 - xa2*xa2D*za1 
      + xa2*xa2D*za2 - ya1*ya1D*za1 - 2.0*ya1*ya2*za1D + ya1*ya1D*za2 + ya1*ya2D*za1 + ya2*ya1D*za1 - ya1*ya2D*za2 - ya2*ya1D*za2 
      - ya2*ya2D*za1 + ya2*ya2D*za2)/pow((pow((xa1 - xa2),2.0) + pow((ya1 - ya2),2.0) + pow((za1 - za2),2.0)),(3.0/2.0));

      dotnl[0]=xnlD;
      dotnl[1]=ynlD;
      dotnl[2]=znlD;
      // to publish
      custom_dotnl.position.x=dotnl[0];
      custom_dotnl.position.y=dotnl[1];
      custom_dotnl.position.z=dotnl[2];      
      // then wl
      wl=nl.cross(dotnl);
      // to publish
      custom_wl.position.x=wl[0];
      custom_wl.position.y=wl[1];
      custom_wl.position.z=wl[2];   
    } 
    else{
      // Numerical integration of states of UAV1
      uav1_velocity=uav1_velocity + uav1_acc*_prediction_dt_;
      custom_uav1_vel.position.x=uav1_velocity[0];      //to publish as pose;
      custom_uav1_vel.position.y=uav1_velocity[1];
      custom_uav1_vel.position.z=uav1_velocity[2];

      uav1_position=uav1_position + uav1_velocity*_prediction_dt_;
      custom_uav1_pose.position.x=uav1_position[0];      //to publish as pose;
      custom_uav1_pose.position.y=uav1_position[1];
      custom_uav1_pose.position.z=uav1_position[2];

      // Numerical integration of states of UAV2

      uav2_velocity=uav2_velocity + uav2_acc*_prediction_dt_;
      custom_uav2_vel.position.x=uav2_velocity[0];      //to publish as pose;
      custom_uav2_vel.position.y=uav2_velocity[1];
      custom_uav2_vel.position.z=uav2_velocity[2];

      uav2_position=uav2_position + uav2_velocity*_prediction_dt_;
      custom_uav2_pose.position.x=uav2_position[0];      //to publish as pose;
      custom_uav2_pose.position.y=uav2_position[1];
      custom_uav2_pose.position.z=uav2_position[2];

      //Numerical integration payload variables
      wl=wl+dotwl*_prediction_dt_;
      custom_wl.position.x=wl[0];
      custom_wl.position.y=wl[1];
      custom_wl.position.z=wl[2];
      nl=nl+dotnl*_prediction_dt_;
      custom_nl.position.x=nl[0];
      custom_nl.position.y=nl[1];
      custom_nl.position.z=nl[2];
      Payload_velocity=Payload_velocity+Payload_acc*_prediction_dt_;
      custom_Payload_velocity.position.x=Payload_velocity[0];
      custom_Payload_velocity.position.y=Payload_velocity[1];
      custom_Payload_velocity.position.z=Payload_velocity[2];

      Payload_position=Payload_position+Payload_velocity*_prediction_dt_;
      custom_Payload_position.position.x=Payload_position[0];// poses to publish
      custom_Payload_position.position.y=Payload_position[1];
      custom_Payload_position.position.z=Payload_position[2];
      // Get anchoring points states for the controller ( and the graphs )
      //positions
      uav1_anchoring_point_position=Payload_position+d1*nl;
      custom_uav1_anchoring_point_pose.position.x=uav1_anchoring_point_position[0];
      custom_uav1_anchoring_point_pose.position.y=uav1_anchoring_point_position[1];
      custom_uav1_anchoring_point_pose.position.z=uav1_anchoring_point_position[2];
      uav2_anchoring_point_position=Payload_position+d2*nl;
      custom_uav2_anchoring_point_pose.position.x=uav2_anchoring_point_position[0];
      custom_uav2_anchoring_point_pose.position.y=uav2_anchoring_point_position[1];
      custom_uav2_anchoring_point_pose.position.z=uav2_anchoring_point_position[2];
      //velocities
      uav1_anchoring_point_velocity=Payload_velocity+d1*dotnl;
      custom_uav1_anchoring_point_vel.position.x=uav1_anchoring_point_velocity[0];
      custom_uav1_anchoring_point_vel.position.y=uav1_anchoring_point_velocity[1];
      custom_uav1_anchoring_point_vel.position.z=uav1_anchoring_point_velocity[2];
      uav2_anchoring_point_velocity=Payload_velocity+d2*dotnl;
      custom_uav2_anchoring_point_vel.position.x=uav2_anchoring_point_velocity[0];
      custom_uav2_anchoring_point_vel.position.y=uav2_anchoring_point_velocity[1];
      custom_uav2_anchoring_point_vel.position.z=uav2_anchoring_point_velocity[2];
      // ROS_INFO_STREAM("uav1_velocity \n"<<uav1_velocity);
      // ROS_INFO_STREAM("uav2_velocity\n"<<uav2_velocity);
      // ROS_INFO_STREAM("uav1_position\n"<<uav1_position);
      // ROS_INFO_STREAM("uav2_position\n"<<uav2_position);
      // ROS_INFO_STREAM("uav1_anchoring_point_position\n"<<uav1_anchoring_point_position);
      // ROS_INFO_STREAM("uav2_anchoring_point_position\n"<<uav2_anchoring_point_position);
      // ROS_INFO_STREAM("uav1_anchoring_point_velocity\n"<<uav1_anchoring_point_velocity); //looks okay
      // ROS_INFO_STREAM("uav2_anchoring_point_velocity\n"<<uav2_anchoring_point_velocity);

      //Get UAV1 attitude uav1_R from output of the controller uav1_attitude_rate_pred.
      uav1_skew_Ow << 0     , -uav1_attitude_rate_pred(2), uav1_attitude_rate_pred(1),
                uav1_attitude_rate_pred(2) , 0,       -uav1_attitude_rate_pred(0),
                -uav1_attitude_rate_pred(1), uav1_attitude_rate_pred(0),  0;// assume omega is omega desired
      // ensure Ow is updated correctly in predictions....
      uav1_Rdot = uav1_skew_Ow*uav1_R; // or add - (equivalent to transpose?) UNUSED
      double custom_dt2 = _prediction_dt_;//   /10.0;
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
      uav1_R = (I + uav1_skew_Ow*custom_dt2 + (0.5)*uav1_skew_Ow*custom_dt2*uav1_skew_Ow*custom_dt2 + (1.0/6.0)*uav1_skew_Ow*custom_dt2*uav1_skew_Ow*custom_dt2*uav1_skew_Ow*custom_dt2)*uav1_R;
      
      //Get UAV2 attitude uav2_R from output of the controller uav2_attitude_rate_pred.
      uav2_skew_Ow << 0.0     , -uav2_attitude_rate_pred(2), uav2_attitude_rate_pred(1),
                uav2_attitude_rate_pred(2) , 0.0,       -uav2_attitude_rate_pred(0),
                -uav2_attitude_rate_pred(1), uav2_attitude_rate_pred(0),  0.0;// assume omega is omega desired
      // ensure Ow is updated correctly in predictions....
      uav2_Rdot = uav2_skew_Ow*uav2_R; 
      uav2_R = (I + uav2_skew_Ow*custom_dt2 + (0.5)*uav2_skew_Ow*custom_dt2*uav2_skew_Ow*custom_dt2 + (1.0/6.0)*uav2_skew_Ow*custom_dt2*uav2_skew_Ow*custom_dt2*uav2_skew_Ow*custom_dt2)*uav2_R;
    } 
    // pushback all the predicted quantity in their pose array vector, to be able to publish the 3s predictions as one big poseArray, called "predicted_varname_out_" and publish using "predicted_varname_pub" as publisher.
    //for now only publishing these informations, but can be extended (and uncomment the needed Posearrays)
    predicted_uav1_poses_out_.poses.push_back(custom_uav1_pose);
    predicted_uav1_vel_out_.poses.push_back(custom_uav1_vel);
    // predicted_uav1_acc_out_.poses.push_back(custom_uav1_acceleration);//tochange when defined
    predicted_uav2_poses_out_.poses.push_back(custom_uav2_pose);
    predicted_uav2_vel_out_.poses.push_back(custom_uav2_vel);
    // predicted_uav2_acc_out_.poses.push_back(custom_uav2_acceleration);//tochange when defined

    predicted_uav1_anchoring_point_pose_out_.poses.push_back(custom_uav1_anchoring_point_pose); 
    predicted_uav1_anchoring_point_vel_out_.poses.push_back(custom_uav1_anchoring_point_vel);
    // predicted_uav1_anchoring_point_acc_out_.poses.push_back(custom_uav1_anchoring_point_acceleration);
    predicted_uav2_anchoring_point_pose_out_.poses.push_back(custom_uav2_anchoring_point_pose);
    predicted_uav2_anchoring_point_vel_out_.poses.push_back(custom_uav2_anchoring_point_vel);
    // predicted_uav2_anchoring_point_acc_out_.poses.push_back(custom_uav2_anchoring_point_acceleration);

 
    predicted_uav1_attituderate.position.x = uav1_attitude_rate_pred(0,0);
    predicted_uav1_attituderate.position.y = uav1_attitude_rate_pred(1,0);
    predicted_uav1_attituderate.position.z = uav1_attitude_rate_pred(2,0);
    predicted_uav2_attituderate.position.x = uav2_attitude_rate_pred(0,0);
    predicted_uav2_attituderate.position.y = uav2_attitude_rate_pred(1,0);
    predicted_uav2_attituderate.position.z = uav2_attitude_rate_pred(2,0);
    predicted_uav1_attitude_rate_out_.poses.push_back(predicted_uav1_attituderate);
    predicted_uav2_attitude_rate_out_.poses.push_back(predicted_uav2_attituderate);


    // predicted_payload_position_out_.poses.push_back(custom_Payload_position);
    // predicted_payload_vel_out_.poses.push_back(custom_Payload_velocity);
    // predicted_payload_acc_out_.poses.push_back(custom_Payload_acc);

    // predicted_nl_out_.poses.push_back(custom_nl);
    // predicted_wl_out_.poses.push_back(custom_wl);
    // predicted_dotnl_out_.poses.push_back(custom_dotnl);
    // predicted_dotwl_out_.poses.push_back(custom_dotwl);

    //Commented as not published yet.

    //------------------------Controller's part-----------------------//
    // Need to put the arguments of the SimulateSe3CopyFunction in the correct form, for both UAVs. 
    //Controller function needs uav_state msg type as input, not a Vector3d.
    uav1_state.pose.position.x=uav1_position[0];
    uav1_state.pose.position.y=uav1_position[1];
    uav1_state.pose.position.z=uav1_position[2]; 

    uav1_state.velocity.linear.x=uav1_velocity[0];
    uav1_state.velocity.linear.y=uav1_velocity[1];
    uav1_state.velocity.linear.z=uav1_velocity[2];
    //same for uav2_state
    uav2_state.pose.position.x=uav2_position[0];
    uav2_state.pose.position.y=uav2_position[1];
    uav2_state.pose.position.z=uav2_position[2]; 

    uav2_state.velocity.linear.x=uav2_velocity[0];
    uav2_state.velocity.linear.y=uav2_velocity[1];
    uav2_state.velocity.linear.z=uav2_velocity[2];
    

    
    // Print input of UAV1 controller.
    // ROS_INFO_STREAM("uav1_state = \n" << uav1_state);
    // ROS_INFO_STREAM("uav1_anchoring_point_position = \n" << uav1_anchoring_point_position);
    // ROS_INFO_STREAM("uav1_anchoring_point_velocity = \n" << uav1_anchoring_point_velocity);
    // ROS_INFO_STREAM("position_cmd = \n" << position_cmd);

    // // //Print input of UAV2 controller.
      // ROS_INFO_STREAM("uav2_state = \n" << uav2_state);
    //   ROS_INFO_STREAM("uav2_anchoring_point_position = \n" << uav2_anchoring_point_position);
    //   ROS_INFO_STREAM("uav2_anchoring_point_velocity = \n" << uav2_anchoring_point_velocity);
      // ROS_INFO_STREAM("uav2_position_cmd_ = \n" << position_cmd_follower_for_leader_);

   //std::tie to create tuple from the two variables and stock the data that is returned by SimulateController fct.
    // Eigen::Vector3d f1;
    double thrust_force1;
    double theta1;
    std::tie(thrust_force1,uav1_attitude_rate_pred,theta1)=ComputeSe3CopyController(uav1_state, uav1_R, uav1_anchoring_point_position , uav1_anchoring_point_velocity ,position_cmd , Rp1 , Rv1 , Rpl1 , Ra1 );
    // ROS_INFO_STREAM("f1 = \n" << f1);
    double thrust_force2;
    double theta2;
    std::tie(thrust_force2,uav2_attitude_rate_pred,theta2)=ComputeSe3CopyController(uav2_state, uav2_R, uav2_anchoring_point_position , uav2_anchoring_point_velocity ,position_cmd_follower_for_leader_ , Rp2 , Rv2 , Rpl2 , Ra2);
    // ROS_INFO_STREAM("f2 = \n" << f2);
    predicted_uav1_thrust_norm.position.x = thrust_force1; // change later to a non vec type; Why not f?
    predicted_uav1_thrust_out_.poses.push_back(predicted_uav1_thrust_norm);
    predicted_uav2_thrust_norm.position.x = thrust_force2; // change later to a non vec type; Why not f?
    predicted_uav2_thrust_out_.poses.push_back(predicted_uav2_thrust_norm);
    
    

    //Compute the applied force, in direction of z_B
    Eigen::Vector3d f1=thrust_force1*uav1_R.col(2); //TODORAPHAEL change to T1,T2 and the tension below to Tc1 Tc2. or keep f.
    Eigen::Vector3d f2=thrust_force2*uav2_R.col(2);
    //-------------------------Prediction with eom part------------------------// (pandolfo model + My kinematic/kinetic study equations)

    // Compute unit vector indicating cable orientation in the given configuration
    Eigen::Vector3d mu1 = (uav1_position-uav1_anchoring_point_position).normalized();
    Eigen::Vector3d mu2 = (uav2_position-uav2_anchoring_point_position).normalized();

    // compute d(x,u)
    Eigen::MatrixXd d_matrix = Eigen::MatrixXd(2,1);
    d_matrix(0,0)=(ml/m1)*mu1.dot(f1) + (ml/_cable_length_)*(uav1_velocity-uav1_anchoring_point_velocity).squaredNorm() + ml*d1*wl.squaredNorm()*mu1.dot(nl);
    d_matrix(1,0)=(ml/m2)*mu2.dot(f2) + (ml/_cable_length_)*(uav2_velocity-uav2_anchoring_point_velocity).squaredNorm() + ml*d2*wl.squaredNorm()*mu2.dot(nl);
  
    // compute D(x)


    Eigen::MatrixXd D1_matrix = Eigen::MatrixXd(2,2); //first sub matrix to compute D
    D1_matrix(0,0)=ml/m1; D1_matrix(0,1)=0.0; 
    D1_matrix(1,0)=0.0; D1_matrix(1,1)=ml/m2;
    // ROS_INFO_STREAM("D1   = \n" << D1_matrix);

    Eigen::MatrixXd D2_matrix = Eigen::MatrixXd(2,2);
    D2_matrix(0,0)=1.0; D2_matrix(0,1)=mu1.dot(mu2); 
    D2_matrix(1,0)=mu2.dot(mu1); D2_matrix(1,1)=1.0;
    // ROS_INFO_STREAM("D2 = \n" << D2_matrix);

    Eigen::MatrixXd D3_matrix = Eigen::MatrixXd(2,3);
    Eigen::MatrixXd D3_matrixLine1=Eigen::MatrixXd(1,3);
    Eigen::MatrixXd D3_matrixLine2=Eigen::MatrixXd(1,3);

    D3_matrixLine1=d1*(nl.cross(mu1)).transpose();
    D3_matrixLine2=d2*(nl.cross(mu2)).transpose();

    D3_matrix(0,0)=D3_matrixLine1(0,0);
    D3_matrix(0,1)=D3_matrixLine1(0,1);
    D3_matrix(0,2)=D3_matrixLine1(0,2); 
    D3_matrix(1,0)=D3_matrixLine2(0,0);
    D3_matrix(1,1)=D3_matrixLine2(0,1);
    D3_matrix(1,2)=D3_matrixLine2(0,2); 
    // ROS_INFO_STREAM("D3 = \n" << D3_matrix);

    Eigen::MatrixXd D4_matrix = Eigen::MatrixXd(3,2);
    Eigen::MatrixXd D4_matrixCol1=Eigen::MatrixXd(3,1);
    Eigen::MatrixXd D4_matrixCol2=Eigen::MatrixXd(3,1);

    D4_matrixCol1=d1*(nl.cross(mu1));
    D4_matrixCol2=d2*(nl.cross(mu2));
    D4_matrix(0,0)=D4_matrixCol1(0,0);
    D4_matrix(1,0)=D4_matrixCol1(1,0);
    D4_matrix(2,0)=D4_matrixCol1(2,0);
    D4_matrix(0,1)=D4_matrixCol2(0,0);
    D4_matrix(1,1)=D4_matrixCol2(1,0);
    D4_matrix(2,1)=D4_matrixCol2(2,0);
    // ROS_INFO_STREAM("D4  = \n" << D4_matrix);

    Eigen::MatrixXd D_matrix = Eigen::MatrixXd(2,2);
    D_matrix=D1_matrix+D2_matrix+D3_matrix*(ml/J_l)*D4_matrix;
    // ROS_INFO_STREAM("J_l= \n" <<J_l);
    // ROS_INFO_STREAM("D   = \n" << D_matrix);
    //compute tension matrix T
    Eigen::MatrixXd T_matrix = Eigen::MatrixXd(2,1);
    T_matrix=D_matrix.inverse()*d_matrix;
    double T1=T_matrix(0);
    double T2=T_matrix(1);
    // ROS_INFO_STREAM("T   = \n" << T_matrix);

    // Compute cable tension predictions:
    custom_uav1_tension_force.position.x = T1;
    predicted_uav1_tension_force_out_.poses.push_back(custom_uav1_tension_force);
    custom_uav2_tension_force.position.x = T2;
    predicted_uav2_tension_force_out_.poses.push_back(custom_uav2_tension_force);
    
    // EOM to get the accelerations.
    Eigen::Vector3d zw(0,0,1.0);
    ////////////////

    Payload_acc=(1.0/ml)*T1*mu1+(1.0/ml)*T2*mu2-common_handlers_->g*zw;
    dotnl = wl.cross(nl); 
    dotwl=(d1/J_l)*T1*nl.cross(mu1)+(d2/J_l)*T2*nl.cross(mu2);
    uav1_acc=(1.0/m1)*f1-(1.0/m1)*T1*mu1-common_handlers_->g*zw;
    uav2_acc=(1.0/m2)*f2-(1.0/m2)*T2*mu2-common_handlers_->g*zw;
    // ROS_INFO_STREAM("Payload_acc   = \n" << Payload_acc);
    // ROS_INFO_STREAM("dotnl   = \n" << dotnl);
    // ROS_INFO_STREAM("uav1_acc   = \n" << uav1_acc);
    // ROS_INFO_STREAM("uav2_acc   = \n" << uav2_acc);

    // To publish acc values as poses :
    custom_Payload_acc.position.x=Payload_acc[0];
    custom_Payload_acc.position.y=Payload_acc[1];
    custom_Payload_acc.position.z=Payload_acc[2];

    custom_dotnl.position.x=dotnl[0];
    custom_dotnl.position.y=dotnl[1];
    custom_dotnl.position.z=dotnl[2];

    custom_dotwl.position.x=dotwl[0];
    custom_dotwl.position.y=dotwl[1];
    custom_dotwl.position.z=dotwl[2];

    custom_uav1_acceleration.position.x=uav1_acc[0];
    custom_uav1_acceleration.position.y=uav1_acc[1];
    custom_uav1_acceleration.position.z=uav1_acc[2];

    custom_uav2_acceleration.position.x=uav2_acc[0];
    custom_uav2_acceleration.position.y=uav2_acc[1];
    custom_uav2_acceleration.position.z=uav2_acc[2];


    // Compute swing angle predictions:
    Eigen::Vector3d zB1 = uav1_R.col(2);
    // ROS_INFO_STREAM("zB1"<<zB1);
    double swing_angle_uav1 = acos((mu1.dot(zB1))/(mu1.norm()*zB1.norm())); //Compute this swing angle (positive only, between 0 and pi)
    // ROS_INFO_STREAM("swing_angle_uav1"<<swing_angle_uav1);
    predicted_uav1_swing_angle.position.x = swing_angle_uav1;
    predicted_uav1_swing_angle_out_.poses.push_back(predicted_uav1_swing_angle); 
    Eigen::Vector3d zB2 = uav2_R.col(2);
    // ROS_INFO_STREAM("zB1"<<zB1);
    double swing_angle_uav2 = acos((mu2.dot(zB2))/(mu2.norm()*zB2.norm())); //Compute this swing angle (positive only, between 0 and pi)
    // ROS_INFO_STREAM("swing_angle_uav2"<<swing_angle_uav2);
    predicted_uav2_swing_angle.position.x = swing_angle_uav2;
    predicted_uav2_swing_angle_out_.poses.push_back(predicted_uav2_swing_angle); 
  } // end for loop prediction

  //Publish the PoseArray that have been filled during the predictions.
  // ROS_INFO_STREAM("Gets to the publishing");
  try {
    predicted_uav1_poses_publisher_.publish(predicted_uav1_poses_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_poses_publisher_.getTopic().c_str());
  }

  try {
    predicted_uav2_poses_publisher_.publish(predicted_uav2_poses_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_poses_publisher_.getTopic().c_str());
  }
  try {
    predicted_uav1_vel_publisher_.publish(predicted_uav1_vel_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_vel_publisher_.getTopic().c_str());
  }
    try {
    predicted_uav2_vel_publisher_.publish(predicted_uav2_vel_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_vel_publisher_.getTopic().c_str());
  }
    try {
    predicted_uav1_anchoring_point_pose_publisher_.publish(predicted_uav1_anchoring_point_pose_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_anchoring_point_pose_publisher_.getTopic().c_str());
  }

  try {
    predicted_uav2_anchoring_point_pose_publisher_.publish(predicted_uav2_anchoring_point_pose_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_anchoring_point_pose_publisher_.getTopic().c_str());
  }
  try {
    predicted_uav1_anchoring_point_vel_publisher_.publish(predicted_uav1_anchoring_point_vel_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_anchoring_point_vel_publisher_.getTopic().c_str());
  }
  try {
    predicted_uav2_anchoring_point_vel_publisher_.publish(predicted_uav2_anchoring_point_vel_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_anchoring_point_vel_publisher_.getTopic().c_str());
  }

  try {
    predicted_uav1_thrust_publisher_.publish(predicted_uav1_thrust_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_thrust_publisher_.getTopic().c_str());
  }
  try {
    predicted_uav2_thrust_publisher_.publish(predicted_uav2_thrust_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_thrust_publisher_.getTopic().c_str());
  }
  // predicted_uav1_attitude_rate_publisher_:
  try {
    predicted_uav1_attitude_rate_publisher_.publish(predicted_uav1_attitude_rate_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_attitude_rate_publisher_.getTopic().c_str());
  }
  // predicted_uav2_attitude_rate_publisher_:
  try {
    predicted_uav2_attitude_rate_publisher_.publish(predicted_uav2_attitude_rate_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_attitude_rate_publisher_.getTopic().c_str());
  }
  // predicted_uav1_swing_angle_publisher_:
  try {
    predicted_uav1_swing_angle_publisher_.publish(predicted_uav1_swing_angle_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_swing_angle_publisher_.getTopic().c_str());
  }
  // predicted_uav2_swing_angle_publisher_:
  try {
    predicted_uav2_swing_angle_publisher_.publish(predicted_uav2_swing_angle_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_swing_angle_publisher_.getTopic().c_str());
  }
  // predicted_uav1_tension_force_publisher_:
  try {
    predicted_uav1_tension_force_publisher_.publish(predicted_uav1_tension_force_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav1_tension_force_publisher_.getTopic().c_str());
  }

  // predicted_uav2_tension_force_publisher_:
  try {
    predicted_uav2_tension_force_publisher_.publish(predicted_uav2_tension_force_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", predicted_uav2_tension_force_publisher_.getTopic().c_str());
  }

}

std::tuple< double, Eigen::Vector3d,double> DergbryanTracker::ComputeSe3CopyController(const mrs_msgs::UavState uavi_state, Eigen::Matrix3d uavi_R, Eigen::Vector3d Payloadposition_vector, Eigen::Vector3d Payloadvelocity_vector,mrs_msgs::PositionCommand uavi_position_cmd ,Eigen::Vector3d uavi_Rp,Eigen::Vector3d uavi_Rv,Eigen::Vector3d uavi_Rpl,Eigen::Vector3d uavi_Ra){
  // Computes the Se3CopyController for 1 UAV with or without payload.
  // But since for 2 UAVs with payload, the control law is distributed, one can use this controller.
  // Make sure to put as argument the data of this UAV, and in the correct type, and it will return the force vector (in frame W) and the attitude rate vector (also in frame W).
  // As this is used for predictions, no integral action is added.
  // The following is just the exact same lines as in the controller section, no additoinnal comments needed.
  // |--------------------------------------------------------------------------------------|
  // TODO: try to make this function using ony C++ eigen and all ROS specific (msg, pub/sub should be called in additional functions on if needed to simulate this.)
  // TODO: Don't rely on globally defined vars, but get the required params from explicit argument assignment.
  // TODO: Generalize I/O of this function such that get args on RobotState, RobotReference, RobotParams, ControlParams it returns based on model and  ControlAction
  // TODO: implement functionaity to saturate variables or not.
  // TODO: Make a lib so this function can be used both in the controller itself and in the tracker and there is only one place where changes are required.

  // init arguments to correct type and name
  
  // | --------------------- define system states --------------------- |
  mrs_msgs::UavState uav_state = uavi_state; // init the state of the UAV number i to a local var, from its pointer in the arguments.

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d Ov(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);

  // R - current uav attitude
  // Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);
  Eigen::Matrix3d R = uavi_R; //Orientation of the UAV, rotation matrix

  // Ow - UAV angular rate
  // Eigen::Vector3d Ow(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z); // TODO: actually not used (see Se3Controller CTU)

  // payload anchoring point state:

  // Opl - position load in global frame
  // Ovl - velocity load in global frame

  // TODO: why here not using Eigen vectors for Opl and Ovl as in controller?
  geometry_msgs::Vector3 load_pose_position; // transform the vector3d into a vector3, as its used like this after in the code. 
  load_pose_position.x = Payloadposition_vector[0];
  load_pose_position.y = Payloadposition_vector[1];
  load_pose_position.z = Payloadposition_vector[2];
  Eigen::Vector3d load_lin_vel = Payloadvelocity_vector;

  Eigen::Vector3d Opl(load_pose_position.x, load_pose_position.y, load_pose_position.z);
  Eigen::Vector3d Ovl(load_lin_vel[0], load_lin_vel[1], load_lin_vel[2]);

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------
  
  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference

  Eigen::Vector3d Rp = uavi_Rp;
  Eigen::Vector3d Rv = uavi_Rv;
  Eigen::Vector3d Ra = uavi_Ra;
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3); // TODO: add as input of function

  mrs_msgs::PositionCommand position_cmd = uavi_position_cmd;
  // TODO: why don't we put the if cases to construct Rp, Rv, Ra based on a position_cmd here? 

  /* test streaming the references Rp, Rv, Ra when the uav is moving.*/
  // ROS_INFO_STREAM("Rp = \n" << Rp);
  // ROS_INFO_STREAM("Rv = \n" << Rv);
  // ROS_INFO_STREAM("Ra = \n" << Ra);


  // | -------------- calculate the control errors -------------- |
  // position control error
  Eigen::Vector3d Ep = Eigen::Vector3d::Zero(3);

  if (position_cmd.use_position_horizontal || position_cmd.use_position_vertical) {
    Ep = Op - Rp;
  }
  // velocity control error
  Eigen::Vector3d Ev = Eigen::Vector3d::Zero(3);

  if (position_cmd.use_velocity_horizontal || position_cmd.use_velocity_vertical ||
      position_cmd.use_position_vertical) {  // even when use_position_vertical to provide dampening
    Ev = Ov - Rv;
  }
  /*VALIDATE: test streaming the errors.*/
  // ROS_INFO_STREAM("Ep = \n" << Ep);
  // ROS_INFO_STREAM("Ev = \n" << Ev);

  // | --------------------------LOAD--------------------------|
  // --------------------------------------------------------------
  // |          load the control reference and errors             |
  // --------------------------------------------------------------
  // TODO: Rpl and Rvl not defined, nor used. Should be the uav ref cable length down. Not used in actual error of Pandolfo.
  // Rpl - position reference load in global frame
  // Rvl - velocity reference load in global frame
  //Eigen::Vector3d Rpl = uavi_Rpl; // actually not used any further as current Pandolfo controller does not require this.

  Eigen::Vector3d Epl = Eigen::Vector3d::Zero(3); // Load position control error
  Eigen::Vector3d Evl = Eigen::Vector3d::Zero(3); // Load velocity control error

  if (_run_type_ == "simulation" && payload_spawned_){
    // load position control error
    if (position_cmd.use_position_horizontal || position_cmd.use_position_vertical) {
      Eigen::Vector3d e3(0.0, 0.0, 1.0);
      Epl = Op - _cable_length_*e3 - Opl; // relative to uav base frame poiting from the anchoring point to the stable equilibrium beneuth the uav, according to Pandolfo / thesis Raphael: Anchoring point realignment
      // 2021 student said this : Op - Opl is super unstable!! However, this is the way Pandolfo thesis explained it. And I think what they did (Rp - Opl) will always be more unstable, as for a very far references, the control actions of the error of the UAV and the one of the Payload will superpose and generate a huge Td, which can easilly saturates the actuators and creates instability. 
      // TODO: I thought Raphael only looks to the posiiton error in xy (ignoring z)? check this.
    }
    // load velocity control error
    if (position_cmd.use_velocity_horizontal || position_cmd.use_velocity_vertical ||
      position_cmd.use_position_vertical) {  // even when use_position_vertical to provide dampening
      Evl = Ov - Ovl; // speed relative to base frame
    }
    // Epl[2] = 0;
    // Evl[2] = 0;
  } 
  else if(_run_type_ == "uav" && payload_spawned_){ 

    if (position_cmd.use_position_horizontal || position_cmd.use_position_vertical) {
      Eigen::Vector3d e3(0.0, 0.0, 1.0);
      Epl = Op - _cable_length_*e3 - Opl; 
    }
    // load velocity control error
    if (position_cmd.use_velocity_horizontal || position_cmd.use_velocity_vertical ||
      position_cmd.use_position_vertical) {  // even when use_position_vertical to provide dampening
      Evl = Ov - Ovl; // speed relative to base frame
    }

    if (Epl.norm()>_cable_length_*sqrt(2)){ //Sanity check : Largest possible error when cable is oriented 90degrees. If Epl is larger, there must be an error in the FK, or in the values returned by the encoders.
      ROS_ERROR("[se3_copy_controller]: Control error of the anchoring point Epl was larger than expected, it has been set to zero");
      Epl= Eigen::Vector3d::Zero(3);
      payload_spawned_=false; // TODO: check this case
    }
      

  }

  // Ignore small load position errors to deal with load offsets and prevent agressive actions on small errors
  // TODO: make a yaml param of 0.05 threshold  
  if(payload_spawned_ && Epl.norm() < 0.05){ // When the payload is very close to equilibrium vertical position, the error is desactivated so the UAV doesn't try to compensate and let it damp naturally.
    Epl = Eigen::Vector3d::Zero(3);
  }

  // | --------------------- load the gains --------------------- |
  // NOTE: do not move the gains outside the for loop or this function! Due to "Kp = Kp * (uav_mass_ + uav_mass_difference_);"
  /*NOTE: for now on disable the filterGains used in the original se3 controller*/
  /*QUESTION: shouldn't we add this back later?*/
  //filterGains(control_reference->disable_position_gains, dt);

  Eigen::Vector3d Ka = Eigen::Vector3d::Zero(3);
  Eigen::Array3d  Kp = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kv = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kq = Eigen::Array3d::Zero(3);

  {
    std::scoped_lock lock(mutex_gains_);

    if (position_cmd.use_position_horizontal) {
      Kp[0] = kpxy_;
      Kp[1] = kpxy_;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (position_cmd.use_position_vertical) {
      Kp[2] = kpz_;
    } else {
      Kp[2] = 0;
    }

    if (position_cmd.use_velocity_horizontal) {
      Kv[0] = kvxy_;
      Kv[1] = kvxy_;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }
    
    if (position_cmd.use_velocity_vertical) {
      Kv[2] = kvz_;
    } 
    else if (position_cmd.use_position_vertical) {  // special case: want to control z-pos but not the velocity => at least provide z dampening
      Kv[2] = kvz_;
    } else {
      Kv[2] = 0;
    }

    if (position_cmd.use_acceleration) {
      Ka << kaxy_, kaxy_, kaz_;
    } else {
      Ka << 0, 0, 0;
    }

    // Those gains are set regardless of control_reference setting,
    // because we need to control the attitude.
    Kq << kqxy_, kqxy_, kqz_;
  }
  // Print to test if the gains are loaded correctly:
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Ka_x = %f", Ka(0));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Ka_y = %f", Ka(1));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Ka_z = %f", Ka(2));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kq_x = %f", Kq(0));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kq_y = %f", Kq(1));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kq_z = %f", Kq(2));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_x = %f", Kp(0));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_y = %f", Kp(1));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_z = %f", Kp(2));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_x = %f", Kv(0));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_y = %f", Kv(1));
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_z = %f", Kv(2));

  // | --------------------------LOAD--------------------------|
  // Load the gains for the Anchoring point realignment method of Pandolfo: 
  Eigen::Array3d  Kpl = Eigen::Array3d::Zero(3); 
  Eigen::Array3d  Kdl = Eigen::Array3d::Zero(3); 

  if(_type_of_system_ == "1uav_payload" ||_type_of_system_ == "2uavs_and_payload"){
    if (position_cmd.use_position_horizontal) { //TODO load them in global variables as for the other param.  
      Kpl[0] = kplxy_;
      Kpl[1] = kplxy_;
    } else {
      Kpl[0] = 0.0;
      Kpl[1] = 0.0;
    }

    if (position_cmd.use_velocity_horizontal) {
      Kdl[0] = kvlxy_;
      Kdl[1] = kvlxy_;
    } else {
      Kdl[0] = 0.0;
      Kdl[1] = 0.0;
    }  
    // Do not compensate the anchoring point position/speed in the z direction, so its gains are kept on zero
    
    // ROS_INFO_STREAM("Kpl = \n" << Kpl);
    // ROS_INFO_STREAM("Kdl = \n" << Kdl);
  }

  // | ------------------------compute total_mass --------------------------------------|
  //ROS_INFO_STREAM("uav_mass_ \n" << uav_mass_ ); // uav_mass_ is set at the start of the update function and represents the estimated mass (includes uav_mass_difference_ computed in controller)
  
  double total_mass;
  if((_type_of_system_== "1uav_payload" ||_type_of_system_ == "2uavs_and_payload")&& payload_spawned_){ // If system has a payload and this payload has already spawn, we add its mass to the UAV mass to get the total mass of the system.
    if(payload_spawned_){ // for simulation, but also on the hardware, when the controller activates, the load mass is always already suspended by the uav
      total_mass = uav_mass_ + _load_mass_;
      //ROS_INFO_STREAM("payload spwaned" << std::endl << total_mass);
    }else{
      total_mass = uav_mass_; 
      //ROS_INFO_STREAM("payload NOT spwaned" << std::endl << total_mass);
    }
  }else{ // no load case
    total_mass = uav_mass_; //Either no payload in the system, or not yet spawned, total mass is then only the estimated mass of the UAV.
  }
  
  // Scale the gains with the total estimated mass of the system
  Kp = Kp * total_mass;
  Kv = Kv * total_mass;
  Kpl = Kpl *total_mass;
  Kdl = Kdl *total_mass;

  // gains after being mutiplied with total_mass
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_x*m = %f", Kp(0));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_y*m = %f", Kp(1));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kp_z*m = %f", Kp(2));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_x*m = %f", Kv(0));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_y*m = %f", Kv(1));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kv_z*m = %f", Kv(2));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kpl_x*m = %f", Kpl(0));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kpl_y*m= %f", Kpl(1));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kpl_z*m = %f", Kpl(2));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kvl_x*m = %f", Kdl(0));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kvl_y*m = %f", Kdl(1));
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: Kvl_z*m = %f", Kdl(2));
  
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: uav_mass_ (estimated) = %f", uav_mass_);
  if(_type_of_system_ == "1uav_payload" || _type_of_system_ == "2uavs_payload" ){
    ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: _load_mass_ = %f", _load_mass_);
  }
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: total_mass (estimated)= %f", total_mass);
  
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: n_motors = %d", common_handlers_->motor_params.n_motors);
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: motor_params.A = %f", common_handlers_->motor_params.A);
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: motor_params.B = %f", common_handlers_->motor_params.B);

  // | --------------- desired orientation matrix --------------- |

  // ignoring get body integral in the world frame

  // construct the desired force vector
  // Compute control actions 
  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array(); // note: Ep was defined as current - desired
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array(); // note: Ev was defined as current - desired
  Eigen::Vector3d integral_feedback;
  integral_feedback << 0, 0, 0;// ignore integral feedback

  // Load errors were initialized to zero if load not spawned
  Eigen::Vector3d load_position_feedback = -Kpl * Epl.array(); // pushes uav in opposed direction of the position error from anchoring point to load at equilibrium
  Eigen::Vector3d load_velocity_feedback = -Kdl * Evl.array(); // pushes uav in opposed direction of the velocity error from anchoring point to load at equilibrium

  // | -------------------------------se3+load------------------------ | 
  Eigen::Vector3d f = position_feedback + velocity_feedback + load_position_feedback + load_velocity_feedback + feed_forward;
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: fx= %.2f, fy= %.2f, fz=%.2f ",f[0],f[1],f[2]);


  // | ----------- limiting the desired downwards acceleration and maximum tilt angle ---------- |
  // TODO: check with MPC guidage code if this actually makes sense as i remember to have improved it
  if (f[2] < 0) {
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);
    // f << 0, 0, 1; ctu original
    f << f[0], f[1], 0.0; // saturate the z-component on zero such that the desired tilt angle stays in the upper hemisphere
  }
  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // ROS_INFO_STREAM("theta (deg) = \n" << theta*180/3.1415);
  // ROS_INFO_STREAM("phi (deg) = \n" << phi*180/3.1415);


  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[DergbryanTracker]: NaN detected in variable 'theta', predictions can't be trusted!!!!");
    erg_predictions_theta_trusted_ = false;
    // TODO: as it is used here for predictions, we don't want to return as for controllers, but trow the error
    //   return mrs_msgs::AttitudeCommand::ConstPtr();
  } else{
    erg_predictions_theta_trusted_ = true;
  }

  // TODO: as it is used here for predictions, we don't want to return as for controllers, but print error
  // if (_tilt_angle_failsafe_enabled_ && theta > _tilt_angle_failsafe_) {

  //   ROS_ERROR("[DergbryanTracker]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta,
  //             (180.0 / M_PI) * _tilt_angle_failsafe_);
  //   ROS_INFO("[DergbryanTracker]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
  //   ROS_INFO("[DergbryanTracker]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
  //   ROS_INFO("[DergbryanTracker]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
  //   ROS_INFO("[DergbryanTracker]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
  //   ROS_INFO("[DergbryanTracker]: position_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", control_reference->position.x, control_reference->position.y,
  //            control_reference->position.z, control_reference->heading);
  //   ROS_INFO("[DergbryanTracker]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state.pose.position.x, uav_state.pose.position.y,
  //            uav_state.pose.position.z, uav_heading);

  //   return mrs_msgs::AttitudeCommand::ConstPtr();
  // }


  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  if (fabs(constraints.tilt) > 1e-3 && theta > constraints.tilt) {
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta = constraints.tilt;
  }
  //TODORAPHAEL predicted_tilt_angle to publish, do it in here ?? Put it as output ?
  // ROS_INFO_STREAM("theta_sat = \n" << theta*180/3.1415);
  

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | ------------- construct the (desired) rotational matrix ------------ |

  Eigen::Matrix3d Rd;

  if (position_cmd.use_orientation) {
    // ROS_INFO_STREAM("IF position_cmd.use_orientation = \n" << position_cmd.use_orientation);
    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(position_cmd.orientation);

    if (position_cmd.use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(position_cmd.heading);
      } catch (...) {
        ROS_ERROR("[DergbryanTracker]: could not set the desired heading");
      }
    }

  } else {
    // ROS_INFO_STREAM("ELSE = \n" << position_cmd.use_orientation);
    Eigen::Vector3d bxd;  // desired heading vector

    if (position_cmd.use_heading) {
      bxd << cos(position_cmd.heading), sin(position_cmd.heading), 0;
      // ROS_INFO_STREAM("bxd = \n" << bxd);
    } else {
      ROS_ERROR_THROTTLE(1.0, "[DergbryanTracker]: desired heading was not specified, using current heading instead!");
      double uav_heading = uav_heading_;
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    // fill in the desired orientation based on the state feedback
    /* TODO: make it compatible with DRS, now skipped first if */
    //if (drs_params.rotation_type == 0) {
    if(0) {
      // ROS_INFO_STREAM("if(0) = \n");
      Rd.col(2) = f_norm;
      Rd.col(1) = Rd.col(2).cross(bxd);
      Rd.col(1).normalize();
      Rd.col(0) = Rd.col(1).cross(Rd.col(2));
      Rd.col(0).normalize();

    } else {

      // | ------------------------- body z ------------------------- |
      Rd.col(2) = f_norm;

      // | ------------------------- body x ------------------------- |

      // construct the oblique projection
      Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - f_norm * f_norm.transpose());

      // create a basis of the body-z complement subspace
      Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
      A.col(0)          = projector_body_z_compl.col(0);
      A.col(1)          = projector_body_z_compl.col(1);

      // create the basis of the projection null-space complement
      Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
      B.col(0)          = Eigen::Vector3d(1, 0, 0);
      B.col(1)          = Eigen::Vector3d(0, 1, 0);

      // oblique projector to <range_basis>
      Eigen::MatrixXd Bt_A               = B.transpose() * A;
      Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
      Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

      Rd.col(0) = oblique_projector * bxd;
      Rd.col(0).normalize();

      // | ------------------------- body y ------------------------- |

      Rd.col(1) = Rd.col(2).cross(Rd.col(0));
      Rd.col(1).normalize();
    }
  }
  
  // test printing Rd:
  // ROS_INFO_STREAM("Rd = \n" << Rd);

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------
  
  // bool temp_bool = position_cmd.use_attitude_rate;
  // ROS_INFO_STREAM("position_cmd.use_attitude_rate = \n" << temp_bool);

  Eigen::Matrix3d E = Eigen::Matrix3d::Zero();

  if (!position_cmd.use_attitude_rate) {
    E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
  }

  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  /* output */
  double thrust_force = f.dot(R.col(2));
  double thrust = 0.0;
  thrust_saturation_physical_ = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, _thrust_saturation_);
  
  if (!position_cmd.use_thrust) {
    if (thrust_force >= 0) {
      thrust = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, thrust_force);
    } else {
      ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: just so you know, the desired thrust force is negative (%.2f)", thrust_force);
    }
  }
  else {
    // the thrust is overriden from the tracker command
    thrust = position_cmd.thrust;
    ROS_INFO_STREAM("position_cmd.use_thrust = \n" << position_cmd.use_thrust);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[DergbryanTracker]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");

  } else if (thrust > _thrust_saturation_) {

    thrust = _thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: saturating thrust to %.2f", _thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: saturating thrust to 0");
  }

  thrust_force = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, thrust);

  // prepare the attitude feedback
  Eigen::Vector3d q_feedback = -Kq * Eq.array();

  if (position_cmd.use_attitude_rate) {
    Rw << position_cmd.attitude_rate.x, position_cmd.attitude_rate.y, position_cmd.attitude_rate.z;
  } else if (position_cmd.use_heading_rate) {

    // to fill in the feed forward yaw rate
    double desired_yaw_rate = 0;

    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(Rd).getYawRateIntrinsic(position_cmd.heading_rate);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: exception caught while calculating the desired_yaw_rate feedforward");
    }

    Rw << 0, 0, desired_yaw_rate;
  }

  // feedforward angular acceleration
  Eigen::Vector3d q_feedforward = Eigen::Vector3d(0, 0, 0);

  /* TODO: change if case using drs_params */
  //if (drs_params.jerk_feedforward) {
  if (false) {

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(position_cmd.jerk.x, position_cmd.jerk.y, position_cmd.jerk.z);
    q_feedforward                = (I.transpose() * Rd.transpose() * desired_jerk) / (thrust_force / total_mass);
  }

  // angular feedback + angular rate feedforward
  Eigen::Vector3d t = q_feedback + Rw + q_feedforward;

  // compensate for the parasitic heading rate created by the desired pitch and roll rate
  Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

  /* TODO: change if case using drs_params */
  if (true) {
  //if (drs_params.pitch_roll_heading_rate_compensation) {

    Eigen::Vector3d q_feedback_yawless = t;
    q_feedback_yawless(2)              = 0;  // nullyfy the effect of the original yaw feedback

    double parasitic_heading_rate = 0;

    try {
      // TODO: update uav_state.pose.orientation (only R updated now, not the quaternion)
      parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeadingRate(q_feedback_yawless);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: exception caught while calculating the parasitic heading rate!");
    }

    try {
      // TODO: update uav_state.pose.orientation (only R updated now, not the quaternion)
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: exception caught while calculating the parasitic heading rate compensation!");
    }
  }

  t += rp_heading_rate_compensation;

  // ignore integrate the world error 
  // ignore integrate the body error
  // ignore integrate the mass difference

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  // ignore | ------------ compensated desired acceleration ------------ |

  // | --------------- saturate the attitude rate --------------- |

  if (got_constraints_) {

    auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
    
    // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.roll_rate = %f", constraints.roll_rate);
    // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.pitch_rate = %f", constraints.pitch_rate);
    // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.yaw_rate = %f", constraints.yaw_rate);

    if (t[0] > constraints.roll_rate) {
      t[0] = constraints.roll_rate;
    } else if (t[0] < -constraints.roll_rate) {
      t[0] = -constraints.roll_rate;
    }

    if (t[1] > constraints.pitch_rate) {
      t[1] = constraints.pitch_rate;
    } else if (t[1] < -constraints.pitch_rate) {
      t[1] = -constraints.pitch_rate;
    }

    if (t[2] > constraints.yaw_rate) {
      t[2] = constraints.yaw_rate;
    } else if (t[2] < -constraints.yaw_rate) {
      t[2] = -constraints.yaw_rate;
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: missing dynamics constraints");
  }
  
  // | --------------- fill the resulting command --------------- |
  Eigen::Vector3d attitude_rate_pred = t;
    
  return {thrust_force, attitude_rate_pred, theta};
}





Eigen::Vector3d DergbryanTracker::calcCirculationField(std::string type, double dist_x, double dist_y, double dist_z, double dist ){
  Eigen::Vector3d cirulation_field = Eigen::Vector3d::Zero(3);
  if (type == "xy"){
    cirulation_field(0) = _alpha_a_*(dist_y / dist);
    cirulation_field(1) = -_alpha_a_*(dist_x / dist);
  }
  if (type == "xz"){
    cirulation_field(0) = _alpha_a_*(dist_z / dist);
    cirulation_field(2) = -_alpha_a_*(dist_x / dist);
  }
  if (type == "yz"){
    cirulation_field(1) = _alpha_a_*(dist_z / dist);
    cirulation_field(2) = -_alpha_a_*(dist_y / dist);
  }
  if (type == "xyz"){
    cirulation_field(0) = _alpha_a_*(-dist_y + dist_z)/dist;
    cirulation_field(1) = _alpha_a_*( dist_x - dist_z)/dist;
    cirulation_field(2) = _alpha_a_*(-dist_x + dist_y)/dist;  
  }
  return cirulation_field;
}
void DergbryanTracker::computeERG(){

  // Computational time:
  // method Kelly
  // start_ERG_ = std::chrono::system_clock::now();
  // TODO: implement computational time calc also via other methods as done for traj predictions
  std::clock_t c_start = std::clock();
  // | -------------------------------------------------------------------------- |
  //                  ______________________________________________
  //                 | Navigation Field (NF) + Dynamic Safety Margin|
  //                  **********************************************
  // | -------------------------- attraction field -------------------------------|
  Eigen::Vector3d NF_att   = Vector3d::Zero(3, 1); // total attraction field
  Eigen::Vector3d NF_att_leader_follower_rot = Vector3d::Zero(3, 1); // attraction field rotation component
  Eigen::Vector3d NF_att_leader_follower_transl = Vector3d::Zero(3, 1); // attraction field rotation component
    // TODO: declare vars, publish in follower, subscribe in leader, check their delay
  double _eta_transl_ = _eta_; // TODO make yaml
  if(_type_of_system_!="2uavs_payload"){
    Eigen::Vector3d ref_dist = Vector3d::Zero(3, 1); // difference between target reference r and applied reference v
    ref_dist(0) = goal_x_ - applied_ref_x_;
    ref_dist(1) = goal_y_ - applied_ref_y_;
    ref_dist(2) = goal_z_ - applied_ref_z_;
    NF_att = ref_dist/std::max(ref_dist.norm(), _eta_);
  }
  else if(_type_of_system_=="2uavs_payload"){
    Eigen::Vector3d ref_dist_leader = Vector3d::Zero(3, 1); // difference between leader's target reference r and applied reference v
    ref_dist_leader(0) = goal_x_ - applied_ref_x_;
    ref_dist_leader(1) = goal_y_ - applied_ref_y_;
    ref_dist_leader(2) = goal_z_ - applied_ref_z_;
    Eigen::Vector3d ref_dist_follower = Vector3d::Zero(3, 1); // difference between follower's target reference r and applied reference v
    ref_dist_follower(0) = goal_position_cmd_follower_for_leader_.position.x - position_cmd_follower_for_leader_.position.x;
    ref_dist_follower(1) = goal_position_cmd_follower_for_leader_.position.y - position_cmd_follower_for_leader_.position.y;
    ref_dist_follower(2) = goal_position_cmd_follower_for_leader_.position.z - position_cmd_follower_for_leader_.position.z;
    Eigen::Vector3d ref_dist = Vector3d::Zero(3, 1); // difference between the average target reference r and applied reference v of leader and follower
    ref_dist = (ref_dist_leader + ref_dist_follower)/2.0; // difference between the average target reference r and applied reference v of leader and follower
    NF_att_leader_follower_transl = ref_dist/std::max(ref_dist.norm(), _eta_transl_);
  }


  // 
  double V_Lyap;
  Eigen::MatrixXd P_Lyap(6,6);
  Eigen::VectorXd state_vec_V_Lyap(6,1);
  if (_DSM_type_== 1){ //Level-set based
  // TODO: consider making a function for the invariant sets as was done for the trajectory predictions
    state_vec_V_Lyap << uav_state_.pose.position.x - applied_ref_x_,
                        uav_state_.pose.position.y - applied_ref_y_,
                        uav_state_.pose.position.z - applied_ref_z_,
                        uav_state_.velocity.linear.x,
                        uav_state_.velocity.linear.y,
                        uav_state_.velocity.linear.z;
    if (_Lyapunov_type_ == 1){
      // TODO compute analytic expression for kxy different from kz. The point is that the theory assumes the worst case (min Gamma) that works for arbitrary directions.
      // So one should compute for all xyz direction and take the worst case.
      P_Lyap << kpxy_+_epsilon_*pow(kvxy_,2.0), 0.0, 0.0, _epsilon_*kvxy_, 0.0, 0.0,
            0.0, kpxy_+_epsilon_*pow(kvxy_,2.0), 0.0, 0.0, _epsilon_*kvxy_, 0.0,
            0.0, 0.0, kpz_+_epsilon_*pow(kvz_,2.0), 0.0, 0.0, _epsilon_*kvz_,
            _epsilon_*kvxy_, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, _epsilon_*kvxy_, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, _epsilon_*kvz_, 0.0, 0.0, 1.0;
      P_Lyap = 0.5*P_Lyap;
      V_Lyap = state_vec_V_Lyap.transpose()*P_Lyap*state_vec_V_Lyap;
    }
  }

  // | ------------------------ control input constraints ----------------------- |
  // Total thrust saturation:
  // TODO: type 1 nog applicable yet for load transport, use type 2 instead!
  if (_DSM_type_ == 1){ // Level-set based
      double Gamma_sTmax;
      double Gamma_sTmin;
      double total_mass = common_handlers_->getMass(); // total estimated mass calculated by controller
      thrust_saturation_physical_ = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, _thrust_saturation_);
      
    if (_Lyapunov_type_ == 1){
      // TODO compute analytic expression for kxy different from kz. The point is that the theory assumes the worst case (min Gamma) that works for arbitrary directions.
      // So one should compute for all xyz direction and take the worst case.
      Gamma_sTmax = 0.5*pow((thrust_saturation_physical_-total_mass*common_handlers_->g),2.0)/pow(total_mass,2.0)*(kpxy_+_epsilon_*(1-_epsilon_)*pow(kvxy_,2.0))/(pow(kpxy_,2.0)+pow(kvxy_,2.0)*(kpxy_+_epsilon_*pow(kvxy_,2.0)-2.0*_epsilon_*kpxy_));
      Gamma_sTmin = 0.5*pow((_T_min_-total_mass*common_handlers_->g),2.0)/pow(total_mass,2.0)*(kpxy_+_epsilon_*(1.0-_epsilon_)*pow(kvxy_,2.0))/(pow(kpxy_,2.0)+pow(kvxy_,2.0)*(kpxy_+_epsilon_*pow(kvxy_,2.0)-2.0*_epsilon_*kpxy_));
    }
    else if (_Lyapunov_type_ == 2){ // Optimally aligned case:
      // TODO generalize, computed offline for kpxy=3, kdxy=2 and mass=3.5kg
      P_Lyap << 0.7248, 0.0, 0.0, 0.4240, 0.0, 0.0,
                0.0, 0.7248, 0.0, 0.0, 0.4240, 0.0,
                0.0, 0.0, 0.7248, 0.0, 0.0, 0.4240,
                0.4240, 0.0, 0.0, 0.3510, 0.0, 0.0,
                0.0, 0.4240, 0.0, 0.0, 0.3510, 0.0,
                0.0, 0.0, 0.4240, 0.0, 0.0, 0.3510;
      V_Lyap = state_vec_V_Lyap.transpose()*P_Lyap*state_vec_V_Lyap;
      //double denum = 74.88; // TODO generalize, computed offline for kpxy=3, kdxy=2 and mass=2.40 f450
      double denum = 159.25; // TODO generalize, computed offline for kpxy=3, kdxy=2 and mass=3.50 t650
      Gamma_sTmax = pow((thrust_saturation_physical_-total_mass*common_handlers_->g),2.0)/denum;
      Gamma_sTmin = pow((_T_min_-total_mass*common_handlers_->g),2.0)/denum;
    }
    DSM_sT_ = _kappa_sT_*(std::min(Gamma_sTmax,Gamma_sTmin) - V_Lyap)/std::min(Gamma_sTmax,Gamma_sTmin);
    
    // double DSM_sT_max = _kappa_sT_*(Gamma_sTmax - V_Lyap)/Gamma_sTmax;
    // double DSM_sT_min = _kappa_sT_*(Gamma_sTmin - V_Lyap)/Gamma_sTmin;
    // DSM_sT_ = std::min(DSM_sT_max,DSM_sT_min)
    DSM_msg_.DSM_s = DSM_sT_;
  }
  else if (_DSM_type_== 2){ // Trajectory based
    if(_type_of_system_!="2uavs_payload"){
      DSM_sT_ = computeDSM_sT_trajpred(predicted_thrust_out_);
      DSM_msg_.DSM_s = DSM_sT_;
      //ROS_INFO_STREAM("DSM_sT_ = \n" << DSM_sT_);
    } else{ // _type_of_system_=="2uavs_payload"
      // uav1:
      DSM_sT_uav1_ = computeDSM_sT_trajpred(predicted_uav1_thrust_out_);
      DSM_uav1_msg_.DSM_s = DSM_sT_uav1_;
      // ROS_INFO_STREAM("DSM_sT_uav1_ = \n" << DSM_sT_uav1_);
      // uav2:
      DSM_sT_uav2_ = computeDSM_sT_trajpred(predicted_uav2_thrust_out_);
      DSM_uav2_msg_.DSM_s = DSM_sT_uav2_;
      // ROS_INFO_STREAM("DSM_sT_uav2_ = \n" << DSM_sT_uav2_);
      // min of both uav1 and uav2:
      DSM_sT_ = std::min(DSM_sT_uav1_, DSM_sT_uav2_);
      DSM_msg_.DSM_s = DSM_sT_;
      // ROS_INFO_STREAM("DSM_sT_ = \n" << DSM_sT_);
    }
  }
 
  //ROS_INFO_STREAM("before Body rate saturation \n");
  // Body rate saturation:
  if (got_constraints_ && _DSM_type_ == 2 && _enable_dsm_sw_) { // _DSM_type_ = 1 not possible sinc elevel-set based on outer loop only
    if(_type_of_system_!="2uavs_payload"){
      if (_predicitons_use_body_inertia_) { // we implement constraints on the actual body rates
        DSM_sw_ = computeDSM_sw_trajpred(predicted_attituderate_out_);
      }
      else{
        DSM_sw_ = computeDSM_sw_trajpred(predicted_des_attituderate_out_);
      }
      DSM_msg_.DSM_sw = DSM_sw_;
      //ROS_INFO_STREAM("DSM_sw_ = \n" << DSM_sw_);
    } else{ // _type_of_system_=="2uavs_payload"
      // uav1 & uav2:
      if (_predicitons_use_body_inertia_) { // TODO: implement this case
        DSM_sw_uav1_ = computeDSM_sw_trajpred(predicted_uav1_attitude_rate_out_);
        DSM_sw_uav2_ = computeDSM_sw_trajpred(predicted_uav2_attitude_rate_out_);
        ROS_ERROR_STREAM("[DergbryanTracker]: _predicitons_use_body_inertia_ ignored as this case is not implemented yet for 2uavs_payload!");
        // DSM_sw_uav1_ = computeDSM_sw_trajpred(predicted_uav1_des_attituderate_out_);
        // DSM_sw_uav2_ = computeDSM_sw_trajpred(predicted_uav2_des_attituderate_out_);
      }
      else{ // we implement constraints on the actual body rates
        DSM_sw_uav1_ = computeDSM_sw_trajpred(predicted_uav1_attitude_rate_out_);
        DSM_sw_uav2_ = computeDSM_sw_trajpred(predicted_uav2_attitude_rate_out_);
      }
      DSM_uav1_msg_.DSM_sw = DSM_sw_uav1_;
      //ROS_INFO_STREAM("DSM_sw_uav1_ = \n" << DSM_sw_uav1_);
      DSM_uav2_msg_.DSM_sw = DSM_sw_uav2_;
      //ROS_INFO_STREAM("DSM_sw_uav2_ = \n" << DSM_sw_uav2_);
      // min of both uav1 and uav2:
      DSM_sw_ = std::min(DSM_sw_uav1_, DSM_sw_uav2_);
      DSM_msg_.DSM_sw = DSM_sw_;
      //ROS_INFO_STREAM("DSM_sw_ = \n" << DSM_sw_);
    }
  } else {
    if(_type_of_system_!="2uavs_payload"){
      DSM_sw_ = _kappa_sw_; // the highest possible value
      DSM_msg_.DSM_sw = DSM_sw_;
    } else{ // _type_of_system_=="2uavs_payload"
      DSM_sw_ = _kappa_sw_; // the highest possible value
      DSM_sw_uav1_ = DSM_sw_;
      DSM_sw_uav2_ = DSM_sw_;
      DSM_uav1_msg_.DSM_sw = DSM_sw_uav1_;
      DSM_uav2_msg_.DSM_sw = DSM_sw_uav2_;
      DSM_msg_.DSM_sw = DSM_sw_;
    }
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: missing dynamics constraints");
  }

  //ROS_INFO_STREAM("before swing angle constraints \n");
  // Cable suspended UAV constraints (swing angle and taut cable)
  if (_DSM_type_== 2){ // Trajectory based
    if(_type_of_system_== "1uav_payload" && payload_spawned_ && _enable_dsm_swing_c_){ //Swing angle DSM
      DSM_swing_c_ = computeDSM_swing_trajpred(predicted_swing_angle_out_);
      DSM_msg_.DSM_swing_c = DSM_swing_c_;
      //ROS_INFO_STREAM("DSM_swing_c_ = \n" << DSM_swing_c_);
    }
    else if(_type_of_system_=="2uavs_payload" && payload_spawned_ && callback_data_follower_no_delay_ && _enable_dsm_swing_c_){
      // uav1:
      DSM_swing_c_uav1_ = computeDSM_swing_trajpred(predicted_uav1_swing_angle_out_);
      DSM_uav1_msg_.DSM_swing_c = DSM_swing_c_uav1_;
      // ROS_INFO_STREAM("DSM_swing_c_uav1_ = \n" << DSM_swing_c_uav1_);
      // uav2:
      DSM_swing_c_uav2_ = computeDSM_swing_trajpred(predicted_uav2_swing_angle_out_);
      DSM_uav2_msg_.DSM_swing_c = DSM_swing_c_uav2_;
      // ROS_INFO_STREAM("DSM_swing_c_uav1_ = \n" << DSM_swing_c_uav1_);
      // min of both uav1 and uav2:
      DSM_swing_c_ = std::min(DSM_swing_c_uav1_, DSM_swing_c_uav2_);
      DSM_msg_.DSM_s = DSM_swing_c_;
      // ROS_INFO_STREAM("DSM_swing_c_ = \n" << DSM_swing_c_);
    }

    //ROS_INFO_STREAM("before cable tension constraints \n");
    if(_type_of_system_== "1uav_payload" && payload_spawned_ && _enable_dsm_Tc_){ // Tension cable DSM
      DSM_Tc_ = computeDSM_Tc_trajpred(predicted_Tc_out_);
      DSM_msg_.DSM_Tc = DSM_Tc_;
      //ROS_INFO_STREAM("DSM_Tc_ = \n" << DSM_Tc_);
    }
    else if(_type_of_system_=="2uavs_payload" && payload_spawned_ && callback_data_follower_no_delay_ && _enable_dsm_Tc_){
      // uav1:
      DSM_Tc_uav1_ = computeDSM_Tc_trajpred(predicted_uav1_tension_force_out_);
      DSM_uav1_msg_.DSM_Tc = DSM_Tc_uav1_;
      // ROS_INFO_STREAM("DSM_Tc_uav1_ = \n" << DSM_Tc_uav1_);
      // uav2:
      DSM_Tc_uav2_ = computeDSM_Tc_trajpred(predicted_uav2_tension_force_out_);
      DSM_uav2_msg_.DSM_Tc = DSM_Tc_uav2_;
      // ROS_INFO_STREAM("DSM_Tc_uav2_ = \n" << DSM_Tc_uav2_);
      // min of both uav1 and uav2:
      DSM_Tc_ = std::min(DSM_Tc_uav1_, DSM_Tc_uav2_);
      DSM_msg_.DSM_Tc = DSM_Tc_;
      // ROS_INFO_STREAM("DSM_Tc_ = \n" << DSM_Tc_);
    }
  }

  // | --------------------------- repulsion fields ------------------------------|  
  //  - agent: uav collision avoidance
  Eigen::Vector3d NF_a_co  = Vector3d::Zero(3, 1); // conservative part
  Eigen::Vector3d NF_a_nco = Vector3d::Zero(3, 1); // non-conservative part

  Eigen::MatrixXd all_NF_a_co  = MatrixXd::Zero(3, 1); // all conservative parts
  Eigen::MatrixXd all_NF_a_nco = MatrixXd::Zero(3, 1); // all non-conservative parts


  // TODO: for wall and obstacle constraints (group A's code)
  // MatrixXd NF_a_co  = MatrixXd::Zero(3, 1); // conservative part
  // MatrixXd NF_a_nco = MatrixXd::Zero(3, 1); // non-conservative part
  // MatrixXd NF_w = MatrixXd::Zero(3,1);
  // MatrixXd NF_o = MatrixXd::Zero(3,1);
  // MatrixXd NF_o_co = MatrixXd::Zero(3,1);
  // MatrixXd NF_o_nco = MatrixXd::Zero(3,1);
  // tictoc_stack.push(clock());
  // MatrixXd NF_w = MatrixXd::Zero(3,1);
  // MatrixXd NF_o = MatrixXd::Zero(3,1);
  // MatrixXd NF_o_co = MatrixXd::Zero(3,1);
  // MatrixXd NF_o_nco = MatrixXd::Zero(3,1);
  //Frank : Add NF_o_co, NF_o_nco, NF_w AND/Reactivate the lines below
  //Frank : How to code such that differents obstacles are treated the right way ? Back then it worked pretty well but how boutnow ? Check with 1 first then see

  // | --------------------------- repulsion walls -------------------------------|
  // double max_repulsion_wall1;
  // //float arm_radius=0.325; // radius of the quadrotor
  // _d_w_(1,0) = wall_p_x - arm_radius;
  // max_repulsion_wall1= (_zeta_w_-(abs(_d_w_(1,0)-applied_ref_x_)))/(_zeta_w_-_delta_w_);
  // if (0 > max_repulsion_wall1){
  // max_repulsion_wall1=0;
  // }
  // NF_w(0,0)=-max_repulsion_wall1;
  // NF_w(1,0)=0;
  // NF_w(2,0)=0;
  // c_w(0,0)=1;
  // c_w(1,0)=0;
  // c_w(2,0)=0;
  // // _d_w_(1,0) = wall_p_x - arm_radius;
  // min_wall_distance= abs(_d_w_(1,0) - predicted_poses_out_.poses[0].position.x);//custom_trajectory_out.poses[0].position.x);
  // for (size_t i = 0; i < _num_pred_samples_; i++) {
  //   if (abs(_d_w_(1,0) - predicted_poses_out_.poses[i].position.x) < min_wall_distance){
  //     min_wall_distance=abs(_d_w_(1,0) - predicted_poses_out_.poses[i].position.x);
  //   }
  // }
  // DSM_w_=_kappa_w_*min_wall_distance;
  //ROS_INFO_STREAM("DSM_w_ = \n" << DSM_w_);
  // | ----------------------- repulsion static obstacles ------------------------|
  // Conservative part
  // MatrixXd dist_ref_obs_ = MatrixXd::Zero(4, 1);
  // MatrixXd dist_obs_ = MatrixXd::Zero(4, 1); //Frank : 3x1 vector of position with  4th element being the norm
  // double max_repulsion_obs1; 
  // double min_obs_distance; 
  // dist_ref_obs_(0)=obs_position(0,0)-applied_ref_x_; // x distance between v and obstacle 1
  // dist_ref_obs_(1)=obs_position(1,0)-applied_ref_y_; // y distance between v and obstacle 1
  // dist_ref_obs_(3)=sqrt(dist_ref_obs_(0)*dist_ref_obs_(0)+dist_ref_obs_(1)*dist_ref_obs_(1));

  // max_repulsion_obs1=arm_radius+(_zeta_o_-(dist_ref_obs_(3)-R_o_))/(_zeta_o_-_delta_o_); //Frank : Added the arm radius to add a safety around the cylinder
  // if (0>max_repulsion_obs1) {
  // max_repulsion_obs1=0;
  // }

  // NF_o_co(0,0)=-(max_repulsion_obs1*(dist_ref_obs_(0)/dist_ref_obs_(3)));
  // NF_o_co(1,0)=-(max_repulsion_obs1*(dist_ref_obs_(1)/dist_ref_obs_(3)));
  // NF_o_co(2,0)=0;

  // // Non-conservative part
  // NF_o_nco(2,0)=0;
  // if (_zeta_o_>=dist_ref_obs_(3)-R_o_) {
  // NF_o_nco(0,0)=_alpha_o_*(dist_ref_obs_(1)/dist_ref_obs_(3));
  // NF_o_nco(1,0)=-_alpha_o_*(dist_ref_obs_(0)/dist_ref_obs_(3));
  // } else {
  // NF_o_nco(0,0)=0;
  // NF_o_nco(1,0)=0;
  // }
  // // Both combined
  // NF_o(0,0)=NF_o_co(0,0)+NF_o_nco(0,0);
  // NF_o(1,0)=NF_o_co(1,0)+NF_o_nco(1,0);
  // NF_o(2,0)=NF_o_co(2,0)+NF_o_nco(2,0);
  
  //DSM_o_ computation
  //// Frank: Implement this and see if it changes the performances or not
  //// Implementation with the lambda 
      //  Eigen::Vector3d point_mu_link0;
     // Eigen::Vector3d point_nu_link1;
     // std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_applied_ref, point_link_star, point_link_applied_ref_other_uav, point_link_star_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);

     // double dist_x = point_nu_link1(0) - point_mu_link0(0);
     // double dist_y = point_nu_link1(1) - point_mu_link0(1);
     // double dist_z = point_nu_link1(2) - point_mu_link0(2);

    //  double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
  ///// Prediction part  with lambda 
  // for (size_t i = 0; i < _num_pred_samples_; i++) {
  //    // shortest distance from predicted pos to tube link
  //    Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
  //    double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos); 
  //    double norm;
  //    if ((lambda <= 1) && (lambda > 0)) {
  //      Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
  //     norm = (point_link_lambda - point_predicted_pos).norm();  
  //    }
  //    else if (lambda <= 0){
  //      norm = (point_link_applied_ref - point_predicted_pos).norm(); 
  //    }
  //    else { // (lambda > 1)
  //      norm = (point_link_star - point_predicted_pos).norm(); 
  //    }
  //
  //    DSM_a_temp = _kappa_a_*(_Sa_perp_max_ - norm)/_Sa_perp_max_; // in _kappa_a_*[0 , 1]
  //    if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
  //      DSM_a_ = DSM_a_temp;
  //    }

  // MatrixXd dist_ref_obs_ = MatrixXd::Zero(4, 1);
  // MatrixXd dist_obs_ = MatrixXd::Zero(4, 1); //Frank : 3x1 vector of position with  4th element being the norm
  // double max_repulsion_obs1; 
  // double min_obs_distance;


  // dist_obs_(0)=predicted_poses_out_.poses[0].position.x-obs_position(0,0);
  // dist_obs_(1)=predicted_poses_out_.poses[0].position.y-obs_position(1,0);
  // dist_obs_(3)=sqrt(dist_obs_(0)*dist_obs_(0)+dist_obs_(1)*dist_obs_(1));

  // min_obs_distance=dist_obs_(3)-R_o_;
  // for (size_t i = 1; i < _num_pred_samples_; i++) {
  //   dist_obs_(0)=predicted_poses_out_.poses[i].position.x-obs_position(0,0);
  //   dist_obs_(1)=predicted_poses_out_.poses[i].position.y-obs_position(1,0);

  //   dist_obs_(3)=sqrt(dist_obs_(0)*dist_obs_(0)+dist_obs_(1)*dist_obs_(1));
  //   if (dist_obs_(3)-R_o_<min_obs_distance) {
  //     min_obs_distance=dist_obs_(3)-R_o_;
  //   }
  // }

  // DSM_o_=_kappa_o_*min_obs_distance;
  // ROS_INFO_STREAM("DSM o, s, and w calculation took = \n "<< (double)(clock()- tictoc_stack.top())/CLOCKS_PER_SEC << "seconds.");
  // tictoc_stack.pop();
  // tictoc_stack.push(clock());
  
  //ROS_INFO_STREAM("before collision avoidance \n");
  // | ------------------------ agent collision avoidance repulsion ----------------------- |
  if (_enable_dsm_a_ && _DERG_strategy_id_ == 0) {
    /* D-ERG strategy 0:
    The agents share reference positions as the centers of fixed-size spheres.
    TODO one could also consider morphing spheres instead of fixed size spheres.
    */

    max_repulsion_other_uav_ = 0.0; // amplitude of max repulsive term
    max_NF_a_co_  = Vector3d::Zero(3, 1); // NF term related with max_repulsion_other_uav_

    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it = other_uavs_applied_references_.begin();
    // ROS_INFO_STREAM("p^v_x = \n" << it->second.points[0].x);
    // ROS_INFO_STREAM("p^v_y = \n" << it->second.points[0].y);
    // ROS_INFO_STREAM("p^v_z = \n" << it->second.points[0].z);
    while (it != other_uavs_applied_references_.end()) {
      double other_uav_ref_x = it->second.points[0].x; // Second means accessing the second part of the iterator. Here it is FutureTrajectory.
      double other_uav_ref_y = it->second.points[0].y;
      double other_uav_ref_z = it->second.points[0].z;

      double dist_x = other_uav_ref_x - applied_ref_x_;
      double dist_y = other_uav_ref_y - applied_ref_y_;
      double dist_z = other_uav_ref_z - applied_ref_z_;

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }

      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
      //ROS_INFO_STREAM("dist = \n" << dist);
      // Conservative part:
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - 2*_Sa_max_))/(_zeta_a_ - _delta_a_);
      if (0.0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0.0;
      }

      Eigen::Vector3d NF_a_co_temp = Vector3d::Zero(3, 1);
      NF_a_co_temp(0,0) = -norm_repulsion_other_uav*(dist_x/dist);
      NF_a_co_temp(1,0) = -norm_repulsion_other_uav*(dist_y/dist);
      NF_a_co_temp(2,0) = -norm_repulsion_other_uav*(dist_z/dist);

      if (norm_repulsion_other_uav >= 1.0){ // assuming the NF only penetrates 1 constraint boundary delta   t a time (i.e. >1)
        max_repulsion_other_uav_ = norm_repulsion_other_uav;
        max_NF_a_co_ = NF_a_co_temp;
        consider_projection_NF_on_max_NF_a_co_ = true;
      }

      NF_a_co(0,0)=NF_a_co(0,0) + NF_a_co_temp(0,0);
      NF_a_co(1,0)=NF_a_co(1,0) + NF_a_co_temp(1,0);
      NF_a_co(2,0)=NF_a_co(2,0) + NF_a_co_temp(2,0);

      // Non-conservative part:
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - 2*_Sa_max_) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it++;
    }

    DSM_a_ = 100000; // large value
    double DSM_a_temp;
    double Gamma_a;
    if (_DSM_type_  == 1){ // Invariant level set based DSM: 
      if (_Lyapunov_type_ == 1){
        Gamma_a = 0.5*(kpxy_ + _epsilon_*(1.0-_epsilon_)*pow(kvxy_,2.0))*pow(_Sa_max_,2.0); // TODO: compute analytic expression for kxy different rom kz
      }
      else if (_Lyapunov_type_ == 2){ // Optimally aligned case:
        // TODO generalize, computed offline for kpxy=3, kdxy=2 and mass=3.5kg
        P_Lyap << 1.0225, 0.0, 0.0, 0.0620, 0.0, 0.0,
                  0.0, 1.0225, 0.0, 0.0, 0.0620, 0.0,
                  0.0, 0.0, 1.0225, 0.0, 0.0, 0.0620,
                  0.0620, 0.0, 0.0, 0.1706, 0.0, 0.0,
                  0.0, 0.0620, 0.0, 0.0, 0.1706, 0.0,
                  0.0, 0.0, 0.0620, 0.0, 0.0, 0.1706;
        V_Lyap = state_vec_V_Lyap.transpose()*P_Lyap*state_vec_V_Lyap;
        double denum = 1.0; // TODO always normalized
        Gamma_a = pow(_Sa_max_,2.0)/denum;
    }
      
      
      
      
      double DSM_a_temp = _kappa_a_*(Gamma_a - V_Lyap)/Gamma_a; // in _kappa_a_*[0 , 1]
        
      if (DSM_a_temp < DSM_a_){  
        DSM_a_ = DSM_a_temp;
      }
    }
    else if(_DSM_type_ == 2){ // Trajectory based DSM:
      for (size_t i = 0; i < _num_pred_samples_; i++) {
        double pos_error_x = applied_ref_x_ - predicted_poses_out_.poses[i].position.x;
        double pos_error_y = applied_ref_y_ - predicted_poses_out_.poses[i].position.y;
        double pos_error_z = applied_ref_z_ - predicted_poses_out_.poses[i].position.z;

        double pos_error = sqrt(pos_error_x*pos_error_x + pos_error_y*pos_error_y + pos_error_z*pos_error_z);
        
        DSM_a_temp = _kappa_a_*(_Sa_max_ - pos_error)/_Sa_max_; // in _kappa_a_*[0 , 1]
        
        if (DSM_a_temp < DSM_a_){  
        DSM_a_ = DSM_a_temp;
        }
      } 
    }

    




  }
  // TODO: clean up strategy 1 2 3, 5
  if (_enable_dsm_a_ && _DERG_strategy_id_ == 1) {
    /* D-ERG strategy 1
    */

    // this uav:
    DSM_a_ = 100000; // large value //Frank : Explanation of the lambda function
    double DSM_a_temp;
    Eigen::Vector3d point_link_pos(predicted_poses_out_.poses[0].position.x, predicted_poses_out_.poses[0].position.y, predicted_poses_out_.poses[0].position.z);
    Eigen::Vector3d point_link_applied_ref(applied_ref_x_, applied_ref_y_, applied_ref_z_);
    Eigen::Vector3d point_link_star; // lambda = 1
    if ((point_link_pos - point_link_applied_ref).norm() > 0.001){
      point_link_star = point_link_applied_ref + _Sa_long_max_*(point_link_pos - point_link_applied_ref)/(point_link_pos - point_link_applied_ref).norm();
    }
    else{ // avoid /0
      point_link_star = point_link_applied_ref;
    }
    // other uavs:
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it1 = other_uavs_applied_references_.begin();
    //std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it2 = other_uavs_positions_.begin();

    // ROS_INFO_STREAM("pv uav1 = \n" << other_uavs_applied_references_["uav1"]);

    while ((it1 != other_uavs_applied_references_.end()) ) {// Frank : Maybe do the same for the other DSMs
      // make sure we use the same uavid for both iterators. It must be robust to possible difference in lenths of the iterators (if some communication got lost) or a different order (e.g. if not auto alphabetical).
      std::string this_uav_id = it1->first;
      try
      {
        mrs_msgs::FutureTrajectory temp_pose = other_uavs_positions_[this_uav_id];
        //ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
      }
      catch(...)
      {
        // other_uavs_positions_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }
            
      
      // ROS_INFO_STREAM("inside \n");
      // other uavs:
      double other_uav_applied_ref_x = it1->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      double other_uav_applied_ref_y = it1->second.points[0].y;
      double other_uav_applied_ref_z = it1->second.points[0].z;
      Eigen::Vector3d point_link_applied_ref_other_uav(other_uav_applied_ref_x, other_uav_applied_ref_y, other_uav_applied_ref_z);
      
      // double other_uav_pos_x = it2->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      // double other_uav_pos_y = it2->second.points[0].y;
      // double other_uav_pos_z = it2->second.points[0].z;
      double other_uav_pos_x = other_uavs_positions_[this_uav_id].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[this_uav_id].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[this_uav_id].points[0].z;
      Eigen::Vector3d point_link_pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);

      Eigen::Vector3d point_link_star_other_uav; // lambda = 1
      // isfinite to check if data are communicated and received well
      // TODO: check if data received are not too "old"!!!!!!
      if (isfinite(point_link_pos_other_uav.norm()) && isfinite(point_link_applied_ref_other_uav.norm()) && ((point_link_pos_other_uav - point_link_applied_ref_other_uav).norm() > 0.001)){
        point_link_star_other_uav = point_link_applied_ref_other_uav + _Sa_long_max_*(point_link_pos_other_uav - point_link_applied_ref_other_uav)/(point_link_pos_other_uav - point_link_applied_ref_other_uav).norm();
      }
      else{ // avoid /0
        point_link_star_other_uav = point_link_applied_ref_other_uav;
        // TODO now also force uav to switch to small _Sa_perp_max_ !!!!
      }

      Eigen::Vector3d point_mu_link0;
      Eigen::Vector3d point_nu_link1;
      std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_applied_ref, point_link_star, point_link_applied_ref_other_uav, point_link_star_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);

      double dist_x = point_nu_link1(0) - point_mu_link0(0);
      double dist_y = point_nu_link1(1) - point_mu_link0(1);
      double dist_z = point_nu_link1(2) - point_mu_link0(2);

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }

      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
      
      bool use_min_tube_radius = false;
      if (!use_min_tube_radius){
        // DSM: 2 tubes of radius _Sa_perp_max_ may not overlap
        // DSM scaled over the influence margin
        DSM_a_temp = _kappa_a_*(dist - 2*Ra_ - 2*_Sa_perp_max_) / _zeta_a_; // in _kappa_a_*[0 , 1]
        // OR (see below)
        // TODO other option!!! Own predicted trajectory (= own orange tube) may not overlap blue tube of other agent
        // --> should give less conservative performance
        if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the agents
          DSM_a_ = DSM_a_temp;
        }
      }


      // Conservative part
      if (!_use_tube_)
      {
        double dist = (point_link_applied_ref_other_uav - point_link_applied_ref).norm();
      }
      
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - 2*_Sa_perp_max_))/(_zeta_a_ - _delta_a_);
      if (0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0;
      }

      NF_a_co(0,0) = NF_a_co(0,0) - norm_repulsion_other_uav*(dist_x / dist);
      NF_a_co(1,0) = NF_a_co(1,0) - norm_repulsion_other_uav*(dist_y / dist);

      if(!_use_tube_){ // If tube used NF_a_co in z should be 0.
      NF_a_co(2,0) = NF_a_co(2,0) - norm_repulsion_other_uav*(dist_z / dist);
      }

      // Non-conservative part
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - 2*_Sa_perp_max_) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it1++;
      //it2++;
    }

    
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      // shortest distance from predicted pos to tube link
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos); 
      double norm;
      if ((lambda <= 1) && (lambda > 0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
        norm = (point_link_lambda - point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref - point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_star - point_predicted_pos).norm(); 
      }

      DSM_a_temp = _kappa_a_*(_Sa_perp_max_ - norm)/_Sa_perp_max_; // in _kappa_a_*[0 , 1]
      if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
        DSM_a_ = DSM_a_temp;
      }

      // if (use_min_tube_radius){
      //   // TODO other option!!! Own predicted trajectory (= own orange tube) may not overlap blue tube of other agent
      //   // --> should give less conservative performance
      //   // DSM scaled over the influence margin
      //   DSM_a_temp = _kappa_a_*(dist - 2*Ra_ - 2*_Sa_perp_max_) / _zeta_a_; // in _kappa_a_*[0 , 1]

      //   if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the agents
      //     DSM_a_ = DSM_a_temp;
      //   }
      // }

      
    }
    DSM_msg_.DSM_a = DSM_a_;
    // added by Titouan and Jonathan
    if(_enable_visualization_){
      tf::pointEigenToMsg(point_link_star, point_link_star_.position);  // conversion from Eigen::Vector3d to geometry_msgs::Point
    }
  }

  if (_enable_dsm_a_ && _DERG_strategy_id_ == 2) {
    /* D-ERG strategy 2: a tube of variable length _Sa_long_ and fixed width _Sa_perp_max_. 
    One hemisphere is centered in the applied reference and the other hemisphere is centered
    in point_link_pos, which lies on the longitudinal tube's axis "link" a distance _Sa_long_max_ 
    in a direction from the applied reference towards the current position.
    */
    // this uav:
    DSM_a_ = 100000; // large value
    double DSM_a_temp;
    Eigen::Vector3d point_link_pos(predicted_poses_out_.poses[0].position.x, predicted_poses_out_.poses[0].position.y, predicted_poses_out_.poses[0].position.z);
    Eigen::Vector3d point_link_applied_ref(applied_ref_x_, applied_ref_y_, applied_ref_z_);
    Eigen::Vector3d point_link_star; // lambda = 1
    if ((point_link_pos - point_link_applied_ref).norm() > 0.001){
      point_link_star = point_link_applied_ref + _Sa_long_max_*(point_link_pos - point_link_applied_ref)/(point_link_pos - point_link_applied_ref).norm();
    }
    else{ // avoid /0
      point_link_star = point_link_applied_ref;
    }

    // other uavs:
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it1 = other_uavs_applied_references_.begin();
    //std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it2 = other_uavs_positions_.begin();

    // ROS_INFO_STREAM("pv uav1 = \n" << other_uavs_applied_references_["uav1"]);

    while ((it1 != other_uavs_applied_references_.end()) ) {
      // make sure we use the same uavid for both iterators. It must be robust to possible difference in lenths of the iterators (if some communication got lost) or a different order (e.g. if not auto alphabetical).
      std::string this_uav_id = it1->first;
      try
      {
        mrs_msgs::FutureTrajectory temp_pose = other_uavs_positions_[this_uav_id];
        //ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
      }
      catch(...)
      {
        // other_uavs_positions_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }
            
      
      // ROS_INFO_STREAM("inside \n");
      // other uavs:
      double other_uav_applied_ref_x = it1->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      double other_uav_applied_ref_y = it1->second.points[0].y;
      double other_uav_applied_ref_z = it1->second.points[0].z;
      Eigen::Vector3d point_link_applied_ref_other_uav(other_uav_applied_ref_x, other_uav_applied_ref_y, other_uav_applied_ref_z);
      
      // double other_uav_pos_x = it2->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      // double other_uav_pos_y = it2->second.points[0].y;
      // double other_uav_pos_z = it2->second.points[0].z;
      double other_uav_pos_x = other_uavs_positions_[this_uav_id].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[this_uav_id].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[this_uav_id].points[0].z;
      Eigen::Vector3d point_link_pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);

    
      Eigen::Vector3d point_mu_link0;
      Eigen::Vector3d point_nu_link1;
      std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_applied_ref, point_link_pos, point_link_applied_ref_other_uav, point_link_pos_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);
      
      double dist_x = point_nu_link1(0) - point_mu_link0(0);
      double dist_y = point_nu_link1(1) - point_mu_link0(1);
      double dist_z = point_nu_link1(2) - point_mu_link0(2);

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }

      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
      

      bool use_min_tube_radius = false;
      if (!use_min_tube_radius){
        // DSM: 2 tubes of radius _Sa_perp_max_ may not overlap
        // DSM scaled over the influence margin
        DSM_a_temp = _kappa_a_*(dist - 2*Ra_ - 2*_Sa_perp_max_) / _zeta_a_; // in _kappa_a_*[0 , 1]
        // OR (see below)
        // TODO other option!!! Own predicted trajectory (= own orange tube) may not overlap blue tube of other agent
        // --> should give less conservative performance
        if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the agents
          DSM_a_ = DSM_a_temp;
        }
      }


      // Conservative part
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - 2*_Sa_perp_max_))/(_zeta_a_ - _delta_a_);
      if (0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0;
      }

      NF_a_co(0,0)=NF_a_co(0,0) - norm_repulsion_other_uav*(dist_x/dist);
      NF_a_co(1,0)=NF_a_co(1,0) - norm_repulsion_other_uav*(dist_y/dist);
      NF_a_co(2,0)=NF_a_co(2,0) - norm_repulsion_other_uav*(dist_z/dist);

      // Non-conservative part
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - 2*_Sa_perp_max_) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it1++;
      //it2++;
    }

    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_star - point_predicted_pos).norm(); 
      }
      DSM_a_temp = _kappa_a_*(_Sa_perp_max_-norm)/_Sa_perp_max_;
      if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
        DSM_a_ = DSM_a_temp;
      }


      
    }
    DSM_msg_.DSM_a = DSM_a_;
    // added by Titouan and Jonathan
    if(_enable_visualization_){
      tf::pointEigenToMsg(point_link_star, point_link_star_.position);  // conversion from Eigen::Vector3d to geometry_msgs::Point
    }
  }
  if (_enable_dsm_a_ && _DERG_strategy_id_ == 3) {
    /* D-ERG strategy 3: a tube of variable length _Sa_long_ and fixed width _Sa_perp_max_. 
    One hemisphere is centered in the applied reference and the other hemisphere is centered
    in point_link_pos, which lies on the longitudinal tube's axis "link" a distance _Sa_long_max_ 
    in a direction from the applied reference towards the current position.
    */
    // this uav:
    DSM_a_ = 100000; // large value
    double DSM_a_temp;
    Eigen::Vector3d point_link_pos(predicted_poses_out_.poses[0].position.x, predicted_poses_out_.poses[0].position.y, predicted_poses_out_.poses[0].position.z);
    Eigen::Vector3d point_link_applied_ref(applied_ref_x_, applied_ref_y_, applied_ref_z_);
    Eigen::Vector3d point_link_star; // lambda = 1
    if ((point_link_pos - point_link_applied_ref).norm() > 0.001){
      point_link_star = point_link_applied_ref + _Sa_long_max_*(point_link_pos - point_link_applied_ref)/(point_link_pos - point_link_applied_ref).norm();
    }
    else{ // avoid /0
      point_link_star = point_link_applied_ref;
    }
    // tube p, pstar
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_star - point_predicted_pos).norm(); 
      }
      DSM_a_temp = _kappa_a_*(_Sa_perp_max_-norm)/_Sa_perp_max_;
      if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
        DSM_a_ = DSM_a_temp;
      } 
    }
    // compute Sa_perp_min
    // * cannot use previous for loop since we did not compute yet the distances between predicted points in hemisphere centred in p to p itself
    double Sa_perp_min = 0; // initialize at the min possible value
    // tube p , pv
    // TODO !!!we loop over large number of same point resulting in same distance, as in prev loop. Skip these ehre to reduce compute load.
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_pos, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_pos - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_pos - point_predicted_pos).norm(); 
      }

     
      if (norm > Sa_perp_min){  // choose largest Sa_perp_min over the predicted trajectory to obtain smallest tube radius
        Sa_perp_min = norm;
      } 
    }
    Sa_perp_.data = Sa_perp_min; // update global variable    
    // publish Sa_perp_ so other uavs can use it (just a test, not actually used)
    try {
      tube_min_radius_publisher_.publish(Sa_perp_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", tube_min_radius_publisher_.getTopic().c_str());
    }
    // we actually use this
    trackers_brubotics::FutureTrajectoryTube future_tube;
    future_tube.stamp = uav_state_.header.stamp;//ros::Time::now();
    future_tube.uav_name = _uav_name_;
    future_tube.priority = avoidance_this_uav_priority_;
    // future_tube.collision_avoidance = true;
    future_tube.min_radius = Sa_perp_.data;
    try {
      future_tube_publisher_.publish(future_tube);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", future_tube_publisher_.getTopic().c_str());
    }



    

    // other uavs:
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it1 = other_uavs_applied_references_.begin();
    //std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it2 = other_uavs_positions_.begin();
    //std::map<std::string, trackers_brubotics::FutureTrajectoryTube>::iterator it3 = other_uav_tube_.begin();

    
    while ((it1 != other_uavs_applied_references_.end())){// || (it2 != other_uavs_positions_.end()) || (it3 != other_uav_tube_.end()) ) {
       // make sure we use the same uavid for both iterators. It must be robust to possible difference in lenths of the iterators (if some communication got lost) or a different order (e.g. if not auto alphabetical).
      std::string this_uav_id = it1->first;
      try
      {
        mrs_msgs::FutureTrajectory temp_pose = other_uavs_positions_[this_uav_id];
      }
      catch(...)
      {
        // other_uavs_positions_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }

      try
      {
        trackers_brubotics::FutureTrajectoryTube temp_tube = other_uav_tube_[this_uav_id];
      }
      catch(...)
      {
        // other_uav_tube_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated tube information corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }
      
      
      
      // other uavs:
      double other_uav_ref_x = it1->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      double other_uav_ref_y = it1->second.points[0].y;
      double other_uav_ref_z = it1->second.points[0].z;
      Eigen::Vector3d point_link_applied_ref_other_uav(other_uav_ref_x, other_uav_ref_y, other_uav_ref_z);

      // double other_uav_pos_x = it2->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      // // ROS_INFO_STREAM("other_uav_pos_x = \n" << other_uav_pos_x);
      // double other_uav_pos_y = it2->second.points[0].y;
      // double other_uav_pos_z = it2->second.points[0].z;
      double other_uav_pos_x = other_uavs_positions_[this_uav_id].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[this_uav_id].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[this_uav_id].points[0].z;
      Eigen::Vector3d point_link_pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);
      

      Eigen::Vector3d point_mu_link0;
      Eigen::Vector3d point_nu_link1;
      std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_applied_ref, point_link_pos, point_link_applied_ref_other_uav, point_link_pos_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);
      
      double dist_x = point_nu_link1(0) - point_mu_link0(0);
      double dist_y = point_nu_link1(1) - point_mu_link0(1);
      double dist_z = point_nu_link1(2) - point_mu_link0(2);

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }

      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    

      bool use_min_tube_radius = true; // delete min in name since it just changes take ti as global variable
      double Sa_perp_other_uav;
      if (use_min_tube_radius){
        // DSM: 2 tubes of radius _Sa_perp_min_ and _Sa_perp_min_other_uav_ may not overlap
        // DSM scaled over the influence margin
        // either other uav uses max or share its own min of Sa
        //(_Sa_perp_min_ + _Sa_perp_max_other)
        //or
        // (_Sa_perp_min_ + _Sa_perp_min_other)
        // make subsciber and publisher for this!!
        //note : Sa_perp_ = Sa_perp_min

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        // Sa_perp_other_uav = it3->second.min_radius;
        Sa_perp_other_uav = other_uav_tube_[this_uav_id].min_radius;
        
        // OR
        // Sa_perp_min_other_uav = _Sa_perp_max_     // --> should give more conservative performance
        
        
        
        
        //ROS_INFO_STREAM("Sa_perp_other_uav = \n" << Sa_perp_other_uav);
         
        DSM_a_temp = _kappa_a_*(dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)) / _zeta_a_; // in _kappa_a_*[0 , 1]
        
     
        if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the agents
          DSM_a_ = DSM_a_temp;
        }
      }


      // Conservative part
      // same comment as before!!!
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)))/(_zeta_a_ - _delta_a_);
      if (0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0;
      }

      NF_a_co(0,0)=NF_a_co(0,0) - norm_repulsion_other_uav*(dist_x/dist);
      NF_a_co(1,0)=NF_a_co(1,0) - norm_repulsion_other_uav*(dist_y/dist);
      NF_a_co(2,0)=NF_a_co(2,0) - norm_repulsion_other_uav*(dist_z/dist);

      // Non-conservative part
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it1++;
      //it2++;
      //it3++;
    }
    DSM_msg_.DSM_a = DSM_a_;
    // added by Titouan and Jonathan
    if(_enable_visualization_){
      tf::pointEigenToMsg(point_link_star, point_link_star_.position);  // conversion from Eigen::Vector3d to geometry_msgs::Point
    }
  }

  if (_enable_dsm_a_ && _DERG_strategy_id_ == 4 && _DSM_type_== 2){ // Trajectory based) {
    /* D-ERG strategy 4: 
    */
    max_repulsion_other_uav_ = 0.0; // amplitude of max repulsive term
    max_NF_a_co_  = Vector3d::Zero(3, 1); // NF term related with max_repulsion_other_uav_

    // this uav:
    DSM_a_ = 100000; // large value
    double DSM_a_temp;
    Eigen::Vector3d point_link_pos(predicted_poses_out_.poses[0].position.x, predicted_poses_out_.poses[0].position.y, predicted_poses_out_.poses[0].position.z);
    Eigen::Vector3d point_link_applied_ref(applied_ref_x_, applied_ref_y_, applied_ref_z_);
    Eigen::Vector3d point_link_star; // lambda = 1
    if ((point_link_pos - point_link_applied_ref).norm() > 0.001){
      point_link_star = point_link_applied_ref + _Sa_long_max_*(point_link_pos - point_link_applied_ref)/(point_link_pos - point_link_applied_ref).norm();
    }
    else{ // avoid /0
      point_link_star = point_link_applied_ref;
    }
    // tube p, pstar
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_star - point_predicted_pos).norm(); 
      }
      DSM_a_temp = _kappa_a_*(_Sa_perp_max_-norm)/_Sa_perp_max_;
      if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
        DSM_a_ = DSM_a_temp;
      } 
    }
    // compute Sa_perp_min, pk0, pk1
    // * cannot use previous for loop since we did not compute yet the distances between predicted points in hemisphere centred in p to p itself
    double Sa_perp_min = 0; // initialize at the min possible value
    Eigen::Vector3d point_link_p0 = point_link_applied_ref; // initialize at the min possible value
    Eigen::Vector3d point_link_p1 = point_link_pos; // initialize at the min possible value
    double norm_long_p0 = 0; // initialize at the min possible value
    double norm_long_p1 = 0; // initialize at the min possible value
    // tube p , pv
    // TODO !!!we loop over large number of same point resulting in same distance, as in prev loop. Skip these ehre to reduce compute load.
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_pos, point_predicted_pos);
      double norm_perp;

      Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_pos - point_link_applied_ref);
      norm_perp = (point_link_lambda - point_predicted_pos).norm();
      // if ((lambda <= 1) && (lambda > 0)) { 
      //   norm = (point_link_lambda - point_predicted_pos).norm();
      // }
      if (lambda <= 0){
        // norm = (point_link_applied_ref-point_predicted_pos).norm(); 
        double norm_long_p0_temp = (point_link_applied_ref - point_link_lambda).norm();
        if (norm_long_p0_temp > norm_long_p0){  // choose largest norm_long_pk0 over the predicted trajectory to obtain smallest tube length
          norm_long_p0 = norm_long_p0_temp;
          point_link_p0 = point_link_lambda;
        } 
      }
      else if (lambda > 1){ // 
        // norm = (point_link_pos - point_predicted_pos).norm(); 
        double norm_long_p1_temp = (point_link_pos - point_link_lambda).norm();
        if (norm_long_p1_temp > norm_long_p1){  // choose largest norm_long_pk1 over the predicted trajectory to obtain smallest tube length
          norm_long_p1 = norm_long_p1_temp;
          point_link_p1 = point_link_lambda;
        } 
      }
    ////////////////////
    // what's happens in best case:
    // point_link_p0 = point_link_applied_ref; // initialize at the min possible value
    // point_link_p1 = point_link_pos; // initialize at the min possible value
    // norm_long_p0 = 0; // initialize at the min possible value
    // norm_long_p1 = 0; // initialize at the min possible value
    ///////////////////////////
      if (norm_perp > Sa_perp_min){  // choose largest Sa_perp_min over the predicted trajectory to obtain smallest tube radius
        Sa_perp_min = norm_perp;
      } 
    }
    Sa_perp_.data = Sa_perp_min; // update global variable    
    // publish Sa_perp_ so other uavs can use it (just a test, not actually used)
    try {
      tube_min_radius_publisher_.publish(Sa_perp_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", tube_min_radius_publisher_.getTopic().c_str());
    }
    // we actually use this
    trackers_brubotics::FutureTrajectoryTube future_tube;
    future_tube.stamp = uav_state_.header.stamp;//ros::Time::now();
    future_tube.uav_name = _uav_name_;
    future_tube.priority = avoidance_this_uav_priority_;
    // future_tube.collision_avoidance = true;
    future_tube.min_radius = Sa_perp_.data;
    future_tube.p0.x = point_link_p0[0];
    future_tube.p0.y = point_link_p0[1];
    future_tube.p0.z = point_link_p0[2];
    future_tube.p1.x = point_link_p1[0];
    future_tube.p1.y = point_link_p1[1];
    future_tube.p1.z = point_link_p1[2];
    try {
      future_tube_publisher_.publish(future_tube);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", future_tube_publisher_.getTopic().c_str());
    }



    

    // other uavs:
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it1 = other_uavs_applied_references_.begin();
    //std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it2 = other_uavs_positions_.begin();
    //std::map<std::string, trackers_brubotics::FutureTrajectoryTube>::iterator it3 = other_uav_tube_.begin();

    
    while ((it1 != other_uavs_applied_references_.end())){// || (it2 != other_uavs_positions_.end()) || (it3 != other_uav_tube_.end()) ) {
       // make sure we use the same uavid for both iterators. It must be robust to possible difference in lenths of the iterators (if some communication got lost) or a different order (e.g. if not auto alphabetical).
      std::string this_uav_id = it1->first;
      trackers_brubotics::FutureTrajectoryTube temp_tube;
      try
      {
        mrs_msgs::FutureTrajectory temp_pose = other_uavs_positions_[this_uav_id];
      }
      catch(...)
      {
        // other_uavs_positions_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }

      try
      {
        temp_tube = other_uav_tube_[this_uav_id];
      }
      catch(...)
      {
        // other_uav_tube_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated tube information corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }
      
      
      
      // other uavs:
      double other_uav_ref_x = it1->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      double other_uav_ref_y = it1->second.points[0].y;
      double other_uav_ref_z = it1->second.points[0].z;
      Eigen::Vector3d point_link_applied_ref_other_uav(other_uav_ref_x, other_uav_ref_y, other_uav_ref_z);

      double other_uav_pos_x = other_uavs_positions_[this_uav_id].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[this_uav_id].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[this_uav_id].points[0].z;
      Eigen::Vector3d point_link_pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);
      
      double other_uav_p0_x = temp_tube.p0.x;//Second means accessing the second part of the iterator.
      double other_uav_p0_y = temp_tube.p0.y;
      double other_uav_p0_z = temp_tube.p0.z;
      Eigen::Vector3d point_link_p0_other_uav(other_uav_p0_x, other_uav_p0_y, other_uav_p0_z);

      double other_uav_p1_x = temp_tube.p1.x;//Second means accessing the second part of the iterator.
      double other_uav_p1_y = temp_tube.p1.y;
      double other_uav_p1_z = temp_tube.p1.z;
      Eigen::Vector3d point_link_p1_other_uav(other_uav_p1_x, other_uav_p1_y, other_uav_p1_z);

      Eigen::Vector3d point_mu_link0;
      Eigen::Vector3d point_nu_link1;
      std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_p0, point_link_p1, point_link_p0_other_uav, point_link_p1_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);
      
      double dist_x = point_nu_link1(0) - point_mu_link0(0);
      double dist_y = point_nu_link1(1) - point_mu_link0(1);
      double dist_z = point_nu_link1(2) - point_mu_link0(2);

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }


      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    

      bool use_min_tube_radius = true; // delete min in name since it just changes take ti as global variable
      double Sa_perp_other_uav;
      if (use_min_tube_radius){
        // DSM: 2 tubes of radius _Sa_perp_min_ and _Sa_perp_min_other_uav_ may not overlap
        // DSM scaled over the influence margin
        // either other uav uses max or share its own min of Sa
        //(_Sa_perp_min_ + _Sa_perp_max_other)
        //or
        // (_Sa_perp_min_ + _Sa_perp_min_other)
        // make subsciber and publisher for this!!
        //note : Sa_perp_ = Sa_perp_min

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        // Sa_perp_other_uav = it3->second.min_radius;
        Sa_perp_other_uav = other_uav_tube_[this_uav_id].min_radius;
        
        // OR
        // Sa_perp_min_other_uav = _Sa_perp_max_     // --> should give more conservative performance
        
        
        
        
        //ROS_INFO_STREAM("Sa_perp_other_uav = \n" << Sa_perp_other_uav);
         
        DSM_a_temp = _kappa_a_*(dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)) / _zeta_a_; // in _kappa_a_*[0 , 1]
        
     
        if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the agents
          DSM_a_ = DSM_a_temp;
        }
      }


      // Conservative part
      // same comment as before!!!
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)))/(_zeta_a_ - _delta_a_);
      if (0.0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0.0;
      }

      Eigen::Vector3d NF_a_co_temp = Vector3d::Zero(3, 1);
      NF_a_co_temp(0,0) = -norm_repulsion_other_uav*(dist_x/dist);
      NF_a_co_temp(1,0) = -norm_repulsion_other_uav*(dist_y/dist);
      NF_a_co_temp(2,0) = -norm_repulsion_other_uav*(dist_z/dist);

      if (norm_repulsion_other_uav >= 1.0){ // assuming the NF only penetrates 1 constraint boundary delta   t a time (i.e. >1)
        max_repulsion_other_uav_ = norm_repulsion_other_uav;
        max_NF_a_co_ = NF_a_co_temp;
        consider_projection_NF_on_max_NF_a_co_ = true;
      }

      NF_a_co(0,0)=NF_a_co(0,0) + NF_a_co_temp(0,0);
      NF_a_co(1,0)=NF_a_co(1,0) + NF_a_co_temp(1,0);
      NF_a_co(2,0)=NF_a_co(2,0) + NF_a_co_temp(2,0);
      // https://www.py4u.net/discuss/81139
      all_NF_a_co.conservativeResize(all_NF_a_co.rows(), all_NF_a_co.cols()+1);
      all_NF_a_co.col(all_NF_a_co.cols()-1) = NF_a_co_temp;


      // Non-conservative part
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it1++;
      //it2++;
      //it3++;
    }

    bool enable_only_max_rep_term = false; // don't sum all agent repulsions, but only the largest repulsive contribution
    if(enable_only_max_rep_term) {
      double max_norm = 0.0; //init
      for (size_t i = 0; i < all_NF_a_co.cols(); i++) 
        if ((all_NF_a_co.col(i)).norm()>max_norm) {
          max_norm = (all_NF_a_co.col(i)).norm();
          NF_a_co = all_NF_a_co.col(i);
          // TODO later NF_a_nco =  
        }
      }
    DSM_msg_.DSM_a = DSM_a_;
    // added by Titouan and Jonathan
    if(_enable_visualization_){
      tf::pointEigenToMsg(point_link_star, point_link_star_.position);  // conversion from Eigen::Vector3d to geometry_msgs::Point
    }
  }

  if (_enable_dsm_a_ && _DERG_strategy_id_ == 5) {
    /* D-ERG strategy 5: 
      
    */
    // this uav:
    DSM_a_ = 100000; // large value
    double DSM_a_temp;
    Eigen::Vector3d point_link_pos(predicted_poses_out_.poses[0].position.x, predicted_poses_out_.poses[0].position.y, predicted_poses_out_.poses[0].position.z);
    Eigen::Vector3d point_link_applied_ref(applied_ref_x_, applied_ref_y_, applied_ref_z_);
    Eigen::Vector3d point_link_star; // lambda = 1
    if ((point_link_pos - point_link_applied_ref).norm() > 0.001){
      point_link_star = point_link_applied_ref + _Sa_long_max_*(point_link_pos - point_link_applied_ref)/(point_link_pos - point_link_applied_ref).norm();
    }
    else{ // avoid /0
      point_link_star = point_link_applied_ref;
    }
    // tube p, pstar
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_star, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_star - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_star - point_predicted_pos).norm(); 
      }
      DSM_a_temp = _kappa_a_*(_Sa_perp_max_-norm)/_Sa_perp_max_;
      if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
        DSM_a_ = DSM_a_temp;
      } 
    }



    // compute Sa_perp_min
    // * cannot use previous for loop since we did not compute yet the distances between predicted points in hemisphere centred in p to p itself
    double Sa_perp_min = 0; // initialize at the min possible value
    // tube p , pv
    // TODO !!!we loop over large number of same point resulting in same distance, as in prev loop. Skip these ehre to reduce compute load.
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      Eigen::Vector3d point_predicted_pos(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
      double lambda = getLambda(point_link_applied_ref, point_link_pos, point_predicted_pos);
      double norm;
      if ((lambda <= 1) && (lambda >0)) {
        Eigen::Vector3d point_link_lambda = point_link_applied_ref + lambda*(point_link_pos - point_link_applied_ref);
        norm = (point_link_lambda-point_predicted_pos).norm();  
      }
      else if (lambda <= 0){
        norm = (point_link_applied_ref-point_predicted_pos).norm(); 
      }
      else { // (lambda > 1)
        norm = (point_link_pos - point_predicted_pos).norm(); 
      }

     
      if (norm > Sa_perp_min){  // choose largest Sa_perp_min over the predicted trajectory to obtain smallest tube radius
        Sa_perp_min = norm;
      } 
    }
    Sa_perp_.data = Sa_perp_min; // update global variable    
    // publish Sa_perp_ so other uavs can use it (just a test, not actually used)
    try {
      tube_min_radius_publisher_.publish(Sa_perp_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", tube_min_radius_publisher_.getTopic().c_str());
    }
    // we actually use this
    trackers_brubotics::FutureTrajectoryTube future_tube;
    future_tube.stamp = uav_state_.header.stamp;//ros::Time::now();
    future_tube.uav_name = _uav_name_;
    future_tube.priority = avoidance_this_uav_priority_;
    // future_tube.collision_avoidance = true;
    future_tube.min_radius = Sa_perp_.data;
    try {
      future_tube_publisher_.publish(future_tube);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", future_tube_publisher_.getTopic().c_str());
    }




    // trajectory
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it = other_uav_avoidance_trajectories_.begin();
    while (it != other_uav_avoidance_trajectories_.end()) {
      // ROS_INFO_STREAM("INSIDE WHILE = \n");
      for (size_t i = 0; i < _num_pred_samples_; i++) {
        Eigen::Vector3d point_traj(predicted_poses_out_.poses[i].position.x, predicted_poses_out_.poses[i].position.y, predicted_poses_out_.poses[i].position.z);
        // ROS_INFO_STREAM("UAV position = \n" << point_traj);
        double other_uav_pos_x = it->second.points[i].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
        double other_uav_pos_y = it->second.points[i].y;
        double other_uav_pos_z = it->second.points[i].z;
        Eigen::Vector3d point_other_uav_traj(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);
        //ROS_INFO_STREAM("Other UAV position = \n" << point_other_uav_traj);
        double norm = (point_traj - point_other_uav_traj).norm()-2*Ra_; 
        DSM_a_temp = _kappa_a_*(norm)/ _zeta_a_; // in _kappa_a_*[0 , 1]
        if (DSM_a_temp < DSM_a_){  // choose smallest DSM_a_ over the predicted trajectory
          DSM_a_ = DSM_a_temp;
        }
      }
      it++;
    }

    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it1 = other_uavs_applied_references_.begin();
    //std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it2 = other_uavs_positions_.begin();
    
    while ((it1 != other_uavs_applied_references_.end())) { //
      std::string this_uav_id = it1->first;
      try
      {
        mrs_msgs::FutureTrajectory temp_traj = other_uav_avoidance_trajectories_[this_uav_id];
      }
      catch(...)
      {
        // other_uav_avoidance_trajectories_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated trajectory prediction corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }

      try
      {
        trackers_brubotics::FutureTrajectoryTube temp_tube = other_uav_tube_[this_uav_id];
      }
      catch(...)
      {
        // other_uav_tube_[this_uav_id] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated tube information corresponding to the applied reference of %s \n", this_uav_id.c_str());
        it1++;
        continue;
      }

      // otherUAV
      double other_uav_ref_x = it1->second.points[0].x;//Second means accessing the second part of the iterator. Here it is FutureTrajectory
      double other_uav_ref_y = it1->second.points[0].y;
      double other_uav_ref_z = it1->second.points[0].z;
      Eigen::Vector3d point_link_applied_ref_other_uav(other_uav_ref_x, other_uav_ref_y, other_uav_ref_z);

      double other_uav_pos_x = other_uavs_positions_[this_uav_id].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[this_uav_id].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[this_uav_id].points[0].z;
      Eigen::Vector3d point_link_pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);


      Eigen::Vector3d point_mu_link0;
      Eigen::Vector3d point_nu_link1;
      std::tie(point_mu_link0, point_nu_link1) = getMinDistDirLineSegments(point_link_applied_ref, point_link_pos, point_link_applied_ref_other_uav, point_link_pos_other_uav);//(point0_link0, point1_link0, point0_link1, point1_link1);
      
      double dist_x = point_nu_link1(0) - point_mu_link0(0);
      double dist_y = point_nu_link1(1) - point_mu_link0(1);
      double dist_z = point_nu_link1(2) - point_mu_link0(2);

      // if we only want to account for distances in the xy plane:
      if (_use_distance_xy_){
        dist_z = 0.0;
      }


      // double dist_x = point_link_applied_ref_other_uav(0) - point_link_applied_ref(0);
      // double dist_y = point_link_applied_ref_other_uav(1) - point_link_applied_ref(1);
      // double dist_z = point_link_applied_ref_other_uav(2) - point_link_applied_ref(2);

      double dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

      bool use_min_tube_radius = true; // delete min in name since it just changes take ti as global variable
      double Sa_perp_other_uav;
      if (use_min_tube_radius){
        
        Sa_perp_other_uav = other_uav_tube_[this_uav_id].min_radius;
      
      }



      // Conservative part
      double norm_repulsion_other_uav = (_zeta_a_ - (dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)))/(_zeta_a_ - _delta_a_);
      if (0 > norm_repulsion_other_uav) {
        norm_repulsion_other_uav = 0;
      }

      NF_a_co(0,0)=NF_a_co(0,0) - norm_repulsion_other_uav*(dist_x/dist);
      NF_a_co(1,0)=NF_a_co(1,0) - norm_repulsion_other_uav*(dist_y/dist);
      NF_a_co(2,0)=NF_a_co(2,0) - norm_repulsion_other_uav*(dist_z/dist);

      // Non-conservative part
      if (_alpha_a_ >= 0.0001){
        if (_zeta_a_ >= dist - 2*Ra_ - (Sa_perp_.data + Sa_perp_other_uav)) {
          NF_a_nco = NF_a_nco + calcCirculationField(_circ_type_, dist_x, dist_y, dist_z, dist);
        }
      }
      it1++;
    } 

  DSM_msg_.DSM_a = DSM_a_;
  // added by Titouan and Jonathan
  if(_enable_visualization_){
    tf::pointEigenToMsg(point_link_star, point_link_star_.position);  // conversion from Eigen::Vector3d to geometry_msgs::Point
  }
  }

  //ROS_INFO_STREAM("before DSM total calc \n");
  // //////////////////////// Determining DSM_total= minimum {DSM_a,DSM_s,DSM_o,DSM_w}////////////////////////////////
  DSM_total_ = 100000; // initialize very high

  if(_enable_dsm_sT_){
    ROS_INFO_THROTTLE(1.0,"[DergbryanTracker]: DSM_sT_ = %f", DSM_sT_);
    if(DSM_sT_ <= DSM_total_){
      DSM_total_ = DSM_sT_;
    }
  }

  if(_enable_dsm_sw_){
    ROS_INFO_THROTTLE(1.0,"[DergbryanTracker]: DSM_sw_ = %f", DSM_sw_);
    if(DSM_sw_ <= DSM_total_){
      DSM_total_ = DSM_sw_;
    }
  }

  if(_enable_dsm_a_){
    ROS_INFO_THROTTLE(1.0,"[DergbryanTracker]: DSM_a_ = %f", DSM_a_);
    if(DSM_a_ <= DSM_total_){
      DSM_total_ = DSM_a_;
    }
  }

  if(_enable_dsm_swing_c_ && payload_spawned_){
    ROS_INFO_THROTTLE(1.0,"[DergbryanTracker]: DSM_swing_c_ = %f", DSM_swing_c_);
    if(DSM_swing_c_ <= DSM_total_ ){
      DSM_total_ = DSM_swing_c_;
    }
  }

  if(_enable_dsm_Tc_ && payload_spawned_){
    ROS_INFO_THROTTLE(1.0,"[DergbryanTracker]: DSM_Tc_ = %f", DSM_Tc_);
    if(DSM_Tc_ <= DSM_total_){
      DSM_total_ = DSM_Tc_;
    }
  }

  if(!_enable_dsm_sT_ && !_enable_dsm_sw_ && !_enable_dsm_a_){
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: all default UAV DSMs (thrust, body rate, agents) are disabled! \n");
  }
  if(!_enable_dsm_swing_c_ && !_enable_dsm_Tc_ && payload_spawned_){
    ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: all payload specific DSMs (swing, cable tension) are disabled although payload is spawned! \n");
  }

  if(_constant_dsm_ >= 0.0){ // if all of the DSMs are disabled or a positive constant DSM is added, TODO add other DSMs here
    DSM_total_ = _constant_dsm_;
    // in case the _constant_dsm_ is negative it will be set to 0 in the next if
  }

  if(DSM_total_ < 0.0){ // make sure it is >= 0
    DSM_total_ = 0.0;
  }

  
  // Frank : uncomment these lines when everything is implemented
  //if(DSM_w <= DSM_total){
  //DSM_total=DSM_w;
  //}
  // if((DSM_o_ <= DSM_total_) && (_enable_dsm_o_)){
  //   DSM_total_=DSM_o_;
  // }
  // //Frank : Add the remaining cases for the other DSMs

  // if((DSM_w_ <= DSM_total_) && (_enable_dsm_w_)){
  //   DSM_total_ = DSM_w_;
  // }




  
  // ROS_INFO_STREAM("DSM_total_ = \n" << DSM_total_);
  // ROS_INFO_STREAM("DSM_sT_ = \n" << DSM_sT_);

  //ROS_INFO_STREAM("DSM_a_ = \n" << DSM_a_);

  // 
 
  //TODO Adding terminal constraint!!!!!





  // Both combined
  Eigen::Vector3d NF_a = NF_a_co + NF_a_nco;
  
  // | ------------------------- total repulsion field ---------------------------|
  Eigen::Vector3d NF_total = Vector3d::Zero(3, 1);
  //Frank : Rework the code above and make lines achieving this result. 
  // NF_total(0,0)=NF_att(0,0)+ NF_a(0,0) +NF_o(0,0) + NF_w(0,0);
  // NF_total(1,0)=NF_att(1,0) + NF_a(1,0) +NF_o(1,0) + NF_w(1,0);
  // NF_total(2,0)=NF_att(2,0) + NF_a(2,0) +NF_o(2,0) + NF_w(2,0);
  NF_total = NF_att + _enable_repulsion_a_*NF_a;// + _enable_repulsion_w_*NF_w + _enable_repulsion_o_*NF_o; 
  //ROS_INFO_STREAM("NF_total = \n" << NF_total);
  //ROS_INFO_STREAM("NF_a = \n" << NF_a);

  if (consider_projection_NF_on_max_NF_a_co_ == true && _enable_repulsion_a_){
    double temp = max_NF_a_co_.dot(NF_total);
    if (temp < 0.0){ // NF_total points inside static delta boundary
      // subtract from the original NF_total the part of NF_total that penetrates into max_NF_a_co_
      NF_total = NF_total - (temp/(max_NF_a_co_.dot(max_NF_a_co_)))*max_NF_a_co_; // https://www.youtube.com/watch?v=qz3Q3v84k9Y
      // this avoids a pure NF consisting of multiple max unitary vectors from penetrating the obstacle
      // this avoids the DSM to become exactly 0 for a strictly positiive delta allowing slow movement near the constraints
    }
    consider_projection_NF_on_max_NF_a_co_ = false; // reset consider_projection_NF_on_max_NF_a_co_ to false
    // Note: the above implementation assumes there is only 1 delta region penetrated simultaneously.
    // TODO: improve that you check for all constraints and recursively apply the projection untill no constraints are penetrated anymore
    // This is currently only implemented for agent constraints under strategy 0 (fixed size spheres) and 4 (morphing tubes) yet. 
  }
  
  // | -------------------------------------------------------------------------- |


  //ROS_INFO_STREAM("before D-ERG output \n");

  // | ------------------------------ D-ERG output -------------------------------|
  MatrixXd applied_ref_dot = MatrixXd::Zero(3, 1); // derivative of the applied reference
  if(_type_of_system_!="2uavs_payload"){
    applied_ref_dot = DSM_total_*NF_total;
    if(erg_predictions_theta_trusted_){
      applied_ref_x_ = applied_ref_x_ + applied_ref_dot(0)*dt_;
      applied_ref_y_ = applied_ref_y_ + applied_ref_dot(1)*dt_;
      applied_ref_z_ = applied_ref_z_ + applied_ref_dot(2)*dt_;
    }
  }
  else if(_type_of_system_=="2uavs_payload"){// && _uav_name_ == "uav1" && payload_spawned_ && callback_data_follower_no_delay_){
    // store leader and follower v and r in Eigen::vector3d
    // applied_ref_leader:
    Eigen::Vector3d applied_ref_leader;
    applied_ref_leader[0] = applied_ref_x_;
    applied_ref_leader[1] = applied_ref_y_;
    applied_ref_leader[2] = applied_ref_z_;
    // applied_ref_follower:
    Eigen::Vector3d applied_ref_follower;
    applied_ref_follower[0] = position_cmd_follower_for_leader_.position.x;
    applied_ref_follower[1] = position_cmd_follower_for_leader_.position.y;
    applied_ref_follower[2] = position_cmd_follower_for_leader_.position.z;
    // goal_leader:  
    Eigen::Vector3d goal_leader;
    goal_leader[0] = goal_x_;
    goal_leader[1] = goal_y_;
    goal_leader[2] = goal_z_;
    // goal_follower:
    Eigen::Vector3d goal_follower;
    goal_follower[0] = goal_position_cmd_follower_for_leader_.position.x;
    goal_follower[1] = goal_position_cmd_follower_for_leader_.position.y;
    goal_follower[2] = goal_position_cmd_follower_for_leader_.position.z;

    // check for steady-state admissibility of references, if invalid adapt follower's reference: 
    // TODO: shouldn't it be better to do this immediately as we receive data? Currently also used in DSM.
    // applied_ref:
    if((applied_ref_follower-applied_ref_leader).norm() != _load_length_){
      applied_ref_follower = applied_ref_leader + _load_length_*(applied_ref_follower-applied_ref_leader)/(applied_ref_follower-applied_ref_leader).norm();
    }
    // goal:
    if((goal_follower-goal_leader).norm() != _load_length_){
      goal_follower = goal_leader + _load_length_*(goal_follower-goal_leader)/(goal_follower-goal_leader).norm();
    }
    // the transformations that follow hereafter (translation and rotation) do not affect the equality constraint
    
    // update applied_ref for translation part:
    applied_ref_dot = DSM_total_*NF_att_leader_follower_transl;
    applied_ref_leader = applied_ref_leader + applied_ref_dot*dt_;
    applied_ref_follower = applied_ref_follower + applied_ref_dot*dt_;

    // update applied_ref for rotation part:
    // compute unit axis of rotation of v:
    Eigen::Vector3d rot_axis = (applied_ref_leader-applied_ref_follower).cross(goal_leader-goal_follower);
    // ROS_INFO_STREAM("rot_axis.x  = " << rot_axis[0]);
    // ROS_INFO_STREAM("rot_axis.y  = " << rot_axis[1]);
    // ROS_INFO_STREAM("rot_axis.z  = " << rot_axis[2]);
    // ROS_INFO_STREAM("rot_axis.norm() = " << rot_axis.norm());
    // max rotation angle required to align v with r
    double max_rot_angle = std::acos((applied_ref_leader-applied_ref_follower).dot(goal_leader-goal_follower)/((applied_ref_leader-applied_ref_follower).norm()*(goal_leader-goal_follower).norm()));
    // define angle of rotation for this v update
    double rot_angle = 0.1*(M_PI/180.0)*DSM_total_;//30.0; // rad, //TODO: make yaml param; // required rotation to be computed for all cases
    if(rot_axis.norm()<0.001){ // both lines v,r between leader-follower are already parallel
      // in case angle near M_PI
      if(max_rot_angle >= M_PI*0.95){
        // choose any unit vector perpendicular to bar. We choose the one with max z coordinate:
        rot_axis = ((applied_ref_leader-applied_ref_follower).cross(Eigen::Vector3d::UnitZ())).cross(applied_ref_leader-applied_ref_follower);
        rot_axis = rot_axis/rot_axis.norm(); // norm is never 0 as first cross is never between parallel vectors as drone references can never physically be only dispalced in z
      } else{ // close to 0
        rot_angle = 0.0;
        rot_axis = Eigen::Vector3d::UnitZ(); // could be any axis
      }
    } 
    else{ // do a rotation with fixed angle
      rot_axis=rot_axis/rot_axis.norm(); // make unitary
    }
    rot_angle = std::min(rot_angle, max_rot_angle);
    // apply the rotation
    Eigen::Affine3d T_rt(Eigen::AngleAxisd(rot_angle, rot_axis)); // homogeneous transformation matrix
    applied_ref_leader = 0.5*(applied_ref_leader + applied_ref_follower) + T_rt.rotation() * (applied_ref_leader-applied_ref_follower)*0.5;
    applied_ref_follower = 0.5*(applied_ref_leader + applied_ref_follower) + T_rt.rotation() * (applied_ref_follower-applied_ref_leader)*0.5;
    // ROS_INFO_STREAM("rotate applied references leader-follower around unit vector "<< rot_axis <<", over angle of = "<< rot_angle);
    // update final v of leader:
    if(erg_predictions_theta_trusted_){
      applied_ref_x_ = applied_ref_leader[0];
      applied_ref_y_ = applied_ref_leader[1];
      applied_ref_z_ = applied_ref_leader[2];
    }
    // update final v to be published to follower:
    position_cmd_follower_from_leader_.header.stamp    = uav_state_.header.stamp;
    position_cmd_follower_from_leader_.header.frame_id = uav_state_.header.frame_id;
    position_cmd_follower_from_leader_.position.x = applied_ref_follower[0];
    position_cmd_follower_from_leader_.position.y = applied_ref_follower[1];
    position_cmd_follower_from_leader_.position.z = applied_ref_follower[2];
    position_cmd_follower_from_leader_.heading = position_cmd_follower_for_leader_.heading;
     // update final r to be published to follower:
    goal_position_cmd_follower_from_leader_.header.stamp    = uav_state_.header.stamp;
    goal_position_cmd_follower_from_leader_.header.frame_id = uav_state_.header.frame_id;
    goal_position_cmd_follower_from_leader_.position.x = goal_follower[0];
    goal_position_cmd_follower_from_leader_.position.y = goal_follower[1];
    goal_position_cmd_follower_from_leader_.position.z = goal_follower[2];
    goal_position_cmd_follower_from_leader_.heading = goal_position_cmd_follower_for_leader_.heading;
    
    
    // ROS_INFO_STREAM("goal_leader  = \n" << goal_leader);
    // ROS_INFO_STREAM("applied_ref_leader  = \n" << applied_ref_leader);
    // ROS_INFO_STREAM("goal_follower  = \n" << goal_follower);
    // ROS_INFO_STREAM("applied_ref_follower  = \n" << applied_ref_follower);

    // Publish:
    if(erg_predictions_theta_trusted_){
      try {
        position_cmd_follower_from_leader_pub_.publish(position_cmd_follower_from_leader_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", position_cmd_follower_from_leader_pub_.getTopic().c_str());
      }

      try {
        goal_position_cmd_follower_from_leader_pub_.publish(goal_position_cmd_follower_from_leader_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", goal_position_cmd_follower_from_leader_pub_.getTopic().c_str());
      }
    }
  }

  // Computational time:
  // end_ERG_ = std::chrono::system_clock::now();
  // ComputationalTime_ERG_ = end_ERG_ - start_ERG_;
  // ComputationalTime_msg_.ERG = ComputationalTime_ERG_.count();

  std::clock_t c_end = std::clock();
  double part = 1000.0*(c_end-c_start) / (double)CLOCKS_PER_SEC;
  ComputationalTime_msg_.parts.push_back(part);
  std::string name_part = "D-ERG: NF and DSM";
  ComputationalTime_msg_.name_parts.push_back(name_part);

  // Prepare DSM_msg_:
  DSM_msg_.stamp = uav_state_.header.stamp;
  DSM_uav1_msg_.stamp = uav_state_.header.stamp;
  DSM_uav2_msg_.stamp = uav_state_.header.stamp;

  DSM_msg_.DSM = DSM_total_;
 

  // DSM_msg_.DSM_o = DSM_o_;
  // DSM_msg_.DSM_w = DSM_w_;

  /*
  // TODO= check mpc_tracker to "check" if communicated info it outdated too much
  */
  // Publishers always required for collision avoidance:
  //  - Prepare the uav_applied_ref_out_ msg:
  uav_applied_ref_out_.stamp = uav_state_.header.stamp; //ros::Time::now();
  uav_applied_ref_out_.uav_name = _uav_name_;
  uav_applied_ref_out_.priority = avoidance_this_uav_priority_; 
  mrs_msgs::FuturePoint new_point;
  new_point.x = applied_ref_x_;
  new_point.y = applied_ref_y_;
  new_point.z = applied_ref_z_;
  uav_applied_ref_out_.points.push_back(new_point);
  //  - Publish the uav_applied_ref_out_ msg:
  try {
     avoidance_applied_ref_publisher_.publish(uav_applied_ref_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", avoidance_applied_ref_publisher_.getTopic().c_str());
  }

  // Prepare the uav_position_out_ msg:
  uav_position_out_.stamp = uav_state_.header.stamp; //ros::Time::now();
  uav_position_out_.uav_name = _uav_name_;
  uav_position_out_.priority = avoidance_this_uav_priority_; 
  // mrs_msgs::FuturePoint new_point;
  new_point.x = uav_state_.pose.position.x; // or predicted_poses_out_.poses[0].position.x;
  new_point.y = uav_state_.pose.position.y; // or predicted_poses_out_.poses[0].position.y;
  new_point.z = uav_state_.pose.position.z;// or predicted_poses_out_.poses[0].position.z;
  uav_position_out_.points.push_back(new_point);
  // Publish the uav_position_out_ msg:
  try {
     avoidance_pos_publisher_.publish(uav_position_out_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", avoidance_pos_publisher_.getTopic().c_str());
  }

  if (_enable_dsm_a_ && (_DERG_strategy_id_ == 5 || _enable_visualization_)) { // avoidance_trajectory_publisher_ only used in _DERG_strategy_id_ == 5 or when RVIZ is enabled
    // Prepare the future_trajectory_out_ msg:
    future_trajectory_out_.stamp = uav_state_.header.stamp; //ros::Time::now();
    future_trajectory_out_.uav_name = _uav_name_;
    future_trajectory_out_.priority = avoidance_this_uav_priority_; 
    // mrs_msgs::FuturePoint new_point;
    for (size_t i = 0; i < _num_pred_samples_; i++) {
      new_point.x = predicted_poses_out_.poses[i].position.x;
      new_point.y = predicted_poses_out_.poses[i].position.y;
      new_point.z = predicted_poses_out_.poses[i].position.z;
      future_trajectory_out_.points.push_back(new_point);
    }
    // Publish the future_trajectory_out_ msg:
    try {
      avoidance_trajectory_publisher_.publish(future_trajectory_out_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", avoidance_trajectory_publisher_.getTopic().c_str());
    }
  }
  
  // Other publishers:

  // Publish DSM_msg_:
  try {
     DSM_publisher_.publish(DSM_msg_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", DSM_publisher_.getTopic().c_str());
  }

  try {
     DSM_uav1_publisher_.publish(DSM_uav1_msg_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", DSM_uav1_publisher_.getTopic().c_str());
  }

  try {
     DSM_uav2_publisher_.publish(DSM_uav2_msg_);
  }
  catch (...) {
    ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", DSM_uav2_publisher_.getTopic().c_str());
  }

  // Diagnostics Publisher - distance to collisions:
  if(_enable_diagnostics_pub_){
    /*
    - compute relevant info for: collision avoidance.
    - this method avoids post-processing timesynchronization problems in matlab
    */
    // Colision avoidance:
    double min_distance_this_uav2other_uav = 100000; // initialized with very high value
    // this uav:
    Eigen::Vector3d pos_this_uav(uav_state_.pose.position.x, uav_state_.pose.position.y, uav_state_.pose.position.z);
    // other uavs:
    std::map<std::string, mrs_msgs::FutureTrajectory>::iterator it = other_uavs_positions_.begin();
    while ((it != other_uavs_positions_.end()) ) {
      // make sure we use the same uavid for both iterators. It must be robust to possible difference in lenths of the iterators (if some communication got lost) or a different order (e.g. if not auto alphabetical).
      std::string other_uav_name = it->first;
      try
      {
        mrs_msgs::FutureTrajectory temp_pos = other_uavs_positions_[other_uav_name];
      }
      catch(...)
      {
        // other_uavs_positions_[other_uav_name] does not exist. Skip this iteration directly.
        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: Lost communicated position corresponding to the position of %s \n", other_uav_name.c_str());
        it++;
        continue;
      }     
      double other_uav_pos_x = other_uavs_positions_[other_uav_name].points[0].x;
      double other_uav_pos_y = other_uavs_positions_[other_uav_name].points[0].y;
      double other_uav_pos_z = other_uavs_positions_[other_uav_name].points[0].z;
      Eigen::Vector3d pos_other_uav(other_uav_pos_x, other_uav_pos_y, other_uav_pos_z);
      // compute minimum distance from this_uav to all other_uav:
      double distance_this_uav2other_uav = (pos_this_uav - pos_other_uav).norm()-2*Ra_;
      DistanceBetweenUavs_msg_.distances_this_uav2other_uavs.push_back(distance_this_uav2other_uav);
      DistanceBetweenUavs_msg_.other_uav_names.push_back(other_uav_name);
      if (distance_this_uav2other_uav <= 0.0){ // a collision is detected
        // trigger elanding
        deactivate();
      }
      if (distance_this_uav2other_uav < min_distance_this_uav2other_uav){
        min_distance_this_uav2other_uav = distance_this_uav2other_uav; // min_distance_this_uav2other_uav = std::min(min_distance_this_uav2other_uav, distance_this_uav2other_uav);
        DistanceBetweenUavs_msg_.min_distance_this_uav2other_uav = min_distance_this_uav2other_uav;
        DistanceBetweenUavs_msg_.other_uav_name = other_uav_name;
      }
      it++;
    }
    DistanceBetweenUavs_msg_.stamp = uav_state_.header.stamp;
    try {
      DistanceBetweenUavs_publisher_.publish(DistanceBetweenUavs_msg_);
    }
    catch (...) {
      ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", DistanceBetweenUavs_publisher_.getTopic().c_str());
    }
    // clear()
    DistanceBetweenUavs_msg_.distances_this_uav2other_uavs.clear();
    DistanceBetweenUavs_msg_.other_uav_names.clear();
  }
  
  // RVIZ Publisher:
  if(_enable_visualization_){
    DERG_strategy_id.data = _DERG_strategy_id_;
    derg_strategy_id_publisher_.publish(DERG_strategy_id);
    point_link_star_publisher_.publish(point_link_star_);
    Sa_max_.data = _Sa_max_;
    sa_max_publisher_.publish(Sa_max_);
    Sa_perp_max.data = _Sa_perp_max_;
    sa_perp_max_publisher_.publish(Sa_perp_max);
  }
}

/* The function computeDSM_sT_trajpred returns the DSM of the total thrust */
double DergbryanTracker::computeDSM_sT_trajpred(geometry_msgs::PoseArray predicted_thrust){
  double DSM_sT;
  // TODO: predicted_thrust_out_.poses is not good programming for a scalar. Maybe try using Class: Std_msgs::Float32MultiArray and test plot still work
  double diff_T = thrust_saturation_physical_; // initialization at the highest possible positive difference value
  // ROS_INFO_STREAM("thrust_saturation_physical_ = \n" << thrust_saturation_physical_);
  for (size_t i = 0; i < _num_pred_samples_; i++) {
    double diff_Tmax = thrust_saturation_physical_ - predicted_thrust.poses[i].position.x;
    double diff_Tmin = predicted_thrust.poses[i].position.x - _T_min_;
    if (diff_Tmax < diff_T) {
      diff_T = diff_Tmax;
    }
    if (diff_Tmin < diff_T) {
      diff_T = diff_Tmin;
    }
  }
  DSM_sT = _kappa_sT_*diff_T/(0.5*(thrust_saturation_physical_ - _T_min_)); // scaled DSM in _kappa_sT_*[0, 1] from the average between the lower and upper limit to the respective limits 
  return DSM_sT; 
}

double DergbryanTracker::computeDSM_sw_trajpred(geometry_msgs::PoseArray predicted_attrate){
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.roll_rate = %f", constraints.roll_rate);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.pitch_rate = %f", constraints.pitch_rate);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.yaw_rate = %f", constraints.yaw_rate);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.roll_rate = %f", constraints.roll_rate);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.pitch_rate = %f", constraints.pitch_rate);
  // ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: constraints.yaw_rate = %f", constraints.yaw_rate);
  // initialization at the highest possible positive difference value (1.0) relative to the constraint.
  // diff_bodyrate_* = 1.0 means the body rate is 0 and diff_bodyrate_= 0 means it's hitting the constraint
  double rel_diff_bodyrate = 1.0; // used for roll, pitch and yaw
  double body_rate_roll;
  double body_rate_pitch;
  double body_rate_yaw;
  for (size_t i = 0; i < _num_pred_samples_; i++) {
  // Depending if the predictions _predicitons_use_body_inertia_:
    if (_predicitons_use_body_inertia_) { // we implement constraints on the actual body rates
      body_rate_roll = predicted_attrate.poses[i].position.x;
      body_rate_pitch = predicted_attrate.poses[i].position.y;
      body_rate_yaw = predicted_attrate.poses[i].position.z;
    } else{ // we implement constraints on the desired body rates
      body_rate_roll = predicted_attrate.poses[i].position.x;
      body_rate_pitch = predicted_attrate.poses[i].position.y;
      body_rate_yaw = predicted_attrate.poses[i].position.z;
    }
    //ROS_INFO_STREAM("body_rate_roll = \n" << body_rate_roll); //
    double rel_diff_bodyrate_roll_temp = (constraints.roll_rate - std::abs(body_rate_roll))/constraints.roll_rate;
    double rel_diff_bodyrate_pitch_temp = (constraints.pitch_rate - std::abs(body_rate_pitch))/constraints.pitch_rate;
    double rel_diff_bodyrate_yaw_temp = (constraints.yaw_rate - std::abs(body_rate_yaw))/constraints.yaw_rate;
    //ROS_INFO_STREAM("rel_diff_bodyrate_roll_temp = \n" << rel_diff_bodyrate_roll_temp); // printing this gives error
    if (rel_diff_bodyrate_roll_temp < rel_diff_bodyrate) {
      rel_diff_bodyrate = rel_diff_bodyrate_roll_temp;
    }
    if (rel_diff_bodyrate_pitch_temp < rel_diff_bodyrate) {
      rel_diff_bodyrate = rel_diff_bodyrate_pitch_temp;
    }
    if (rel_diff_bodyrate_yaw_temp < rel_diff_bodyrate) {
      rel_diff_bodyrate = rel_diff_bodyrate_yaw_temp;
    }
  }
  double DSM_sw = _kappa_sw_*rel_diff_bodyrate; // scaled DSM in _kappa_sw_*[0, 1]  
  return DSM_sw;
}

double DergbryanTracker::computeDSM_swing_trajpred(geometry_msgs::PoseArray predicted_swing_angle){
  double diff_alpha_min = _constraint_swing_c_; //init the difference as the highest possible value
  for (size_t i = 0; i < _num_pred_samples_; i++) {
    double diff_alpha = _constraint_swing_c_ - predicted_swing_angle.poses[i].position.x;
    if (diff_alpha < diff_alpha_min) { //if we find a lower value we store it
      diff_alpha_min = diff_alpha;  //diff_alpha_min will be the minimal value we get over the whole prediction horizon.
    }
  }
  double DSM_swing_c = _kappa_swing_c_ * diff_alpha_min / _constraint_swing_c_; // scaled between 0 and _kappa_swing_c_    
  return DSM_swing_c;
}

double DergbryanTracker::computeDSM_Tc_trajpred(geometry_msgs::PoseArray predicted_cable_tension){
  double min_Tc = 100000.0; //init at very high value;
  for (size_t i = 0; i < _num_pred_samples_; i++) {
    if (predicted_cable_tension.poses[i].position.x < min_Tc) { //if we find a lower value we store it
      min_Tc = predicted_cable_tension.poses[i].position.x;  //diff_alpha_min will be the minimal value we get over the whole prediction horizon.
    }
  }
  // TODO: adapt scaling fot 2UAV case
  double DSM_Tc = _kappa_Tc_ * min_Tc / (_load_mass_ * common_handlers_->g); // at hover the min_Tc = _load_mass_ * common_handlers_->g and then DSM = _kappa_Tc_
  return DSM_Tc;
}




// FROM kelly's repo
/* The function getLambda returns the parametrization factor lambda for link i wrt spherical obstacle j */
double DergbryanTracker::getLambda(Eigen::Vector3d &point_link_0, Eigen::Vector3d &point_link_1, Eigen::Vector3d &point_sphere){
  double lambda;
    double denum = (point_link_1 - point_link_0).dot(point_link_1 - point_link_0);
    if (std::abs(denum) <0.001){ // don't devide over 0
      return 0;
    }
      lambda = (point_link_1-point_link_0).dot(point_sphere-point_link_0)/denum;
      // if (lambda(i,j) < 0) {
      //   lambda(i,j) = 0;
      // }
      // else if(lambda(i,j) >1) {
      //   lambda(i,j) = 1;
      // }  
  return lambda; 
}
/* The function getMuSijTij returns the parametrization factor mu for link i wrt cylindrical obstacle j 
and returns the positions of the closest distance between link i and cylinder j */ //Frank : Use this now for obstacle avoidance
std::tuple< Eigen::Vector3d, Eigen::Vector3d> DergbryanTracker::getMinDistDirLineSegments(Eigen::Vector3d &point0_link0, Eigen::Vector3d &point1_link0, Eigen::Vector3d &point0_link1, Eigen::Vector3d &point1_link1){
  // Eigen::Vector3d direction_link0_to_link1; 
  // double distance;

  double mu;  
  double nu;

  Eigen::Vector3d point_mu_link0;
  Eigen::Vector3d point_nu_link1;

  Eigen::Vector3d a;
  Eigen::Vector3d b;
  Eigen::Vector3d c_0;
  Eigen::Vector3d c_1;
  // Eigen::Matrix<double, 3, 1> cylinder_start;
  // Eigen::Matrix<double, 3, 1> cylinder_end;

  // for (int i=0; i<panda_jointpositions.cols()-1;i++) {// #joints-1 (-1, because in code +1 to denote next joint)

    a = point1_link0 - point0_link0; 
  //   for (int j=0; j<number_obst_cylinders_; j++){ // # obstacles
  //     cylinder_start = cylinder_startendpoints_.block(0,j,3,1); 
  //     cylinder_end = cylinder_startendpoints_.block(3,j,3,1); 

      b = point1_link1 - point0_link1; 
      c_0 = point0_link1 - point0_link0;
      c_1 = point1_link1 - point0_link0;

      if (a.norm()<0.001 || b.norm()<0.001){ // too small
        if (a.norm()<0.001 && b.norm()<0.001) {
          point_mu_link0 = point0_link0;
          point_nu_link1 = point0_link1;
        }
        else if(a.norm()<0.001){
          point_mu_link0 = point0_link0;
          nu = (b.dot(point_mu_link0 - point0_link1))/(b.dot(b));  // LAMBDA FUNCTION !!!!
          point_nu_link1 = point0_link1 + nu * (point1_link1 - point0_link1);  
        }
        else if(b.norm()<0.001){
          point_nu_link1 = point0_link1;
          mu = (a.dot(point_nu_link1 - point0_link0))/(a.dot(a));   // LAMBDA FUNCTION !!!!
          point_mu_link0 = point0_link0 + mu * (point1_link0 - point0_link0);
        }
        return {point_mu_link0, point_nu_link1};
      }


      // CASE OF PARALLEL SEGMENTS 
      if ( ((a/a.norm()).cross(b/b.norm())).norm() < 0.001 ) 
      {
        double d_0 = (a/a.norm()).dot(c_0); 
        double d_1 = (a/a.norm()).dot(c_1);

        if(d_0<=0.0 && d_1<=0.0) // link1 before link0 (viewpoint of point0_link0 to point1_link0)
        {
          mu = 0.0; // mu = 0, point_mu_link0 = point0_link0
          if (std::abs(d_0) < std::abs(d_1))
          {
            nu = 0.0; // nu = 0, point_nu_link1 = point0_link1 
          }
          else if(std::abs(d_0) > std::abs(d_1))
          {
            nu = 1.0; // nu = 1, point_nu_link1 = point1_link1 
          }
        }
        else if(d_0>=a.norm() && d_1>=a.norm()) // link1 after link0  (viewpoint of point0_link0 to point1_link0)
        {
          mu = 1.0; // mu = 1, point_mu_link0 = point1_link0
          if (std::abs(d_0)<std::abs(d_1))
          {
            nu = 0.0; // nu = 0, point_nu_link1 = point0_link1 
          }
          else if(std::abs(d_0)>std::abs(d_1))
          {
            nu = 1.0; // nu = 1, point_nu_link1 = point1_link1 
          }
        }
        else // link1 and link0 (partly) overlapping
        {
          double nu_parallel = (b.dot((point0_link0 + point1_link0)/2 - point0_link1))/(b.dot(b)); // LAMBDA FUNCTION !!!!
          if (0.0<=nu_parallel && nu_parallel <=1.0) 
          { 
            mu = 0.5; // mu =0.5, point_mu_link0 = (point0_link0 + point1_link0)/2
            nu = nu_parallel; // nu computed as in point-line case
          }
          else if(0.0<=d_0 && d_0<=a.norm()) // = if nu_parallel < 0
          {
            if (d_1>a.norm())
            {
              mu = 1.0; // mu = 1, point_mu_link0 = point1_link0
              nu = (b.dot(point1_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case
            }
            else if(d_1 < 0.0)
            {
              mu = 0.0; // mu = 0, point_mu_link0 = point0_link0
              nu = (b.dot(point0_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case
            }
          }
          else if (0.0<=d_1 && d_1<=a.norm()) // % = if nu_parallel > 1
          {
            if (d_0>a.norm())
            {
              mu = 1.0; // mu = 1, point_mu_link0 = point1_link0
              nu = (b.dot(point1_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case
            }
            else if (d_0 < 0.0)
            {
              mu = 0.0; // mu = 0, point_mu_link0 = point0_link0
              nu = (b.dot(point0_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case
            }
          }                 
        }
        point_mu_link0 = point0_link0 + mu * (point1_link0 - point0_link0);
        point_nu_link1 = point0_link1 + nu * (point1_link1 - point0_link1);  
      }

      // CASE OF NON-PARALLEL SEGMENTS (SKEW/CUTTING)
      else {
        mu = (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-a.dot(b)*b.dot(a)); // mu computed for skew line-line case
        nu = (a.dot(a)/b.dot(a)) * (b.dot(b)*c_0.dot(a)-c_0.dot(b)*b.dot(a))/(b.dot(b)*a.dot(a)-b.dot(a)*b.dot(a))-(c_0.dot(a)/b.dot(a)); // nu computed for skew line-line case
        if ( (mu >=0.0 && mu<=1.0) && (nu>=0.0 && nu<=1.0) ){ // mu in [0,1] and nu in [0,1]
          point_mu_link0 = point0_link0 + mu * (point1_link0 - point0_link0);
          point_nu_link1 = point0_link1 + nu * (point1_link1 - point0_link1);  
        }
        else if ( (mu<0.0 || mu>1.0) && (nu>=0.0 && nu<=1.0) ){ // mu not in [0,1] and nu in [0,1]
          if (mu<0.0){
            mu = 0.0;
          }
          else if (mu>1.0){
            mu = 1.0; 
          }
          point_mu_link0 = point0_link0 + mu * (point1_link0 - point0_link0);
          nu = (b.dot(point_mu_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case   // LAMBDA FUNCTION !!!!
          if(nu<0.0){
            nu = 0.0;
          }
          else if (nu>1.0){
            nu = 1.0;
          }
          point_nu_link1 = point0_link1 + nu * (point1_link1 - point0_link1);  
        }
        else if ( (mu >=0.0 && mu<=1.0) && (nu<0.0 || nu>1.0) ){ // mu in [0,1] and nu not in [0,1]
          if (nu<0.0){
            nu = 0.0;
          }
          else if (nu>1.0){
            nu = 1.0;
          }
          point_nu_link1 = point0_link1 + nu * (point1_link1 - point0_link1);
          mu = (a.dot(point_nu_link1 - point0_link0))/(a.dot(a)); // mu computed as in point-line case // LAMBDA FUNCTION !!!!
          point_mu_link0 = point0_link0 + mu * (point1_link0 - point0_link0);
        }
        else { // mu not in [0,1] and nu not in [0,1]
          double nu_for_mu0 = (b.dot(point0_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case 
          double nu_for_mu1 = (b.dot(point1_link0 - point0_link1))/(b.dot(b)); // nu computed as in point-line case   
          if(nu_for_mu0<0.0){
            nu_for_mu0 = 0.0;
          }
          else if(nu_for_mu0>1.0){
            nu_for_mu0 = 1.0;
          }
          if(nu_for_mu1<0.0){
            nu_for_mu1 = 0.0;
          }
          else if(nu_for_mu1>1.0){
            nu_for_mu1 = 1.0;
          }    
          Eigen::Matrix<double, 3, 1> point_nu_link1_for_mu0 = point0_link1 + nu_for_mu0 * (point1_link1 - point0_link1);
          Eigen::Matrix<double, 3, 1> point_nu_link1_for_mu1 = point0_link1 + nu_for_mu1 * (point1_link1 - point0_link1);
          if( (point0_link0 - point_nu_link1_for_mu0).norm() < (point1_link0 - point_nu_link1_for_mu1).norm() ){
            mu = 0.0;
            point_mu_link0 = point0_link0;
            point_nu_link1 = point_nu_link1_for_mu0;
          }
          else{
            mu = 1.0;
            point_mu_link0 = point1_link0;
            point_nu_link1 = point_nu_link1_for_mu1;
          }
        }
      }
      // direction_link0_to_link1 = point_nu_link1 - point_mu_link0;
      // distance = (point_mu_link0 - point_nu_link1).norm();
  return {point_mu_link0, point_nu_link1}; 
}


// | ------------------------ callbacks ----------------------- |

// | ---------------------------LOAD----------------------------------------------- | 
// | ----------------- load subscribtion callback ---------------- |
// TODO: streamline, account for prev comments and document the load callbacks below
void DergbryanTracker::GazeboLoadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg) {
    // TODO: moreover, as this callabck changes global load state variables at asynchronous and higher rates than the tracker update function, one needs to ensure that the global variables used for the state are always the same everywhere (avoid using different global information as the update function is sequentially executed). For this I think at the start of the update function one can "freeze" those global variables. So create 2 sets of global load variables and use everywhere except in this callabck the frozen variables.
    // This callback function is only triggered when doing simulation, and will be used to unpack the data coming from the Gazebo topics.
    int anchoring_pt_index; // Stores the index at which the anchoring point appears in the message that is received from Gazebo. 
    std::vector<std::string> link_names = loadmsg->name; // Take a vector containing the name of each link in the simulation. Among these there is the links that are related to the payload. 
    for(size_t i = 0; i < link_names.size(); i++){ // Go through all the link names
      if (_type_of_system_ == "1uav_payload"){
        if(link_names[i] == "point_mass::link_01"){ //link_01 is the point mass payload link (see mass_point.xacro). When the link_name corresponds to the one of the payload, defined in simulation/models/suspended_payload xacro files of the testing folder.
          anchoring_pt_index = i; //Store the index of the name, as it will be used as the index to access all the states of this link, in the loadmsg later.
          payload_spawned_ = true; //Notify that the payload has spawned. This will only be triggered once and allow predictions to start.
        }
      }
      if (_type_of_system_ == "2uavs_payload"){ // 2UAVs transporting beam payload case. Need to return different link if this UAV is the uav1 or 2. 
        if (_uav_name_=="uav1"){
          if(link_names[i] == "bar::link_04"){ //link_04 corresponds to the anchoring point of uav1 (see bar.xacro)
              anchoring_pt_index = i;
              payload_spawned_ = true;
              //ROS_INFO_STREAM("payload_spawned_ for uav1 = \n" << payload_spawned_);
          }
        }
        else if (_uav_name_=="uav2") { // for uav2
          if(link_names[i] == "bar::link_01"){ //link_01 corresponds to the anchoring point of uav1 (see bar.xacro)
              anchoring_pt_index = i;
              payload_spawned_ = true;
              //ROS_INFO_STREAM("payload_spawned_ for uav2 = \n" << payload_spawned_);
          }
        }
      }
    }
    // Extract the value from the received loadmsg. 
    geometry_msgs::Pose anchoring_pt_pose = loadmsg->pose[anchoring_pt_index]; // Now that we know which index refers to the anchoring point we search for (depending on which system we have), we can use it to get the actual state of this point.  
    anchoring_pt_pose_position_[0] = anchoring_pt_pose.position.x;
    anchoring_pt_pose_position_[1] = anchoring_pt_pose.position.y;
    anchoring_pt_pose_position_[2] = anchoring_pt_pose.position.z;

    anchoring_pt_velocity_ = loadmsg->twist[anchoring_pt_index];
    // This is anchoring point linear velocity, it will be used to deduce rotational velocity of the beam in the first iteration of the prediction
    anchoring_pt_lin_vel_[0]= anchoring_pt_velocity_.linear.x;
    anchoring_pt_lin_vel_[1]= anchoring_pt_velocity_.linear.y;
    anchoring_pt_lin_vel_[2]= anchoring_pt_velocity_.linear.z;

    // TODO: if we want to measure other variables (e.g., 2uav bar load COM linear and rotatinal velocity)
  //-------------------------------------------------//
  // if we don't print something, we get an error. TODO: figure out why, see emails with Raphael.
  ROS_INFO_THROTTLE(15.0,"[DergbryanTracker]: publish this here or you get strange error");
}

// TODO: document and test if this works on hardware
// TODO: combine with BacaLoadStatesCallback of controller as (almost) same code
void DergbryanTracker::BacaLoadStatesCallback(const mrs_msgs::BacaProtocolConstPtr& msg) {
  // TODO: similar to GazeboLoadStatesCallback, update the variable payload_spawned_ if this the data is correctly received from arduino
  int message_id;
  int payload_1;
  int payload_2;
  message_id = msg->payload[0];
  payload_1 = msg->payload[1];
  payload_2 = msg->payload[2];

  int16_t combined = payload_1 << 8;
  combined |= payload_2;

  float encoder_output = (float) combined/ 1000.0;

  if (message_id == 24)
  {
    encoder_angle_1_ = encoder_output;
  }else if (message_id == 25)
  {
    encoder_angle_2_ = encoder_output;
  }else if (message_id == 32)
  {
    encoder_velocity_1_ = encoder_output;
  }else if (message_id == 33)
  {
    encoder_velocity_2_ = encoder_output;
  }

  // Sanity check
  if (!std::isfinite(encoder_angle_1_)||!std::isfinite(encoder_angle_2_)) {
    ROS_ERROR("[Se3CopyController]: NaN detected in encoder angles");
    payload_spawned_=false; //Put payload_spawned back to false in case the encoder stops giving finite values during a flight. Epl stays equal to zero when this flag is false, avoiding strange behaviors or non finite Epl. 
  }
  else if (!std::isfinite(encoder_velocity_1_)||!std::isfinite(encoder_velocity_2_)) {
    ROS_ERROR("[Se3CopyController]: NaN detected in encoder angular velocities");
    payload_spawned_=false;  
  }
  else{
    ROS_INFO_THROTTLE(20.0,"[Se3CopyController]: Encoder angles returned are finite values");
    payload_spawned_=true; // Values are finite and thus can be used to compute a finite
  }

  Eigen::Vector3d anchoring_pt_pose_position_rel ; // position of the payload in the body frame B
  Eigen::Vector3d anchoring_pt_lin_vel_rel; //Velocity of the payload in the body frame B.

  //Compute position of the anchoring point in body base B. (i.e., relative to drone COM)
  anchoring_pt_pose_position_rel[0]=_cable_length_*sin(encoder_angle_1_)*cos(encoder_angle_2_);
  anchoring_pt_pose_position_rel[1]=_cable_length_*sin(encoder_angle_2_);
  anchoring_pt_pose_position_rel[2]=-_cable_length_*cos(encoder_angle_1_)*cos(encoder_angle_2_);
  
  //Compute absolute position of the payload.
  Eigen::Vector3d Op(uav_state_.pose.position.x, uav_state_.pose.position.y, uav_state_.pose.position.z);
  Eigen::Vector3d Ov(uav_state_.velocity.linear.x, uav_state_.velocity.linear.y, uav_state_.velocity.linear.z);
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state_.pose.orientation);

  anchoring_pt_pose_position_=Op+R*anchoring_pt_pose_position_rel;
  
  //Compute absolute velocity of the anchoring point.

  Eigen::Vector3d Ow(uav_state_.velocity.angular.x, uav_state_.velocity.angular.y, uav_state_.velocity.angular.z); 
  Eigen::Matrix3d skew_Ow;
  skew_Ow << 0.0     , -Ow(2), Ow(1),
            Ow(2) , 0.0,       -Ow(0),
            -Ow(1), Ow(0),  0.0;
  Eigen::Matrix3d Rdot = skew_Ow*R;
    
  anchoring_pt_lin_vel_rel[0]=_cable_length_*(cos(encoder_angle_1_)*cos(encoder_angle_2_)*encoder_velocity_1_
    - sin(encoder_angle_1_)*sin(encoder_angle_2_)*encoder_velocity_2_);
  anchoring_pt_lin_vel_rel[1]=_cable_length_*cos(encoder_angle_2_)*encoder_velocity_2_;
  anchoring_pt_lin_vel_rel[2]=_cable_length_*(sin(encoder_angle_2_)*cos(encoder_angle_1_)*encoder_velocity_2_+
  sin(encoder_angle_1_)*cos(encoder_angle_2_)*encoder_velocity_1_);
  
  anchoring_pt_lin_vel_=Ov+Rdot*anchoring_pt_pose_position_rel+R*anchoring_pt_lin_vel_rel;

  geometry_msgs::Pose anchoring_pt_pose;
  anchoring_pt_pose.position.x=anchoring_pt_pose_position_[0];
  anchoring_pt_pose.position.y=anchoring_pt_pose_position_[1];
  anchoring_pt_pose.position.z=anchoring_pt_pose_position_[2];
  anchoring_pt_velocity_.linear.x=anchoring_pt_lin_vel_[0];
  anchoring_pt_velocity_.linear.y=anchoring_pt_lin_vel_[1];
  anchoring_pt_velocity_.linear.z=anchoring_pt_lin_vel_[2]; 
}

// TODO: update these callbacks which contain too hardcoded info as uav2
void DergbryanTracker::uav_state_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg){
  uav_state_follower_for_leader_ = *msg; //for the header stamp and other quantities that might be interesting
  uav_position_follower_[0]=msg->pose.position.x; // as need to use these as vector in model
  uav_position_follower_[1]=msg->pose.position.y;
  uav_position_follower_[2]=msg->pose.position.z;
  // ROS_INFO_STREAM("Received uav2 position \n"<< uav_position_follower_);
  uav_velocity_follower_[0]=msg->velocity.linear.x;
  uav_velocity_follower_[1]=msg->velocity.linear.y;
  uav_velocity_follower_[2]=msg->velocity.linear.z;
  // ROS_INFO_STREAM("Received uav2 velocity \n"<< uav_velocity_follower_);
}

// TODO: update these callbacks which contain too hardcoded info as uav2
void DergbryanTracker::anchoring_point_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg){
  anchoring_point_follower_for_leader_ = *msg; //for the header stamp and other quantities that might be interesting
  anchoring_point_follower_position_[0]=msg->pose.position.x;
  anchoring_point_follower_position_[1]=msg->pose.position.y;
  anchoring_point_follower_position_[2]=msg->pose.position.z;
  // ROS_INFO_STREAM("Received anchoring point position of follower uav \n"<< anchoring_point_follower_position_);
  anchoring_point_follower_velocity_[0]=msg->velocity.linear.x;
  anchoring_point_follower_velocity_[1]=msg->velocity.linear.y;
  anchoring_point_follower_velocity_[2]=msg->velocity.linear.z;
  // ROS_INFO_STREAM("Received anchoring point velocity of follower uav \n"<< anchoring_point_follower_velocity_);
}

void DergbryanTracker::position_cmd_follower_for_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
  position_cmd_follower_for_leader_=*msg;
  // ROS_INFO_STREAM("Received position cmd of follower from for the leader: "<< position_cmd_follower_for_leader_);
}

void DergbryanTracker::goal_position_cmd_follower_for_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
  goal_position_cmd_follower_for_leader_=*msg;
  // ROS_INFO_STREAM("Received goal position cmd of follower from for the leader: "<< goal_position_cmd_follower_for_leader_);
}
  
void DergbryanTracker::position_cmd_follower_from_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
  position_cmd_follower_from_leader_=*msg;
  // ROS_INFO_STREAM("Received position cmd of follower from leader \n"<< position_cmd_follower_from_leader_);
}

void DergbryanTracker::goal_position_cmd_follower_from_leader_callback(const mrs_msgs::PositionCommand::ConstPtr& msg){
  goal_position_cmd_follower_from_leader_=*msg;
  // ROS_INFO_STREAM("Received position cmd of follower from leader \n"<< goal_position_cmd_follower_from_leader_);
}

void DergbryanTracker::callbackOtherUavAppliedRef(const mrs_msgs::FutureTrajectoryConstPtr& msg) {
  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherUavAppliedRef");
  //ROS_INFO_STREAM("in callbackOtherUavAppliedRef!! \n");
  mrs_msgs::FutureTrajectory temp_pose = *msg;
  other_uavs_applied_references_[msg->uav_name] = temp_pose;
  //ROS_INFO_STREAM("pv uav1 = \n" << other_uavs_applied_references_["uav1"]);
  //ROS_INFO_STREAM("pv uav2 = \n" << other_uavs_applied_references_["uav2"]);
}

void DergbryanTracker::callbackOtherUavPosition(const mrs_msgs::FutureTrajectoryConstPtr& msg) {
  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherUavPosition");
  // ROS_INFO_STREAM("in callbackOtherUavPosition!! \n");
  mrs_msgs::FutureTrajectory temp_pose = *msg;
  other_uavs_positions_[msg->uav_name] = temp_pose;
}

void DergbryanTracker::callbackOtherUavTrajectory(const mrs_msgs::FutureTrajectoryConstPtr& msg) {
  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherUavTrajectory");
  // ROS_INFO_STREAM("in callbackOtherUavTrajectory!! \n");
  mrs_msgs::FutureTrajectory temp_traj = *msg;
  other_uav_avoidance_trajectories_[msg->uav_name] = temp_traj;
}

void DergbryanTracker::chatterCallback(mrs_lib::SubscribeHandler<std_msgs::String>& sh_ptr){
  //ROS_INFO_STREAM("in chatterCallback!! \n");
  mrs_lib::Routine profiler_routine = profiler.createRoutine("chatterCallback");
  //// ROS_INFO("I heard: [%s]", sh_ptr->data.c_str()); DOES NOT WORK as in default tutorial ROS
  //ROS_INFO_STREAM("Received: '" << sh_ptr.getMsg()->data << "' from topic '" << sh_ptr.topicName() << "'"); // see examples SubscribeHandler mrs lib
}

void DergbryanTracker::callbackOtherUavTubeMinRadius(mrs_lib::SubscribeHandler<std_msgs::Float32>& sh_ptr) {
  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherUavTubeMinRadius");
  // double temp_radius = *sh_ptr; // does not work!
  //ROS_INFO_STREAM("Received: '" << sh_ptr.getMsg()->data << "' from topic '" << sh_ptr.topicName() << "'");
  double temp_radius = sh_ptr.getMsg()->data;
  //ROS_INFO_STREAM("temp_radius = \n" << temp_radius);
}

void DergbryanTracker::callbackOtherUavFutureTrajectoryTube(mrs_lib::SubscribeHandler<trackers_brubotics::FutureTrajectoryTube>& sh_ptr){
  mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherUavFutureTrajectoryTube");
  // ROS_INFO_STREAM("Received minimal radius: '" << sh_ptr.getMsg()->min_radius << "' from topic '" << sh_ptr.topicName() << "'");
  trackers_brubotics::FutureTrajectoryTube temp_tube = *sh_ptr.getMsg(); // use *!
  other_uav_tube_[sh_ptr.getMsg()->uav_name] = temp_tube;
  // double temp_radius = sh_ptr.getMsg()->min_radius;
  // other_uav_tube_min_radius_[sh_ptr.getMsg()->uav_name] = temp_radius;


}
// void DergbryanTracker::callbackOtherMavTrajectory(mrs_lib::SubscribeHandler<mrs_msgs::FutureTrajectory>& sh_ptr) {

//   ROS_INFO_STREAM("inside callbackOtherMavTrajectory \n");

//   if (!is_initialized_) {
//     return;
//   }

//   mrs_lib::Routine profiler_routine = profiler.createRoutine("callbackOtherMavTrajectory");

//   auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

//   mrs_msgs::FutureTrajectory trajectory = *sh_ptr.getMsg();

//   // the times might not be synchronized, so just remember the time of receiving it
//   trajectory.stamp = ros::Time::now();

//   // transform it from the utm origin to the currently used frame
//   auto res = common_handlers_->transformer->getTransform("utm_origin", uav_state.header.frame_id, ros::Time::now(), true);

//   if (!res) {

//     std::string message = "[DergbryanTracker]: can not transform other drone trajectory to the current frame";
//     ROS_WARN_STREAM_ONCE(message);
//     ROS_DEBUG_STREAM_THROTTLE(1.0, message);

//     return;
//   }

//   mrs_lib::TransformStamped tf = res.value();

//   for (int i = 0; i < int(trajectory.points.size()); i++) {

//     geometry_msgs::PoseStamped original_pose;

//     original_pose.pose.position.x = trajectory.points[i].x;
//     original_pose.pose.position.y = trajectory.points[i].y;
//     original_pose.pose.position.z = trajectory.points[i].z;

//     original_pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

//     auto res = common_handlers_->transformer->transform(tf, original_pose);

//     if (res) {
//       trajectory.points[i].x = res.value().pose.position.x;
//       trajectory.points[i].y = res.value().pose.position.y;
//       trajectory.points[i].z = res.value().pose.position.z;
//     } else {

//       std::string message = "[DergbryanTracker]: could not transform point of other uav future trajectory!";
//       ROS_WARN_STREAM_ONCE(message);
//       ROS_DEBUG_STREAM_THROTTLE(1.0, message);

//       return;
//     }
//   }

//   {
//     std::scoped_lock lock(mutex_other_uav_avoidance_trajectories_);

//     // update the diagnostics
//     ROS_INFO_STREAM("trajectory.uav_name = \n" << trajectory.uav_name);

//     ROS_INFO_STREAM("trajectory = \n" << trajectory);
//     other_uav_avoidance_trajectories_[trajectory.uav_name] = trajectory;
//   }
// }





// | -------------------- referece setting -------------------- |
// = copy of MpcTracker
/* //{ loadTrajectory() */

// method for setting desired trajectory
std::tuple<bool, std::string, bool> DergbryanTracker::loadTrajectory(const mrs_msgs::TrajectoryReference msg) {
//   // bryan: they don't seem to be used in this function, why use thme here????
  // copy the member variables
  // auto x         = mrs_lib::get_mutexed(mutex_mpc_x_, mpc_x_); !!!!
  // auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);!!!!
  std::stringstream ss;

  /* check the trajectory dt //{ */

  double trajectory_dt;
  if (msg.dt <= 1e-4) {
    trajectory_dt = 0.2;
    ROS_WARN_THROTTLE(10.0, "[DergbryanTracker]: the trajectory dt was not specified, assuming its the old 0.2 s");
  } else if (msg.dt < _dt1_) {
    trajectory_dt = 0.2;
    ss << std::setprecision(3) << "the trajectory dt (" << msg.dt << " s) is too small (smaller than the tracker's internal step size: " << _dt1_ << " s)";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());
    return std::tuple(false, ss.str(), false);
  } else {
    trajectory_dt = msg.dt;
  }

//   //}

  int trajectory_size = msg.points.size();

//   /* sanitize the time-ness of the trajectory //{ */

  int    trajectory_sample_offset    = 0;  // how many samples in past is the trajectory
//   int    trajectory_subsample_offset = 0;  // how many simulation inner loops ahead of the first valid sample
  double trajectory_time_offset      = 0;  // how much time in past in [s]

//   // btw, "trajectory_time_offset = trajectory_dt*trajectory_sample_offset + _dt1_*trajectory_subsample_offset" should hold
  if (msg.fly_now) {

    ros::Time trajectory_time = msg.header.stamp;

    // the desired time is 0 => the current time
    // the trajecoty is a single point => the current time
    if (trajectory_time == ros::Time(0) || int(msg.points.size()) == 1) {

      trajectory_time_offset = 0.0;

      // the desired time is specified
    } else {

      trajectory_time_offset = (ros::Time::now() - trajectory_time).toSec();

      // when the time offset is negative, thus in the future
      // just say it, but use it like its from the current time
      if (trajectory_time_offset < 0.0) {

        ROS_WARN_THROTTLE(1.0, "[DergbryanTracker]: received trajectory with timestamp in the future by %.2f s", -trajectory_time_offset);

        trajectory_time_offset = 0.0;
      }
    }

    // if the time offset is set, check if we need to "move the first idx"
    if (trajectory_time_offset > 0) {

      // calculate the offset in samples
      trajectory_sample_offset = int(floor(trajectory_time_offset / trajectory_dt));

      // and get the subsample offset, which will be used to initialize the interpolator
      //trajectory_subsample_offset = int(floor(fmod(trajectory_time_offset, trajectory_dt) / _dt1_));!!!

      //ROS_DEBUG_THROTTLE(1.0, "[DergbryanTracker]: sanity check: %.3f", trajectory_dt * trajectory_sample_offset + _dt1_ * trajectory_subsample_offset);!!!

      // if the offset is larger than the number of points in the trajectory
      // the trajectory can not be used
      if (trajectory_sample_offset >= trajectory_size) {

        ss << "trajectory timestamp is too old (time difference = " << trajectory_time_offset << ")";
        ROS_ERROR_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());
        return std::tuple(false, ss.str(), false);

      } else {

        // If the offset is larger than one trajectory sample,
        // offset the start
        if (trajectory_time_offset >= trajectory_dt) {

          // decrease the trajectory size
          trajectory_size -= trajectory_sample_offset;

          ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: got trajectory with timestamp '" << trajectory_time_offset << " s' in the past");

        } else {

          trajectory_sample_offset = 0;
        }
      }
    }
  }

  //}

  ROS_DEBUG_THROTTLE(1.0, "[DergbryanTracker]: trajectory sample offset: %d", trajectory_sample_offset);
//   ROS_DEBUG_THROTTLE(1.0, "[DergbryanTracker]: trajectory subsample offset: %d", trajectory_subsample_offset); !!!!

//   // after this, we should have the correct value of
//   // * trajectory_size
//   // * trajectory_sample_offset
//   // * trajectory_subsample_offset

//   /* copy the trajectory to a local variable //{ */

  // copy only the part from the first valid index
//   // _mpc_horizon_len_ = _num_pred_samples_
// !!!! deleted parts in the original block: _mpc_horizon_len_ = 0
  MatrixXd des_x_whole_trajectory       = VectorXd::Zero(trajectory_size, 1);
  MatrixXd des_y_whole_trajectory       = VectorXd::Zero(trajectory_size, 1);
  MatrixXd des_z_whole_trajectory       = VectorXd::Zero(trajectory_size, 1);
  MatrixXd des_heading_whole_trajectory = VectorXd::Zero(trajectory_size, 1);

  for (int i = 0; i < trajectory_size; i++) {

    des_x_whole_trajectory(i)       = msg.points[trajectory_sample_offset + i].position.x;
    des_y_whole_trajectory(i)       = msg.points[trajectory_sample_offset + i].position.y;
    des_z_whole_trajectory(i)       = msg.points[trajectory_sample_offset + i].position.z;
    des_heading_whole_trajectory(i) = msg.points[trajectory_sample_offset + i].heading;
  }

//   //}

//   /* set looping //{ */

  bool loop = false;

  if (msg.loop) {

    double first_x = des_x_whole_trajectory(0);
    double first_y = des_y_whole_trajectory(0);
    double first_z = des_z_whole_trajectory(0);

    double last_x = des_x_whole_trajectory(trajectory_size - 1);
    double last_y = des_y_whole_trajectory(trajectory_size - 1);
    double last_z = des_z_whole_trajectory(trajectory_size - 1);

    // check whether the trajectory is loopable
    // TODO should check heading aswell
    if (mrs_lib::geometry::dist(vec3_t(first_x, first_y, first_z), vec3_t(last_x, last_y, last_z)) < 3.141592653) {

      ROS_INFO_THROTTLE(1.0, "[DergbryanTracker]: looping enabled");
      loop = true;

    } else {

      ss << "can not loop trajectory, the first and last points are too far apart";
      ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());
      return std::tuple(false, ss.str(), false);
    }

  } else {

    loop = false;
  }

  //}

  // by this time, the values of these should be set:
  // * loop
//!!!!! seems like the final point are all the same over mpc horizon
//   /* add tail (the last point repeated to fill the prediction horizon) //{ */

//   if (!loop) {

//     // extend it so it has smooth ending
//     for (int i = 0; i < _num_pred_samples_; i++) {

//       des_x_whole_trajectory(i + trajectory_size)       = des_x_whole_trajectory(i + trajectory_size - 1);
//       des_y_whole_trajectory(i + trajectory_size)       = des_y_whole_trajectory(i + trajectory_size - 1);
//       des_z_whole_trajectory(i + trajectory_size)       = des_z_whole_trajectory(i + trajectory_size - 1);
//       des_heading_whole_trajectory(i + trajectory_size) = des_heading_whole_trajectory(i + trajectory_size - 1);
//     }
//   }

//   //}

  // by this time, the values of these should be set correctly:
  // * trajectory_size
  // * des_x_whole_trajectory
  // * des_y_whole_trajectory
  // * des_z_whole_trajectory
  // * des_heading_whole_trajectory

  /* update the global variables //{ */

  {
    std::scoped_lock lock(mutex_des_whole_trajectory_, mutex_des_trajectory_, mutex_trajectory_tracking_states_);
//     auto mpc_x_heading = mrs_lib::get_mutexed(mutex_mpc_x_, mpc_x_heading_); !!!!

    trajectory_tracking_in_progress_ = msg.fly_now;
    trajectory_track_heading_        = msg.use_heading;

    // allocate the vectors
    // !!!! deleted parts in the original block: _mpc_horizon_len_ = 0
    des_x_whole_trajectory_       = std::make_shared<VectorXd>(trajectory_size, 1);
    des_y_whole_trajectory_       = std::make_shared<VectorXd>(trajectory_size, 1);
    des_z_whole_trajectory_       = std::make_shared<VectorXd>(trajectory_size, 1);
    des_heading_whole_trajectory_ = std::make_shared<VectorXd>(trajectory_size, 1);
    // !!!! deleted parts in the original block: _mpc_horizon_len_ = 0
    for (int i = 0; i < trajectory_size; i++) {

      (*des_x_whole_trajectory_)(i) = des_x_whole_trajectory(i);
      (*des_y_whole_trajectory_)(i) = des_y_whole_trajectory(i);
      (*des_z_whole_trajectory_)(i) = des_z_whole_trajectory(i);

      if (trajectory_track_heading_) {
        (*des_heading_whole_trajectory_)(i) = des_heading_whole_trajectory(i);
      } else {
        //(*des_heading_whole_trajectory_).fill(mpc_x_heading(0, 0)); !!!
        (*des_heading_whole_trajectory_).fill(uav_heading_); // !!!!
      }
    }

    // if we are tracking trajectory, copy the setpoint
    if (trajectory_tracking_in_progress_) {

      toggleHover(false);

//       /* interpolate the trajectory points and fill in the desired_trajectory vector //{ */

//       for (int i = 0; i < _mpc_horizon_len_; i++) {

//         double first_time = _dt1_ + i * _dt2_ + trajectory_subsample_offset * _dt1_;

//         int first_idx  = floor(first_time / trajectory_dt);
//         int second_idx = first_idx + 1;

//         double interp_coeff = std::fmod(first_time / trajectory_dt, 1.0);

//         if (trajectory_tracking_loop_) {

//           if (second_idx >= trajectory_size) {
//             second_idx -= trajectory_size;
//           }

//           if (first_idx >= trajectory_size) {
//             first_idx -= trajectory_size;
//           }
//         } else {

//           if (second_idx >= trajectory_size) {
//             second_idx = trajectory_size - 1;
//           }

//           if (first_idx >= trajectory_size) {
//             first_idx = trajectory_size - 1;
//           }
//         }

//         des_x_trajectory_(i, 0) = (1 - interp_coeff) * des_x_whole_trajectory(first_idx) + interp_coeff * des_x_whole_trajectory(second_idx);
//         des_y_trajectory_(i, 0) = (1 - interp_coeff) * des_y_whole_trajectory(first_idx) + interp_coeff * des_y_whole_trajectory(second_idx);
//         des_z_trajectory_(i, 0) = (1 - interp_coeff) * des_z_whole_trajectory(first_idx) + interp_coeff * des_z_whole_trajectory(second_idx);

//         des_heading_trajectory_(i, 0) = sradians::interp(des_heading_whole_trajectory(first_idx), des_heading_whole_trajectory(second_idx), interp_coeff);
//       }

      //!!!!


      //}
    }

    trajectory_size_             = trajectory_size;
    trajectory_tracking_idx_     = 0;
//     trajectory_tracking_sub_idx_ = trajectory_subsample_offset;
    trajectory_set_              = true;
    trajectory_tracking_loop_    = loop;
    trajectory_dt_               = trajectory_dt;
    trajectory_count_++;

    timer_trajectory_tracking_.setPeriod(ros::Duration(trajectory_dt));
  }

//   //}

  if (trajectory_tracking_in_progress_) {
    timer_trajectory_tracking_.start();
  }

  ROS_INFO_THROTTLE(1, "[DergbryanTracker]: received trajectory with length %d", trajectory_size);

//   /* publish the debugging topics of the post-processed trajectory //{ */

//   {

//     geometry_msgs::PoseArray debug_trajectory_out;
//     debug_trajectory_out.header.stamp    = ros::Time::now();
//     debug_trajectory_out.header.frame_id = common_handlers_->transformer->resolveFrameName(msg.header.frame_id);

//     {
//       std::scoped_lock lock(mutex_des_whole_trajectory_);

//       for (int i = 0; i < trajectory_size; i++) {

//         geometry_msgs::Pose new_pose;

//         new_pose.position.x = (*des_x_whole_trajectory_)(i);
//         new_pose.position.y = (*des_y_whole_trajectory_)(i);
//         new_pose.position.z = (*des_z_whole_trajectory_)(i);

//         new_pose.orientation = mrs_lib::AttitudeConverter(0, 0, (*des_heading_whole_trajectory_)(i));

//         debug_trajectory_out.poses.push_back(new_pose);
//       }
//     }

//     try {
//       pub_debug_processed_trajectory_poses_.publish(debug_trajectory_out);
//     }
//     catch (...) {
//       ROS_ERROR("[MpcTracker]: exception caught during publishing topic %s", pub_debug_processed_trajectory_poses_.getTopic().c_str());
//     }

//     visualization_msgs::MarkerArray msg_out;

//     visualization_msgs::Marker marker;

//     marker.header.stamp     = ros::Time::now();
//     marker.header.frame_id  = common_handlers_->transformer->resolveFrameName(msg.header.frame_id);
//     marker.type             = visualization_msgs::Marker::LINE_LIST;
//     marker.color.a          = 1;
//     marker.scale.x          = 0.05;
//     marker.color.r          = 1;
//     marker.color.g          = 0;
//     marker.color.b          = 0;
//     marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

//     {
//       std::scoped_lock lock(mutex_des_whole_trajectory_);

//       for (int i = 0; i < trajectory_size - 1; i++) {

//         geometry_msgs::Point point1;

//         point1.x = des_x_whole_trajectory(i);
//         point1.y = des_y_whole_trajectory(i);
//         point1.z = des_z_whole_trajectory(i);

//         marker.points.push_back(point1);

//         geometry_msgs::Point point2;

//         point2.x = des_x_whole_trajectory(i + 1);
//         point2.y = des_y_whole_trajectory(i + 1);
//         point2.z = des_z_whole_trajectory(i + 1);

//         marker.points.push_back(point2);
//       }
//     }

//     msg_out.markers.push_back(marker);

//     try {
//       pub_debug_processed_trajectory_markers_.publish(msg_out);
//     }
//     catch (...) {
//       ROS_ERROR("exception caught during publishing topic %s", pub_debug_processed_trajectory_markers_.getTopic().c_str());
//     }
//   }

//   //}

//   publishDiagnostics();

  return std::tuple(true, "trajectory loaded", false);
}

/* gotoTrajectoryStartImpl() //{ */

std::tuple<bool, std::string> DergbryanTracker::gotoTrajectoryStartImpl(void) {

  std::stringstream ss;

  if (trajectory_set_) {

    toggleHover(false);

    trajectory_tracking_in_progress_ = false;
    timer_trajectory_tracking_.stop();

    {
      std::scoped_lock lock(mutex_des_whole_trajectory_);

      setGoal((*des_x_whole_trajectory_)[0], (*des_y_whole_trajectory_)[0], (*des_z_whole_trajectory_)[0], (*des_heading_whole_trajectory_)[0],
              trajectory_track_heading_);
    }

    //publishDiagnostics();

    ss << "flying to the start of the trajectory";
    ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    return std::tuple(true, ss.str());

  } else {

    ss << "can not fly to the start of the trajectory, the trajectory is not set";
    ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    return std::tuple(false, ss.str());
  }
}

//}




//
/* toggleHover() //{ */

void DergbryanTracker::toggleHover(bool in) {

  if (in == false && hovering_in_progress_) {

    ROS_DEBUG("[DergbryanTracker]: stoppping the hover timer");

    while (hover_timer_runnning_) {

      ROS_DEBUG("[DergbryanTracker]: the hover is in the middle of an iteration, waiting for it to finish");
      ros::Duration wait(0.001);
      wait.sleep();
    
      if (!hover_timer_runnning_) {
        ROS_DEBUG("[ControlManager]: hover timer finished");
        break;
      }
    }
    // timer_hover_.stop();

    hovering_in_progress_ = false;

  } else if (in == true && !hovering_in_progress_) {

    ROS_DEBUG("[DergbryanTracker]: starting the hover timer");

    hovering_in_progress_ = true;

    // timer_hover_.start();
  }
}

//}


// | ------------------- trajectory tracking ------------------ |

/* startTrajectoryTrackingImpl() //{ */

std::tuple<bool, std::string> DergbryanTracker::startTrajectoryTrackingImpl(void) {

  std::stringstream ss;

  if (trajectory_set_) {

    toggleHover(false);

    {
      std::scoped_lock lock(mutex_des_trajectory_);

      trajectory_tracking_in_progress_ = true;
      trajectory_tracking_idx_         = 0;
      // trajectory_tracking_sub_idx_     = 0;
    }

    timer_trajectory_tracking_.setPeriod(ros::Duration(trajectory_dt_));
    timer_trajectory_tracking_.start();

    ss << "trajectory tracking started";
    ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    return std::tuple(true, ss.str());

  } else {

    ss << "can not start trajectory tracking, the trajectory is not set";
    ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    return std::tuple(false, ss.str());
  }
}

//}


/* resumeTrajectoryTrackingImpl() //{ */

std::tuple<bool, std::string> DergbryanTracker::resumeTrajectoryTrackingImpl(void) {

  std::stringstream ss;

  if (trajectory_set_) {

    toggleHover(false);

    auto trajectory_tracking_idx = mrs_lib::get_mutexed(mutex_trajectory_tracking_states_, trajectory_tracking_idx_);

    if (trajectory_tracking_idx < (trajectory_size_ - 1)) {

      {
        std::scoped_lock lock(mutex_des_trajectory_);

        trajectory_tracking_in_progress_ = true;
      }

      timer_trajectory_tracking_.setPeriod(ros::Duration(trajectory_dt_));
      timer_trajectory_tracking_.start();

      ss << "trajectory tracking resumed";
      ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

      //publishDiagnostics();

      return std::tuple(true, ss.str());

    } else {

      ss << "can not resume trajectory tracking, trajectory is already finished";
      ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

      return std::tuple(false, ss.str());
    }

  } else {

    ss << "can not resume trajectory tracking, ther trajectory is not set";
    ROS_WARN_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    return std::tuple(false, ss.str());
  }
}

//}
/* stopTrajectoryTrackingImpl() //{ */

std::tuple<bool, std::string> DergbryanTracker::stopTrajectoryTrackingImpl(void) {

  std::stringstream ss;

  if (trajectory_tracking_in_progress_) {

    trajectory_tracking_in_progress_ = false;
    timer_trajectory_tracking_.stop();

    toggleHover(true);

    ss << "stopping trajectory tracking";
    ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());

    //publishDiagnostics();

  } else {

    ss << "can not stop trajectory tracking, already at stop";
    ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: " << ss.str());
  }

  return std::tuple(true, ss.str());
}

//}


/* timerTrajectoryTracking() //{ */

void DergbryanTracker::timerTrajectoryTracking(const ros::TimerEvent& event) {

  auto trajectory_size = mrs_lib::get_mutexed(mutex_des_trajectory_, trajectory_size_);
  auto trajectory_dt   = mrs_lib::get_mutexed(mutex_trajectory_tracking_states_, trajectory_dt_);

  mrs_lib::Routine profiler_routine = profiler.createRoutine("timerTrajectoryTracking", int(1.0 / trajectory_dt), 0.01, event);

  {
    std::scoped_lock lock(mutex_trajectory_tracking_states_);

    // do a step of the main tracking idx

    // reset the subsampling counter
    //trajectory_tracking_sub_idx_ = 0;

    if(trajectory_tracking_idx_ == 0){
      //ROS_INFO_STREAM("[DergbryanTracker]: trajectory_tracking_idx_ = \n" << trajectory_tracking_idx_);
      // set the global variable to keep track of the time when the trajectory was started
      time_at_start_point_ = (ros::Time::now()).toSec();
      // publish it below (add to msg) 
    }
    // INCREMENT THE TRACKING IDX
    trajectory_tracking_idx_++;
    ROS_INFO_STREAM_THROTTLE(1.0, "[DergbryanTracker]: trajectory_tracking_idx_" << trajectory_tracking_idx_);
    // !!!!! bryan, inspired by original timerMpc
    // looping  
    if (trajectory_tracking_loop_) {
      if (trajectory_tracking_idx_ >= trajectory_size) {
        trajectory_tracking_idx_ -= trajectory_size;
      }
    } else {
      if (trajectory_tracking_idx_ >= trajectory_size) {
        trajectory_tracking_idx_ = trajectory_tracking_idx_ - 1;
      }
    }
    {
      std::scoped_lock lock(mutex_des_whole_trajectory_, mutex_goal_);
      goal_x_ = (*des_x_whole_trajectory_)[trajectory_tracking_idx_];
      goal_y_ = (*des_y_whole_trajectory_)[trajectory_tracking_idx_];
      goal_z_ = (*des_z_whole_trajectory_)[trajectory_tracking_idx_];
      goal_heading_ = (*des_heading_whole_trajectory_)[trajectory_tracking_idx_];

      // Diagnostics of the TrajectoryTracking:
      if(_enable_diagnostics_pub_){
        // ROS_INFO("[DergbryanTracker]: Diagnostics of the TrajectoryTracking");

        TrajectoryTracking_msg_.stamp = uav_state_.header.stamp;
        traj_start_point_.x = (*des_x_whole_trajectory_)[0];
        traj_start_point_.y = (*des_y_whole_trajectory_)[0];
        traj_start_point_.z = (*des_z_whole_trajectory_)[0];
        TrajectoryTracking_msg_.traj_start_point = traj_start_point_;
        traj_end_point_.x = (*des_x_whole_trajectory_)[trajectory_size_-1];
        traj_end_point_.y = (*des_y_whole_trajectory_)[trajectory_size_-1];
        traj_end_point_.z = (*des_z_whole_trajectory_)[trajectory_size_-1];
        TrajectoryTracking_msg_.traj_end_point = traj_end_point_;
        TrajectoryTracking_msg_.arrival_norm_pos_error_treshold = arrival_norm_pos_error_treshold_; // static
        TrajectoryTracking_msg_.arrival_period_treshold = arrival_period_treshold_; // static
        TrajectoryTracking_msg_.time_at_start_point = time_at_start_point_; // initialized higher up
        // Check if we arrived at the traj_end_point:
        Eigen::Vector3d pos_error2goal;
        pos_error2goal[0] = traj_end_point_.x - uav_state_.pose.position.x;
        pos_error2goal[1] = traj_end_point_.y - uav_state_.pose.position.y;
        pos_error2goal[2] = traj_end_point_.z - uav_state_.pose.position.z;
        double norm_pos_error2goal = pos_error2goal.norm();
        TrajectoryTracking_msg_.norm_pos_error2goal = norm_pos_error2goal; 
        if(norm_pos_error2goal <= arrival_norm_pos_error_treshold_){ // within position treshold
          if (flag_running_timer_at_traj_end_point_ == false)  { // if timer not started
            flag_running_timer_at_traj_end_point_ = true; // set the flag to ...
            time_started_timer_at_traj_end_point_ = ros::Time::now(); // ... start the timer
          }
          double elapsed_time = (ros::Time::now() - time_started_timer_at_traj_end_point_).toSec();
          TrajectoryTracking_msg_.elapsed_time_in_pos_error_treshold = elapsed_time; 
          if(elapsed_time >= arrival_period_treshold_){
            arrived_at_traj_end_point_ = true;
            time_at_end_point_ = (ros::Time::now()).toSec();
          } 
          } else {
          flag_running_timer_at_traj_end_point_ = false; // if the uav would escape the pos error treshold this will we reset the timer if uav goes back within treshold later
          arrived_at_traj_end_point_ = false;
          time_at_end_point_ = 0.0; //reset
          }
          TrajectoryTracking_msg_.time_at_end_point = time_at_end_point_;
          TrajectoryTracking_msg_.arrived_at_traj_end_point = arrived_at_traj_end_point_;
        try {
          TrajectoryTracking_publisher_.publish(TrajectoryTracking_msg_);
        }
        catch (...) {
          ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", TrajectoryTracking_publisher_.getTopic().c_str());
        }
      }
    }


    // if the tracking idx hits the end of the trajectory
    if (trajectory_tracking_idx_ == trajectory_size) {

      if (trajectory_tracking_loop_) {

        // reset the idx
        trajectory_tracking_idx_ = 0;

        ROS_INFO("[DergbryanTracker]: trajectory looped");

      } else {

        trajectory_tracking_in_progress_ = false;

        // set the idx to the last idx of the trajectory
        trajectory_tracking_idx_ = trajectory_size - 1;

        timer_trajectory_tracking_.stop();

        ROS_INFO("[DergbryanTracker]: done tracking trajectory");
      }
    }
  }

  //publishDiagnostics();
}

//}



}  // namespace dergbryan_tracker
}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::dergbryan_tracker::DergbryanTracker, mrs_uav_managers::Tracker)
