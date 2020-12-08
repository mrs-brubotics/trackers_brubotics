#define VERSION "0.0.5.0"

/*includes//{*/
#include <ros/ros.h>
#include <mrs_uav_managers/tracker.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

/*begin includes added by bryan:*/
#include <mrs_lib/attitude_converter.h>
/*end includes added by bryan:*/

//}
#define OUTPUT_ATTITUDE_RATE 0
#define OUTPUT_ATTITUDE_QUATERNION 1

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
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle                                     nh_;
  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  std::string _version_;
  // // | ------------------------ uav state ----------------------- |
  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false; // now added by bryan
  std::mutex         mutex_uav_state_; // now added by bryan

  double uav_x_; // now added by bryan
  double uav_y_; // now added by bryan
  double uav_z_; // now added by bryan

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

  double                        _uav_mass_;
  double                        uav_mass_difference_;
  // double                        _g_;
  // mrs_uav_managers::MotorParams _motor_params_;


  // gains that are used and already filtered
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
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

  std::mutex mutex_gains_;       // locks the gains the are used and filtered



  // | ------------ controller limits and saturations ----------- |
  double _tilt_angle_failsafe_;
  double _thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  //mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  //ros::Time last_update_time_;
  //bool      first_iteration_ = true;

  // | ----------------------- output mode ---------------------- |

  int        output_mode_;  // attitude_rate / acceleration
  std::mutex mutex_output_mode_;


  // | ------------------------ profiler_ ------------------------ |
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ---------------------- desired goal ---------------------- |
  double     goal_x_;
  double     goal_y_;
  double     goal_z_;
  double     goal_heading_;
  double     have_goal_ = false;
  std::mutex mutex_goal_;



  bool is_initialized_ = false;
  bool is_active_      = false;
  bool hover_          = false;

  bool starting_bool=true;
};
//}

//WRITE THE FUNCTIONS HERE.


/*initialize()//{*/
void DergbryanTracker::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string uav_name,
                             [[maybe_unused]] std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {
  /*QUESTION: using se3_controller here since I want to load the se3 controllers paramters and not overwrite them. How to add paramters specific to derg tracker? How to overwrite se3 control paramters if we would use our own se3 controller?*/
  //ros::NodeHandle nh_(parent_nh, "dergbryan_tracker");
  ros::NodeHandle nh_(parent_nh, "se3_controller");
  common_handlers_ = common_handlers;
  /*QUESTION: how to load these paramters commented below as was done in the se3controller?*/
  // _motor_params_   = motor_params;
  // _uav_mass_       = uav_mass;
  // _g_              = g;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |
  //mrs_lib::ParamLoader param_loader(nh_, "DergbryanTracker");
  mrs_lib::ParamLoader param_loader(nh_, "Se3Controller");

  param_loader.loadParam("version", _version_);

  /*QUESTION: this block triggers the error always, anyone knows solution to this?*/
  // if (_version_ != VERSION) {
  //   ROS_ERROR("[DergbryanTracker]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
  //   ros::shutdown();
  // }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  // lateral gains
  param_loader.loadParam("default_gains/horizontal/kp", kpxy_);
  param_loader.loadParam("default_gains/horizontal/kv", kvxy_);
  param_loader.loadParam("default_gains/horizontal/ka", kaxy_);

  param_loader.loadParam("default_gains/horizontal/kiw", kiwxy_);
  param_loader.loadParam("default_gains/horizontal/kib", kibxy_);

  // height gains
  param_loader.loadParam("default_gains/vertical/kp", kpz_);
  param_loader.loadParam("default_gains/vertical/kv", kvz_);
  param_loader.loadParam("default_gains/vertical/ka", kaz_);

  // attitude gains
  param_loader.loadParam("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("default_gains/vertical/attitude/kq", kqz_);

  // mass estimator
  param_loader.loadParam("default_gains/mass_estimator/km", km_);
  param_loader.loadParam("default_gains/mass_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.loadParam("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.loadParam("constraints/tilt_angle_failsafe", _tilt_angle_failsafe_);
  param_loader.loadParam("constraints/thrust_saturation", _thrust_saturation_);

  // output mode
  param_loader.loadParam("output_mode", output_mode_);

  //param_loader.loadParam("rotation_matrix", drs_params_.rotation_type);



  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[DergbryanTracker]: could not load all parameters!");
    ros::shutdown();
  }
  // | ---------------- prepare stuff from params --------------- |

  /* QUESTION: do we need this? */
  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[Se3Controller]: output mode has to be {0, 1}!");
    ros::shutdown();
  }

  // convert to radians
  _tilt_angle_failsafe_ = (_tilt_angle_failsafe_ / 180.0) * M_PI;
  

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // initialization completed
  is_initialized_ = true;
  ROS_INFO("[DergbryanTracker]: initialized");
}
//}
/*activate()//{*/
std::tuple<bool, std::string> DergbryanTracker::activate(const mrs_msgs::PositionCommand::ConstPtr &last_position_cmd) {
  std::stringstream ss;
  if (!got_uav_state_) {
    ss << "odometry not set";
    ROS_ERROR_STREAM("[DergbryanTracker]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  ss << "Activated";
  is_active_ = true;
  ROS_INFO("[DergbryanTracker]: activated");
  return std::tuple(true, ss.str());
}

/*deactivate()//{*/
void DergbryanTracker::deactivate(void) {
  is_active_ = false;
  ROS_INFO("[DergbryanTracker]: deactivated");
}
//}

/*resetStatic()//{*/
bool DergbryanTracker::resetStatic(void) {
  ROS_INFO("[DergbryanTracker]: no states to reset");
  return true;
}
//}

/*update()//{*/
const mrs_msgs::PositionCommand::ConstPtr DergbryanTracker::update(const mrs_msgs::UavState::ConstPtr &                        uav_state,
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
  /* TODO: make it compatible with DRS */
  //auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return mrs_msgs::PositionCommand::Ptr();
  }
  mrs_msgs::PositionCommand position_cmd;

  // set the header
  position_cmd.header.stamp    = uav_state->header.stamp;
  position_cmd.header.frame_id = uav_state->header.frame_id;
  if (starting_bool) {
    // stay in place
    goal_x_= uav_state->pose.position.x;
    goal_y_= uav_state->pose.position.y;
    goal_z_= uav_state->pose.position.z;
    // set heading based on current odom
    try {
      goal_heading_ = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
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

    position_cmd.use_position_vertical   = 1;
    position_cmd.use_position_horizontal = 1;
    position_cmd.use_velocity_vertical   = 1;
    position_cmd.use_velocity_horizontal = 1;
    position_cmd.use_acceleration        = 0;
    position_cmd.use_jerk                = 0;
    position_cmd.use_heading             = 1;
    position_cmd.use_heading_rate        = 1;

    starting_bool=false;

    ROS_INFO("[Dergbryan tracker - odom]: [goal_x_=%.2f],[goal_y_=%.2f],[goal_z_=%.2f, goal_heading_=%.2f]",goal_x_,goal_y_,goal_z_,goal_heading_);
 
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}
/* begin copy of se3controller*/

// | -------------------- calculate the dt -------------------- |

  double dt;
  /* TODO: this if condition is commented since we will only allow this tracker to be activated if before the se3 was activated. 
  so will need to see how to use const mrs_msgs::ControllerStatus Se3Controller::getStatus() { of the controller in here to check if it is active*/

  // if (first_iteration_) {

  //   last_update_time_ = uav_state->header.stamp;

  //   first_iteration_ = false;

  //   ROS_INFO("[Se3Controller]: first iteration");

  //   return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));

  // } else {

  /* NOTE: assume a fixed sampling time used in the controller for this DergTracker*/
  /* TODO: load dt (se3 control sample time) as inverse of control sampling frequency defined somewhere in mrs tracker.yaml files (e.g. see mpc, line trackers) and set to 100Hz for controller (*/
  dt = 0.01; 
  // dt                = (uav_state->header.stamp - last_update_time_).toSec();
  // last_update_time_ = uav_state->header.stamp;
  // }

  /* NOTE: commented this section bacause we assume the controller has a fixed sample time of dt*/
  // if (fabs(dt) <= 0.001) {

  //   ROS_DEBUG("[Se3Controller]: the last odometry message came too close (%.2f s)!", dt);

  //   if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

  //     return last_attitude_cmd_;

  //   } else {

  //     return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
  //   }
  // }

  // | ----------------- get the current heading ---------------- |

  double uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[DergbryanTracker]: could not calculate the UAV heading");
  }

  

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3);
  
  position_cmd.use_position_vertical   = 1;
  position_cmd.use_position_horizontal = 1;
  // QUESTION: Why can't I print this? No value is printed, empty.
  // ROS_INFO_STREAM("position_cmd.use_position_horizontal = \n" << position_cmd.use_position_horizontal);
  // std::cout << "with std: "<< position_cmd.use_position_horizontal << std::endl;
  position_cmd.use_velocity_vertical   = 1;
  position_cmd.use_velocity_horizontal = 1;
  position_cmd.use_acceleration        = 0;
  position_cmd.use_jerk                = 0;
  position_cmd.use_heading             = 1;
  position_cmd.use_heading_rate        = 1;

  /*TODO,ADDED: SET THE APPLIED REFERNECE POSE (POSITION & HEADING) TEMPORARILY TO THE DESIRED GOAL*/
  position_cmd.position.x     = goal_x_;
  position_cmd.position.y     = goal_y_;
  position_cmd.position.z     = goal_z_;
  position_cmd.heading        = goal_heading_;

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
  /*VALIDATE: test streaming the references Rp, Rv, Ra when the drone is moving.*/
  // ROS_INFO_STREAM("Rp = \n" << Rp);
  // ROS_INFO_STREAM("Rv = \n" << Rv);
  // ROS_INFO_STREAM("Ra = \n" << Ra);
 
  /*QUESTION: where to print Rw? When it is set*/
  // ROS_INFO_STREAM("Rw = \n" << Rw);

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state->pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // // | -------------- calculate the control errors -------------- |

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
  /*VALIDATE: test streaming the references Op, Ov, R, Ow, Ep, Ev when the drone is moving.*/
  // ROS_INFO_STREAM("Op = \n" << Op);
  // ROS_INFO_STREAM("Ov = \n" << Ov);
  // ROS_INFO_STREAM("R  = \n" << R);
  // ROS_INFO_STREAM("Ow = \n" << Ow);
  // ROS_INFO_STREAM("Ep = \n" << Ep);
  // ROS_INFO_STREAM("Ev = \n" << Ev);

  // | --------------------- load the gains --------------------- |
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
    } else if (position_cmd.use_position_vertical) {  // special case: want to control z-pos but not the velocity => at least provide z dampening
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

  
  /*TODO: answer question below and define Kp, Kv accordingly, for now deined hardcoded*/
  double _uav_mass_ = 2.0; // TODO: change later
  double uav_mass_difference_ = 0.0;  // TODO: change later
  // QUESTION: how to get _uav_mass_ analoguous to controllers, in the controllers ?
  Kp = Kp * (_uav_mass_ + uav_mass_difference_);
  Kv = Kv * (_uav_mass_ + uav_mass_difference_);

  // a print to test if the gains change so you know where to change:
  // ROS_INFO_STREAM("Kp = \n" << Kp);
  // ROS_INFO_STREAM("Kv = \n" << Kv);
  // ROS_INFO_STREAM("Ka = \n" << Ka);
  // ROS_INFO_STREAM("Kq = \n" << Kq);
  // QUESTION: some gains printed above do not correspond to the gains set in the yaml file (e.G. Kpz). Why is that?


  // | --------------- desired orientation matrix --------------- |

  // get body integral in the world frame


/* NOTE: this part is added by Bryan, was originally in the controllers activate function */
    Ib_b_[0] = -last_attitude_cmd->disturbance_bx_b;
    Ib_b_[1] = -last_attitude_cmd->disturbance_by_b;

    Iw_w_[0] = -last_attitude_cmd->disturbance_wx_w;
    Iw_w_[1] = -last_attitude_cmd->disturbance_wy_w;

    // ROS_INFO(
    //     "[Se3Controller]: setting the mass difference and integrals from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: "
    //     "%.2f, %.2f N",
    //     uav_mass_difference_, Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
/* NOTE: TILL HERE*/

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);

  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle(uav_state_.header.frame_id, Ib_b_stamped);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the Ib_b_ to the world frame");
    }
  }


  // construct the desired force vector
  /* TODO: define _g_ as done in controllers */
  double _g_ = -9.81;
  double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, _g_) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }

  Eigen::Vector3d f = position_feedback + velocity_feedback + integral_feedback + feed_forward;

  // test printing force:
  // ROS_INFO_STREAM("f = \n" << f);

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  double theta = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta)) {

    ROS_ERROR("[Se3Controller]: NaN detected in variable 'theta', returning null");

    /* TODO: return something in this function */
    //return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_ > 1e-3 && theta > _tilt_angle_failsafe_) {

    // ROS_ERROR("[Se3Controller]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta,
    //           (180.0 / M_PI) * _tilt_angle_failsafe_);
    // ROS_INFO("[Se3Controller]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    // ROS_INFO("[Se3Controller]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
    // ROS_INFO("[Se3Controller]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
    // ROS_INFO("[Se3Controller]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    // ROS_INFO("[Se3Controller]: position_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", control_reference->position.x, control_reference->position.y,
    //          control_reference->position.z, control_reference->heading);
    // ROS_INFO("[Se3Controller]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state->pose.position.x, uav_state->pose.position.y,
    //          uav_state->pose.position.z, uav_heading);

    /* TODO: return something in this function */
    //return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

  if (theta > constraints.tilt) {
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta = constraints.tilt;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta) * cos(phi);
  f_norm[1] = sin(theta) * sin(phi);
  f_norm[2] = cos(theta);

  // | ------------- construct the rotational matrix ------------ |

  Eigen::Matrix3d Rd;

  if (position_cmd.use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(position_cmd.orientation);

    if (position_cmd.use_heading) {
      Rd = mrs_lib::AttitudeConverter(Rd).setHeading(position_cmd.heading);
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (position_cmd.use_heading) {
      bxd << cos(position_cmd.heading), sin(position_cmd.heading), 0;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    // fill in the desired orientation based on the state feedback
    /* TODO: make it compatible with DRS, now skipped first if */
    //if (drs_params.rotation_type == 0) {
      if (0) {

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

  /* orientation error */
  Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  /* output */
  double thrust_force = f.dot(R.col(2));

  double thrust = 0;

  if (thrust_force >= 0) {
    /*QUESTION: how to acces in the tracker code: _motor_params_.A + _motor_params_.B??*/
    //thrust = sqrt(thrust_force) * _motor_params_.A + _motor_params_.B;
    /*TODO change code below unhardcoded*/
    double Aparam = 0.01;
    double Bparam = 0.0001;
    thrust = sqrt(thrust_force) * Aparam + Bparam;
  } else {
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: just so you know, the desired thrust force is negative (%.2f)", thrust_force);
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[Se3Controller]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");

  } else if (thrust > _thrust_saturation_) {

    thrust = _thrust_saturation_;
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: saturating thrust to %.2f", _thrust_saturation_);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: saturating thrust to 0");
  }

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
      ROS_ERROR("[Se3Controller]: exception caught while calculating the desired_yaw_rate feedforward");
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
      parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeadingRate(q_feedback_yawless);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the parasitic heading rate!");
    }

    try {
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3Controller]: exception caught while calculating the parasitic heading rate compensation!");
    }
  }

  t += rp_heading_rate_compensation;


  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (position_cmd.use_position_horizontal) {
      Iw_w_ -= kiwxy_ * Ep.head(2) * dt;
    } else if (position_cmd.use_velocity_horizontal) {
      Iw_w_ -= kiwxy_ * Ev.head(2) * dt;
    }

    // saturate the world X
    double world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's world Y integral is being saturated!");
    }
  }

   //}

  /* body error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the body error                  |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0);  // position error in the untilted frame of the UAV
    Eigen::Vector2d Ev_fcu_untilted = Eigen::Vector2d(0, 0);  // velocity error in the untilted frame of the UAV

    // get the position control error in the fcu_untilted frame
    {

      geometry_msgs::Vector3Stamped Ep_stamped;

      Ep_stamped.header.stamp    = ros::Time::now();
      Ep_stamped.header.frame_id = uav_state_.header.frame_id;
      Ep_stamped.vector.x        = Ep(0);
      Ep_stamped.vector.y        = Ep(1);
      Ep_stamped.vector.z        = Ep(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the position error to fcu_untilted");
      }
    }

    // get the velocity control error in the fcu_untilted frame
    {
      geometry_msgs::Vector3Stamped Ev_stamped;

      Ev_stamped.header.stamp    = ros::Time::now();
      Ev_stamped.header.frame_id = uav_state_.header.frame_id;
      Ev_stamped.vector.x        = Ev(0);
      Ev_stamped.vector.y        = Ev(1);
      Ev_stamped.vector.z        = Ev(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ev_stamped);

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (position_cmd.use_position_horizontal) {
      Ib_b_ -= kibxy_ * Ep_fcu_untilted * dt;
    } else if (position_cmd.use_velocity_horizontal) {
      Ib_b_ -= kibxy_ * Ev_fcu_untilted * dt;
    }

    // saturate the body
    double body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: SE3's body roll integral is being saturated!");
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);
    /*QUESTION: do we need to make it work with rampup_active_ or do we assume erg does not need to simulate this phase? */
    if (position_cmd.use_position_vertical){// && !rampup_active_) {
      uav_mass_difference_ -= km_ * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3Controller]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {

    Eigen::Matrix3d des_orientation = mrs_lib::AttitudeConverter(Rd);
    Eigen::Vector3d thrust_vector   = thrust_force * des_orientation.col(2);

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = (thrust_vector[2] / total_mass) - _g_;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state->header.frame_id;
    world_accel.vector.x        = world_accel_x;
    world_accel.vector.y        = world_accel_y;
    world_accel.vector.z        = world_accel_z;

    auto res = common_handlers_->transformer->transformSingle("fcu", world_accel);

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------- saturate the attitude rate --------------- |

  if (got_constraints_) {

    auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

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
    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: missing dynamics constraints");
  }

  
  // | --------------- fill the resulting command --------------- |

  auto output_mode = mrs_lib::get_mutexed(mutex_output_mode_, output_mode_);

  // fill in the desired attitude anyway, since we know it
  output_command->attitude = mrs_lib::AttitudeConverter(Rd);

  if (output_mode == OUTPUT_ATTITUDE_RATE) {

    // output the desired attitude rate
    output_command->attitude_rate.x = t[0];
    output_command->attitude_rate.y = t[1];
    output_command->attitude_rate.z = t[2];

    output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  } else if (output_mode == OUTPUT_ATTITUDE_QUATERNION) {

    output_command->mode_mask = output_command->MODE_ATTITUDE;

    ROS_WARN_THROTTLE(1.0, "[Se3Controller]: outputting desired orientation (this is not normal)");
  }

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  /*QUESTION: do we need rampup_active_ in traj prediction? now commented*/
  // if (rampup_active_) {

  //   // deactivate the rampup when the times up
  //   if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

  //     rampup_active_         = false;
  //     output_command->thrust = thrust;

  //     ROS_INFO("[Se3Controller]: rampup finished");

  //   } else {

  //     double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

  //     rampup_thrust_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

  //     rampup_last_time_ = ros::Time::now();

  //     output_command->thrust = rampup_thrust_;

  //     ROS_INFO_THROTTLE(0.1, "[Se3Controller]: ramping up thrust, %.4f", output_command->thrust);
  //   }

  // } else {
  //   output_command->thrust = thrust;
  // }
  output_command->thrust = thrust;
  /*QUESTION: do we need rampup_active_ in traj prediction? now commented*/
  // output_command->ramping_up = rampup_active_;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = -Ib_b_[0];
  output_command->disturbance_by_b = -Ib_b_[1];

  output_command->disturbance_bx_w = -Ib_w[0];
  output_command->disturbance_by_w = -Ib_w[1];

  output_command->disturbance_wx_w = -Iw_w_[0];
  output_command->disturbance_wy_w = -Iw_w_[1];

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "Se3Controller";

  last_attitude_cmd_ = output_command;

  /*QUESTION: what to do now with the output_command?*/
  //return output_command;




/* end copy of se3controller*/
 // set the desired states from the input of the goto function


  

  // return a position command
  return mrs_msgs::PositionCommand::ConstPtr(new mrs_msgs::PositionCommand(position_cmd));
}
//}

/*getStatus()//{*/
const mrs_msgs::TrackerStatus DergbryanTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;
  tracker_status.active = is_active_;
  tracker_status.tracking_trajectory = false;
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
  std_srvs::TriggerResponse res;
  res.message = "hover initiated";
  res.success = true;

  hover_ = true;

  return std_srvs::TriggerResponse::ConstPtr(new std_srvs::TriggerResponse(res));
}
//}

/*startTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*stopTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*resumeTrajectoryTracking()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*gotoTrajectoryStart()//{*/
const std_srvs::TriggerResponse::ConstPtr DergbryanTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}
//}

/*setConstraints()//{*/
const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr DergbryanTracker::setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  got_constraints_ = true;

   ROS_INFO("[Se3Controller]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}
//}

/*setReference()//{*/
const mrs_msgs::ReferenceSrvResponse::ConstPtr DergbryanTracker::setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  mrs_msgs::ReferenceSrvResponse res;

  {
    std::scoped_lock lock(mutex_goal_);

  goal_x_=cmd->reference.position.x;
  goal_y_=cmd->reference.position.y;
  goal_z_=cmd->reference.position.z;
  goal_heading_=cmd->reference.heading;

  }

  hover_ = false;

  res.success = true;
  res.message = "reference set";

  return mrs_msgs::ReferenceSrvResponse::ConstPtr(new mrs_msgs::ReferenceSrvResponse(res));
}
//}

/*setTrajectoryReference()//{*/
const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr DergbryanTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}
//}

}  // namespace dergbryan_tracker
}  // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::dergbryan_tracker::DergbryanTracker, mrs_uav_managers::Tracker)