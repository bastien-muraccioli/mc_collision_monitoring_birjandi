#include "CollisionMonitoringBirjandi.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

constexpr unsigned CollisionMonitoringBirjandi::STATE_SIZE;
constexpr unsigned CollisionMonitoringBirjandi::MEASUREMENT_SIZE;
constexpr unsigned CollisionMonitoringBirjandi::INPUT_SIZE;

CollisionMonitoringBirjandi::~CollisionMonitoringBirjandi() = default;

CollisionMonitoringBirjandi::CollisionMonitoringBirjandi(): filter_(STATE_SIZE, MEASUREMENT_SIZE, INPUT_SIZE, false),  stateDynamics_(0.001){}

void CollisionMonitoringBirjandi::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();

  // Get the configuration
  auto plugin_config = config("collision_monitoring_birjandi");
  imuBodyName_.assign(plugin_config("imuBodyName"));
  robotBodyName_.assign(plugin_config("robotBodyName"));
  jointBodyName_.assign(plugin_config("jointBodyName"));
  kt = plugin_config("kt");
  gear_ratio = plugin_config("gear_ratio", 100.0);
  threshold_offset_ = plugin_config("threshold_offset", 2.0);
  threshold_filtering_ = plugin_config("threshold_filtering", 0.05);
  

  // Copy the robot to the controller
  robot_copied_ = mc_rbdyn::Robots::make();
  robot_copied_->robotCopy(robot, robot.name());
  robot_copied_->robotCopy(realRobot, "inputRobot");

  auto & inputRobot = robot_copied_->robot("inputRobot");
  forwardDynamics_hat_ = rbd::ForwardDynamics(inputRobot.mb());
  forwardDynamics_ = rbd::ForwardDynamics(realRobot.mb());

  jointNumber_ = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  lpf_threshold_.setValues(threshold_offset_, threshold_filtering_, jointNumber_);
  
  imu_not_yet_initialized_ = true;
  dt_ = ctl.timestep();

  stateDynamics_.setSamplingPeriod(dt_);
  filter_.setFunctor(&stateDynamics_);

  R_joint_ = realRobot.bodyPosW(jointBodyName_).rotation();

  accelero_ = Eigen::Vector3d::Zero();
  ya_ = Eigen::Vector3d::Zero();
  gyro_ = Eigen::Vector3d::Zero();
  angveldot_ = Eigen::Vector3d::Zero();
  acclin_joint_ = Eigen::Vector3d::Zero();
  angvel_joint_ = Eigen::Vector3d::Zero();

  q_ = realRobot.encoderValues()[jointNumber_-2]; // Initial joint position from encoders
  mc_rtc::log::info("[CollisionMonitoringBirjandi] Initial q where IMU is set: {} rad", q_);
  qdot_hat_ = 0.0;
  qddot_hat_ = 0.0;

  qdot_ = 0.0;
  qddot_ = 0.0;
  qddot_qp_ = 0.0;
  qddot_accelero_ = 0.0;

  // Compute the covariance matrix
  Eigen::VectorXd v(7); // Dimension of q + accelero + gyro
  v << qCovariance_, acceleroCovariance_, acceleroCovariance_, acceleroCovariance_, gyroCovariance_, gyroCovariance_, gyroCovariance_;
  r_covariance_ = v.asDiagonal();
  q_covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * stateCov;
  mc_rtc::log::info("[CollisionMonitoringBirjandi] R size: {}x{}", r_covariance_.rows(), r_covariance_.cols());
  mc_rtc::log::info("[CollisionMonitoringBirjandi] Q size: {}x{}", q_covariance_.rows(), q_covariance_.cols());

  // Initialize EKF covariances
  filter_.setQ(q_covariance_);
  filter_.setR(r_covariance_);
  filter_.setStateCovariance(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * stateInitCov);
  
  Eigen::VectorXd x0(STATE_SIZE);
  x0 << q_, 0, 0, 0;
  filter_.setState(x0, 0);

  Eigen::VectorXd y0(MEASUREMENT_SIZE);
  y0 << q_, 0, 0, 0, 0, 0, 0;
  filter_.setMeasurement(y0, 0);
  tau_m = Eigen::VectorXd::Zero(jointNumber_);
  tau_fric = Eigen::VectorXd::Zero(jointNumber_);

  tau_high_ = Eigen::VectorXd::Zero(jointNumber_);
  tau_low_ = Eigen::VectorXd::Zero(jointNumber_);

  addLog(ctl);
  addGui(ctl);

  mc_rtc::log::info("CollisionMonitoringBirjandi::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionMonitoringBirjandi::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::reset called");
}

void CollisionMonitoringBirjandi::before(mc_control::MCGlobalController & controller)
{
  counter_ += dt_;
  if(activate_plot_ && !plot_added_)
  {
    addPlot(controller);
    plot_added_ = true;
  }
  collisionDetection(controller);
  // mc_rtc::log::info("CollisionMonitoringBirjandi::before");
}

void CollisionMonitoringBirjandi::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionMonitoringBirjandi::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void CollisionMonitoringBirjandi::computeTauExtRealRobot(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();

    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();

    forwardDynamics_.computeC(robot.mb(), robot.mbc());
    forwardDynamics_.computeH(robot.mb(), robot.mbc());

    auto coriolis = new rbd::Coriolis(robot.mb());
    coriolisMatrix_ = coriolis->coriolis(robot.mb(), robot.mbc());
    auto forwardDynamics = rbd::ForwardDynamics(robot.mb());

    forwardDynamics.computeH(robot.mb(), robot.mbc());
    inertiaMatrix_ = forwardDynamics.H() - forwardDynamics.HIr();

    if(ctl.controller().datastore().has("torque_fric"))
    {
      tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
    }
    else
    {
      tau_fric.setZero(jointNumber_);
      mc_rtc::log::error("[CollisionMonitoringBirjandi] No torque_fric in datastore");
    }
    int jointIndex = 0;
    for(auto const& [key, val] : kt)
    {
        if (jointIndex >= 0 && jointIndex < jointNumber_) {
            // Calculate motor torque and assign it to tau_m
            double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
            tau_m[jointIndex] = tau_mot;
        } else {
            mc_rtc::log::error("[CollisionMonitoringBirjandi] Invalid joint name: {} or index out of bounds", key);
        }
        jointIndex++;
    }

    auto coriolisGravityTerm = forwardDynamics_.C(); //C*qdot + g
    tau_ext_ = inertiaMatrix_ * qddot_hat_ + coriolisGravityTerm + tau_fric - tau_m;
}

void CollisionMonitoringBirjandi::computeTauExtInputRobot(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();
    auto & inputRobot = robot_copied_->robot("inputRobot");

    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();

    inputRobot.forwardKinematics();
    inputRobot.forwardVelocity();
    inputRobot.forwardAcceleration();


    auto & realQ = robot.mbc().q;
    auto & realAlpha = robot.mbc().alpha;
    auto & realAlphaD = robot.mbc().alphaD;

    Eigen::VectorXd realQ_v(jointNumber_);
    Eigen::VectorXd realAlpha_v(jointNumber_);
    Eigen::VectorXd realAlphaD_v(jointNumber_);

    rbd::paramToVector(robot.mbc().q, realQ_v);
    rbd::paramToVector(robot.mbc().alpha, realAlpha_v);
    rbd::paramToVector(robot.mbc().alphaD, realAlphaD_v);

    Eigen::VectorXd realQ_v_hat(jointNumber_);
    Eigen::VectorXd realAlpha_v_hat(jointNumber_);
    Eigen::VectorXd realAlphaD_v_hat(jointNumber_);

    qddot_qp_ = realAlphaD_v[5];

    realQ_v_hat = realQ_v;
    realAlpha_v_hat = realAlpha_v;
    realAlphaD_v_hat = realAlphaD_v;
    realAlpha_v_hat[5] = qdot_hat_;
    realAlphaD_v_hat[5] = qddot_hat_;


    rbd::vectorToParam(realQ_v_hat, realQ);
    rbd::vectorToParam(realAlpha_v_hat, realAlpha);
    rbd::vectorToParam(realAlphaD_v_hat, realAlphaD);


    std::copy(std::next(realQ.begin()), realQ.end(), std::next(inputRobot.mbc().q.begin()));
    std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(inputRobot.mbc().alpha.begin()));
    std::copy(std::next(realAlphaD.begin()), realAlphaD.end(), std::next(inputRobot.mbc().alphaD.begin()));


    Eigen::VectorXd realQ_v_hat_temp(jointNumber_);
    Eigen::VectorXd realAlpha_v_hat_temp(jointNumber_);
    Eigen::VectorXd realAlphaD_v_hat_temp(jointNumber_);

    
    rbd::paramToVector(inputRobot.mbc().q, realQ_v_hat_temp);
    rbd::paramToVector(inputRobot.mbc().alpha, realAlpha_v_hat_temp);
    rbd::paramToVector(inputRobot.mbc().alphaD, realAlphaD_v_hat_temp);

    forwardDynamics_hat_.computeC(inputRobot.mb(), inputRobot.mbc());
    forwardDynamics_hat_.computeH(inputRobot.mb(), inputRobot.mbc());

    auto coriolis_hat = new rbd::Coriolis(inputRobot.mb());
    coriolisMatrix_hat_ = coriolis_hat->coriolis(inputRobot.mb(), inputRobot.mbc());
    

    forwardDynamics_hat_.computeH(inputRobot.mb(), inputRobot.mbc());
    inertiaMatrix_hat_ = forwardDynamics_hat_.H() - forwardDynamics_hat_.HIr();

    if(ctl.controller().datastore().has("torque_fric"))
    {
      tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
    }
    else
    {
      tau_fric.setZero(jointNumber_);
      mc_rtc::log::error("[CollisionMonitoringBirjandi] No torque_fric in datastore");
    }
    int jointIndex = 0;
    for(auto const& [key, val] : kt)
    {
        if (jointIndex >= 0 && jointIndex < jointNumber_) {
            // Calculate motor torque and assign it to tau_m
            double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
            tau_m[jointIndex] = tau_mot;
        } else {
            mc_rtc::log::error("[CollisionMonitoringBirjandi] Invalid joint name: {} or index out of bounds", key);
        }
        jointIndex++;
    }

    auto coriolisGravityTerm = forwardDynamics_hat_.C(); //C*qdot + g
    tau_ext_hat_ = inertiaMatrix_hat_ * qddot_hat_ + coriolisGravityTerm + tau_fric - tau_m;
}

void CollisionMonitoringBirjandi::collisionDetection(mc_control::MCGlobalController & ctl)
{
    updateFilter(ctl);
    computeTauExtRealRobot(ctl);
    computeTauExtInputRobot(ctl);
    obstacle_detected_ = false;
    tau_high_ = lpf_threshold_.adaptiveThreshold(tau_ext_hat_, true);
    tau_low_ = lpf_threshold_.adaptiveThreshold(tau_ext_hat_, false);
    if(tau_ext_hat_[5] > tau_high_[5] || tau_ext_hat_[5] < tau_low_[5])
    {
        obstacle_detected_ = true;
        if(collision_stop_activated_)
        {
          ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
        }
    }
}

void CollisionMonitoringBirjandi::updateFilter(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();

    if(!robot.hasBodySensor(imuBodyName_))
    {
        mc_rtc::log::error("[CollisionMonitoringBirjandi] Body sensor {} does not exist in the robot. Jerk Estimation is impossible", imuBodyName_);
        return;
    }

    const mc_rbdyn::BodySensor & imu = robot.bodySensor(imuBodyName_);

    if(imu.linearAcceleration().norm() != 0.0 && imu.angularVelocity().norm() != 0.0 && imu_not_yet_initialized_)
    {
        imu_not_yet_initialized_ = false;
        accelero_ = imu.linearAcceleration();
        gyro_ = imu.angularVelocity();
    }

    if(imu_not_yet_initialized_)
    {
        return;
    }

    
    // Measurements
    q_ = realRobot.encoderValues()[jointNumber_-2];
    auto R_imu_w = realRobot.bodyPosW(robotBodyName_).rotation();
    auto R_joint_w = realRobot.bodyPosW(jointBodyName_).rotation();
    R_joint_ = R_joint_w * R_imu_w.transpose();
    stateDynamics_.setJointOrientation(R_joint_);
    

    // Computing the acceleration and angular velocity in the joint frame
    Eigen::VectorXd prev_angvel_ = angvel_joint_;
    
    auto P_imu = realRobot.bodySensor(imuBodyName_).position();
    auto P_joint = realRobot.bodyPosW(jointBodyName_).translation();
    auto X_pos = R_joint_.transpose() * (P_imu - P_joint);
    stateDynamics_.setJointPosition(X_pos);

    gyro_ = R_joint_.transpose() * imu.angularVelocity();
    gyro_[1] = -gyro_[1];
    accelero_ = R_joint_.transpose() * imu.linearAcceleration();
    // accelero_[1] = -accelero_[1];
    
    Eigen::VectorXd y(7);
    y << q_, accelero_.x(), accelero_.y(), accelero_.z(), gyro_.x(), gyro_.y(), gyro_.z();
    
    Eigen::MatrixXd A(4, 4);
    A <<  0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, 0, 0;
    // Discretize A
    Eigen::MatrixXd A_discrete(4, 4);
    A_discrete = (A * dt_).exp();
    filter_.setA(A_discrete);
   
    
    const stateObservation::Vector dx = filter_.stateVectorConstant(1) * 1e-8;
    auto CFD = filter_.getCMatrixFD(dx);
    filter_.setC(CFD);

    auto time = filter_.getCurrentTime();
    filter_.setMeasurement(y, time + 1);
    
    // Run EKF and update state estimate
    auto xk = filter_.getEstimatedState(time + 1);
    qdot_hat_ = xk(1);
    qddot_hat_ = xk(2);
    ya_ = stateDynamics_.getYaMeasurement(); // Dynamic of the acceleration measurement

    // Ground truth (derived from encoders)
    double qdot_prev = qdot_;
    qdot_ = realRobot.encoderVelocities()[jointNumber_-2];
    double qddot_prev = qddot_;
    qddot_ = (qdot_ - qdot_prev) / dt_;
    // filtered
    qddot_ = a_filter_ * qddot_ + (1-a_filter_) * qddot_prev;
    qddot_accelero_ = (imu.linearAcceleration()[1]-9.81)/distance_imu_joint_;
}

void CollisionMonitoringBirjandi::addPlot(mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    gui.addPlot(
        "qdot CollisionMonitoringBirjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("gyro", [this]() { return gyro_[1]; }, mc_rtc::gui::Color::Blue),
        mc_rtc::gui::plot::Y("qdot_hat", [this]() { return qdot_hat_; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("qdot", [this]() { return qdot_; }, mc_rtc::gui::Color::Green)
    );
    gui.addPlot(
        "qddot CollisionMonitoringBirjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("qddot_accelero", [this]() { return qddot_accelero_; }, mc_rtc::gui::Color::Cyan),
        mc_rtc::gui::plot::Y("qddot_qp", [this]() { return qddot_qp_; }, mc_rtc::gui::Color::Magenta),
        mc_rtc::gui::plot::Y("ya", [this]() { return ya_[1]; }, mc_rtc::gui::Color::Blue),
        mc_rtc::gui::plot::Y("qddot_hat", [this]() { return qddot_hat_; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("qddot", [this]() { return qddot_; }, mc_rtc::gui::Color::Green)
    );

    gui.addPlot(
        "tau_ext CollisionMonitoringBirjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("tau_high", [this]() { return tau_high_[5]; }, mc_rtc::gui::Color::Gray),
        mc_rtc::gui::plot::Y("tau_low", [this]() { return tau_low_[5]; }, mc_rtc::gui::Color::Gray),
        mc_rtc::gui::plot::Y("tau_ext_hat", [this]() { return tau_ext_hat_[5]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("tau_ext", [this]() { return tau_ext_[5]; }, mc_rtc::gui::Color::Green)
    );
}

void CollisionMonitoringBirjandi::addLog(mc_control::MCGlobalController & ctl)
{
    ctl.controller().logger().addLogEntry("Birjandi_q", [this]() { return q_; });
    ctl.controller().logger().addLogEntry("Birjandi_qdot_hat", [this]() { return qdot_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_qdot", [this]() { return qdot_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot_hat", [this]() { return qddot_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot", [this]() { return qddot_; });
    ctl.controller().logger().addLogEntry("Birjandi_acclin_joint", [this]() { return acclin_joint_; });
    ctl.controller().logger().addLogEntry("Birjandi_angvel_joint", [this]() { return angvel_joint_; });
    ctl.controller().logger().addLogEntry("Birjandi_gyro", [this]() { return gyro_; });
    ctl.controller().logger().addLogEntry("Birjandi_accelero", [this]() { return accelero_; });
    ctl.controller().logger().addLogEntry("Birjandi_angveldot", [this]() { return angveldot_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_ext_hat", [this]() { return tau_ext_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_ext", [this]() { return tau_ext_; });
    ctl.controller().logger().addLogEntry("Birjandi_ya", [this]() { return ya_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot_accelero", [this]() { return qddot_accelero_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot_qp", [this]() { return qddot_qp_; });
    ctl.controller().logger().addLogEntry("Birjandi_obstacle_detected", [this]() { return obstacle_detected_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_high", [this]() { return tau_high_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_low", [this]() { return tau_low_; });
}

void CollisionMonitoringBirjandi::addGui(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  
  gui.addElement({"Plugins", "CollisionMonitoringBirjandi"}, 
    // Add button to activate the plots
    mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }),
    // Add checkbox to activate the collision stop
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_), 
    // Add Threshold offset input
    mc_rtc::gui::NumberInput("Threshold offset", [this](){return this->threshold_offset_;},
       [this](double offset)
      { 
        threshold_offset_ = offset;
        lpf_threshold_.setOffset(threshold_offset_); 
      }),
    // Add Threshold filtering input
    mc_rtc::gui::NumberInput("Threshold filtering", [this](){return this->threshold_filtering_;},
       [this](double filtering)
      { 
        threshold_filtering_ = filtering;
        lpf_threshold_.setFiltering(threshold_filtering_); 
      })
  );
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionMonitoringBirjandi", mc_plugin::CollisionMonitoringBirjandi)
