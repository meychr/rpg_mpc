/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "rpg_mpc/mpc_controller.h"

#include <ctime>

namespace rpg_mpc {

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& topic) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    // TODO adapt for foldable drone
    est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
                                                  0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.785, 0.785, 0.785, 0.785).finished()),
    servo_angles_((Eigen::Matrix<T, 4, 1>() << 0.785, 0.785, 0.785, 0.785).finished()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    point_of_interest_(Eigen::Matrix<T, 3, 1>::Zero()) {
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(topic, 1);

  sub_point_of_interest_ = nh_.subscribe("mpc/point_of_interest", 1,
                                         &MpcController<T>::pointOfInterestCallback, this);
  sub_autopilot_off_ = nh_.subscribe("autopilot/off", 1,
                                     &MpcController<T>::offCallback, this);
  // TODO check if better solution for publishing and subscribing
  // to get correct state estimate of servo angles: foldable_drone_feedback
  // but does not work good with control command update => problem with velocity vs position control!
  // see updateControlCommand()
  sub_servo_angles_ = nh_.subscribe("set_servo", 1,
                                         &MpcController<T>::servoAnglesCallback, this);
  pub_servo_angle_cmd_ =  nh_.advertise<foldable_drone_msgs::FoldableDroneServoAngles>("set_servo", 1);

  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template<typename T>
void MpcController<T>::pointOfInterestCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  point_of_interest_(0) = msg->point.x;
  point_of_interest_(1) = msg->point.y;
  point_of_interest_(2) = msg->point.z;
  mpc_wrapper_.setPointOfInterest(point_of_interest_);
}

template<typename T>
void MpcController<T>::servoAnglesCallback(
    const foldable_drone_msgs::FoldableDroneServoAngles::ConstPtr& msg){
  servo_angles_(0) = msg->angle_servo_0;
  servo_angles_(1) = msg->angle_servo_1;
  servo_angles_(2) = msg->angle_servo_2;
  servo_angles_(3) = msg->angle_servo_3;
}

template<typename T>
void MpcController<T>::offCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  solve_from_scratch_ = true;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::off() {
  quadrotor_common::ControlCommand command;

  command.zero();

  return command;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const quadrotor_common::Trajectory& reference_trajectory,
    const MpcParams<T>& params) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  preparation_thread_.join();

  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  // reference values are the ones used to define the desired state => (x - x_reference), can be left at zero for servo angles of foldable_drone
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with hover as initial guess.");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Publish the predicted trajectory.
  // TODO Does not publish predicted servo angles
  publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  return updateControlCommand(predicted_states_.col(0),
                              predicted_inputs_.col(0),
                              call_time);
}

template<typename T>
bool MpcController<T>::setStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate) {
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  est_state_(kServo0) = servo_angles_(0);
  est_state_(kServo1) = servo_angles_(1);
  est_state_(kServo2) = servo_angles_(2);
  est_state_(kServo3) = servo_angles_(3);
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
  return quaternion_norm_ok;
}

template<typename T>
bool MpcController<T>::setReference(
    const quadrotor_common::Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory.points.size() == 1) {
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
    // TODO adapt for foldable drone, use 0.785? (for X config as equilibrium)
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().velocity.template cast<T>(),
        0.785, 0.785, 0.785, 0.785  // servo angles necessary or not?
    ).finished().replicate(1, kSamples + 1);

    acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
    // TODO adapt for foldable drone, use 0.0 servo angle rate
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>()
      << acceleration.norm(),
      reference_trajectory.points.front().bodyrates.template cast<T>(),
      0.0, 0.0, 0.0, 0.0 // servo angle rate
    ).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      q_orientation = q_heading * iterator->orientation.template cast<T>();
      // TODO adapt for foldable drone, use 0.785? (for X config as equilibrium)
      reference_states_.col(i) << iterator->position.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          iterator->velocity.template cast<T>(),
          0.785, 0.785, 0.785, 0.785;  // servo angles
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      acceleration << iterator->acceleration.template cast<T>() - gravity;
      // TODO adapt for foldable drone, use 0.0 servo angle rate
      reference_inputs_.col(i) << acceleration.norm(),
          iterator->bodyrates.template cast<T>(),
          0.0, 0.0, 0.0, 0.0;  // servo angle rates
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
quadrotor_common::ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));
  input_bounded(INPUT::kRateServo0) = std::max(-params_.max_servo_angle_rate_,
                                          std::min(params_.max_servo_angle_rate_, input_bounded(INPUT::kRateServo0)));
  input_bounded(INPUT::kRateServo1) = std::max(-params_.max_servo_angle_rate_,
                                          std::min(params_.max_servo_angle_rate_, input_bounded(INPUT::kRateServo1)));
  input_bounded(INPUT::kRateServo2) = std::max(-params_.max_servo_angle_rate_,
                                          std::min(params_.max_servo_angle_rate_, input_bounded(INPUT::kRateServo2)));
  input_bounded(INPUT::kRateServo3) = std::max(-params_.max_servo_angle_rate_,
                                          std::min(params_.max_servo_angle_rate_, input_bounded(INPUT::kRateServo3)));

  const T dt_mpc = mpc_wrapper_.getTimestep();  // in secs
  foldable_drone_msgs::FoldableDroneServoAngles servo_angle_command;
  servo_angle_command.header.stamp = time;
  servo_angle_command.angle_servo_0 = std::max(params_.min_servo_angle_,
                                         std::min(params_.max_servo_angle_, state(STATE::kServo0) + input_bounded(INPUT::kRateServo0) * dt_mpc));
  servo_angle_command.angle_servo_1 = std::max(params_.min_servo_angle_,
                                         std::min(params_.max_servo_angle_, state(STATE::kServo1) + input_bounded(INPUT::kRateServo1) * dt_mpc));
  servo_angle_command.angle_servo_2 = std::max(params_.min_servo_angle_,
                                         std::min(params_.max_servo_angle_, state(STATE::kServo2) + input_bounded(INPUT::kRateServo2) * dt_mpc));
  servo_angle_command.angle_servo_3 = std::max(params_.min_servo_angle_,
                                         std::min(params_.max_servo_angle_, state(STATE::kServo3) + input_bounded(INPUT::kRateServo3) * dt_mpc));

  pub_servo_angle_cmd_.publish(servo_angle_command);

  quadrotor_common::ControlCommand command;

  command.timestamp = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);
  return command;
}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i < kSamples; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);

  return true;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_, params.min_servo_angle_, params.max_servo_angle_, params.max_servo_angle_rate_);
  mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace rpg_mpc
