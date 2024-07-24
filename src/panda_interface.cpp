#include "panda_switch_controller/panda_interface.hpp"


void Panda::IdleControl(){
  _stop_ctrl = false;
  std::function<bool(const franka::RobotState&)> loop_fn_panda; 
  loop_fn_panda = [&](const franka::RobotState& panda_state){
    UpdateState(panda_state);
    return !_stop_ctrl; //if stop_ctrl is true, break
  };
}

franka::Torques Panda::CalculateJointImpedanceCtrl(const franka::RobotState& state){
  std::array<double, 7> tau_d;
  std::array<double, 7> coriolis = model.coriolis(state);
  
  for (size_t i = 0; i < 7; i++) {
    tau_d[i] =
        joint_k_array[i] * (state.q_d[i] - state.q[i]) - joint_d_array[i] * state.dq[i] + coriolis[i];
  }
  return tau_d;
}

franka::Torques Panda::CalculateTaskImpedanceCtrl(const franka::RobotState& state){
  std::array<double, 7> coriolis = model.coriolis(state);
  std::array<double, 42> jacobian_array =
    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d;
  // orientation error
  // "difference" quaternion
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7);

  // Spring damper system with damping ratio=1
  tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
  tau_d << tau_task + coriolis;
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
  return tau_d_array;
}

void Panda::TorqueControl(TorqueCtrlLoopFn ctrl_fn){
  TorqueControlLoopFn loop_fn_panda;
  _stop_ctrl = false;

  loop_fn_panda = [&](const franka::RobotState& panda_state, franka::Duration) -> franka::Torques{
    if (m.try_lock()){
      UpdateState(panda_state);
      RobotTorque7 torque = ctrl_fn(panda_state);
      m.unlock();
    }
    // stop : if stop_ctrl flag turned 'true', this loop should break
    if (_stop_ctrl){
      return franka::MotionFinished(franka::Torques(torque));
    }
    return torque;
  };

  UpdateState(panda.readOnce()); //read state once for initialize
  panda.control(loop_fn_panda); //run loop
}
