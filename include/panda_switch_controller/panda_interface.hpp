#pragma once

//frankalib
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

#include "panda_switch_controller/core.hpp"

typedef std::function<franka::Torques(const franka::RobotState&, franka::Duration)> TorqueCtrlLoopPanda;

class Panda{
protected:
  std::string ip;
  franka::Robot panda;
  franka::Model model;
  bool _stop_ctrl;
  bool _has_data;
  std::mutex m;

public:
  franka::RobotState state;
  std::string joint_names[7] = {
    "panda_joint1", 
    "panda_joint2", 
    "panda_joint3", 
    "panda_joint4", 
    "panda_joint5", 
    "panda_joint6", 
    "panda_joint7"
  };
  void IdleControl(); //Idle loop
  void UpdateState(franka::RobotState panda_state){
    if (m.try_lock()){
      _has_data = true;
      state = panda_state;
    }
  }
  bool IsDataReceived(){return _has_data;}
  void StopControl(){_stop_ctrl = true;}
  void CalculateJointImpedanceCtrl();
  void CalculateTaskImpedanceCtrl();

  Panda(const std::string &ip)
    : panda(ip)
    , model(panda.loadModel())
    , _has_data(false)
    , _stop_ctrl(false)
  {
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                      Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                          Eigen::MatrixXd::Identity(3, 3);
  };
  ~Panda(){_stop_ctrl = true;};
};