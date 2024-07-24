#include "panda_switch_controller/switch_controller.hpp"

void SwitchController::SetController(int ctrl_mode_cmd){
  if (_is_ctrl_running){
    StopCtrl();
  }
  _is_ctrl_running = true;
  _ctrl_mode = ctrl_mode_cmd;
  EnqueueJob([this](){ this->RunCtrl(); }); 
}

void SwitchController::RunCtrl(){
  try{
    switch(_ctrl_mode){
      case CTRLMODE_STOP:
        std::cout << "stopped" << std::endl;
        StopCtrl();
      break;
      case CTRLMODE_IDLE:
        std::cout << "idle" << std::endl;
      break;
      case CTRLMODE_JOINT_IMP:
        std::cout << "joint imp ctrl" << std::endl;
      break;
      case CTRLMODE_TASK_IMP:
        std::cout << "task imp ctrl" << std::endl;
      break;
    }
  } catch(int expn) {
    StopCtrl();
    _is_stopped_by_error = true;
  }
}

void SwitchController::StopCtrl(){
  _ctrl_mode = CTRLMODE_STOP;
  // robot->StopControl();
  std::cout << "Robot control stopped" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

