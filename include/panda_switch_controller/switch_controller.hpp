#pragma once
#include "panda_switch_controller/core.hpp"
#include "panda_switch_controller/thread_pool.hpp"
#include "panda_switch_controller/panda_interface.hpp"

enum CtrlMode{
  CTRLMODE_STOP,
  CTRLMODE_IDLE,
  CTRLMODE_JOINT_IMP,
  CTRLMODE_TASK_IMP,
};

class SwitchController : protected ThreadPool{
public:
  Panda *robot;
  
  SwitchController(Panda *robot_ptr)
    : _ctrl_mode(0)
    , robot(robot_ptr)
    , _is_ctrl_running(false)
    , _is_stopped_by_error(false)
  {};
  ~SwitchController(){ 
    _ctrl_mode = CTRLMODE_STOP;
    StopCtrl();
  };

  void SetCtrlMode(int mode) {_ctrl_mode = mode;};
  int GetCtrlMode() {return _ctrl_mode;};
  bool IsStoppedByError() {return _is_stopped_by_error;}
  void ClearError() {_is_stopped_by_error = false;}

  void StopCtrl();
  void RunCtrl();
  void SetController(int ctrl_mode);

private:
  int _ctrl_mode;
  bool _is_ctrl_running;
  bool _is_stopped_by_error;
};