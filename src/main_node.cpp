#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "panda_switch_controller/switch_controller.hpp"
#include "panda_switch_controller/panda_interface.hpp"

using namespace std::chrono_literals;

class SwitchControllerNode : public rclcpp::Node, public SwitchController
{
public:
  SwitchControllerNode(Panda *robot_ptr)
    : Node("switch_controller")
    , SwitchController(robot_ptr)
  {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    main_loop_timer_ = this->create_wall_timer(
      10ms, std::bind(&SwitchControllerNode::LoopCallback, this));
  }

private:
  void LoopCallback()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    // fill msg. ee pose(robot_state_publisher)
    joint_state_pub_-> publish(msg);
    
    if(IsStoppedByError()){
      RCLCPP_INFO(this->get_logger(), "Error: set to idle mode");
      SetController(CTRLMODE_IDLE);
      ClearError();
    }
  }
  rclcpp::TimerBase::SharedPtr main_loop_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
  const std::string ip = "172.16.0.2";
  std::shared_ptr<Panda> panda = std::make_shared<Panda>(ip);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwitchControllerNode>(panda));
  rclcpp::shutdown();
  return 0;
}
// TODO: key input, panda state, ...