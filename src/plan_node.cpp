#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class PlanNode : public rclcpp::Node
{
public:
  PlanNode() : Node("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Subscribe to target position topic (Float32MultiArray)
    target_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", 10,
      std::bind(&PlanNode::target_position_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for target position on /target_position topic...");
  }

  void initialize()
  {
    // Create the MoveIt MoveGroup Interface after the node is fully constructed
    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    
    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized");
  }

private:
  void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveIt interface not initialized yet");
      return;
    }

    if (msg->data.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "Received target position with insufficient data");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target position: [%.3f, %.3f, %.3f]", 
                msg->data[0], msg->data[1], msg->data[2]);

    // Convert Float32MultiArray to Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = msg->data[0];
    target_pose.position.y = msg->data[1];
    target_pose.position.z = msg->data[2];
    
    // Set default orientation
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    // Set the target pose
    move_group_interface_->setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Only log if planning is successful
    if(success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful!");
      RCLCPP_INFO(this->get_logger(), "Executing plan...");
      move_group_interface_->execute(plan);
    }
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<PlanNode>();
  
  // Initialize MoveIt after the node is fully created
  node->initialize();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}