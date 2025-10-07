#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathConverterNode : public rclcpp::Node
{
public:
  PathConverterNode() : Node("path_converter_node")
  {
    // Subscription to the robot trajectory
    trajectory_sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
      "/moveit_path", 10,
      std::bind(&PathConverterNode::trajectory_callback, this, std::placeholders::_1));
    
    // Publisher for Cartesian path (nav_msgs::Path)
    cartesian_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/cartesian_path", 10);
    
    // Publisher for Cartesian poses (PoseArray for visualization)
    cartesian_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/cartesian_poses", 10);
    
    RCLCPP_INFO(this->get_logger(), "Path converter node initialized");
    RCLCPP_INFO(this->get_logger(), "Waiting for robot trajectory on /moveit_path");
  }

  void initialize()
  {
    // Initialize the robot model (called after construction)
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
      return;
    }
    
    // Get the joint model group
    joint_model_group_ = robot_model_->getJointModelGroup("ur_manipulator");
    if (!joint_model_group_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group 'ur_manipulator'");
      return;
    }
    
    // Create robot state
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully");
  }

private:
  void trajectory_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
  {
    if (!initialized_) {
      RCLCPP_WARN(this->get_logger(), "Robot model not initialized yet, skipping trajectory");
      return;
    }
    
    if (msg->joint_trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu waypoints", 
                msg->joint_trajectory.points.size());
    
    // Prepare output messages
    nav_msgs::msg::Path cartesian_path_msg;
    geometry_msgs::msg::PoseArray cartesian_poses_msg;
    
    cartesian_path_msg.poses.clear();
    cartesian_poses_msg.poses.clear();
    
    // Set headers
    auto stamp = this->get_clock()->now();
    cartesian_path_msg.header.frame_id = "base_link";  // Adjust frame as needed
    cartesian_path_msg.header.stamp = stamp;
    cartesian_poses_msg.header.frame_id = "base_link";
    cartesian_poses_msg.header.stamp = stamp;
    
    // Convert each waypoint from joint space to Cartesian space
    for (const auto& point : msg->joint_trajectory.points) {
      if (point.positions.size() != joint_model_group_->getVariableCount()) {
        RCLCPP_WARN(this->get_logger(), "Joint positions size mismatch: expected %zu, got %zu",
                    joint_model_group_->getVariableCount(), point.positions.size());
        continue;
      }
      
      // Set joint positions in robot state
      robot_state_->setJointGroupPositions(joint_model_group_, point.positions);
      
      // Get the end-effector pose using forward kinematics
      const Eigen::Isometry3d& end_effector_pose = 
        robot_state_->getGlobalLinkTransform("tool0");  // Adjust link name as needed
      
      // Extract position
      Eigen::Vector3d position = end_effector_pose.translation();
      
      // Extract orientation (quaternion)
      Eigen::Quaterniond orientation(end_effector_pose.rotation());
      
      // Create PoseStamped for Path message
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "base_link";
      pose_stamped.header.stamp = stamp;
      pose_stamped.pose.position.x = position.x();
      pose_stamped.pose.position.y = position.y();
      pose_stamped.pose.position.z = position.z();
      pose_stamped.pose.orientation.x = orientation.x();
      pose_stamped.pose.orientation.y = orientation.y();
      pose_stamped.pose.orientation.z = orientation.z();
      pose_stamped.pose.orientation.w = orientation.w();
      
      cartesian_path_msg.poses.push_back(pose_stamped);
      
      // Create Pose for PoseArray message (for visualization)
      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = position.x();
      pose_msg.position.y = position.y();
      pose_msg.position.z = position.z();
      pose_msg.orientation.x = orientation.x();
      pose_msg.orientation.y = orientation.y();
      pose_msg.orientation.z = orientation.z();
      pose_msg.orientation.w = orientation.w();
      
      cartesian_poses_msg.poses.push_back(pose_msg);
    }
    
    // Publish the results
    cartesian_path_pub_->publish(cartesian_path_msg);
    cartesian_poses_pub_->publish(cartesian_poses_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published %zu Cartesian waypoints to /cartesian_path", 
                cartesian_path_msg.poses.size());
    
    // Log first and last positions for debugging
    if (!cartesian_path_msg.poses.empty()) {
      const auto& first_pose = cartesian_path_msg.poses.front().pose;
      const auto& last_pose = cartesian_path_msg.poses.back().pose;
      RCLCPP_INFO(this->get_logger(), "First position: [%.3f, %.3f, %.3f]",
                  first_pose.position.x, first_pose.position.y, first_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Last position: [%.3f, %.3f, %.3f]",
                  last_pose.position.x, last_pose.position.y, last_pose.position.z);
    }
  }

  // ROS components
  rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cartesian_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cartesian_poses_pub_;
  
  // MoveIt components
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::shared_ptr<moveit::core::RobotState> robot_state_;
  
  bool initialized_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathConverterNode>();
  
  // Initialize the robot model after the node is fully constructed
  node->initialize();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

