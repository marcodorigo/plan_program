// Initial joint values and joint limits (necessary to avoid that the robot takes random path towards target) are found in the Universal_Robots_ROS2_Description>Config>ur5e

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class PlanNode : public rclcpp::Node
{
public:
  PlanNode() : Node("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    target_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", 10,
      std::bind(&PlanNode::target_position_callback, this, std::placeholders::_1));

    // Publisher for the planned path
    path_publisher_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("/moveit_path", 10);

    // Timer for planning every second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PlanNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for target position on /target_position topic...");
  }

  void initialize()
  {
    // Create the MoveIt MoveGroup Interface after the node is fully constructed
    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    
    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id = move_group_interface_->getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id;
      collision_object.id = "sphere1";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the sphere
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[primitive.SPHERE_RADIUS] = 0.07;

      // Define the pose of the sphere (relative to the frame_id)
      geometry_msgs::msg::Pose sphere_pose;
      sphere_pose.orientation.w = 1.0;
      sphere_pose.position.x = 0.08;
      sphere_pose.position.y = 0.5;
      sphere_pose.position.z = 0.44;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(sphere_pose);
      collision_object.operation = collision_object.ADD;

      return collision_object;
    }();

    // Add the collision object to the scene
    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    planning_scene_interface_->applyCollisionObject(collision_object);
    
    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized");
    RCLCPP_INFO(this->get_logger(), "Added spherical obstacle at [0.08, 0.5, 0.44] with radius 0.07");
  }

private:
  void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN(this->get_logger(), "Received target position with insufficient data");
      return;
    }
    latest_target_position_ = *msg;
    has_target_ = true;
    RCLCPP_INFO(this->get_logger(), "Received target position: [%.3f, %.3f, %.3f]", 
                msg->data[0], msg->data[1], msg->data[2]);
  }

  void timer_callback()
  {
    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveIt interface not initialized yet");
      return;
    }
    if (!has_target_) {
      RCLCPP_INFO(this->get_logger(), "No target position received yet.");
      return;
    }

    // Convert Float32MultiArray to Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = latest_target_position_.data[0];
    target_pose.position.y = latest_target_position_.data[1];
    target_pose.position.z = latest_target_position_.data[2];
    
    // Set default orientation
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group_interface_->setPoseTarget(target_pose);

    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();

    if(success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful!");
      
      // Publish the planned trajectory
      path_publisher_->publish(plan.trajectory_);
      RCLCPP_INFO(this->get_logger(), "Published path to /moveit_path");
      
      RCLCPP_INFO(this->get_logger(), "Executing plan...");
      move_group_interface_->execute(plan);
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed!");
    }
  }

  std_msgs::msg::Float32MultiArray latest_target_position_;
  bool has_target_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr path_publisher_;
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


// joint_limits:
//   shoulder_pan_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 150.0
//     max_position: !degrees  370.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees -10.0
//   shoulder_lift_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 150.0
//     max_position: !degrees  10.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees -100.0
//   elbow_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 150.0
//     # we artificially limit this joint to half its actual joint position limit
//     # to avoid (MoveIt/OMPL) planning problems, as due to the physical
//     # construction of the robot, it's impossible to rotate the 'elbow_joint'
//     # over more than approx +- 1 pi (the shoulder lift joint gets in the way).
//     #
//     # This leads to planning problems as the search space will be divided into
//     # two sections, with no connections from one to the other.
//     #
//     # Refer to https://github.com/ros-industrial/universal_robot/issues/265 for
//     # more information.
//     max_position: !degrees  180.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees 0.0
//   wrist_1_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 28.0
//     max_position: !degrees  -40.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees -140.0
//   wrist_2_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 28.0
//     max_position: !degrees  -60.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees -120.0
//   wrist_3_joint:
//     # acceleration limits are not publicly available
//     has_acceleration_limits: false
//     has_effort_limits: true
//     has_position_limits: true
//     has_velocity_limits: true
//     max_effort: 28.0
//     max_position: !degrees  360.0
//     max_velocity: !degrees  180.0
//     min_position: !degrees -360.0


// shoulder_pan_joint: 1.57
// shoulder_lift_joint: -1.57
// elbow_joint: 1.57
// wrist_1_joint: -1.57
// wrist_2_joint: -1.57
// wrist_3_joint: 0.0
