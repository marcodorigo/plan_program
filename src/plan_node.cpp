// Initial joint values and joint limits (necessary to avoid that the robot takes random path towards target) are found in the Universal_Robots_ROS2_Description>Config>ur5e

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

class PlanNode : public rclcpp::Node
{
public:
  PlanNode() : Node("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    target_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/target_position", 10,
      std::bind(&PlanNode::target_position_callback, this, std::placeholders::_1));

    // Subscription for spherical obstacles
    spherical_obstacles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/spherical_obstacles", 10,
      std::bind(&PlanNode::spherical_obstacles_callback, this, std::placeholders::_1));

    // Publisher for the planned path
    path_publisher_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("/moveit_path", 10);

    // Timer for planning every five seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PlanNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for target position on /target_position topic...");
    RCLCPP_INFO(this->get_logger(), "Waiting for spherical obstacles on /spherical_obstacles topic...");
  }

  void initialize()
  {
    // Create the MoveIt MoveGroup Interface after the node is fully constructed
    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");
    
    // Initialize planning scene interface
    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    
    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized");
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

  void spherical_obstacles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!planning_scene_interface_) {
      RCLCPP_WARN(this->get_logger(), "Planning scene interface not initialized yet");
      return;
    }

    // Clear existing collision objects
    std::vector<std::string> object_ids;
    for (size_t i = 0; i < spherical_obstacles_.size(); ++i) {
      object_ids.push_back("sphere" + std::to_string(i));
    }
    if (!object_ids.empty()) {
      planning_scene_interface_->removeCollisionObjects(object_ids);
    }

    spherical_obstacles_.clear();
    
    // Parse the received data: [x1, y1, z1, radius1, x2, y2, z2, radius2, ...]
    for (size_t i = 0; i + 3 < msg->data.size(); i += 4) {
      SphericalObstacle obstacle;
      obstacle.center_x = msg->data[i];
      obstacle.center_y = msg->data[i + 1];
      obstacle.center_z = msg->data[i + 2];
      obstacle.radius = msg->data[i + 3];
      
      // Only add obstacles with non-zero radius
      if (obstacle.radius > 0.0) {
        spherical_obstacles_.push_back(obstacle);
      }
    }

    // Create and add collision objects for each spherical obstacle
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    for (size_t i = 0; i < spherical_obstacles_.size(); ++i) {
      const auto& obstacle = spherical_obstacles_[i];
      
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
      collision_object.id = "sphere" + std::to_string(i);
      
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[primitive.SPHERE_RADIUS] = obstacle.radius;

      geometry_msgs::msg::Pose sphere_pose;
      sphere_pose.orientation.w = 1.0;
      sphere_pose.position.x = obstacle.center_x;
      sphere_pose.position.y = obstacle.center_y;
      sphere_pose.position.z = obstacle.center_z;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(sphere_pose);
      collision_object.operation = collision_object.ADD;
      
      collision_objects.push_back(collision_object);
    }

    // Apply all collision objects to the scene
    if (!collision_objects.empty()) {
      planning_scene_interface_->applyCollisionObjects(collision_objects);
      RCLCPP_INFO(this->get_logger(), "Added %zu spherical obstacles to the scene", 
                  spherical_obstacles_.size());
    } else {
      RCLCPP_INFO(this->get_logger(), "No valid spherical obstacles to add");
    }

    has_obstacles_ = true;
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
    if (!has_obstacles_) {
      RCLCPP_INFO(this->get_logger(), "No obstacles received yet.");
      return;
    }

    // Skip if already planning
    if (is_planning_) {
      RCLCPP_INFO(this->get_logger(), "Already planning, skipping this cycle");
      return;
    }

    is_planning_ = true;

    // Set planning parameters
    move_group_interface_->setPlanningTime(5.0);  // Increase planning time
    move_group_interface_->setNumPlanningAttempts(5);
    move_group_interface_->setPlannerId("RRTConnect");  // Try specific planner
    move_group_interface_->allowReplanning(true);
    
    // Convert Float32MultiArray to Pose with fixed orientation
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = latest_target_position_.data[0];
    target_pose.position.y = latest_target_position_.data[1];
    target_pose.position.z = latest_target_position_.data[2];
    
    // Set fixed orientation as specified
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group_interface_->setPoseTarget(target_pose);

    // RCLCPP_INFO(this->get_logger(), "Planning to position: [%.3f, %.3f, %.3f] with fixed orientation: [1.0, 0.0, 0.0, 0.0]",
    //             target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // Set up orientation constraints
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.link_name = "tool0";
    orientation_constraint.header.frame_id = move_group_interface_->getPlanningFrame();
    orientation_constraint.orientation.x = 1.0;
    orientation_constraint.orientation.y = 0.0;
    orientation_constraint.orientation.z = 0.0;
    orientation_constraint.orientation.w = 0.0;
    
    // Set tolerance for orientation constraint
    orientation_constraint.absolute_x_axis_tolerance = 0.3;
    orientation_constraint.absolute_y_axis_tolerance = 0.3;
    orientation_constraint.absolute_z_axis_tolerance = 0.3;
    orientation_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(orientation_constraint);
    move_group_interface_->setPathConstraints(constraints);

    // Plan the trajectory with constraints
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto planning_result = move_group_interface_->plan(plan);
    
    
    // RCLCPP_INFO(this->get_logger(), "Planning result: %d", planning_result.val);
    bool has_trajectory = !plan.trajectory_.joint_trajectory.points.empty();
    // RCLCPP_INFO(this->get_logger(), "Trajectory has %zu points", plan.trajectory_.joint_trajectory.points.size());

    path_publisher_->publish(plan.trajectory_);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints to /moveit_path", 
                  plan.trajectory_.joint_trajectory.points.size());
    
    // Publish if we have a valid trajectory
    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS || has_trajectory) {
      RCLCPP_INFO(this->get_logger(), "Planning successful or trajectory available!");
      
      // Publish the planned trajectory
      path_publisher_->publish(plan.trajectory_);
      // RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints to /moveit_path", 
      //             plan.trajectory_.joint_trajectory.points.size());
      
      // RCLCPP_INFO(this->get_logger(), "Plan ready for execution (execution disabled to avoid segfault)");
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed! Error code: %d", planning_result.val);
    }

    // Clear targets and constraints
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    
    // Reset planning flag
    is_planning_ = false;
  }

  struct SphericalObstacle {
    float center_x, center_y, center_z, radius;
  };

  std_msgs::msg::Float32MultiArray latest_target_position_;
  std::vector<SphericalObstacle> spherical_obstacles_;
  bool has_target_ = false;
  bool has_obstacles_ = false;
  bool is_planning_ = false;  // Add planning state flag
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr spherical_obstacles_sub_;
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
