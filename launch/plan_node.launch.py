from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    # Process the URDF xacro file
    urdf_xacro_path = "/home/alebarte/ros2_marco/src/UR5e_ros2_helper/ur5e/ur5e_description/urdf/ur5e_merlin.urdf.xacro"
    urdf_result = subprocess.run(['xacro', urdf_xacro_path], capture_output=True, text=True)
    
    if urdf_result.returncode != 0:
        raise Exception(f"Failed to process URDF xacro: {urdf_result.stderr}")
    
    robot_description = urdf_result.stdout
    
    # Create custom SRDF with virtual joint and planning group
    custom_srdf_content = '''<?xml version="1.0"?>
<robot name="ur5e">
    <!--VIRTUAL JOINT: Purpose: this element defines a virtual joint between the robot's base and the environment -->
    <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base_link"/>
    
    <!--GROUP: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->
    <group name="ur5e_manipulator">
        <chain base_link="base_link" tip_link="tool0"/>
    </group>
    
    <!--GROUP STATES: Purpose: Define a named state for a particular group, in terms of joint values. This is useful to define states like 'folded arms'-->
    <group_state name="home" group="ur_manipulator">
        <joint name="shoulder_pan_joint" value="0"/>
        <joint name="shoulder_lift_joint" value="-1.5708"/>
        <joint name="elbow_joint" value="0"/>
        <joint name="wrist_1_joint" value="-1.5708"/>
        <joint name="wrist_2_joint" value="0"/>
        <joint name="wrist_3_joint" value="0"/>
    </group_state>
    
    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->
    <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
    <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
    <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
    <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
    <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
    <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
</robot>'''
    
    # Create kinematics configuration
    kinematics_content = '''ur_manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005'''
    
    # Define the node
    plan_node = Node(
        package="plan_program",
        executable="plan_node",
        name="plan_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": custom_srdf_content},
            {"robot_description_kinematics": kinematics_content},
        ]
    ),

    # Path converter node - converts joint trajectories to Cartesian paths
    Node(
        package='plan_program',
        executable='path_converter_node',
        name='path_converter_node',
        output='screen',
        parameters=[
            # Add any parameters here if needed
        ]
    ),

    return LaunchDescription([
        plan_node,
        path_converter_node
    ])