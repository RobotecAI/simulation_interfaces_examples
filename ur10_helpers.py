"""Helper functions for controlling UR10 robot arm across different simulation backends."""

import time
from rclpy.node import Node


def move_ur10_joints(node: Node, loop_iteration: int, sim_backend: str):
    """Publish joint positions for UR10 for the given iteration.
    
    Args:
        node: ROS2 node
        loop_iteration: Iteration number (0-2) to determine joint positions
        sim_backend: Simulation backend ("isaacsim", "o3de", or "gazebo")
    """
    joint_positions_by_iteration = [
        [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
        [0.7854, -1.0472, 0.7854, -0.7854, -1.5708, 0.5236],
        [-0.5236, -2.0944, 2.0944, -2.0944, -1.5708, -0.7854],
    ]
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    positions = joint_positions_by_iteration[loop_iteration]
    
    if sim_backend == "isaacsim":
        move_ur10_joints_topic(node, joint_names, positions)
    elif sim_backend == "o3de":
        move_ur10_joints_action(node, joint_names, positions)
    elif sim_backend == "gazebo":
        move_ur10_joint_array_topic(node, positions)
    else:
        print(f"Unknown simulation backend: {sim_backend}")


def move_ur10_joints_topic(node: Node, joint_names: list, positions: list):
    """Move UR10 joints using JointState topic publisher.
    
    Used for Isaac Sim backend.
    
    Args:
        node: ROS2 node
        joint_names: List of joint names
        positions: List of joint positions (radians)
    """
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    
    ur10_joint_pub = node.create_publisher(JointState, "/joint_commands", 10)
    joint_cmd = JointState()
    joint_cmd.header = Header()
    joint_cmd.header.stamp = node.get_clock().now().to_msg()
    joint_cmd.name = joint_names
    joint_cmd.position = positions
    
    for _ in range(10):
        ur10_joint_pub.publish(joint_cmd)
        time.sleep(0.2)


def move_ur10_joint_array_topic(node: Node, positions: list):
    """Move UR10 joints using Float64MultiArray publisher.
    
    Used for Gazebo backend.
    
    Args:
        node: ROS2 node
        positions: List of joint positions (radians)
    """
    from std_msgs.msg import Float64MultiArray
    
    ur10_joint_pub = node.create_publisher(Float64MultiArray, "/joint_commands", 10)
    joint_cmd = Float64MultiArray()
    joint_cmd.data = positions
    
    for _ in range(10):
        ur10_joint_pub.publish(joint_cmd)
        time.sleep(0.2)


def move_ur10_joints_action(node: Node, joint_names: list, positions: list):
    """Move UR10 joints using FollowJointTrajectory action.
    
    Used for O3DE backend.
    
    Args:
        node: ROS2 node
        joint_names: List of joint names
        positions: List of joint positions (radians)
    """
    import rclpy
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    
    action_client = ActionClient(
        node, 
        FollowJointTrajectory, 
        "/joint_trajectory_controller/follow_joint_trajectory"
    )
    
    while not action_client.wait_for_server(timeout_sec=1.0):
        print("Waiting for action server...")
    
    # Build trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * len(joint_names)
    point.effort = [0.0] * len(joint_names)
    point.time_from_start = Duration(sec=1, nanosec=0)
    trajectory.points.append(point)
    
    # Build goal with tolerances
    goal_tolerance = [JointTolerance(position=0.01) for _ in range(2)]
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = trajectory
    goal_msg.goal_tolerance = goal_tolerance

    # Send goal and wait for result
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if not goal_handle.accepted:
        print("Goal rejected")
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
