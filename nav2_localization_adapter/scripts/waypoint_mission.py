#!/usr/bin/env python3
"""
多点导航脚本
通过Nav2 NavigateThroughPoses/FollowWaypoints action发送航点序列
从config/waypoints.yaml配置文件读取航点和导航模式

Usage:
  # 首先启动导航:
  ros2 launch nav2_localization_adapter waypoint_navigation.launch.py
  
  # 然后运行此脚本:
  ros2 run nav2_localization_adapter waypoint_mission.py

配置文件 config/waypoints.yaml 中可设置:
  - mission_config.loop: 是否循环执行 (true/false)
  - mission_config.mode: 导航模式 ("pass_through" 或 "stop_at_waypoint")
  - waypoints: 航点列表
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import math
import yaml
import os


class WaypointMission(Node):
    """多点导航节点"""
    
    def __init__(self):
        super().__init__('waypoint_mission')
        
        # 加载配置
        self.config = self._load_config()
        
        # 从配置中读取模式设置
        mission_config = self.config.get('mission_config', {})
        self.loop_mode = mission_config.get('loop', False)
        self.mode = mission_config.get('mode', 'pass_through')
        
        # 创建FollowWaypoints action client
        self._waypoint_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints'
        )
        
        # NavigateThroughPoses (pass through模式)
        self._nav_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )
        
        # 从配置文件加载航点
        self.waypoints = self._load_waypoints_from_config()
        
        self.get_logger().info(f'Waypoint Mission initialized with {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Loop mode: {self.loop_mode}')
    
    def _load_config(self) -> dict:
        """加载YAML配置文件"""
        # 使用功能包中的配置文件
        try:
            pkg_share = get_package_share_directory('nav2_localization_adapter')
            config_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        except Exception:
            # 开发模式下的备用路径
            config_file = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'waypoints.yaml'
            )
        
        self.get_logger().info(f'Loading config from: {config_file}')
        
        if not os.path.exists(config_file):
            self.get_logger().error(f'Config file not found: {config_file}')
            return {}
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                return config if config else {}
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            return {}
    
    def _load_waypoints_from_config(self) -> list:
        """从配置文件加载航点"""
        waypoints = []
        waypoints_config = self.config.get('waypoints', [])
        
        if not waypoints_config:
            self.get_logger().warn('No waypoints found in config, using default waypoints')
            # 默认航点
            waypoints_config = [
                {'x': 11.15, 'y': 11.81, 'yaw': 0.0},
                {'x': 1.23, 'y': 17.36, 'yaw': -1.57},
            ]
        
        for i, wp in enumerate(waypoints_config):
            x = wp.get('x', 0.0)
            y = wp.get('y', 0.0)
            yaw = wp.get('yaw', 0.0)
            
            pose = self._create_pose_stamped(x, y, yaw)
            waypoints.append(pose)
            self.get_logger().info(f'Waypoint {i+1}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
        return waypoints
    
    def _create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将yaw角转换为四元数
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def run(self):
        """根据配置的模式运行导航任务"""
        if self.mode == 'stop_at_waypoint':
            self.get_logger().info('Using FollowWaypoints (stop at each waypoint)')
            return self.send_waypoints()
        else:
            self.get_logger().info('Using NavigateThroughPoses (pass-through mode)')
            return self.send_navigate_through_poses()
    
    def send_waypoints(self):
        """发送航点到FollowWaypoints action server"""
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        
        if not self._waypoint_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowWaypoints action server not available!')
            return False
        
        self.get_logger().info('FollowWaypoints action server connected!')
        
        # 更新时间戳
        for pose in self.waypoints:
            pose.header.stamp = self.get_clock().now().to_msg()
        
        # 创建goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints
        
        self.get_logger().info(f'Sending {len(self.waypoints)} waypoints for navigation...')
        
        # 发送goal
        send_goal_future = self._waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self._waypoint_feedback_callback
        )
        send_goal_future.add_done_callback(self._waypoint_goal_response_callback)
        
        return True
    
    def send_navigate_through_poses(self):
        """
        使用NavigateThroughPoses发送航点 (pass-through模式)
        机器人会经过所有点但不会在每个点停留
        """
        self.get_logger().info('Waiting for NavigateThroughPoses action server...')
        
        if not self._nav_through_poses_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateThroughPoses action server not available!')
            return False
        
        self.get_logger().info('NavigateThroughPoses action server connected!')
        
        # 更新时间戳
        for pose in self.waypoints:
            pose.header.stamp = self.get_clock().now().to_msg()
        
        # 创建goal
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        
        self.get_logger().info(f'Sending {len(self.waypoints)} poses for pass-through navigation...')
        
        # 发送goal
        send_goal_future = self._nav_through_poses_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_through_poses_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_through_poses_goal_response_callback)
        
        return True
    
    def _waypoint_feedback_callback(self, feedback_msg):
        """处理FollowWaypoints的反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current waypoint: {feedback.current_waypoint + 1}/{len(self.waypoints)}')
    
    def _waypoint_goal_response_callback(self, future):
        """处理FollowWaypoints goal响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected!')
            return
        
        self.get_logger().info('Waypoint goal accepted, navigating...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._waypoint_result_callback)
    
    def _waypoint_result_callback(self, future):
        """处理FollowWaypoints结果"""
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        
        if len(missed_waypoints) == 0:
            self.get_logger().info('All waypoints reached successfully!')
        else:
            self.get_logger().warn(f'Missed {len(missed_waypoints)} waypoints: {missed_waypoints}')
        
        # 如果是循环模式，重新发送航点
        if self.loop_mode:
            self.get_logger().info('Loop mode: Restarting waypoint mission...')
            self.send_waypoints()
        else:
            self.get_logger().info('Waypoint mission completed!')
    
    def _nav_through_poses_feedback_callback(self, feedback_msg):
        """处理NavigateThroughPoses的反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Remaining poses: {feedback.number_of_poses_remaining}, '
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )
    
    def _nav_through_poses_goal_response_callback(self, future):
        """处理NavigateThroughPoses goal响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('NavigateThroughPoses goal rejected!')
            return
        
        self.get_logger().info('NavigateThroughPoses goal accepted, navigating...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_through_poses_result_callback)
    
    def _nav_through_poses_result_callback(self, future):
        """处理NavigateThroughPoses结果"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Pass-through navigation completed successfully!')
        else:
            self.get_logger().warn(f'Navigation finished with status: {result.status}')
        
        # 如果是循环模式，重新发送
        if self.loop_mode:
            self.get_logger().info('Loop mode: Restarting pass-through navigation...')
            self.send_navigate_through_poses()
        else:
            self.get_logger().info('Pass-through mission completed!')


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointMission()
    
    try:
        if node.run():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission cancelled by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
