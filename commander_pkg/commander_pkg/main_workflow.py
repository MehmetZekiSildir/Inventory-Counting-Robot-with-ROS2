#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.clock import Clock
from irobot_create_msgs.action import Undock, Dock
from rclpy.action import ActionClient
from rclpy.node import Node

class RobotNavigator(Node):
    def __init__(self, x_positions, y_positions, z_orientations):
        super().__init__('robot_navigator')
        self.undock_client = ActionClient(self, Undock, 'undock')
        self.dock_client = ActionClient(self, Dock, 'dock')
        self.x_positions = x_positions
        self.y_positions = y_positions
        self.z_orientations = z_orientations
        self.navigator = BasicNavigator()
        self.goals = []
        self.current_waypoint = 0
        self.total_time = 0


    def create_goals(self):
        for i in range(len(self.x_positions)):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.x_positions[i]
            goal_pose.pose.position.y = self.y_positions[i]
            goal_pose.pose.orientation.z = self.z_orientations[i]
            goal_pose.pose.orientation.w = 0.99
            self.goals.append(goal_pose)

    def follow_waypoints(self):
        self.navigator.waitUntilNav2Active()
        self.navigator.followWaypoints(self.goals)
        self.start_time = self.navigator.get_clock().now()
        while not self.navigator.isTaskComplete():
            self.print_feedback()

        # Capture time for the last waypoint
        if self.current_waypoint == len(self.goals) - 1:
            end_time = self.navigator.get_clock().now()
            time_taken = (end_time - self.start_time).nanoseconds / 1e9  # Calculate time taken in seconds
            print(f'Time to reach waypoint {self.current_waypoint + 1}: {time_taken:.2f} seconds.')
            self.total_time += time_taken

        self.print_result()

    def print_feedback(self):
        feedback = self.navigator.getFeedback()
        if feedback and feedback.current_waypoint != self.current_waypoint:
            end_time = self.navigator.get_clock().now()
            time_taken = (end_time - self.start_time).nanoseconds / 1e9  # Calculate time taken in seconds
            print(f'Time to reach waypoint {self.current_waypoint + 1}: {time_taken:.2f} seconds.')
            self.start_time = end_time  # Reset start time for next waypoint
            self.total_time += time_taken
            self.current_waypoint = feedback.current_waypoint

    def print_result(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Toplam süre: {self.total_time:.2f} seconds')
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')



def main():
    rclpy.init()
    x_positions = [-2.20,-0.37,0.88,-1.22,0.18,0.40]
    y_positions = [-12.89,-14.29,-11.83,-10.83,-11.11,-9.70]
    z_orientations = [-0.97,-0.58,0.20,0.86,0.19,0.22]
    
    robot_navigator = RobotNavigator(x_positions, y_positions, z_orientations)
    
    robot_navigator.create_goals()
    robot_navigator.follow_waypoints()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
