from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
import math
import time


def calculate_distance(initial_pose, goal_pose):
    distance = math.sqrt((goal_pose.pose.position.x - initial_pose.pose.position.x) ** 2 +
                         (goal_pose.pose.position.y - initial_pose.pose.position.y) ** 2)
    return distance

def get_initial_pose():
    # Example: In a real scenario, subscribe to /odom or /amcl_pose topic to get current position
    # For simulation, assume initial position is (0, 0) in 'map' frame
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'  # Replace with actual frame ID
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    return initial_pose

def get_goal_pose_from_input():
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'  # Replace with actual frame ID
    goal_pose.pose.position.x = float(input("Enter goal X position: "))
    goal_pose.pose.position.y = float(input("Enter goal Y position: "))
    goal_pose.pose.orientation.w = 1.0
    return goal_pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    try:
        # Get initial pose (simulated, assumed to be from /odom or /amcl_pose)
        initial_pose = get_initial_pose()

        # Get goal pose from user input
        goal_pose = get_goal_pose_from_input()

        # Calculate the estimated path distance
        distance_to_goal = calculate_distance(initial_pose, goal_pose)
        print(f"Estimated path distance from current position to goal: {distance_to_goal} meters")

        # Set the initial pose
        nav.setInitialPose(initial_pose)

        # Set the goal pose
        nav.goToPose(goal_pose)

        # Monitor navigation progress
        while rclpy.ok():
            if nav.isTaskComplete():
                print("Robot has arrived at the goal pose!")
                break
            time.sleep(1.0)  # Adjust as needed for desired update frequency

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
