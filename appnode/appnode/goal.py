import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Define a class NavToPose which is a ROS 2 Node
class NavToPose(Node):

    def __init__(self):
        # Initialize the node with the name 'nav_to_pose'
        super().__init__('nav_to_pose')
        # Create an action client that will communicate with the 'navigate_to_pose' action server
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        # Create a new goal message
        goal_msg = NavigateToPose.Goal()
        # Set the pose for the goal
        goal_msg.pose = pose
        # Wait until the action server is available
        self._client.wait_for_server()
        # Send the goal to the action server asynchronously
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        # Add a callback for when the goal response is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the response from the action server
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        # Add a callback for when the result of the goal is received
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Handle the result of the goal
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        # Shutdown the ROS 2 communication
        rclpy.shutdown()


def target(location):
    # Create an instance of the NavToPose node
    node = NavToPose()
    print(location)
    
    # Define known poses as a dictionary and properly initialize PoseStamped objects
    known_poses = {
        'location1': PoseStamped(),
        'location2': PoseStamped(),
        'location3': PoseStamped()
    }

    # Setup example poses (these should be replaced with actual coordinates)
    known_poses['location1'].header.frame_id = 'map'
    known_poses['location1'].pose.position.x = 5.0
    known_poses['location1'].pose.position.y = 0.0
    known_poses['location1'].pose.orientation.w = 1.0

    known_poses['location2'].header.frame_id = 'map'
    known_poses['location2'].pose.position.x = 1.0
    known_poses['location2'].pose.position.y = 1.0
    known_poses['location2'].pose.orientation.w = 1.0

    known_poses['location3'].header.frame_id = 'map'
    known_poses['location3'].pose.position.x = -1.0
    known_poses['location3'].pose.position.y = -1.0
    known_poses['location3'].pose.orientation.w = 1.0


    # Print known poses to ensure they are correctly initialized
    print("Known poses initialized:")
    for key, pose in known_poses.items():
        print(f"{key}: {pose}")

    # Take input from the terminal for the target location
    target = str(location).strip()


    # Check if the target location is in the known poses dictionary
    if target in known_poses:
        # Send the goal to navigate to the target location
        node.send_goal(known_poses[target])
        # Spin the node to keep it alive and handle callbacks
        rclpy.spin(node)
    else:
        # Print an error message if the location is unknown
        print("Unknown location!")


def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    # Create an instance of the NavToPose node
    node = NavToPose()
    target("location1")

    
    
    

if __name__ == '__main__':
    main()
