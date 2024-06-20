import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

class NavToPose(Node):

    def __init__(self):
        super().__init__('nav_to_pose')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
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

#----------------

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.callback_server,
            10)
        
        self.subscription

        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def callback_server(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)



class RequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        self.send_response(405)
        self.end_headers()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        data = body.decode('utf-8')
        list_data = data.split("&")
        dict_data = {_[0]: _[1] for _ in [x.split("=") for x in list_data]}
        print(dict_data) 

        if dict_data['func'] == 'submit_request':
            self.server.callback(data)
            target(dict_data['location']) # location from user
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"rid": 1}}'.encode('utf-8'))

        
   
        elif dict_data['func'] == 'path_request':
            self.server.callback(data)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"path": 100}}'.encode('utf-8')) # Write a response with path = 100


#-------------

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    nav_to_pose = NavToPose()
    


    server = HTTPServer(('localhost', 8080), RequestHandler)
    server.callback = minimal_subscriber.callback_server  # Set the server callback to the node's callback_server method
    server.serve_forever()


    rclpy.spin(minimal_subscriber)
     

    minimal_subscriber.destroy_node()
    nav_to_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
