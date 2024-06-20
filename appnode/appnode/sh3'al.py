from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
import math
import time
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, HTTPServer
import json

position={
    'location1':(5.0,0.0,1.0),
    'location2':(-5.0,0.0,1.0),
    'location3':(4.0,2.0,1.0),
}

rass={
    
}

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
    known_poses['location1'].pose.position.x = position['location1'][0]
    known_poses['location1'].pose.position.y = position['location1'][1]
    known_poses['location1'].pose.orientation.w = position['location1'][2]

    known_poses['location2'].header.frame_id = 'map'
    known_poses['location2'].pose.position.x = position['location2'][0]
    known_poses['location2'].pose.position.y = position['location2'][1]
    known_poses['location2'].pose.orientation.w = position['location2'][2]

    known_poses['location3'].header.frame_id = 'map'
    known_poses['location3'].pose.position.x = position['location3'][0]
    known_poses['location3'].pose.position.y = position['location3'][1]
    known_poses['location3'].pose.orientation.w = position['location3'][2]


    # Print known poses to ensure they are correctly initialized
    print("Known poses initialized:")
    for key, pose in known_poses.items():
        print(f"{key}: {pose}")

    # Take input from the terminal for the target location
    target = str(location).strip()

    distance1=path_distance('rid_1',target)
    print(distance1)
    #distance2=path_distance('rid_2',target)
    distance2=100000000000

    # Check if the target location is in the known poses dictionary
    if target in known_poses:
        # Send the goal to navigate to the target location
        node.send_goal(known_poses[target])
        # Spin the node to keep it alive and handle callbacks
        rclpy.spin(node)
    else:
        # Print an error message if the location is unknown
        print("Unknown location!")

    return 1 if distance1<distance2 else 2


def path_distance(rid,location):


    nav = BasicNavigator()
    distance_to_goal=-1
    try:
        # Get initial pose (simulated, assumed to be from /odom or /amcl_pose)
        initial_pose = get_initial_pose()
        print (initial_pose)

        # Get goal pose from user input
        goal_pose = get_goal_pose_from_input(location)
        print (goal_pose)

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
        return distance_to_goal

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
            ID=target(dict_data['location'])
            print(ID)
            rass[f'rid_{ID}']=dict_data['location']
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"rid": ID}}'.encode('utf-8'))

        
        elif dict_data['func'] == 'path_request':
            self.server.callback(data)
            self.send_response(200)
            self.end_headers()
            distance=path_distance(dict_data['rid'],rass[f'rid_{dict_data["rid"]}'])
            print(distance)
            self.wfile.write(f'{{"path": distance}}'.encode('utf-8')) # Write a response with path = 100


#-------------

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

def get_goal_pose_from_input(location):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'  # Replace with actual frame ID
    goal_pose.pose.position.x = position[location][0]
    goal_pose.pose.position.y = position[location][1]
    goal_pose.pose.orientation.w = position[location][2]
    return goal_pose



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
