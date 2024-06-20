import rclpy #py lib on ros
from rclpy.node import Node # to use NODE class
from std_msgs.msg import String # i know it from [ros2 topic info /(topic_name)]
from http.server import BaseHTTPRequestHandler, HTTPServer  # HTTP server libraries
import json  # JSON library



# Define a class for the ROS 2 minimal subscriber node
class MinimalSubscriber(Node): # to have all functionalities of ros2 "Node class"

    def __init__(self):
        super().__init__('minimal_subscriber') #Node name which will be used
        # Now you initialized the node

        #create subcriber(msg_type, topic_name , callback(what will happen when you subscribe from the topic), Queue_size[buffer])
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.callback_server,
            10)
        
        self.subscription  # prevent unused variable warning

#2lly fo2 da 2l 3aady -----

        # This publisher will be used to publish messages in response to HTTP requests
        self.publisher_ = self.create_publisher(String, 'topic', 10)  # Create a publisher called "self.publisher_"

    # This function is called when the HTTP server receives data
    def callback_server(self, data):
        msg = String() # Create a new String message
        msg.data = data # Set the message data
        print(msg.data) # Print the data to the terminal
        self.publisher_.publish(msg) # the publisher "self.publisher_" Publish the message on the 'topic' topic


# Define a request handler class for the HTTP server
class RequestHandler(BaseHTTPRequestHandler):


    # get bby3rd
    # post ba5d mn user

    # Handle GET requests (not allowed in this case)
    def do_GET(self):
        self.send_response(405)  # Method Not Allowed
        self.end_headers()


    # Handle POST requests
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])  # Get the length of the request body
        body = self.rfile.read(content_length)  # Read the request body
        data = body.decode('utf-8')  # Decode the body to a string
        list_data = data.split("&")  # Split the data by '&'
        dict_data = {_[0]: _[1] for _ in [x.split("=") for x in list_data]}  # Parse the data into a dictionary
        print(dict_data)  # Print the dictionary (user input)

        
        if dict_data['func'] == 'submit_request':
            self.server.callback(data)  # Call the server callback with the data
            self.send_response(200)  # Send a 200 OK response
            self.end_headers()  #ensures the proper structure of HTTP responses by marking the end of the headers section. This allows the client to correctly interpret the response and prepares the server to send the response body if needed.
            self.wfile.write(f'{{"rid": 1}}'.encode('utf-8'))  # Write a response with rid = 1


        elif dict_data['func'] == 'path_request':
            self.server.callback(data)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"path": 100}}'.encode('utf-8')) # Write a response with path = 100



def main(args=None):  # Define the main function that will be executed (setup.py)
    rclpy.init(args=args) #ros2 communication 

    minimal_subscriber = MinimalSubscriber()

    
    server = HTTPServer(('localhost', 8080), RequestHandler) # Create and start the HTTP server
    server.callback = minimal_subscriber.callback_server  # Set the server callback to the node's callback_server method
    server.serve_forever()  # Start the server

    rclpy.spin(minimal_subscriber)  #work as while loop till you kill it [it enables all the callbacks of the node] -------Keep the node running, enabling all callbacks--------
    # this spin makes sure that this node is execute all its callbacks

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    # Clean up and shutdown
    minimal_subscriber.destroy_node() # this used for specific node [node_name.destroy_node()] ----Destroy the node explicitly------
    rclpy.shutdown() ##shutdown ros2 communication


if __name__ == '__main__':

    main() # Call the main function when the script is executed

