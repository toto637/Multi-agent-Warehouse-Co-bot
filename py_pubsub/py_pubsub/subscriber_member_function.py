import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, HTTPServer
import json


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'topic', 10)  # Create a publisher

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def callback_server(self, data):
        msg = String()
        msg.data = data
        print(msg.data)
        self.publisher_.publish(msg)  # Publish the message


class RequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        self.send_response(405)  # Method Not Allowed
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
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"rid": 1}}'.encode('utf-8'))

        elif dict_data['func'] == 'path_request':
            self.server.callback(data)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(f'{{"path": 100}}'.encode('utf-8'))



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    server = HTTPServer(('localhost', 8080), RequestHandler)
    server.callback = minimal_subscriber.callback_server
    server.serve_forever()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
