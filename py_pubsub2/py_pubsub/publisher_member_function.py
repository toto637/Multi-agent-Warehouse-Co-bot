# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from urllib import request
import requests
import rclpy #py lib on ros
from rclpy.node import Node # to use NODE class

from std_msgs.msg import String # i know it from [ros2 topic info /(topic_name)]


def send_data_to_flask(data):
    url = 'http://192.168.1.4:8000/ros_data'
    requests.get(url + '/' + data)

    

class MinimalPublisher(Node): # to have all functionalities of ros2 "Node class"

    def __init__(self):
        super().__init__('minimal_publisher') #Node name which will be used
        # Now you initialized the node

        #self.  publisher_  (i could change it ex. self.pub_  )
        self.publisher_ = self.create_publisher(String, 'topic', 10) #create publisher(msg_type, topic_name ,Queue_size[buffer])
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # create timer which will call (timer_callback) every (timer_period)
        #then each (timer_period) it will callback 
        
        #so this counter just initialized with 0 at first time only
        self.i = 0

    def timer_callback(self): 
        msg = String() #msg obj
        msg.data = 'Hello World: %d' % self.i # msg.data (i know the msg name [data] from ### ros2 interface show (type) ###  )
        # /////////////// send_data_to_flask(msg.data)
        self.publisher_.publish(msg) # could be (self.pub_.publish(msg)) look at creating publisher line to understand 
        self.get_logger().info('Publishing: "%s"' % msg.data) # work as print
        self.i += 1 # but here it will +1 at each callback


def main(args=None):
    rclpy.init(args=args) #ros2 communication 
    node = MinimalPublisher()
    
    rclpy.spin(node) #work as while loop till you kill it [it enables all the callbacks of the node]
    # this spin makes sure that this node is execute all its callbacks
    
	
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    node.destroy_node() # this used for specific node [node_name.destroy_node()]
    rclpy.shutdown() #shutdown ros2 communication


if __name__ == '__main__':
    main()
