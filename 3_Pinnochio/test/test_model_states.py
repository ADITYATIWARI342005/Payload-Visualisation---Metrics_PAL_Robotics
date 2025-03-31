#!/usr/bin/env python3  
"""  
A simple script to publish joint states for testing the robot_model_node.  
"""  

import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import JointState  
from std_msgs.msg import Header  
import math  

class JointStatePublisher(Node):  
    """  
    Node to publish joint states for testing the robot model with Pinocchio.  
    """  
    def __init__(self):  
        # Initialize the Node  
        super().__init__('joint_state_publisher_test')  
        
        # Create a publisher on the 'joint_states' topic  
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)  
        
        # Create a timer to call the timer_callback function periodically  
        self.timer = self.create_timer(0.1, self.timer_callback)  
        
        # Initialize time counter for generating joint positions  
        self.t = 0.0  
        
        # Joint names should match the names defined in the URDF  
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  
        
        # Log a message indicating the node has started  
        self.get_logger().info('Joint State Publisher Test started')  
        
    def timer_callback(self):  
        """  
        Callback function that publishes joint states at regular intervals.  
        """  
        # Create a JointState message  
        joint_state = JointState()  
        joint_state.header = Header()  
        joint_state.header.stamp = self.get_clock().now().to_msg()  # Set the current time  
        joint_state.name = self.joint_names  # Set the names of the joints  
        
        # Generate joint positions using a sinusoidal motion pattern  
        joint_state.position = [  
            0.5 * math.sin(0.5 * self.t),               # Position for joint1  
            0.3 * math.sin(0.7 * self.t + 0.5),         # Position for joint2  
            0.4 * math.sin(0.3 * self.t + 1.0),         # Position for joint3  
            0.2 * math.sin(0.9 * self.t + 1.5),         # Position for joint4  
            0.6 * math.sin(0.4 * self.t + 2.0),         # Position for joint5  
            0.3 * math.sin(0.6 * self.t + 2.5)          # Position for joint6  
        ]  
        
        # Publish the joint_states message  
        self.publisher.publish(joint_state)  
        
        # Increment the time counter for the next callback  
        self.t += 0.1  
        
def main(args=None):  
    # Initialize the ROS client library  
    rclpy.init(args=args)  
    
    # Create an instance of the JointStatePublisher  
    joint_publisher = JointStatePublisher()  
    
    try:  
        # Spin the node to keep it alive and process callbacks  
        rclpy.spin(joint_publisher)  
    except KeyboardInterrupt:  
        pass  # Exit gracefully on keyboard interrupt  
    finally:  
        # Clean up the node and shut down the ROS client library  
        joint_publisher.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':  
    main()  