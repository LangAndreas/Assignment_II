#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class moveNode(Node):
    def __init__(self):
        super().__init__("moveNode")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.create_timer(0.5, self.sendMoveCommand)
        self.get_logger().info("Node started")

    def sendMoveCommand(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Move executed")


def main(args=None):
    rclpy.init(args=args)

    node = moveNode()
    rclpy.spin(node)

    rclpy.shutdown()






if __name__ == '__main__':
    main()