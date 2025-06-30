#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

X_MIN = 0
Y_MIN = 0
X_MAX = 11
Y_MAX = 11

class moveNode(Node):
    def __init__(self):
        super().__init__("moveNode")
        self.state = 1
        self.randomNumber = random.uniform(0,5)
        #Creating publisher and subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.pose_sub = self.create_subscription(Pose,"/turtle1/pose",self.poseCallback,9)
        #Checking the state machine every 0.1 seconds
        self.timer = self.create_timer(0.1, self.stateMachine)
        self.get_logger().info("Node started")

    def sendMoveCommand(self,x,z):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.cmd_vel_pub.publish(msg)

    def poseCallback(self,msg:Pose):
        #self.get_logger().info(str(msg))
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def stateMachine(self):
        match self.state:
            #Turn for a random duration
            case 1:
                self.sendMoveCommand(0.0,1.0)
                self.randomNumber-=0.1
                if(self.randomNumber < 0):
                    self.state = 2
            #Start moving forward, stop turning
            case 2:
                self.sendMoveCommand(1.0,0.0)
                self.state = 3
            #Check boundaries
            case 3:
                if((X_MIN < self.x < X_MAX) and (Y_MIN < self.y < Y_MAX)):
                    self.state = 2
                else:
                    self.state = 4
                    self.oldTheta = self.theta
            #Out of bounds -> start turning
            case 4:
                self.get_logger().info("edge detected!")
                self.sendMoveCommand(0.0,-2.0)

                angle = self.theta - self.oldTheta
                angle = math.fmod(angle, 2 * math.pi)

                if angle > math.pi:
                    angle -= 2 * math.pi
                elif angle < -math.pi:
                    angle += 2 * math.pi
                
                angle = abs(angle)
                
                #If 90° have been reached, move with increased speed to "escape" the wall
                if(angle > math.pi/2):
                    self.sendMoveCommand(2.0,0.0)
                    self.state = 2

def main(args=None):
    rclpy.init(args=args)

    node = moveNode()    

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()