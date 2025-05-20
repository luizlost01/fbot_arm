import rclpy
import rclpy.logging
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from visualization_msgs.msg import Marker
import random

from time import sleep
class MoveToTarget(Node):
    def __init__(self):
        super().__init__('movetotarget')
        #Publisher of the marker
        self.marker_pub = self.create_publisher(Marker, '/ballon', 10)

    #Function to move arm to the marker(Target)
    def movetotarget(self):

        bot = InterbotixManipulatorXS(
            robot_model='wx200',
            group_name='arm',
            gripper_name='gripper'
        )
        #Move arm to marker position
        bot.arm.set_ee_cartesian_trajectory(self.create_marker().x,self.create_marker().y, self.create_marker().z)
        bot.arm.go_to_sleep_pose()

        #Function to create a marker in the world
    def create_marker(self): 
        self.marker = Marker()
        self.marker.header.frame_id = 'wx200/base_link'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.SPHERE
        self.marker.id = 1
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 1.0
        self.marker.color.g = 2.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        #Generate a random position to the marker between 0.0 and 0.30
        self.marker.pose.position.x = random.uniform(0.0, 0.30)
        self.marker.pose.position.z = random.uniform(0.0, 0.30)
        
        self.marker_pub.publish(self.marker)

        marker_points = self.marker.pose.position
        return marker_points

def main(args=None):
    rclpy.init(args=args)
    move = MoveToTarget()
    move.create_marker()
    move.movetotarget()
    rclpy.shutdown()

if __name__ == "__main__":
    main()