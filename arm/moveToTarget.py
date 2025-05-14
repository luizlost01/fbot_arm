import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from visualization_msgs.msg import Marker
from arm.marker import Point

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('movetotarget')
        self.point = Point()

    def movetotarget(self):
        bot = InterbotixManipulatorXS(
            robot_model='wx200',
            group_name='arm',
            gripper_name='gripper'
        )
        bot.arm.set_ee_cartesian_trajectory(self.point.x)
        print(self.point.x)


    
def main(args=None):
    rclpy.init(args=args)
    move = MoveToTarget()
    move.movetotarget()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
