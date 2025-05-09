import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('movetotarget')
        self.movearm_pub = self.create_publisher(JointSingleCommand,'/wx200/commands/joint_single', 10)
    

    

    def movetotarget(self):
        msg = JointSingleCommand()
        msg.name = 'waist'
        msg.cmd = 2.0

        self.movearm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    move = MoveToTarget()
    move.movetotarget()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
