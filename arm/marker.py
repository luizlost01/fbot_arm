import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import time
class Block(Node):
    def __init__(self):
        super().__init__('block')
        self.marker_pub = self.create_publisher(Marker, 'ballon', 10)

        # self.marker_sub = self.create_subscription(Marker, )

        self.marker = Marker()
        self.marker.header.frame_id = 'wx200/base_link'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.POINTS
        self.marker.id = 1
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = 1.0
        self.marker.color.g = 2.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    
    def publish_marker(self):

        #Define a variavel point usando o Points
        point = Point()
        #Define o ponto que o marker vai aparece no mundo
        point.x, point.y, point.z = 0.0, 0.0, 0.0

        #Insere 
        self.marker.points.append(point)


        self.marker_pub.publish(self.marker)
        self.marker.lifetime.sec = 2


def main(args=None):
    rclpy.init(args=args)
    balloon = Block()
    balloon.publish_marker()
    balloon.destroy_node()  
    rclpy.shutdown()
if __name__ == '__main__':
    main()