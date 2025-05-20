import rclpy
import yaml
import os

from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy.wait_for_message
from sensor_msgs.msg import JointState
from collections import OrderedDict
from rclpy.node import Node



#TO DO: self.poses['moveToTarget']['ros__parameters']['targets'][pose_name] Line should have its values renamed to fit. 
# "MoveToTarget" should be the current used node name and "targets" and "pose name" should follow the structure of the yaml file.
'''
Save the current pose in the specified topic to a yaml file.
Created by Gabriel Dorneles on 2024-10-06.
Port to ROS2 by Vitor Anello on 2025-05-7.
Port to Use in Arm By Luiz Sparvoli on 2025-05-20.

'''


class OrderedDumper(yaml.SafeDumper):
    '''
    @brief Custom YAML dumper to handle OrderedDict.
    '''
    def represent_ordereddict(self, data):
        return self.represent_dict(data.items())
    
class OrderedLoader(yaml.SafeLoader):
    pass

def construct_ordered_dict(loader, node):
    '''
    @brief Custom YAML loader to handle OrderedDict.
    '''
    return OrderedDict(loader.construct_pairs(node))

OrderedLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_ordered_dict
)

class PoseWriter (Node):
    '''
    @class PoseWriter
    @brief A ROS 2 node that saves the current pose of a robot to a YAML file.
    This node subscribes to a specified pose topic and allows the user to save the current pose
    by entering a name. The saved poses are stored in a YAML file for later use.
    '''
    
    def __init__(self, node_name):
        '''
        @brief Constructor for the PoseWriter node.
        '''
        super().__init__(node_name=node_name)
    
        self.poses = {'moveToTarget': {'ros__parameters': {'targets': {}}}}

        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../.."))
        self.config_path = os.path.join(ws_dir,'src/fbot_arm/arm/poses')

        while True:
            self.yaml_file = input("Enter the name of the file to save the poses (e.g., 'pose_inspection.yaml'): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                self.get_logger().warning("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = self.config_path + '/' + self.yaml_file
        self.declare_parameter('armPose', '/wx200/joint_states')
        self.pose_topic = self.get_parameter('armPose').get_parameter_value().string_value

        _, message = rclpy.wait_for_message.wait_for_message(msg_type= JointState, node=self, topic='/wx200/joint_states', time_to_wait=10)
        joints = message.name
        values = message.position

        self.joint_values = tuple(zip(joints,values))


        self.get_logger().info(f"Received msg: {self.joint_values}")
        
        self.current_pose = self.joint_values
        
    def save_pose(self) -> None:
        while rclpy.ok():
            arm_name = input("Move the robot to the desired pose and enter its name (e.g., 'garbage_1', 'exit'): ")

            if not arm_name: 
                self.get_logger().warning("No name provided, skipping pose.")
                continue
            
            if not hasattr(self, 'current_pose'):
                self.get_logger().warning("No pose received from topic yet.")
                continue

            self.poses['moveToTarget']['ros__parameters']['targets'][arm_name] = OrderedDict(self.joint_values)

            self.get_logger().info(f"Pose '{arm_name}' saved.")
            while True:
                save_now = input("Do you want to add more poses? (y/n): ").lower()
                if save_now == 'n':
                    self.write_to_yaml()
                    self.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")
                    return

                elif save_now == 'y':
                    break
                else:
                    self.get_logger().warning("Invalid input. Please enter 'y' or 'n'.")

    def write_to_yaml(self):
        OrderedDumper.add_representer(OrderedDict, OrderedDumper.represent_ordereddict)

        if os.path.exists(self.yaml_path):
            self.get_logger().info(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")

            with open(self.yaml_path, 'r') as yaml_file:
                try:
                    existing_data = yaml.load(yaml_file, Loader=OrderedLoader) or OrderedDict()
                except yaml.YAMLError as e:
                    self.get_logger().error(f"Error reading {self.yaml_file}: {e}")
                    existing_data = OrderedDict()
        else:
            self.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")
            existing_data = OrderedDict()

        if 'moveToTarget' not in existing_data:
            existing_data['moveToTarget']= OrderedDict({'ros__parameters':{'targets': OrderedDict()}})

        existing_data['moveToTarget']['ros__parameters']['targets'].update(self.poses['moveToTarget']['ros__parameters']['targets'])
        self.get_logger().info(self.yaml_path)
        with open(self.yaml_path, 'w') as yaml_file:
             yaml.dump(existing_data, yaml_file, default_flow_style=False, Dumper=OrderedDumper)
             
        return


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        saver = PoseWriter(node_name = 'arm')
        saver.save_pose()
        saver.destroy_node()
        rclpy.try_shutdown()
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()