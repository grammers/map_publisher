import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt
import yaml

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        self.get_logger().info('starting')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('path', 'path'),
                ('map_name', 'map')
            ])

        with open(self.get_parameter('path').get_parameter_value().string_value + self.get_parameter('map_name').get_parameter_value().string_value + '.yaml', 'r') as f:
            self.yaml = yaml.load(f, Loader=yaml.FullLoader)


        self.timer = self.create_timer(1.0, self.publish_map)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.grid_map = OccupancyGrid()

        self.read_map()

    def publish_map(self):
        self.get_logger().info('tick')
        self.grid_map.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.grid_map)

    def read_map(self):
        self.get_logger().info('read')

        with open(self.get_parameter('path').get_parameter_value().string_value + self.yaml['image'], 'rb') as pgmf:
            map_img = plt.imread(pgmf)
        
        free_thresh = self.yaml['free_thresh']
        occupied_thresh = self.yaml['occupied_thresh']
        col_len = 0
        for row in map_img:
            col_len = col_len + 1
            row_len = 0
            for nr in row:
                row_len = row_len + 1
                val = nr / 256.0
                if nr > 255 - 255.0 * free_thresh:
                    self.grid_map.data.append(0)
                elif nr < 255 - 255.0 * occupied_thresh:
                    self.grid_map.data.append(100)
                else:
                    self.grid_map.data.append(-1)

        self.grid_map.info.resolution = self.yaml['resolution']
        self.grid_map.info.width = row_len
        self.grid_map.info.height = col_len
        self.grid_map.info.origin.position.x = self.yaml['origin'][0]
        self.grid_map.info.origin.position.y = self.yaml['origin'][1]
        self.grid_map.info.origin.position.z = self.yaml['origin'][2]
        self.grid_map.info.origin.orientation.x = 0.0
        self.grid_map.info.origin.orientation.y = 0.0
        self.grid_map.info.origin.orientation.z = 0.0
        self.grid_map.info.origin.orientation.w = 1.0


        self.grid_map.header.frame_id = 'map'


        


def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)

    map_publsiher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
