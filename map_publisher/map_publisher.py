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
                ('map', 'no path'),
                ('map_topic', '/map'),
                ('rate', 1.0),
            ])


        self.map_publisher = self.create_publisher(OccupancyGrid, self.get_parameter('map_topic').get_parameter_value().string_value, 10)

        self.path = ''
        self.yaml_file = ''
        self.pgm_file = ''
        self.path_name_spliser() 

        #with open(self.get_parameter('path').get_parameter_value().string_value + self.get_parameter('map_name').get_parameter_value().string_value + '.yaml', 'r') as f:
        with open(self.path + self.yaml_file, 'r') as f:
            self.yaml = yaml.load(f, Loader=yaml.FullLoader)

        self.pgm_path()
        self.grid_map = OccupancyGrid()
        self.read_map()

        self.publish_map()

        rate = self.get_parameter('rate').get_parameter_value().double_value
        if rate  != 0.0:
            self.timer = self.create_timer(rate, self.publish_map)
        

    def publish_map(self):
        #self.get_logger().info('tick')
        self.grid_map.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.grid_map)

    def read_map(self):
        #self.get_logger().info('read')

        #with open(self.get_parameter('path').get_parameter_value().string_value + self.yaml['image'], 'rb') as pgmf:
        with open(self.pgm_file, 'rb') as pgmf:
        #with open(self.path + self.yaml['image'], 'rb') as pgmf:
            map_img = plt.imread(pgmf)
        
        free_thresh = self.yaml['free_thresh']
        occupied_thresh = self.yaml['occupied_thresh']
        col_len = 0
        for col in reversed(range(len(map_img))):
            col_len = col_len + 1
            row_len = 0
            for row in range(len(map_img[0])):
                row_len = row_len + 1
                if map_img[col][row] > 255 - 255.0 * free_thresh:
                    #self.grid_map.data.insert(0,0)
                    self.grid_map.data.append(0)
                elif map_img[col][row] < 255 - 255.0 * occupied_thresh:
                    #self.grid_map.data.insert(0,100)
                    self.grid_map.data.append(100)
                else:
                    #self.grid_map.data.insert(0,-1)
                    self.grid_map.data.append(-1)

        self.grid_map.info.resolution = self.yaml['resolution']
        self.grid_map.info.width =  row_len
        self.grid_map.info.height =  col_len
        self.grid_map.info.origin.position.x = self.yaml['origin'][0]
        self.grid_map.info.origin.position.y = self.yaml['origin'][1]
        self.grid_map.info.origin.position.z = self.yaml['origin'][2]
        self.grid_map.info.origin.orientation.x = 0.0
        self.grid_map.info.origin.orientation.y = 0.0
        self.grid_map.info.origin.orientation.z = 0.0 #-0.707
        self.grid_map.info.origin.orientation.w = 1.0 #0.707

        #self.grid_map.data[1] = 50


        self.grid_map.header.frame_id = 'map'

    def path_name_spliser(self):
        full_path = self.get_parameter('map').get_parameter_value().string_value
        spline_path = full_path.split('/')
        self.path = '/'.join(spline_path[0:-1]) + '/'
        self.yaml_file = spline_path[-1]

        self.get_logger().info('in path %s' % full_path)
        self.get_logger().info('path %s' % self.path)
        self.get_logger().info('yaml %s' % self.yaml_file)

    def pgm_path(self):
        pgm = self.yaml['image']

        if len(pgm.split('/')) == 1:
            self.pgm_file = self.path + pgm
        else:
            self.pgm_file = pgm

        


def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)

    map_publsiher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
