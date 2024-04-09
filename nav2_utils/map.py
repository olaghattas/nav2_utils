import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np


class PoseinMap(Node):

    def __init__(self):
        super().__init__('pose_in_map')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription_map = self.create_subscription(OccupancyGrid, "map", self.map_callback, qos_profile)
        self.occ_grid = None
        self.origin = None
        self.map = None
        self.map_matrix = None
        # self.vel_pub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)

    def map_callback(self, msg):
        print("calla")
        if msg:
            self.map = msg

            height = msg.info.height
            width = msg.info.width

            self.map_matrix = np.zeros((width, height))
            index = 0
            for i in range(height):
                for j in range(width):
                    self.map_matrix[j, i] = self.map.data[index]
                    index += 1
            # print(self.map_matrix)
            # pts = [(-0.14252321422100067, -0.60), (0.9456138610839844, -0.06074492260813713),
            #        (1.0387203693389893, 2.751633644104004)]
            pts = [(-4.1449503898620605, -1.9035770893096924), (-1.8553942441940308, -4.012697696685791),
                   (-2.2164196968078613, 0.3310917615890503)]
            for pt in pts:
                print("pt", pt)
                map_x, map_y = self.world_to_map(pt[0], pt[1], self.map)
                print(map_x, map_y)
                print(self.map_matrix[map_x, map_y])
                # print(self.map_matrix[map_y, map_x])

    def map_to_world(self, map_x, map_y, map):
        pos_x = map.info.origin.position.x + (map_x + 0.5) * map.info.resolution
        pos_y = map.info.origin.position.y + (map_y + 0.5) * map.info.resolution
        return pos_x, pos_y

    def world_to_map(self, pos_x, pos_y, map):
        # resolution : Resolution of the map, meters / pixel
        map_x = int(((pos_x - map.info.origin.position.x) / map.info.resolution) - 0.5)
        map_y = int(((pos_y - map.info.origin.position.y) / map.info.resolution) - 0.5)
        return map_x, map_y

    def cmd_callback(self, msg):
        print("calla")
        if msg:
            print("Width: ", msg)


def main(args=None):
    rclpy.init(args=args)
    pose_in_map = PoseinMap()

    while rclpy.ok():
        rclpy.spin_once(pose_in_map, timeout_sec=1.0)
    rclpy.shutdown()
