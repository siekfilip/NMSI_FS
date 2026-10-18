#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class AStarCalculator(Node):
    def __init__(self) -> None:
        super().__init__('a_star')

        self.fixed_frame = 'map'
        self.robot_frame = 'base_link'

        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.pose_callback, 10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dilated_map_subscription = self.create_subscription(
            OccupancyGrid, '/map_dilated', self.dilated_map_callback, map_qos)
        self.cost_map_subscription = self.create_subscription(
            OccupancyGrid, '/map_cost', self.cost_map_callback, map_qos)

        self.publisher = self.create_publisher(Path, '/path', 10)
        self.map_dilated: OccupancyGrid | None = None
        self.map_cost: OccupancyGrid | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, pose: PoseStamped) -> None:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'Could not transform {self.robot_frame} to '
                f'{self.fixed_frame}: {exc}')
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = trans.transform.translation.x
        start.pose.position.y = trans.transform.translation.y
        start.pose.position.z = trans.transform.translation.z
        start.pose.orientation = trans.transform.rotation
        self.a_star(start, pose)

    def dilated_map_callback(self, map_msg: OccupancyGrid) -> None:
        self.map_dilated = map_msg

    def cost_map_callback(self, map_msg: OccupancyGrid) -> None:
        self.map_cost = map_msg

    def a_star(self, start: PoseStamped, end: PoseStamped) -> None:
        calculated_path = Path()
        calculated_path.header.frame_id = 'map'
        calculated_path.header.stamp = self.get_clock().now().to_msg()
        calculated_path.poses.append(start)
        calculated_path.poses.append(end)
        self.publisher.publish(calculated_path)

    def index_conversion (self, PosX, PosY):
        Ox = self.map_dilated.info.origin.position.x
        Oy = self.map_dilated.info.origin.position.y
        r = self.map_dilated.info.resolution

        ix = math.floor((PosX - Ox)/r)
        iy = math.floor((PosY - Oy)/r)

        idx = iy * self.map_dilated.info.width

        return ix, iy

    def meter_conversion(self, ix, iy):
        Ox = self.map_dilated.info.origin.position.x
        Oy = self.map_dilated.info.origin.position.y
        r = self.map_dilated.info.resolution

        x = Ox + (ix + 0.5) * r
        y = Oy + (iy + 0.5) * r

        return x, y

    def inside_map (self, PosX, PosY):
        
        Ox = self.map_dilated.info.origin.position.x
        X_limit = Ox + self.map_dilated.info.width

        Oy = self.map_dilated.info.origin.position.y
        Y_limit = Oy + self.map_dilated.info.hight

        if((Ox > PosX > X_limit) or(Oy > PosY > Y_limit)):


    
    


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
