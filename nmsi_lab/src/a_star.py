#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
import math
from queue import PriorityQueue
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
        self.PathFind(start.pose.position.x,start.pose.position.y, end.pose.position.x, end.pose.position.y)

        calculated_path.poses.append(end)
        self.publisher.publish(calculated_path)

    def index_conversion (self, PosX, PosY):
        Ox = self.map_dilated.info.origin.position.x
        Oy = self.map_dilated.info.origin.position.y
        r = self.map_dilated.info.resolution

        ix = math.floor((PosX - Ox)/r)
        iy = math.floor((PosY - Oy)/r)

        #idx = iy * self.map_dilated.info.width + ix

        return ix, iy

    def meter_conversion(self, ix, iy):
        Ox = self.map_dilated.info.origin.position.x
        Oy = self.map_dilated.info.origin.position.y
        r = self.map_dilated.info.resolution

        x = Ox + (ix + 0.5) * r
        y = Oy + (iy + 0.5) * r

        return x, y

    def inside_map (self, Pos):

        Posx = Pos[0]
        Posy = Pos[1]
        
        Ox = self.map_dilated.info.origin.position.x
        X_limit = Ox + self.map_dilated.info.width

        Oy = self.map_dilated.info.origin.position.y
        Y_limit = Oy + self.map_dilated.info.height

        
        if(not (Ox <= Posx and Posx <= X_limit) or not (Oy <= Posy and Posy <= Y_limit)):
            print("Outside the Map!")
            return False
        else:
            print("Inside the Map!")
            return True

    def heuristic(self, current, target):
        r = self.map_dilated.info.resolution

        diff_x = (current[0] - target[0]) * r
        diff_y = (current[1] - target[1]) * r

        return math.sqrt(diff_x * diff_x + diff_y * diff_y)

    def new_neighbour4(self, Pos):
        x_cord = Pos[0]
        y_cord = Pos[1]

        neighbour1 = x_cord, y_cord + 1
        neighbour2 = x_cord + 1, y_cord
        neighbour3 = x_cord, y_cord - 1
        neighbour4 = x_cord - 1, y_cord

        #potencjalnie check inside map

        neighbours = [neighbour1, neighbour2, neighbour3, neighbour4]

        return neighbours
    
    def new_neighbour8(self, Pos):
        x_cord = Pos[0]
        y_cord = Pos[1] 

        is_straight = True #pierwszy neighbour jest Diagonal

        neighbours = []

        for i in range(-1, 2):
            for j in range(-1, 2):
                is_straight = not is_straight #flipowanie wartości
                if(i == 0 and j == 0):                    
                    continue
                neighbour = x_cord + j, y_cord + i, is_straight
                neighbours.append(neighbour)



        return neighbours


    def reconstruct_Path(self, visited_from, currentNode):
        path = [currentNode]
        while(currentNode in visited_from):
            currentNode = visited_from[currentNode]
            path.insert(0, currentNode) #convert_meter currentNode TO DO potencjalnie
        return path


    def PathFind(self, startPosX, startPosY, targetPosX, targetPosY):
        
        startNode = startPosX, startPosY
        targetNode = targetPosX, targetPosY
        currentNode = startNode

        neighbours = self.new_neighbour4(startNode)

        h_cost = self.heuristic
        g_cost = {}

        visited = {}

        visited_from = {}
        g_cost[startNode] = 0

        #g_cost.update(self.new_neighbour4(startNode))
        #handling sasiada poza mapą
        for cord in g_cost.keys():
            if(self.inside_map([cord])):
                continue
            else:
                g_cost.pop(cord)

        move_cost_straigth = self.heuristic(startNode, neighbours[1][:2])
        move_cost_diagonal = self.heuristic(startNode, neighbours[0][:2])

        Open = {}
        Open[startNode] = g_cost[startNode] + h_cost(startNode, targetNode)

        #vvvvv 4 sasiadow while loop
        """
        while(not Open):
            neighbours = self.new_neighbour4(currentNode)
            f_min = 100000 #wartosc f z jakiegokolwiek wierzchołka w open 

            for cord in Open.keys():
                if(Open[cord] < f_min):
                    f_min = Open[cord]
                    currentNode = cord

            Open.pop(currentNode)
            if(currentNode == targetNode):
                return self.reconstruct_Path(visited_from, currentNode) #TO DO funkcja rekonstrukcji drogi przebytej

            visited.add(currentNode)

            

            for neighbour in neighbours:
                if(neighbour in visited):
                    continue
                temp_g = g_cost[currentNode] + move_cost_straigth
                if(neighbour not in g_cost or temp_g < g_cost[neighbour]):
                    visited_from[neighbour] = currentNode
                    g_cost[neighbour] = temp_g
                    Open[neighbour] = g_cost[neighbour] + h_cost(neighbour)
        """

        #while loop na 8 neighbourów
        g_cost.update(self.new_neighbour8(startNode))

        while(not Open):
            neighbours = self.new_neighbour8(currentNode)
            f_min = 100000 #wartosc f z jakiegokolwiek wierzchołka w open 

            for cord in Open.keys():
                if(Open[cord] < f_min):
                    f_min = Open[cord]
                    currentNode = cord

            Open.pop(currentNode)
            if(currentNode == targetNode):
                return self.reconstruct_Path(visited_from, currentNode) #TO DO funkcja rekonstrukcji drogi przebytej

            visited.add(currentNode)

            

            for neighbour in neighbours:
                neighbour_cord = neighbour[:2]
                is_straight = neighbour[2]
                if(neighbour[:2] in visited):
                    continue
                if(neighbour[2] == True):
                    move_cost = self.heuristic(startNode, neighbours[1][:2]) #straight
                else:
                    move_cost = self.heuristic(startNode, neighbours[0][:2]) #diagonal
                
                temp_g = g_cost[currentNode] + move_cost
                if(neighbour_cord not in g_cost or temp_g < g_cost[neighbour_cord]):
                    visited_from[neighbour_cord] = currentNode
                    g_cost[neighbour_cord] = temp_g
                    Open[neighbour_cord] = g_cost[neighbour_cord] + h_cost(neighbour_cord)


        return False 



            

            





    
    


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarCalculator()

    StartNode = [0, 0]
    TargetNode = [20, 20]

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
