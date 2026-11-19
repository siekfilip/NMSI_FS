#!/usr/bin/env python3

import copy
import math
import random
import time
from enum import Enum, auto

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
import tf2_geometry_msgs  # Registers PoseStamped transforms with tf2.
from tf2_ros import Buffer, TransformException, TransformListener


class TrainingPhase(Enum):
    IDLE = auto()
    NEED_RESET = auto()
    RESETTING = auto()
    WAITING_FOR_LOCALIZATION = auto()
    SETTLING_AFTER_RESET = auto()
    NEED_GOAL = auto()
    WAITING_FOR_PLAN = auto()
    EXECUTING = auto()


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__('path_follower')

        self.epsilon = self.declare_parameter('epsilon', 0.2).value
        self.reset_settle_time = self.declare_parameter('reset_settle_time', 1.0).value
        self.path_settle_time = self.declare_parameter('path_settle_time', 1.0).value
        self.q_file = self.declare_parameter('q_file', 'q_table_path_only.txt').value
        random_seed = self.declare_parameter('random_seed', 42).value
        if not 0.0 <= self.epsilon <= 1.0:
            raise ValueError('epsilon must be between 0 and 1')
        if (not math.isfinite(self.reset_settle_time) or self.reset_settle_time < 0.0
                or not math.isfinite(self.path_settle_time) or self.path_settle_time < 0.0):
            raise ValueError('Settling times must be finite and nonnegative')

        self.fixed_frame = 'map'
        self.robot_frame = 'base_link'
        self.head_radius = 0.4
        # TODO: dobierz liczbę akcji i utrzymaj zgodność tablicy Q oraz formatu pliku.

        
        self.action_count = 5
        self.q_table: dict[tuple[int, ...], list[float]] = {}
        self.random = random.Random(random_seed)
        self.load_q_table(self.q_file)

        self.path: Path | None = None
        self.map: OccupancyGrid | None = None
        self.collided: bool | None = None
        self.training = False
        self.training_phase = TrainingPhase.IDLE
        self.localization_after = self.get_clock().now()
        self.reset_settling_started = self.get_clock().now()
        self.path_received_at = self.get_clock().now()
        self.phase_started = time.monotonic()
        self.training_goal = PoseStamped()
        self.previous_state: tuple[int, ...] | None = None
        self.previous_action = 0
        self.episode_steps = 0
        self.failed_plans = 0
        self.pose_id = 1

        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map_dilated', self.map_callback, latched_qos)
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.collision_subscription = self.create_subscription(
            Bool, '/collision', self.collision_callback, latched_qos)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.stop_service = self.create_service(
            Trigger, '/stop_path_follower', self.stop_callback)
        self.learn_service = self.create_service(
            Trigger, '/start_learning', self.learn_callback)
        self.follow_service = self.create_service(
            Trigger, '/start_following', self.follow_callback)
        self.reset_client = self.create_client(Trigger, '/random_reset')
        self.amcl_update_client = self.create_client(Empty, '/request_nomotion_update')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.follow_path)

    def path_callback(self, path: Path) -> None:
        if not path.poses:
            self.clear_path()
            return
        if len(path.poses) < 2:
            self.get_logger().error(
                f'Path is too short, it has only {len(path.poses)} points and should have at least 2')
            return
        goal = path.poses[-1].pose.position
        expected = self.training_goal.pose.position
        if self.training and (
                self.training_phase != TrainingPhase.WAITING_FOR_PLAN
                or math.hypot(goal.x - expected.x, goal.y - expected.y) > 0.001):
            return
        self.stop_moving()
        self.previous_state = None
        self.episode_steps = 0
        self.pose_id = 1
        self.path = path
        self.path_received_at = self.get_clock().now()
        self.get_logger().info(
            f'Path registered. Goal: ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f})')
        if self.training:
            self.failed_plans = 0
            self.training_phase = TrainingPhase.EXECUTING

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if (self.training and self.training_phase == TrainingPhase.WAITING_FOR_LOCALIZATION
                and Time.from_msg(msg.header.stamp,
                                  clock_type=self.get_clock().clock_type) > self.localization_after):
            self.reset_settling_started = self.get_clock().now()
            self.training_phase = TrainingPhase.SETTLING_AFTER_RESET

    def collision_callback(self, msg: Bool) -> None:
        self.collided = msg.data

    def stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.clear_path()
        response.success = True
        response.message = 'Path follower stopped'
        return response

    def learn_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.training_phase == TrainingPhase.RESETTING:
            response.message = 'A reset is still in progress'
        elif self.training:
            response.message = 'Already learning'
        else:
            self.clear_path()
            self.training = True
            self.training_phase = TrainingPhase.NEED_RESET
            self.get_logger().info(f'Learning started with epsilon = {self.epsilon:.3f}')
            response.success = True
            response.message = 'Learning started'
        return response

    def follow_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.training_phase == TrainingPhase.RESETTING:
            response.message = 'A reset is still in progress'
            return response
        if self.training:
            try:
                self.save_q_table(self.q_file)
            except (OSError, RuntimeError) as exc:
                response.message = str(exc)
                self.get_logger().error(str(exc))
                return response
        self.clear_path()
        self.training = False
        self.training_phase = TrainingPhase.IDLE
        response.success = True
        response.message = 'Following mode: waiting for a path'
        return response

    def follow_path(self) -> None:
        if self.training and self.path is None:
            self.prepare_training_episode()
            return
        if self.path is None or self.collided is None:
            return
        if (self.get_clock().now() - self.path_received_at).nanoseconds < self.path_settle_time * 1e9:
            return
        self.learning_loop()

    def prepare_training_episode(self) -> None:
        elapsed = time.monotonic() - self.phase_started
        if self.training_phase == TrainingPhase.NEED_RESET:
            if not self.reset_client.service_is_ready():
                self.get_logger().warning('Waiting for /random_reset', throttle_duration_sec=5.0)
                return
            if not self.amcl_update_client.service_is_ready():
                self.get_logger().warning(
                    'Waiting for /request_nomotion_update', throttle_duration_sec=5.0)
                return
            self.training_phase = TrainingPhase.RESETTING
            self.phase_started = time.monotonic()
            self.reset_client.call_async(Trigger.Request()).add_done_callback(self.reset_complete)
            return
        if self.training_phase == TrainingPhase.RESETTING:
            if elapsed > 20.0:
                self.get_logger().error(
                    'Waiting for random reset to complete; mode changes are blocked',
                    throttle_duration_sec=5.0)
            return
        if self.training_phase == TrainingPhase.WAITING_FOR_LOCALIZATION:
            if elapsed > 15.0:
                self.get_logger().error('AMCL did not publish a fresh pose after reset; learning stopped')
                self.training = False
                self.training_phase = TrainingPhase.IDLE
            return
        if self.training_phase == TrainingPhase.SETTLING_AFTER_RESET:
            if ((self.get_clock().now() - self.reset_settling_started).nanoseconds
                    < self.reset_settle_time * 1e9):
                return
            self.training_phase = TrainingPhase.NEED_GOAL
        if self.training_phase == TrainingPhase.WAITING_FOR_PLAN and elapsed > 10.0:
            self.failed_plans += 1
            if self.failed_plans >= 5:
                self.get_logger().warning(
                    f'No path after {self.failed_plans} attempts; resetting robot position')
                self.training_phase = TrainingPhase.NEED_RESET
                return
            self.get_logger().warning(
                f'No path to random goal ({self.failed_plans}/5); choosing another')
            self.training_phase = TrainingPhase.NEED_GOAL
        if self.training_phase != TrainingPhase.NEED_GOAL or self.collided is not False:
            return
        if self.map is None or not self.map.data:
            self.get_logger().warning('Waiting for /map_dilated', throttle_duration_sec=5.0)
            return
        try:
            robot = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for robot TF: {exc}', throttle_duration_sec=5.0)
            return
        info = self.map.info
        if (self.map.header.frame_id != self.fixed_frame or info.width == 0 or info.height == 0
                or info.resolution <= 0.0 or len(self.map.data) != info.width * info.height):
            self.get_logger().error('Invalid map for random goal; learning stopped')
            self.training = False
            self.training_phase = TrainingPhase.IDLE
            return
        q = info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        for _ in range(2000):
            index = self.random.randrange(len(self.map.data))
            if self.map.data[index] < 0 or self.map.data[index] >= 65:
                continue
            x = (index % info.width + 0.5) * info.resolution
            y = (index // info.width + 0.5) * info.resolution
            goal = PoseStamped()
            goal.header.frame_id = self.fixed_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = info.origin.position.x + math.cos(yaw) * x - math.sin(yaw) * y
            goal.pose.position.y = info.origin.position.y + math.sin(yaw) * x + math.cos(yaw) * y
            goal.pose.orientation.w = 1.0
            if math.hypot(goal.pose.position.x - robot.transform.translation.x,
                          goal.pose.position.y - robot.transform.translation.y) < 0.8:
                continue
            self.training_goal = goal
            self.goal_publisher.publish(goal)
            self.training_phase = TrainingPhase.WAITING_FOR_PLAN
            self.phase_started = time.monotonic()
            self.get_logger().info(f'Random goal: {goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}')
            return
        self.get_logger().error('Cannot find a random free goal; learning stopped')
        self.training = False
        self.training_phase = TrainingPhase.IDLE

    def reset_complete(self, future) -> None:
        exc = future.exception()
        if exc is not None:
            self.get_logger().error(f'Random reset request failed: {exc}')
            self.training = False
            self.training_phase = TrainingPhase.IDLE
            return
        response = future.result()
        if not response.success:
            self.get_logger().error(f'Random reset failed: {response.message}')
            self.training = False
            self.training_phase = TrainingPhase.IDLE
            return
        self.failed_plans = 0
        self.localization_after = self.get_clock().now()
        self.training_phase = TrainingPhase.WAITING_FOR_LOCALIZATION
        self.phase_started = time.monotonic()
        self.amcl_update_client.call_async(Empty.Request())
        self.get_logger().info('Waiting for AMCL after reset')

    def find_head_point(self) -> PoseStamped | None:
        if self.path is None or not self.path.poses:
            self.get_logger().warning('Cannot find head point: no path')
            return None
        end = self.global_to_local(self.path.poses[-1])
        if self.distance(end) <= self.head_radius:
            return end
        for i in range(len(self.path.poses) - 1, 0, -1):
            start = self.global_to_local(self.path.poses[i - 1])
            x, y = start.pose.position.x, start.pose.position.y
            dx = end.pose.position.x - x
            dy = end.pose.position.y - y
            a = dx * dx + dy * dy
            b = 2.0 * (x * dx + y * dy)
            c = x * x + y * y - self.head_radius * self.head_radius
            discriminant = b * b - 4.0 * a * c
            if a > 0.0 and discriminant >= 0.0:
                root = math.sqrt(discriminant)
                for t in ((-b + root) / (2.0 * a), (-b - root) / (2.0 * a)):
                    if 0.0 <= t <= 1.0:
                        point = copy.deepcopy(end)
                        point.pose.position.x = x + t * dx
                        point.pose.position.y = y + t * dy
                        point.pose.position.z = (start.pose.position.z
                                                 + t * (end.pose.position.z - start.pose.position.z))
                        return point
            end = start
        self.get_logger().warning('Cannot find head point: path does not intersect the lookahead circle')
        return None






    # TODO 1: Zamień obserwację na dyskretny stan; nie używaj brakujących ani niefinitywnych wartości.
    def discretize_state(self, observation: list[float]) -> tuple[int, ...]:
        discreet_state = []
        for element in observation:
            discreet_element = 100000 * element #jak to zdyskretyzować
            discreet_state.append(discreet_element)
        
        return (discreet_state)

    # TODO 2: Wybierz akcję epsilon-zachłannie; poza uczeniem wybieraj najlepszą akcję.
    def choose_action(self, state: tuple[int, ...]) -> int:

        return 0

    # TODO 3: Sprawdź numer akcji, zamień go na prędkości i opublikuj na /cmd_vel.
    def execute_action(self, action: int) -> None:

        pass

    # TODO 4: Zaktualizuj Q-tablicę, uwzględniając stan terminalny.
    def update_q_table(self, state: tuple[int, ...], action: int, reward: float,
                       next_state: tuple[int, ...], terminal: bool) -> None:

        pass

    # TODO 5 i 6: Zapisz i wczytaj Q-tablicę w tym samym formacie.
    # Brak lub pusty plik oznacza pustą tablicę; niepoprawne wiersze odrzucaj w całości.
    def save_q_table(self, filename: str) -> None:

        pass

    def load_q_table(self, filename: str) -> None:

        pass

    # TODO 7: Jeden krok uczenia. Obsłuż brak punktu trasy i wyjątki TF przed wyborem akcji.
    # Zakończ epizod za pomocą finish_episode("powód"); nie twórz pętli kroków.
    def learning_loop(self) -> None:

        pass







    def finish_episode(self, reason: str) -> None:
        self.get_logger().info(
            f'Episode ended: {reason} after {self.episode_steps} steps, {len(self.q_table)} states')
        self.clear_path()
        if self.training:
            self.training_phase = TrainingPhase.NEED_RESET
            try:
                self.save_q_table(self.q_file)
            except (OSError, RuntimeError) as exc:
                self.get_logger().error(str(exc))

    def global_to_local(self, point: PoseStamped) -> PoseStamped:
        point = copy.deepcopy(point)
        point.header.stamp.sec = 0
        point.header.stamp.nanosec = 0
        return self.tf_buffer.transform(point, self.robot_frame)

    def stop_moving(self) -> None:
        self.publisher.publish(Twist())

    def clear_path(self) -> None:
        self.path = None
        self.previous_state = None
        self.pose_id = 1
        self.stop_moving()
        self.get_logger().info('Path cleared, waiting for a new path')

    @staticmethod
    def distance(point: PoseStamped) -> float:
        return math.hypot(point.pose.position.x, point.pose.position.y)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
