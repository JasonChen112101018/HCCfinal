import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Int32MultiArray
from scipy.optimize import least_squares
import numpy as np

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')

        # Publishers
        self.move_to_pub = self.create_publisher(Point, '/tello/move_to', 10)
        self.drone_position_pub = self.create_publisher(Point, '/navigation/drone_position', 10)
        self.unknown_tag_position_pub = self.create_publisher(Point, '/navigation/unknown_tag_position', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/apriltag/pose', self.pose_callback, 10)
        self.detected_ids_sub = self.create_subscription(Int32MultiArray, '/apriltag/detections', self.detections_callback, 10)
        self.command_sub = self.create_subscription(String, '/navigation/command', self.command_callback, 10)

        # 儲存相對位置與已知座標
        self.reference_tags = {
            1: (-150, -150), 2: (-50, -150), 3: (50, -150), 4: (150, -150),
            5: (150, 150), 6: (50, 150), 7: (-50, 150), 8: (-150, 150)
        }
        self.detected_reference = {}  # tag_id → (rel_x, rel_y)
        self.drone_position = None
        self.unknown_tag_positions = {}  # tag_id → (x, y)

        self.get_logger().info("✅ Navigation Node initialized")

    def pose_callback(self, msg: PoseStamped):
        try:
            tag_id = int(msg.header.frame_id.split('_')[1])
            rel = (
                msg.pose.position.x * 100,  # to cm
                msg.pose.position.y * 100,
                msg.pose.position.z * 100
            )

            if tag_id in self.reference_tags:
                self.detected_reference[tag_id] = rel
                self.update_drone_position()
            else:
                if self.drone_position:
                    abs_x = self.drone_position[0] + rel[0]
                    abs_y = self.drone_position[1] + rel[1]
                    self.unknown_tag_positions[tag_id] = (abs_x, abs_y)

                    pos_msg = Point()
                    pos_msg.x = abs_x
                    pos_msg.y = abs_y
                    pos_msg.z = float(tag_id)
                    self.unknown_tag_position_pub.publish(pos_msg)
                    self.get_logger().info(f"🧩 Unknown Tag {tag_id} at ({abs_x:.1f}, {abs_y:.1f})")
        except Exception as e:
            self.get_logger().error(f"Pose processing error: {e}")

    def detections_callback(self, msg: Int32MultiArray):
        # 可以加入：根據 tag IDs 檢查探索進度
        pass

    def update_drone_position(self):
        if len(self.detected_reference) < 2:
            return

        def equations(p):
            x, y = p
            eqs = []
            for tag_id, rel in self.detected_reference.items():
                ref = self.reference_tags[tag_id]
                dx, dy = rel[0], rel[1]
                distance_sq = dx**2 + dy**2
                eqs.append((x - ref[0])**2 + (y - ref[1])**2 - distance_sq)
            return eqs

        result = least_squares(equations, x0=[0, 0])
        if result.success:
            self.drone_position = result.x
            msg = Point()
            msg.x, msg.y = result.x
            msg.z = 0.0
            self.drone_position_pub.publish(msg)
            self.get_logger().info(f"📍 Drone estimated at ({msg.x:.1f}, {msg.y:.1f})")

    def command_callback(self, msg: String):
        parts = msg.data.strip().split(':')
        if parts[0] == "move_to_grid":
            try:
                grid_id = int(parts[1])
                target = self.grid_to_position(grid_id)
                self.navigate_to(target)
            except:
                self.get_logger().error("Invalid grid command")
        elif parts[0] == "explore":
            self.explore_grid()

    def grid_to_position(self, grid_id: int):
        # 九宮格對應位置（以中心為 (0, 0)，單位：cm）
        row = grid_id // 3
        col = grid_id % 3
        x = -100 + col * 100
        y = -100 + row * 100
        return (x, y)

    def navigate_to(self, target):
        if self.drone_position is None:
            self.get_logger().warn("Cannot navigate: unknown drone position")
            return
        dx = target[0] - self.drone_position[0]
        dy = target[1] - self.drone_position[1]
        move = Point()
        move.x = dx
        move.y = dy
        move.z = 0.0
        self.move_to_pub.publish(move)
        self.get_logger().info(f"🧭 Navigating dx={dx:.1f}, dy={dy:.1f}")

    def explore_grid(self):
        if self.drone_position is None:
            self.get_logger().warn("Cannot explore: unknown position")
            return

        # 以 3x3 九宮格方式探索，中心→四周
        points = [
            (0, 0), (-50, 0), (50, 0),
            (0, -50), (0, 50),
            (-50, -50), (50, -50),
            (-50, 50), (50, 50),
        ]
        for pt in points:
            if self.drone_position:
                dx = pt[0] - self.drone_position[0]
                dy = pt[1] - self.drone_position[1]
                move = Point(x=dx, y=dy, z=0.0)
                self.move_to_pub.publish(move)
                self.get_logger().info(f"🔍 Exploring to ({pt[0]}, {pt[1]})")
                rclpy.spin_once(self, timeout_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
