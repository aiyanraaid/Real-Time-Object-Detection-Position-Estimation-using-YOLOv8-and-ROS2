import socket
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 65432))
        self.get_logger().info("Socket server started, waiting for data...")

        self.timer = self.create_timer(0.1, self.publish_marker)

        self.latest_x = 0.0
        self.latest_y = 0.0

    def publish_marker(self):
        try:
            self.sock.settimeout(0.01)
            data, _ = self.sock.recvfrom(1024)
            message = data.decode()
            self.latest_x, self.latest_y = map(float, message.split(','))

            self.get_logger().info(f"Received: {message}")

        except socket.timeout:
            pass

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=self.latest_x / 1000.0, y=self.latest_y / 1000.0, z=0.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
