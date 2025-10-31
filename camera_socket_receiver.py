import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import struct
import pickle
import cv2

class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Setup socket server
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 8485))
        server_socket.listen(1)
        self.get_logger().info('Camera socket server started, waiting for connection...')

        self.conn, _ = server_socket.accept()
        self.get_logger().info('Camera connection established')

        self.receive_frames()

    def receive_frames(self):
        data = b""
        payload_size = struct.calcsize(">L")

        while True:
            while len(data) < payload_size:
                packet = self.conn.recv(4096)
                if not packet:
                    return
                data += packet

            packed_size = data[:payload_size]
            data = data[payload_size:]
            frame_size = struct.unpack(">L", packed_size)[0]

            while len(data) < frame_size:
                data += self.conn.recv(4096)

            frame_data = data[:frame_size]
            data = data[frame_size:]

            frame = pickle.loads(frame_data)

            # Publish to ROS topic
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Published camera frame')

def main(args=None):
    rclpy.init(args=args)
    camera_receiver = CameraReceiver()
    rclpy.spin(camera_receiver)
    camera_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
