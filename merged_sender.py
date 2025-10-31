import socket
import struct
import pickle
import cv2
from ultralytics import YOLO

HOST = '192.168.165.170' 
PORT_DETECTION = 65432   
PORT_IMAGE = 8485      


detection_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

image_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
image_socket.connect((HOST, PORT_IMAGE))

model = YOLO('yolov8n.pt')

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Camera is not working")
    exit()

print("Camera is open and ready")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            continue

        data = pickle.dumps(frame)
        size = struct.pack(">L", len(data))
        image_socket.sendall(size + data)
        print("Sent camera frame to ROS")

        results = model(frame, verbose=False)

        for result in results:
            if result.boxes is not None:
                for box in result.boxes.xyxy:
                    x1, y1, x2, y2 = box.tolist()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    message = f"{center_x},{center_y}"
                    detection_socket.sendto(message.encode(), (HOST, PORT_DETECTION))
                    print(f"Sent detection: {message}")

except KeyboardInterrupt:
    print("Interrupted")

finally:
    cap.release()
    image_socket.close()
    detection_socket.close()
    cv2.destroyAllWindows()
    print("Everything closed safely")