# Real-Time-Object-Detection-Position-Estimation-using-YOLOv8-and-ROS2

🤖 Real-Time Object Detection & Position Estimation using YOLOv8 and ROS2
This project integrates YOLOv8 (You Only Look Once, v8) with the ROS2 (Robot Operating System 2) framework to perform real-time object detection, position estimation, and 3D visualization. The system captures live video from a webcam, detects moving objects, estimates their position, and visualizes them in RViz using dynamically updated spherical markers.

🧠 Objective
To demonstrate the seamless integration of AI-based object detection with robotic visualization and perception tools. The system showcases a hybrid pipeline that bridges computer vision and robotics middleware, enabling real-time detection and visualization on consumer-grade hardware.

🧩 System Architecture
YOLOv8 Object Detection (Windows)
Captures real-time video via laptop camera
Runs YOLOv8n inference using the Ultralytics Python API
Extracts bounding box centers and transmits coordinates via UDP
ROS2 Position Estimation (Ubuntu/WSL2)
Receives object coordinates through a UDP listener
Publishes 3D visualization markers (spheres) to /visualization_marker
Displays the object in RViz as a dynamically moving marker
Cross-Platform Data Flow
Communication handled via lightweight UDP socket
Real-time updates with low latency (~1–2 ms per packet)
Enables modular and scalable architecture for future robotics applications
