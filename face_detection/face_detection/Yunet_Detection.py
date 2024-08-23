#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from receive_theta.msg import FaceInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        self.face_info_publisher = self.create_publisher(FaceInfo, 'face_info', 10)

        self.model_path = "/home/robot/autonomy_stack_diablo_setup/src/utilities/face_detection/face_detection/face_detection_yunet_2023mar.onnx"
        self.face_detector = cv2.FaceDetectorYN_create(self.model_path,
                          "", 
                          (300, 300),
                          score_threshold=0.8,
                          backend_id=5,
                          target_id=6) 

        baseline_img = cv2.imread("/home/robot/autonomy_stack_diablo_setup/src/utilities/face_detection/face_detection/baseline.jpg")
        if baseline_img is None:
            self.get_logger().error("Failed to load baseline image.")
            raise ValueError("Failed to load baseline image.")
        
        self.baseline_face_descriptor = self.extract_face_descriptor(baseline_img)

        self.threshold = 45
	
        self.prev_time = time.time()

    def extract_face_descriptor(self, img):
        height, width, _ = img.shape
        self.face_detector.setInputSize((width, height))
        _, faces = self.face_detector.detect(img)
        
        if faces is None or len(faces) == 0:
            raise Exception("No face detected in the image.")
        
        face = faces[0]
        x, y, w, h = face[:4].astype(int)
        face_roi = img[y:y+h, x:x+w]
        face_descriptor = cv2.resize(face_roi, (128, 128)).flatten() / 255.0
        return face_descriptor

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = frame.shape
        self.face_detector.setInputSize((width, height))
        _, faces = self.face_detector.detect(frame)

        best_face_info = None
        max_distance = -1  # 初始化最大距离为负数，表示尚未找到匹配的脸

        if faces is not None:
            for face in faces:
                x, y, w, h = face[:4].astype(int)
                
                if x < 0 or y < 0 or x + w > width or y + h > height:
                    self.get_logger().warning(f"out of bounds: x={x}, y={y}, w={w}, h={h}")
                    continue
                
                face_roi = frame[y:y+h, x:x+w]

                if face_roi.size == 0:
                    self.get_logger().warning("ROI is None.")
                    continue

                try:
                    face_descriptor = cv2.resize(face_roi, (128, 128)).flatten() / 255.0
                except cv2.error as e:
                    self.get_logger().error(f"Resize ROI return error: {e}")
                    continue

                distance = np.linalg.norm(self.baseline_face_descriptor - face_descriptor)
                face_area = w * h
                if distance > self.threshold:
                # if distance > self.threshold and face_area > 300:
                    if distance > max_distance:
                        max_distance = distance
                        best_face_info = (x, y, w, h)

                    label = "Match"
                    color = (0, 255, 0)
                else:
                    label = "Not Match"
                    color = (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                

        # 发布置信度最高的人脸信息
        if best_face_info is not None:
            x, y, w, h = best_face_info
            face_info_msg = FaceInfo()
            face_info_msg.x = int(x - width / 2)  
            face_info_msg.y = int(y - height / 2)  
            face_info_msg.width = int(w) 
            face_info_msg.height = int(h) 
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "Match", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            self.face_info_publisher.publish(face_info_msg)
            face_area = face_info_msg.width * face_info_msg.height
            self.get_logger().info(f"Best face - (x, y): ({face_info_msg.x}, {face_info_msg.y}), (w, h): ({face_info_msg.width}, {face_info_msg.height}), face area: {face_area}")
                
        current_time = time.time()
        fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time
        fps_text = f"FPS: {fps:.2f}"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # cv2.imshow("Video", frame)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    face_recognition_node = FaceRecognitionNode()
    rclpy.spin(face_recognition_node)
    face_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
