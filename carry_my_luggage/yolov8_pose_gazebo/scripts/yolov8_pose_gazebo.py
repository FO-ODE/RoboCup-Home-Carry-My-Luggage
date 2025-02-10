#!/usr/bin/env python3
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

class PoseDetector:
    def __init__(self):
        rospy.init_node('yolov8_pose_gazebo')
        
        # 初始化模型
        self.model = YOLO("scripts/models/yolov8n-pose.pt").to("cuda")
        self.bridge = CvBridge()
        
        # 参数配置
        self.side_angle_thres = 45   # 侧抬手臂判定阈值
        self.forward_thres = 30      # 前抬手臂垂直偏差阈值
        
        # 发布器
        self.stop_pub = rospy.Publisher("/test_topic", String, queue_size=1)
        self.grasp_pub = rospy.Publisher("/start_grasp", String, queue_size=1)
        
        # 订阅摄像头
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        
        # 初始化OpenCV窗口
        cv2.namedWindow("YOLOv8 Pose Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLOv8 Pose Detection", 1280, 720)

    def calculate_angle(self, a, b, c):
        """计算三个关键点之间的角度(以b为顶点)"""
        ba = a - b
        bc = c - b
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        return math.degrees(np.arccos(cosine_angle))

    def image_callback(self, msg):
        try:
            # 图像转换
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 推理检测
            results = self.model(cv_image, verbose=False)
            
            # 可视化处理
            if results[0].boxes is not None:
                annotated_frame = results[0].plot()  # 自动绘制bbox和关键点
            else:
                annotated_frame = cv_image.copy()
            
            if results[0].keypoints is not None:
                # 关键点处理
                keypoints = results[0].keypoints.xy.cpu().numpy()
                confidences = results[0].keypoints.conf.cpu().numpy()
                
                for person, conf in zip(keypoints, confidences):
                    # 过滤低置信度关键点
                    if conf[5] < 0.5 or conf[6] < 0.5:  # 需要可靠的双肩检测
                        continue
                    
                    # 关键点索引
                    left_shoulder = person[5]
                    left_elbow = person[7]
                    left_wrist = person[9]
                    right_shoulder = person[6]
                    right_elbow = person[8]
                    right_wrist = person[10]

                    # ========== 侧抬手检测 ==========
                    left_arm_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
                    right_arm_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)
                    
                    if left_arm_angle < self.side_angle_thres:
                        self.stop_pub.publish(String(data="stop_follow"))
                        cv2.putText(annotated_frame, "LEFT ARM RAISED", (50, 50),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    elif right_arm_angle < self.side_angle_thres:
                        self.stop_pub.publish(String(data="stop_follow")) 
                        cv2.putText(annotated_frame, "RIGHT ARM RAISED", (50, 50),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    # ========== 前抬手检测 ==========
                    left_vertical_diff = abs(left_wrist[1] - left_shoulder[1])
                    right_vertical_diff = abs(right_wrist[1] - right_shoulder[1])
                    
                    if left_vertical_diff < self.forward_thres:
                        self.grasp_pub.publish(String(data="start_grasp"))
                        cv2.putText(annotated_frame, "FORWARD LEFT ARM", (50, 100),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif right_vertical_diff < self.forward_thres:
                        self.grasp_pub.publish(String(data="start_grasp"))
                        cv2.putText(annotated_frame, "FORWARD RIGHT ARM", (50, 100),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 显示处理后的图像
            cv2.imshow("YOLOv8 Pose Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")
        finally:
            # ROS关闭时自动清理窗口
            if rospy.is_shutdown():
                cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = PoseDetector()
    rospy.spin()