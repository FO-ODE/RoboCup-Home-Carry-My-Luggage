#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Initialize the YOLOv8-Pose model
pose_model = YOLO("yolov8n-pose.pt")  

# Initialize ROS
rospy.init_node("yolov8_pose_gazebo")
bridge = CvBridge()

# Create a ROS topic publisher
pose_pub = rospy.Publisher("/test_topic", String, queue_size=10)

def image_callback(msg):
    try:
        # Convert ROS image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8-Pose detection
        pose_results = pose_model.predict(cv_image, stream=False)

        # Ensure pose_results is not empty
        if not pose_results:
            rospy.loginfo("No people detected.")
            return

        # Iterate through detected people
        for pose_result in pose_results:
            keypoints = pose_result.keypoints  # Get keypoints

            if keypoints is None or len(keypoints.xy[0]) < 11:
                continue  # Skip if no keypoints detected

            left_hand = keypoints.xy[0][9]  # Left hand
            right_hand = keypoints.xy[0][10]  # Right hand
            left_elbow = keypoints.xy[0][7]  # Left elbow
            right_elbow = keypoints.xy[0][8]  # Right elbow

            # Detect hand gestures
            if is_side_raised(left_hand, right_hand, left_elbow, right_elbow):
                rospy.loginfo("Side hand raised detected! Publishing 'stop_follow'.")
                pose_pub.publish("stop_follow")

            elif is_front_raised(left_hand, right_hand, left_elbow, right_elbow):
                rospy.loginfo("Front hand raised detected! Publishing 'start_grasp'.")
                pose_pub.publish("start_grasp")

            # Display the detected results
            annotated_frame = pose_result.plot()
            cv2.imshow("Pose Detection", annotated_frame)
            cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")
        
def is_side_raised(left_hand, right_hand, left_elbow, right_elbow):
    """ Determine whether a side hand raise is detected (left or right hand raised above a certain threshold) """
    threshold = 20  # Hand raise threshold
    left_raised = left_hand[1] < left_elbow[1] - threshold
    right_raised = right_hand[1] < right_elbow[1] - threshold
    return left_raised or right_raised

def is_front_raised(left_hand, right_hand, left_elbow, right_elbow):
    """ Determine whether a front hand raise is detected (left or right hand extended forward) """
    threshold = 20  # Forward extension threshold
    left_forward = abs(left_hand[0] - left_elbow[0]) > threshold
    right_forward = abs(right_hand[0] - right_elbow[0]) > threshold
    return left_forward or right_forward

if __name__ == "__main__":
    # Subscribe to the camera image topic
    rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)
    
    rospy.loginfo("Pose detection node started!")
    rospy.spin()

    cv2.destroyAllWindows()
