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

# Create ROS topic publishers
pose_pub = rospy.Publisher("/test_topic", String, queue_size=10)

def image_callback(msg):
    try:
        # Convert ROS image messages to OpenCV images
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8-Pose detection
        pose_results = pose_model.predict(cv_image, stream=False)

        # Make sure pose_results is not empty
        if not pose_results:
            rospy.loginfo("No people detected.")
            return

        # Iterate over everyone detected
        for pose_result in pose_results:
            keypoints = pose_result.keypoints  # Access to key points

            if keypoints is None or len(keypoints.xy[0]) < 11:
                continue  # Ensure that key points are sufficient

            left_hand = keypoints.xy[0][9]  # Left hand
            right_hand = keypoints.xy[0][10]  ## Right hand
            left_elbow = keypoints.xy[0][7]  # Left elbow
            right_elbow = keypoints.xy[0][8]  # Right elbow

            # Detecting hand-raising postures
            if is_side_raised(left_hand, right_hand, left_elbow, right_elbow):
                rospy.loginfo("Side hand raised detected! Publishing 'stop_follow'.")
                pose_pub.publish("stop_follow")

            elif is_front_raised(left_hand, right_hand, left_elbow, right_elbow):
                rospy.loginfo("Front hand raised detected! Publishing 'start_grasp'.")
                pose_pub.publish("start_grasp")

            # Display test results
            annotated_frame = pose_result.plot()
            cv2.imshow("Pose Detection", annotated_frame)
            cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

def is_side_raised(left_hand, right_hand, left_elbow, right_elbow):
    """ Determine if it is a side lift (left or right hand up over a certain angle) """
    threshold = 20  # Hands up threshold
    left_raised = left_hand[1] < left_elbow[1] - threshold
    right_raised = right_hand[1] < right_elbow[1] - threshold
    return left_raised or right_raised

def is_front_raised(left_hand, right_hand, left_elbow, right_elbow):
    """ Determine if it is a forward lift (left or right hand straight forward) """
    threshold = 20  # Pre-extension threshold
    left_forward = abs(left_hand[0] - left_elbow[0]) > threshold
    right_forward = abs(right_hand[0] - right_elbow[0]) > threshold
    return left_forward or right_forward

if __name__ == "__main__":
    # Subscribe to camera images
    rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)
    
    rospy.loginfo("Pose detection node started!")
    rospy.spin()

    cv2.destroyAllWindows()