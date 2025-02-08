import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


bridge = CvBridge()

def callback(image_msg):

    cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    bgr_image_msg = bridge.cv2_to_imgmsg(bgr_image, "bgr8")
    pub.publish(bgr_image_msg)


rospy.init_node('bgr_to_rgb_converter')

pub = rospy.Publisher('/camera/image_rgb', Image, queue_size=10)
sub = rospy.Subscriber('/camera/image_raw', Image, callback)

rospy.spin()
