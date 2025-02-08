import actionlib
import rospy
import moveit_commander
import tf2_ros
import numpy as np
import math
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import (Constraints, 
                             PositionConstraint, 
                             BoundingVolume, 
                             PlanningScene,
                             CollisionObject,
                             AttachedCollisionObject)


class MoveToObject():
    def __init__(self, safe_distance=0.5, target_objects=["bottle"], final_orientation= 0):
        """
        :param safe_distance: The safe distance to be maintained between the robot and the object (in meters).
        :param target_objects: A list of target object names to be searched for in the /text_markers topic.
        :param final_orientation: The final orientation to face the object.
        """
        self.safe_distance = safe_distance
        self.goal_objects = target_objects if isinstance(target_objects, list) else [target_objects]
        self.final_orientation = final_orientation

        # Initialize publishers, buffers, and action clients
        self.marker_topic = "/text_markers"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.nav_goal_pub = rospy.Publisher('/nav_goal', PoseStamped, queue_size=10)
        self.object_pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Timer for continuous head tracking
        # self.tracking_timer = rospy.Timer(
        #     rospy.Duration(0.2),
        #     self.head_tracking_cb
        # )

    def find_object(self, max_retries=5, retry_interval=0.5):
        """
        Search the /text_markers topic for a Marker whose text matches any object in self.goal_objects.
        
        :return: The matching Marker object if found, otherwise None.
        """
        target_marker = None
        retries = 0

        while retries < max_retries:
            try:
                # rospy.loginfo(f"Attempt {retries+1}/{max_retries}: Waiting for /text_markers...")
                marker_array_msg = rospy.wait_for_message(
                    self.marker_topic, MarkerArray, timeout=5.0
                )
                # rospy.loginfo(f"Found {len(marker_array_msg.markers)} markers in /text_markers.")
                
                for marker in marker_array_msg.markers:
                    rospy.loginfo(f"Marker text: {marker.text}")
                    if marker.text in self.goal_objects:
                        rospy.loginfo(f"Found target object: {marker.text}")
                        target_marker = marker
                        return target_marker  # find the target object

                # rospy.logwarn("No matching target marker found in the MarkerArray.")
            except rospy.ROSException as e:
                rospy.logwarn(f"Timeout. No marker received. Error: {e}")

            retries += 1
            if retries < max_retries:
                rospy.loginfo(f"Retrying in {retry_interval} seconds...")
                rospy.sleep(retry_interval)

        # rospy.logwarn("Max retries reached. No matching marker found.")
        return None
    
    def get_robot_pose(self):
        """
        Get the robot's position and yaw angle in the 'map' coordinate frame.
        
        :return: (position, yaw) where position is a geometry_msgs/Vector3, 
                 and yaw is a float representing the yaw angle in radians.
        """
        try:
            # Lookup the transform from base_link to map
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            # Extract position and orientation
            position = transform.transform.translation
            orientation = transform.transform.rotation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(quaternion)  # Extract the yaw angle
            rospy.loginfo("Robot pose in map: x={}, y={}, z={}, w={}".format(
                position.x, position.y, orientation.z, orientation.w
            ))

            return position, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None, None

    def get_nav_goal(self, marker):
        """
        Compute the navigation goal based on the detected Marker position, 
        maintaining a specified safe distance (self.safe_distance).

        :param marker: The Marker object containing the object's position.
        :return: A MoveBaseGoal to be sent to the move_base action server.
        """
        rospy.sleep(1)
        
        # Wrap marker.pose into a PoseStamped for tf2 transformation
        pose_in_marker_frame = PoseStamped()
        pose_in_marker_frame.header = marker.header
        pose_in_marker_frame.pose = marker.pose

        try:
            # Lookup transform from the marker's frame to map
            transform = self.tf_buffer.lookup_transform("map", marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Transform object's pose into the map frame
            pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_in_marker_frame, transform)

            # Publish the object's pose in the map frame
            self.object_pose_pub.publish(pose_in_map)
            rospy.loginfo("Published object pose in map frame.")
            rospy.loginfo("Object pose in map: x={}, y={}, z={}, w={}".format(
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y,
                pose_in_map.pose.orientation.z,
                pose_in_map.pose.orientation.w
            ))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform object pose to map frame: {e}")

        robot_position, angle_robot = self.get_robot_pose()
        if robot_position is None:
            rospy.logwarn("Robot position not available, cannot compute navigation goal.")
            return None

        dx = pose_in_map.pose.position.x - robot_position.x
        dy = pose_in_map.pose.position.y - robot_position.y
        distance = math.sqrt(dx**2 + dy**2)
        rospy.loginfo(f"dx: {dx:.2f}, dy: {dy:.2f}")
        rospy.loginfo(f"Distance to object: {distance:.2f} m")

        
        final_angle = final_orientation
        rospy.logwarn(f"Final orientation: {final_angle}")

        # If the robot is farther than the safe distance, compute the target position
        if distance > self.safe_distance:
            nx = pose_in_map.pose.position.x - self.safe_distance * math.cos(final_angle)
            ny = pose_in_map.pose.position.y - self.safe_distance * math.sin(final_angle)
        else:
            # If within safe distance, remain at the current position
            nx = robot_position.x
            ny = robot_position.y
            final_angle = angle_robot
            rospy.logwarn("Object is too close. Staying at current position.")

        rospy.loginfo(f"Safe distance target: x={self.safe_distance * math.cos(final_angle)}, y={self.safe_distance * math.sin(final_angle)}")

        # Set the robot's orientation to face the object
        qz = math.sin(final_angle / 2)
        qw = math.cos(final_angle / 2)

        # Construct the navigation goal for move_base
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = "map"
        nav_goal.target_pose.header.stamp = rospy.Time.now()
        nav_goal.target_pose.pose.position.x = nx
        nav_goal.target_pose.pose.position.y = ny
        nav_goal.target_pose.pose.orientation.z = qz
        nav_goal.target_pose.pose.orientation.w = qw

        # Output the transformed pose
        rospy.loginfo("Pose in map: x={}, y={}, z={}, w={}".format(
            nav_goal.target_pose.pose.position.x,
            nav_goal.target_pose.pose.position.y,
            nav_goal.target_pose.pose.orientation.z,
            nav_goal.target_pose.pose.orientation.w
        ))
        
        nav_goal_msg = PoseStamped()
        nav_goal_msg.header = nav_goal.target_pose.header
        nav_goal_msg.pose = nav_goal.target_pose.pose
        self.nav_goal_pub.publish(nav_goal_msg)
        rospy.loginfo("Published navigation goal.")

        return nav_goal

    def _look_direction(self, positions):
        """
        Publish a JointTrajectory command to move the robot's head joints.
        
        :param positions: A list of joint positions [head_1_joint, head_2_joint].
        """
        head_joint_traj = JointTrajectory()
        head_joint_traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.15)
        head_joint_traj.points.append(point)
        self.head_pub.publish(head_joint_traj)

    def look_down(self, angle=0.8):
        """
        Instruct the head to tilt down by a specified angle.
        
        :param angle: The angle by which the head should tilt down.
        """
        self._look_direction([0.0, -angle])
    
    def look_up(self, angle=0):
        """
        Instruct the head to tilt up by a specified angle.
        
        :param angle: The angle by which the head should tilt up.
        """
        self._look_direction([0.0, angle])
    
    def track_object_with_head(self, marker):
        """
        Adjust the head joints to track the given object (Marker) based on its position.
        
        :param marker: The Marker containing the object's pose.
        """
        marker_position = marker.pose.position
        px, py = marker_position.x, marker_position.y
        angle = math.atan2(py, px)
        self._look_direction([angle, -0.8])

    def head_tracking_cb(self, event):
        """
        Timer callback function for continuous target tracking.
        1. Call find_object() to look for the target Marker.
        2. If found, call track_object_with_head() to adjust the head joints.
        """
        obj_marker = self.find_object()
        if obj_marker is not None:
            self.track_object_with_head(obj_marker)

    def navigate_to_object(self):
        """
        Main routine:
          - Look down to find the object if it's below the robot's head tilt range.
          - Continuously track the object.
          - Compute navigation goal and send it to move_base.
          - Once navigation is complete, optionally look up.
        
        :return: True if navigation succeeds, otherwise False.
        """
        self.look_down(angle=0.8)
        obj_marker = self.find_object()

        if obj_marker is None:
            rospy.logerr("Object not found.")
            return False
                
        # Compute the navigation goal
        goal = self.get_nav_goal(obj_marker)
        if goal is None:
            rospy.logwarn("Could not compute navigation goal. Retrying...")
            return False

        rospy.loginfo("Sending goal to move_base...")
        self.move_base_client.send_goal(goal)

        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(10.0))
        if finished_within_time:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation to object succeeded!")
                rospy.sleep(0.3)  # Wait for the robot to stabilize
            else:
                rospy.logwarn("Navigation to object failed.")
                return False
        return True


if __name__ == "__main__":
    # Retrieve parameters from the parameter server
    safe_distance = 0.8
    target_object = ["bottle"]
    final_orientation = 3.14159
    rospy.loginfo(f"final_orientation: {final_orientation}")
    
    rospy.init_node("move_to_object")
    move_to_object = MoveToObject(safe_distance, target_object, final_orientation)
    
    # move_to_object.look_down(angle=0.0)

    if move_to_object.navigate_to_object():
        rospy.loginfo("Finished navigation to the object.")
    pass
