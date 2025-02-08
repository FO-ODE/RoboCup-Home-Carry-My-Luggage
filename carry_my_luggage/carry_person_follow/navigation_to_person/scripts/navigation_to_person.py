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


class NavigationToPerson():
    def __init__(self, safe_distance=0.5, target_object="person"):
        """
        :param safe_distance: Safe distance between the robot and the person (unit: meters)
        :param target_object: Target object text to look for in /text_markers
        """
        self.safe_distance = safe_distance
        self.goal_object = target_object

        # Used to wait/get MarkerArray
        self.marker_topic = "/text_markers"
        
        # TF2 buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers for person pose and navigation goal
        self.nav_goal_pub = rospy.Publisher('/nav_goal', PoseStamped, queue_size=10)
        self.person_pose_pub = rospy.Publisher('/person_pose', PoseStamped, queue_size=10)
        # Head controller publisher
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

        # MoveBase Action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        self.tracking_timer = rospy.Timer(
            rospy.Duration(0.2),  # Execute every 0.2s (5Hz)
            self.head_tracking_cb
        )

    def find_person(self, max_retries=5, retry_interval=0.5):
        """
        Find the Marker matching self.goal_object from /text_markers topic
        Returns: Found Marker object (None if not found)
        """
        target_marker = None
        retries = 0

        while retries < max_retries:
            try:
                rospy.loginfo(f"Attempt {retries+1}/{max_retries}: Waiting for /text_markers...")
                marker_array_msg = rospy.wait_for_message(
                    self.marker_topic, MarkerArray, timeout=5.0
                )
                rospy.loginfo(f"Found {len(marker_array_msg.markers)} markers in /text_markers.")
                
                for marker in marker_array_msg.markers:
                    rospy.loginfo(f"Marker text: {marker.text}")
                    if marker.text == self.goal_object:
                        rospy.loginfo(f"Found target object: {self.goal_object}")
                        target_marker = marker
                        return target_marker  # find the target object

                rospy.logwarn("No matching person marker found in the MarkerArray.")
            except rospy.ROSException as e:
                rospy.logwarn(f"Timeout. No marker received. Error: {e}")

            retries += 1
            if retries < max_retries:
                rospy.loginfo(f"Retrying in {retry_interval} seconds...")
                rospy.sleep(retry_interval)

        rospy.logwarn("Max retries reached. No matching marker found.")
        return None
    
    def get_robot_pose(self):
        """
        Get the robot's position and orientation in the map coordinate frame
        :return: (position, orientation) tuple
        """
        try:
            # Query the transform from base_link to map
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            # Extract position and orientation
            position = transform.transform.translation
            orientation = transform.transform.rotation
            orientation = transform.transform.rotation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(quaternion)  # Extract yaw angle
            rospy.loginfo("Robot pose in map: x={}, y={}, z={}, w={}".format(
                position.x, position.y, orientation.z, orientation.w
            ))
            

            return position, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None, None

    def get_nav_goal(self, marker):
        """
        Calculate the navigation goal based on the found Marker,
        and maintain a safe_distance from the marker's position.
        """
        rospy.sleep(1)
        
        # Wrap marker.pose (Pose) into PoseStamped for tf2_geometry_msgs
        pose_in_marker_frame = PoseStamped()
        pose_in_marker_frame.header = marker.header
        pose_in_marker_frame.pose = marker.pose

        try:
            # Query the transform from marker frame to map frame
            transform = self.tf_buffer.lookup_transform("map", marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Perform coordinate transformation to get the person's pose in the map frame
            pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_in_marker_frame, transform)

            # Publish the person's pose in the map frame
            self.person_pose_pub.publish(pose_in_map)
            rospy.loginfo("Published person pose in map frame.")
            rospy.loginfo("person pose in map: x={}, y={}, z={}, w={}".format(
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y,
                pose_in_map.pose.orientation.z,
                pose_in_map.pose.orientation.w
            ))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform person pose to map frame: {e}")

        robot_position, angle_robot = self.get_robot_pose()
        dx = pose_in_map.pose.position.x - robot_position.x
        dy = pose_in_map.pose.position.y - robot_position.y
        distance = math.sqrt(dx**2 + dy**2)
        rospy.loginfo(f"dx: {dx:.2f}, dy: {dy:.2f}")
        rospy.loginfo(f"Distance to person: {math.sqrt(dx**2 + dy**2):.2f} m")

        # If the distance is greater than the safe distance, calculate the target point
        if distance > self.safe_distance:
            # Calculate the target point (nx, ny) on the line between the person and the robot, at a distance of safe_distance from the person
            angle_to_person = math.atan2(dy, dx)
            nx = pose_in_map.pose.position.x - self.safe_distance * math.cos(angle_to_person)
            ny = pose_in_map.pose.position.y - self.safe_distance * math.sin(angle_to_person)
        else:
            # If the distance is less than or equal to the safe distance, stay in the current position
            nx = robot_position.x
            ny = robot_position.y
            angle_to_person = angle_robot
            rospy.logwarn("Person is too close. Staying at current position.")

        rospy.loginfo(f"Safe distance target: x={nx}, y={ny}")

        # Set the navigation goal's orientation (make the robot face the person)
        # Heading angle = angle_to_person
        qz = math.sin(angle_to_person / 2)
        qw = math.cos(angle_to_person / 2)
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
    
    def get_rotation_goal(self, yaw_angle):
        """
        Generate a navigation goal for in-place rotation
        :param yaw_angle: Angle to rotate (in radians)
        :return: MoveBaseGoal
        """
        # Get the robot's current position
        robot_position, current_yaw = self.get_robot_pose()
        if robot_position is None:
            rospy.logerr("Failed to get robot pose for rotation.")
            return None

        # Calculate the new orientation angle (in-place rotation)
        target_yaw = current_yaw + yaw_angle
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))  # Normalize the angle to [-pi, pi]

        # Generate the target orientation in quaternion
        qz = math.sin(target_yaw / 2)
        qw = math.cos(target_yaw / 2)

        # Construct the rotation goal (keep the position, only change the orientation)
        rotation_goal = MoveBaseGoal()
        rotation_goal.target_pose.header.frame_id = "map"
        rotation_goal.target_pose.header.stamp = rospy.Time.now()
        rotation_goal.target_pose.pose.position.x = robot_position.x
        rotation_goal.target_pose.pose.position.y = robot_position.y
        rotation_goal.target_pose.pose.orientation.z = qz
        rotation_goal.target_pose.pose.orientation.w = qw

        rospy.loginfo(f"Generated rotation goal: yaw={math.degrees(target_yaw):.2f} degrees")
        return rotation_goal
    
    def get_detour_goal(self, base_goal, angle_offset):
        """
        Generate a detour goal based on the original navigation goal with an angle offset
        :param base_goal: Original navigation goal (MoveBaseGoal)
        :param angle_offset: Detour angle offset (in radians)
        :return: Detour navigation goal (MoveBaseGoal)
        """
        # Get the original goal's position and orientation
        goal_position = base_goal.target_pose.pose.position
        goal_orientation = base_goal.target_pose.pose.orientation
        _, _, base_yaw = euler_from_quaternion([goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w])

        # Calculate the detour orientation angle
        detour_yaw = base_yaw + angle_offset
        detour_yaw = math.atan2(math.sin(detour_yaw), math.cos(detour_yaw))  # Normalize to [-pi, pi]

        # Generate the detour quaternion
        qz = math.sin(detour_yaw / 2)
        qw = math.cos(detour_yaw / 2)

        # Construct the detour goal (keep the position, change the orientation)
        detour_goal = MoveBaseGoal()
        detour_goal.target_pose.header.frame_id = "map"
        detour_goal.target_pose.header.stamp = rospy.Time.now()
        detour_goal.target_pose.pose.position.x = goal_position.x
        detour_goal.target_pose.pose.position.y = goal_position.y
        detour_goal.target_pose.pose.orientation.z = qz
        detour_goal.target_pose.pose.orientation.w = qw

        rospy.loginfo(f"Generated detour goal with yaw offset: {math.degrees(angle_offset):.2f} degrees")
        return detour_goal

    def _look_direction(self, positions):
        head_joint_traj = JointTrajectory()
        head_joint_traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.15)
        head_joint_traj.points.append(point)
        self.head_pub.publish(head_joint_traj)
    
    def track_person_with_head(self, marker):
        marker_position = marker.pose.position
        px, py = marker_position.x, marker_position.y
        angle = math.atan2(py, px)
        self._look_direction([angle, 0.0])

    def head_tracking_cb(self, event):
            """
            Timer callback function for continuous target tracking.
            1. Call find_person() to find the target Marker
            2. If found, call track_person_with_head() to adjust the head
            """
            person_marker = self.find_person()
            if person_marker is not None:
                self.track_person_with_head(person_marker)

    def navigate_to_person(self):
        """
        Main process: continuously track the person, navigate to the target, and then reacquire the target
        """
        rate = rospy.Rate(1)  # Set loop frequency
        rotation_goals = [math.radians(angle) for angle in [15, 30, 45, 60, -60, -15, -30, -45, -60, 60]]
        detour_angles = [math.radians(15), math.radians(-15), math.radians(30), math.radians(-30)]
        while not rospy.is_shutdown():
            # Find the person's location
            person_marker = self.find_person()
            # if person_marker is None:
            #     rospy.logwarn("No person marker found. Starting rotation search...")
            #     for angle in rotation_goals:
            #         # Calculate the rotation target angle (in-place rotation)
            #         rotation_goal = self.get_rotation_goal(angle)
            #         self.move_base_client.send_goal(rotation_goal)
            #         finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(10.0))

            #         if finished_within_time and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            #             rospy.loginfo("Rotation completed. Retrying to find person...")
            #             person_marker = self.find_person()
            #             if person_marker:
            #                 rospy.loginfo("Person found after rotation.")
            #                 break
            #         else:
            #             rospy.logwarn("Rotation failed or timed out. Retrying...")
            

            if person_marker is None:
                    rospy.logerr("Person not found after rotation search.")
                    
            # Get the navigation goal
            goal = self.get_nav_goal(person_marker)
            if goal is None:
                rospy.logwarn("Could not compute navigation goal. Retrying...")
                rate.sleep()
                continue

            rospy.loginfo("Sending goal to move_base...")
            self.move_base_client.send_goal(goal)

            finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(4.0))
            if finished_within_time:
                state = self.move_base_client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Navigation to person succeeded!")
                    rospy.sleep(0.3)  # Wait for 0.3 seconds before re-navigating
                    continue
                else:
                    rospy.logwarn(f"Navigation failed with state: {state}. Trying detour...")
            else:
                rospy.logwarn("Navigation timed out. Trying detour...")

            # Try detour after navigation failure
            for i, angle in enumerate(detour_angles):
                detour_goal = self.get_detour_goal(goal, angle)
                self.move_base_client.send_goal(detour_goal)
                finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(15.0))

                if finished_within_time and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Detour attempt {i + 1} succeeded!")
                    break
                else:
                    rospy.logwarn(f"Detour attempt {i + 1} failed.")

            # After navigation is complete or timed out, reacquire the target
            rospy.sleep(0.1)  # Wait for 0.1 seconds before re-navigating


if __name__ == "__main__":
    # Get the safe distance parameter from the launch file
    distance = 1.5  # Default safe distance

    # Initialize the node and NavigationToPerson object
    rospy.init_node("navigation_to_person_node")
    nav_person = NavigationToPerson(safe_distance=distance, target_object="person")
    
    # Start the main process
    try:
        nav_person.navigate_to_person()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
