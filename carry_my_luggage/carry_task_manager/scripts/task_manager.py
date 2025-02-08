import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
import time
import math
import os
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from pal_interaction_msgs.msg import TtsActionGoal
from teleop_tools_msgs.msg import IncrementActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf.transformations as tf



class PublishArmTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Publishing arm trajectory command...")

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.1, 0.0, -1.5, 1.5, -1.6, 1.4, 1.5]
        point.time_from_start = rospy.Duration(5)  

        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)
        rospy.loginfo("Arm trajectory published successfully!")
        rospy.sleep(5.0)
        return 'succeeded'
    
class PublishArmTrajectoryVertical(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Publishing arm trajectory command...")

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]


        point = JointTrajectoryPoint()
        point.positions = [1.5, -1.5, -3.0, 2.5, -1.6, -1.0, 0.0]
        point.time_from_start = rospy.Duration(5)  

        traj_msg.points.append(point)

        self.publisher.publish(traj_msg)
        rospy.loginfo("Arm trajectory published successfully!")
        rospy.sleep(5.0)
        return 'succeeded'


class WaitState(smach.State):
    def __init__(self, wait_time):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.wait_time = wait_time

    def execute(self, userdata):
        rospy.loginfo(f"Waiting for {self.wait_time} seconds...")
        rospy.sleep(self.wait_time)
        return 'succeeded'


class PublishNavGoal(smach.State):
    def __init__(self, x, y, yaw):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.x = x
        self.y = y
        self.yaw = yaw
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.sleep(1)

    def execute(self, userdata):
        rospy.loginfo(f"Publishing 2D Nav Goal to ({self.x}, {self.y}) with yaw {self.yaw} degrees...")

        if rospy.is_shutdown():
            rospy.logerr("ROS is shutting down, cannot publish navigation goal.")
            return 'aborted'

        # 确保有订阅者
        start_time = rospy.Time.now()
        while self.publisher.get_num_connections() == 0 and (rospy.Time.now() - start_time).to_sec() < 3:
            rospy.logwarn("Waiting for a subscriber to connect to /move_base_simple/goal...")
            rospy.sleep(1)

        if self.publisher.get_num_connections() == 0:
            rospy.logerr("No subscriber connected to /move_base_simple/goal, aborting.")
            return 'aborted'

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.x
        goal.pose.position.y = self.y
        goal.pose.position.z = 0.0

        quaternion = tf.quaternion_from_euler(0, 0, math.radians(self.yaw))  # 确保yaw是弧度
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        try:
            self.publisher.publish(goal)
            rospy.loginfo("2D Nav Goal published successfully!")
            
            rospy.loginfo("Waiting for 3 seconds after publishing goal...")
            rospy.sleep(5)
            
            return 'succeeded'
        except Exception as e:
            rospy.logerr(f"Failed to publish navigation goal: {e}")
            return 'aborted'


class WaitForMessage(smach.State):
    def __init__(self, topic, expected_msg):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.topic = topic
        self.expected_msg = expected_msg
        self.received_message = False

    def message_callback(self, msg):
        """ 订阅回调函数，检查是否收到期望的消息 """
        rospy.loginfo(f"Received message: {msg.data}")
        if msg.data == self.expected_msg:
            self.received_message = True

    def execute(self, userdata):
        """ 订阅指定话题，并等待特定消息 """
        rospy.loginfo(f"Waiting for message on topic: {self.topic} with expected content: '{self.expected_msg}'")

        self.received_message = False
        rospy.Subscriber(self.topic, String, self.message_callback)

        rate = rospy.Rate(10)  # 10Hz
        while not self.received_message and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo(f"Received expected message '{self.expected_msg}', transitioning to the next state.")
        return 'succeeded'


class FOLLOW(smach.State):
    def __init__(self, launch_file):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.launch_file = launch_file
        self.stop_signal_received = False

    def execute(self, userdata):
        rospy.loginfo(f"Launching: {self.launch_file}")

        self.process = subprocess.Popen(['roslaunch', *self.launch_file.split()])
        rospy.loginfo(f"Launch process started with PID {self.process.pid}")


        rospy.Subscriber('/test_topic', String, self.stop_callback)


        rate = rospy.Rate(10)  # 10 Hz
        while not self.stop_signal_received and not rospy.is_shutdown():
            rate.sleep()

        if self.process.poll() is None:
            self.process.terminate()
            self.process.wait()
            rospy.loginfo("Launch process terminated.")

        if self.stop_signal_received:
            return 'succeeded'
        else:
            return 'aborted'

    def stop_callback(self, msg):
        if msg.data == 'stop_follow':
            rospy.loginfo("Stop signal received.")
            self.stop_signal_received = True



class ExecuteCommand(smach.State): 
    def __init__(self, cmd):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.cmd = cmd

    def execute(self, userdata):
        rospy.loginfo(f"Executing command: {self.cmd}")
        try:
            env = os.environ.copy()
            process = subprocess.Popen(
                self.cmd,
                shell=True,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.sleep(3)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(f"Command failed: {e}")
            return 'aborted'


class ExecuteCommandWithReturn(smach.State):
    def __init__(self, cmd):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.cmd = cmd

    def execute(self, userdata):
        rospy.loginfo(f"Executing command: {self.cmd}")
        try:
            result = subprocess.call(self.cmd, shell=True, env=os.environ.copy())

            if result == 0:
                rospy.loginfo(f"Command '{self.cmd}' executed successfully.")
                return 'succeeded'
            else:
                rospy.logerr(f"Command '{self.cmd}' failed with exit code {result}.")
                return 'aborted'
        except Exception as e:
            rospy.logerr(f"Command execution failed: {e}")
            return 'aborted'


class HeadControl(smach.State):
    def __init__(self, joint_names, positions, time_from_start):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.joint_names = joint_names
        self.positions = positions
        self.time_from_start = time_from_start
        self.publisher = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.loginfo("HeadControl Publisher initialized.")

    def execute(self, userdata):
        rospy.loginfo("Publishing head control command...")
        
        if not rospy.is_shutdown():
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = self.positions
            point.time_from_start = rospy.Duration(self.time_from_start)

            traj_msg.points = [point]

           
            try:
                self.publisher.publish(traj_msg)
                rospy.loginfo(f"Published head control command: {traj_msg}")
                rospy.sleep(1) 
                return 'succeeded'
            except Exception as e:
                rospy.logerr(f"Failed to publish head control command: {e}")
                return 'aborted'
        else:
            rospy.logerr("ROS is shutting down, unable to publish head control command.")
            return 'aborted'


class SPEAK(smach.State):
    def __init__(self, text, lang_id='en_GB'):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.text = text
        self.lang_id = lang_id
        self.publisher = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        rospy.loginfo("TTS Publisher initialized.")

    def execute(self, userdata):
        rospy.loginfo("Publishing TTS message...")
        
        if not rospy.is_shutdown():
            tts_goal = TtsActionGoal()
            tts_goal.goal.rawtext.text = self.text
            tts_goal.goal.rawtext.lang_id = self.lang_id

            try:
                self.publisher.publish(tts_goal)
                rospy.loginfo(f"Published TTS message: {self.text}")
                rospy.sleep(3)
                return 'succeeded'
            except Exception as e:
                rospy.logerr(f"Failed to publish TTS message: {e}")
                return 'aborted'
        else:
            rospy.logerr("ROS is shutting down, unable to publish TTS message.")
            return 'aborted'


class PublishTorsoIncrement(smach.State):
    def __init__(self, increment_values):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.increment_values = increment_values
        self.publisher = rospy.Publisher('/torso_controller/increment/goal', IncrementActionGoal, queue_size=10)
        rospy.loginfo("Torso Increment Publisher initialized.")

    def execute(self, userdata):
        rospy.loginfo("Publishing torso increment message...")
        
        if not rospy.is_shutdown():
            increment_goal = IncrementActionGoal()
            increment_goal.goal.increment_by = self.increment_values

            try:
                self.publisher.publish(increment_goal)
                rospy.loginfo(f"Published torso increment values: {self.increment_values}")
                rospy.sleep(1) 
                return 'succeeded'
            except Exception as e:
                rospy.logerr(f"Failed to publish torso increment message: {e}")
                return 'aborted'
        else:
            rospy.logerr("ROS is shutting down, unable to publish torso increment message.")
            return 'aborted'




class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing object grasp...')
        result = subprocess.call(["roslaunch", "carry_moveit", "grasp.launch"])
        return 'succeeded' if result == 0 else 'aborted'
    
    
class GraspObjectVertical(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing object grasp...')
        result = subprocess.call(["roslaunch", "carry_moveit", "grasp_vertical.launch"])
        return 'succeeded' if result == 0 else 'aborted'


class DropObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing object drop...')
        result = subprocess.call(["roslaunch", "carry_moveit", "drop.launch"])
        return 'succeeded' if result == 0 else 'aborted'


class ForceGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('WARNING: Force Grasp')
        time.sleep(5)
        result = subprocess.call(["rosservice", "call", "/parallel_gripper_controller/grasp"])
        return 'succeeded' if result == 0 else 'aborted'


class ForceDrop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('WARNING: Force Drop')
        result = subprocess.call(["rosrun", "pal_gripper_controller_configuration_gazebo", "home_gripper.py"])
        return 'succeeded' if result == 0 else 'aborted'





#########################################################################################################
#########################################################################################################
#########################################################################################################
#########################################################################################################




def main():
    rospy.init_node('task_manager', log_level=rospy.INFO)
    rospy.loginfo("Task manager initialized.")


    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    with sm:


        ############################### find the person ###################################
        smach.StateMachine.add('START', 
                               PublishNavGoal(x=0.0, y=0.0, yaw=0),
                               transitions={'succeeded': 'FIND_PERSON', 'aborted': 'aborted'})
        
        smach.StateMachine.add('FIND_PERSON',
                               PublishNavGoal(x=0.0, y=0.0, yaw=-80),
                               transitions={'succeeded': 'SPEAK_HELLO', 'aborted': 'aborted'})
        
        smach.StateMachine.add('SPEAK_HELLO', 
                               SPEAK(text="Hello, nice to meet you. Please let me know which piece of luggage I need to carry."),
                               transitions={'succeeded': 'WAIT_FOR_COMMAND', 'aborted': 'aborted'})
        
        smach.StateMachine.add('WAIT_FOR_COMMAND',
                               WaitForMessage(topic="/test_topic", expected_msg="start_grasp"),
                               transitions={'succeeded': 'RETURN_TO_TABLE'})
        
        smach.StateMachine.add('RETURN_TO_TABLE',
                               PublishNavGoal(x=0.25, y=0.0, yaw=0.0),
                               transitions={'succeeded': 'HEAD_DOWN', 'aborted': 'aborted'}) 
        ############################### find the person ###################################



        ############################### grasp ###################################
        smach.StateMachine.add('HEAD_DOWN', 
                               HeadControl(joint_names=['head_1_joint', 'head_2_joint'], 
                                           positions=[0.0, -1.0], 
                                           time_from_start=1.0),
                               transitions={'succeeded': 'SPEAK_BEFORE_GRASP', 'aborted': 'aborted'})
        
        smach.StateMachine.add('SPEAK_BEFORE_GRASP', 
                               SPEAK(text="found the luggage"),
                               transitions={'succeeded': 'TORSO_LIFTING', 'aborted': 'aborted'})
        
        smach.StateMachine.add('TORSO_LIFTING', 
                               PublishTorsoIncrement(increment_values=[0.3]),
                               transitions={'succeeded': 'ARM_PREPARE', 'aborted': 'aborted'})
    
        smach.StateMachine.add('ARM_PREPARE', 
                               ExecuteCommandWithReturn(cmd="roslaunch carry_moveit arm_prepare.launch"),
                               transitions={'succeeded': 'ARM_INTERMEDIATE_STATE', 'aborted': 'aborted'})
        
        smach.StateMachine.add('ARM_INTERMEDIATE_STATE',
                               PublishArmTrajectory(),
                               transitions={'succeeded': 'TORSO_DOWN'})
        
        smach.StateMachine.add('TORSO_DOWN', 
                               PublishTorsoIncrement(increment_values=[-0.2]),
                               transitions={'succeeded': 'GRASP_SPEAK', 'aborted': 'aborted'})
        
        smach.StateMachine.add('GRASP_SPEAK', 
                               SPEAK(text="prepare for object grasping"),
                               transitions={'succeeded': 'START_SEGMENTATION', 'aborted': 'aborted'})
        
        smach.StateMachine.add('START_SEGMENTATION', 
                               ExecuteCommand(cmd="roslaunch carry_perception plane_segmentation.launch"),
                               transitions={'succeeded': 'START_LABELING_BOTTLE', 'aborted': 'aborted'})
        
        smach.StateMachine.add('START_LABELING_BOTTLE', 
                               ExecuteCommand(cmd="roslaunch carry_perception object_labeling.launch"),
                               transitions={'succeeded': 'GRASP_OBJECT', 'aborted': 'aborted'})
        
        smach.StateMachine.add('GRASP_OBJECT', 
                               GraspObject(),
                               transitions={'succeeded': 'FORCE_GRASP', 'aborted': 'TORSO_LIFTING_2'})
        
        
        #################################### grasp 2nd attempt ################################################
        smach.StateMachine.add('TORSO_LIFTING_2', 
                               PublishTorsoIncrement(increment_values=[0.3]),
                               transitions={'succeeded': 'ARM_INTERMEDIATE_2', 'aborted': 'aborted'})
        
        smach.StateMachine.add('ARM_INTERMEDIATE_2',
                               PublishArmTrajectoryVertical(),
                               transitions={'succeeded': 'TORSO_DOWN_2'})
        
        smach.StateMachine.add('TORSO_DOWN_2', 
                               PublishTorsoIncrement(increment_values=[-0.2]),
                               transitions={'succeeded': 'GRASP_OBJECT_2', 'aborted': 'aborted'})
        
        smach.StateMachine.add('GRASP_OBJECT_2', 
                               GraspObjectVertical(),
                               transitions={'succeeded': 'FORCE_GRASP', 'aborted': 'FORCE_GRASP'})
        #################################### grasp 2nd attempt ################################################
        
        
        smach.StateMachine.add('FORCE_GRASP', 
                               ForceGrasp(),
                               transitions={'succeeded': 'TORSO_LIFT_AFTER_GRASP', 'aborted': 'aborted'})
        
        smach.StateMachine.add('TORSO_LIFT_AFTER_GRASP', 
                               PublishTorsoIncrement(increment_values=[0.3]),
                               transitions={'succeeded': 'HEAD_UP', 'aborted': 'aborted'})
        
        smach.StateMachine.add('HEAD_UP', 
                               HeadControl(joint_names=['head_1_joint', 'head_2_joint'], 
                                           positions=[0.0, 0.0], 
                                           time_from_start=1.0),
                               transitions={'succeeded': 'CARRYING_OBJ', 'aborted': 'aborted'})
        
        smach.StateMachine.add('CARRYING_OBJ', 
                               ExecuteCommandWithReturn(cmd="rosrun carry_moveit tuck_arm.py"),
                               transitions={'succeeded': 'RETURN_BACK_TO_PERSON', 'aborted': 'aborted'})
        ################################ grasp ###################################
        
        
        
        ################################ person following ###################################        
        smach.StateMachine.add('RETURN_BACK_TO_PERSON',
                               PublishNavGoal(x=0.0, y=0.0, yaw=-80),
                               transitions={'succeeded': 'START_POINTCLOUD_PREPROCESS', 'aborted': 'aborted'}) 
        
        
        smach.StateMachine.add('START_POINTCLOUD_PREPROCESS', 
                               ExecuteCommand(cmd="roslaunch preprocessing_pointcloud preprocessing_pointcloud.launch"),
                               transitions={'succeeded': 'START_LABELING_PERSON', 'aborted': 'aborted'})
        
        smach.StateMachine.add('START_LABELING_PERSON', 
                               ExecuteCommand(cmd="roslaunch object_labeling object_labeling.launch"),
                               transitions={'succeeded': 'MOVE_SPEAK', 'aborted': 'aborted'})
        
        
        
        smach.StateMachine.add('MOVE_SPEAK', 
                               SPEAK(text="carrying object, ready to move"),
                               transitions={'succeeded': 'PERSON_FOLLOW', 'aborted': 'aborted'})
        
        smach.StateMachine.add('PERSON_FOLLOW', 
                               FOLLOW('navigation_to_person navigation_to_person.launch'),
                               transitions={'succeeded': 'RETURN_SPEAK', 'aborted': 'RETURN_SPEAK'})
        ################################ person following ###################################
        
        
        
        ################################ return the luggage ###################################
        smach.StateMachine.add('RETURN_SPEAK', 
                               SPEAK(text="here is your luggage"),
                               transitions={'succeeded': 'DROP_TORSO_LIFTING', 'aborted': 'aborted'})
        
        smach.StateMachine.add('DROP_TORSO_LIFTING', PublishTorsoIncrement(increment_values=[0.3]),
                               transitions={'succeeded': 'DROP_OBJECT', 'aborted': 'aborted'})
        
        smach.StateMachine.add('DROP_OBJECT', 
                               DropObject(),
                               transitions={'succeeded': 'ARM_TUCK_END', 'aborted': 'FORCE_DROP'})
        
        smach.StateMachine.add('FORCE_DROP', 
                               ForceDrop(),
                               transitions={'succeeded': 'ARM_TUCK_END', 'aborted': 'ARM_TUCK_END'})

        smach.StateMachine.add('ARM_TUCK_END', 
                               ExecuteCommandWithReturn(cmd="rosrun carry_moveit tuck_arm.py"),
                               transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
        ############################### return the luggage ###################################



    sis = smach_ros.IntrospectionServer('task_manager', sm, '/SM_ROOT')
    sis.start()


    outcome = sm.execute()
    rospy.loginfo('Task Manager Outcome: %s', outcome)

    sis.stop()


if __name__ == '__main__':
    main()
    
    
    
    