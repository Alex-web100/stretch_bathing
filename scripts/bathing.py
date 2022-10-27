#!/usr/bin/env python
import rospy
import tf
import numpy as np
import threading
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import funmaptestcopy as fp
import pickle
from datetime import date
import time

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class BathingNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate=2.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wrist_contact_detector=fp.ContactDetector(get_wrist_pitch,pitch_contact_fn,move_increment=.02)
        self.wrist_contact_detector.get_lift_pos=get_lift_pose

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        self.lift_position, self.lift_velocity, self.lift_effort = hm.get_lift_state(joint_states)
        self.wrist_contact_detector.update(joint_states,self.stop_the_robot_service)

    def grip_helper(self):
        rospy.loginfo('Grip Helper')
        pose = {'gripper_aperture':0.07}
        self.move_to_pose(pose)
        rospy.sleep(5)
        pose = {'gripper_aperture':-0.1}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def move_arm_back(self):
        rospy.loginfo('Moving Arm Back')
        pose = {'wrist_extension': .1, 'joint_wrist_pitch':0, 'joint_wrist_yaw':-1, 'joint_head_pan':-1.6, 'joint_head_tilt':-.5,'joint_lift':.80}
        self.move_to_pose(pose)
        rospy.sleep(3)

    def align_with_tag(self):
        rospy.loginfo('Aligning with Tag')
        try:
            listener.waitForTransform('bathing_start_tag','link_aruco_top_wrist',rospy.Time(0.0),rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('bathing_start_tag','link_aruco_top_wrist',rospy.Time(0.0))
            rospy.loginfo("Bottom right to robot wrist: " + str((trans,rot)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf exception")
        #trans = (y, x, z)
        wrist_extend_by = 0
        lift_move_by = 0
        if trans[0]>.265:
            wrist_extend_by = trans[0]-.265
        if trans[1]>0.01:
            rospy.loginfo("test")
            lift_move_by = trans[1]
        elif trans[1]<-0.01:
            lift_move_by = trans[]
        pose = {'wrist_extension': wrist_extend_by, 'joint_wrist_pitch': 0, 'joint_wrist_yaw': 0, 'translate_mobile_base': lift_move_by}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def move_to_knee(self):
        rospy.loginfo('Moving to Knee')
        today = date.today()
        file_name = '/home/hello-robot/catkin_ws/src/bathing/scripts/pose_detection/transforms/'+str(today)+'.pickle'
        with open(file_name, 'rb') as fobj:
            ((kx, ky),(ax,ay)) =  pickle.load(fobj)
        pose = {'translate_mobile_base':-kx*(.01)}
        self.move_to_pose(pose)
        pose = {'wrist_extension': ky*(.01)}
        self.move_to_pose(pose)
        rospy.sleep(1)

    def move_to_ankle(self):
        rospy.loginfo('Moving to Ankle')
        today = date.today()
        file_name = '/home/hello-robot/catkin_ws/src/bathing/scripts/pose_detection/transforms/'+str(today)+'.pickle'
        with open(file_name, 'rb') as fobj:
            ((kx, ky),(ax,ay)) =  pickle.load(fobj)
        interval = (-ax/15.0)
        shift = .015
        with open(file_name, 'rb') as fobj:
            ((kx, ky),(ax,ay)) =  pickle.load(fobj)
        for i in range (0,15):
            pose = {'translate_mobile_base':interval*(.01),'wrist_extension':ky*.01 + shift}
            self.move_to_pose(pose)
            p = self.joint_states.name.index('joint_wrist_pitch')
            current_effort_pitch = self.joint_states.effort[p]
            if (current_effort_pitch<1.3):
                self.wrist_contact_detector.move_until_contact('joint_lift',.05,-1,self.move_to_pose)
            time.sleep(0.5)
            shift = shift*-1
        rospy.sleep(1)

    
    def trigger_grip(self, request):
        self.grip_helper()

        return TriggerResponse(
            success=True,
            message= 'Gripped Washcloth'
        )
    
    def trigger_move_arm_back(self, request):

        self.move_arm_back()

        return TriggerResponse(
            success=True,
            message= 'Moved arm back!'
        )

    def trigger_align_with_tag(self,request):
        self.align_with_tag()

        return TriggerResponse(
            success=True,
            message= 'Aligned With Tag'
        )

    def trigger_move_to_knee(self,request):
        self.move_to_knee()

        return TriggerResponse(
            success=True,
            message= 'Moved to Knee'
        )

    def trigger_move_to_ankle(self,request):
        self.move_to_ankle()

        return TriggerResponse(
            success=True,
            message= 'Moved to Ankle'
        )

    def main(self):
        
        hm.HelloNode.main(self, 'bathing', 'bathing', wait_for_first_pointcloud = False)
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.trigger_write_hello_service = rospy.Service('grip',
                                                         Trigger,
                                                         self.trigger_grip)

        self.trigger_write_hello_service = rospy.Service('move_arm_back',
                                                         Trigger,
                                                         self.trigger_move_arm_back)

        self.trigger_write_hello_service = rospy.Service('align_with_tag',
                                                         Trigger,
                                                         self.trigger_align_with_tag)

        self.trigger_write_hello_service = rospy.Service('move_to_knee',
                                                         Trigger,
                                                         self.trigger_move_to_knee)

        self.trigger_write_hello_service = rospy.Service('move_to_ankle',
                                                         Trigger,
                                                         self.trigger_move_to_ankle)


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

#referenced existing hello robot code for these in stretch_ros/funmap and stretch_ros/hello_misc.py
def get_wrist_pitch(joint_states):
    joint_name = 'joint_wrist_pitch'
    i = joint_states.name.index(joint_name)
    pitch_pos = joint_states.position[i]
    pitch_velocity = joint_states.velocity[i]
    pitch_effort = joint_states.effort[i]
    return [pitch_pos,pitch_velocity,pitch_effort]

def get_lift_pose(joint_states):
    joint_name = 'joint_lift'
    i = joint_states.name.index(joint_name)
    lift_pos = joint_states.position[i]
    return lift_pos

def pitch_contact_fn(effort, av_effort):
    single_effort_threshold = 1.9
    av_effort_threshold = 1.5

    if (effort >= single_effort_threshold):
        rospy.loginfo('Pitch effort exceeded single_effort_threshold: {0} >= {1}'.format(effort,single_effort_threshold))
    if (av_effort >= av_effort_threshold):
        rospy.loginfo('Pitch average effort exceeded av_effort_threshold: {0} >= {1}'.format(av_effort,av_effort_threshold))

    return ((effort >= single_effort_threshold) or
            (av_effort >= av_effort_threshold))

if __name__ == '__main__':
    try:
        try:
            rospy.init_node('bathing')
            listener=tf.TransformListener()
            node = BathingNode()
            node.main()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf Exception')
    except KeyboardInterrupt:
        rospy.loginfo('Process Closing')