#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading, SegmentList
import time
import numpy as np

class pure_pursuit(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None
        self.segments = None
        self.last_ms = None
        self.pub_counter = 0
        self.actuator_limits = None
        self.L = 0.1

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        
        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.PoseHandling, "lane_filter", queue_size=1)
        self.sub_segments = rospy.Subscriber("~seglist_filtered", SegmentList, self.segHandling, "lane_filter_seg", queue_size=1)

        # FSM
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch,  queue_size=1)     # for this topic, no remapping is required, since it is directly defined in the namespace lane_controller_node by the fsm_node (via it's default.yaml file)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def cbSwitch(self,fsm_switch_msg):
        self.active = fsm_switch_msg.data   # True or False

        rospy.loginfo("active: " + str(self.active))
    # FSM

    def cbMode(self,fsm_state_msg):

        # if self.fsm_state != fsm_state_msg.state and fsm_state_msg.state == "IN_CHARGING_AREA":
        #     self.sleepMaintenance = True
        #     self.sendStop()
        #     rospy.Timer(rospy.Duration.from_sec(2.0), self.unsleepMaintenance)

        self.fsm_state = fsm_state_msg.state    # String of current FSM state
        print "fsm_state changed in lane_controller_node to: " , self.fsm_state

    def segHandling(self, input_seg_msg, seg_source):
        if seg_source == "lane_filter_seg":
            self.segments = input_seg_msg

    def PoseHandling(self, input_pose_msg, pose_source):
        self.updatePose(input_pose_msg)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()
        self.sub_segments.unregister()
        self.sub_switch.unregister()
        self.sub_fsm_mode.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

        rospy.sleep(0.5)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

    def loginfo(self, message):
        rospy.loginfo("[%s] %s" % (self.node_name, message))

    def updatePose(self, pose_msg):
        self.lane_reading = pose_msg

        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        car_control_msg.v = 0.3
        car_control_msg.omega = 0.

        if self.segments:
            total_white = np.array([0., 0.])
            total_yellow = np.array([0., 0.])
            n_white = 0
            n_yellow = 0
            for segment in self.segments.segments:
                centroid = np.array([(segment.points[1].x + segment.points[0].x) / 2, 
                                        (segment.points[1].y + segment.points[0].y) / 2])
                #weight = 1 / (1 + np.abs(np.linalg.norm(centroid) - self.L) ** 2)
                #centroid = weight * centroid
                if segment.color == segment.WHITE:
                    total_white += centroid
                    n_white += 1
                elif segment.color == segment.YELLOW:
                    total_yellow += centroid
                    n_yellow += 1

            ave_white = total_white * 1. / max(1, n_white)
            ave_yellow = total_yellow * 1. / max(1, n_yellow)

            follow_point = 0.5 * (ave_white + ave_yellow)

            if n_white == 0:
                follow_point[1] += 0.25
            elif n_yellow == 0:
                follow_point[1] -= 0.15

            # heading = np.array([np.cos(self.lane_reading.phi), np.sin(self.lane_reading.phi)])

            distance = np.linalg.norm(follow_point)

            # sin_phi = np.cross(follow_point, heading) / ((distance * np.linalg.norm(heading)) + np.exp(-6))

            # cos_phi = np.dot(follow_point, heading) / ((distance * np.linalg.norm(heading)) + np.exp(-6))

            # if sin_phi >= 0:
            #     angle = np.arccos(cos_phi)
            # else:
            #     angle = (2 * np.pi) - np.arccos(cos_phi)
            
            angle = np.arctan2(follow_point[1], follow_point[0])
            car_control_msg.omega = -2 * car_control_msg.v * np.sin(angle) / (distance + np.exp(-6))
        self.publishCmd(car_control_msg)

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_node", anonymous=False)

    pure_pursuit_node = pure_pursuit()
    rospy.spin()
