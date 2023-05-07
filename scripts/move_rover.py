#! /usr/bin/env python

import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from tf.transformations import quaternion_from_euler
import math

class MoveRover:
    def __init__(self):
        self.rover_state = State()
        self.rover_current_pose = PoseStamped()
        self.rover_des_pose = PoseStamped()
        self.rover_vel = TwistStamped()
        self.rover_coordinate = [0, 0, 0]

        self.rate = rospy.Rate(200)
        self.t_start = rospy.get_time()

        self.rover_local_pose_publisher = rospy.Publisher("/uav0/mavros/setpoint_position/local", PoseStamped,
                                                          queue_size=10)
        self.rover_local_pose_subscriber = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped,
                                                            callback=self.rover_Current_pose)
        self.rover_state_sub = rospy.Subscriber("/uav0/mavros/state", State, callback=self.Current_rover_state)

        self.rover_vel_pub = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.offboard_control_rover = False
        self.rover_moved = False

    def Current_rover_state(self,msg):
        self.rover_state = msg

    def rover_Current_pose(self, msg):
        self.rover_current_pose = msg

    def Set_Mode_Rover_Offboard(self):
        rospy.wait_for_service("/uav0/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/uav0/mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/uav0/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("/uav0/mavros/set_mode", SetMode)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        if arming_client.call(arm_cmd).success == True:
            rospy.loginfo(" Rover armed")
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.rover_local_pose_publisher.publish(self.rover_des_pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"

        if set_mode_client.call(offb_set_mode).mode_sent == True:
            self.offboard_control_rover = True
            rospy.loginfo(" Rover OFFBOARD enabled")

        last_req = rospy.Time.now()
        while not rospy.is_shutdown() and not self.offboard_control_rover:
            if self.rover_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if set_mode_client.call(offb_set_mode).mode_sent == True:
                    self.offboard_control_rover = True
                    rospy.loginfo(" Rover OFFBOARD enabled")
                    break
                last_req = rospy.Time.now()
            else:
                if not self.rover_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Rover armed")
            last_req = rospy.Time.now()

    def get_descent(self, x, y, z, yaw):
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp = rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame = 8
        des_vel.type_mask = 3527
        des_vel.velocity.x = x
        des_vel.velocity.y = y
        des_vel.velocity.z = z
        des_vel.yaw_rate = yaw
        return des_vel

    def Rover_Path_PosControl(self):

        rospy.loginfo("Rover Movement started...")
        while not rospy.is_shutdown():

            # self.rover_des_pose.pose.position.x = self.rover_coordinate[0] + (20.0 * math.cos(0.12))
            # self.rover_des_pose.pose.position.y = self.rover_coordinate[1] + (20.0 * math.sin(0.12))
            # self.rover_des_pose.pose.position.z = self.rover_coordinate[2]

            des = self.get_descent(0.2, 0.0, 0.0, 0.02)
            self.rover_vel_pub.publish(des)


    def Rover_Mission_Start(self):

        if not self.offboard_control_rover:
            self.Set_Mode_Rover_Offboard()
        if not self.rover_moved:
            self.Rover_Path_PosControl()

if __name__ == "__main__":
    rospy.init_node("Drone_Landing", anonymous=True)
    MoveRover().Rover_Mission_Start()
    rospy.spin()