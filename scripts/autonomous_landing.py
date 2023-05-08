#! /usr/bin/env python

import rospy
import numpy as np
import math
from mavros_msgs.msg import State, PositionTarget
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from geometry_msgs.msg import PoseStamped
import time


def get_dist(curr, des):
    return math.sqrt(pow(curr.x - des[0], 2) + pow(curr.y - des[1], 2) + pow(curr.z - des[2], 2))


class Mission:
    def __init__(self):
        self.drone_status = State()
        self.present_pose = PoseStamped()
        self.correct_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.curr = self.present_pose.pose.position
        self.tag_pt = None
        self.tag_ori = None
        self.tranformation_mat = np.matrix([[1, 0], [0, -1]])
        self.vel = np.matrix([[0], [0]])
        self.err = np.matrix([[0], [0]])

        self.KP = 0.01
        self.KD = 0.0000
        self.KI = 0.00000
        self.rate = rospy.Rate(40)
        self.rock_vec = np.array([60.2, -12.5, 18])
        self.probe_vec = np.array([40.5, 3.8, 15])
        self.rover_vec = np.array([12.6, -65.0, -3.5])

        self.distThr = 0.2

        self.local_pose_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.local_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, callback=self._pose_cb)
        self.rover_pose_sub = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, callback=self._rover_cb)
        state_sub = rospy.Subscriber("/uav1/mavros/state", State, callback=self._status_cb)
        tag_detect = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback=self._tag_detections)
        self.vel_pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.pose = PoseStamped()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

    def _setmode_offb(self):
        rospy.wait_for_service("/uav1/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/uav1/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        for i in range(10):
            if rospy.is_shutdown():
                break
            self.local_pose_pub.publish(self.goal_pose)
            self.rate.sleep()

        last_req = rospy.Time.now()
        while not rospy.is_shutdown():  # and not self.offboardCheck:

            if self.drone_status.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0):
                if set_mode_client.call(offb_set_mode).mode_sent:
                    # self.offboardCheck = True
                    rospy.loginfo("OFFBOARD enabled")
                    # break

                last_req = rospy.Time.now()
            else:  # if A*B else !(A*B) = !A + !B
                if not self.drone_status.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
            if self.drone_status.mode == "OFFBOARD" and self.drone_status.armed:
                if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    # last_req = rospy.Time.now()
                    break
            self.local_pose_pub.publish(self.pose)
            self.rate.sleep()

    def navigate(self, x, y, z):
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        rospy.loginfo("Desired position: x={}, y={}, z={}".format(x, y, z))
        des = [x, y, z]
        # print(self.curr)
        d = get_dist(self.curr, des)
        while d > self.distThr and not rospy.is_shutdown():
            azimuth = math.atan2(self.rock_vec[1] - self.present_pose.pose.position.y,
                                 self.rock_vec[0] - self.present_pose.pose.position.x)
            # print(azimuth)
            if azimuth > math.pi:
                azimuth -= 2.0 * math.pi
            else:
                azimuth += 2.0 * math.pi
            q = quaternion_from_euler(0, 0, 3.14 / 2)
            # print(q)
            self.goal_pose.pose.orientation.x = q[0]
            self.goal_pose.pose.orientation.y = q[1]
            self.goal_pose.pose.orientation.z = q[2]
            self.goal_pose.pose.orientation.w = q[3]
            d = get_dist(self.curr, des)
            self.local_pose_pub.publish(self.goal_pose)
            self.rate.sleep()
            if d <= self.distThr:
                print(azimuth)
                print(q)
                break

    def _goto_rover(self):
        locations = np.matrix([[self.rovr_pose.x, self.rovr_pose.y + 2, 7.0], ])
        # locations = np.matrix([[-1.0, 0.0, 7.0], ])
        for waypt in locations:
            x, y, z = waypt.tolist()[0]
            self.navigate(x, y, z)

    def get_descent(self, x, y, z):
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp = rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame = 8
        des_vel.type_mask = 3527
        des_vel.velocity.x = x
        des_vel.velocity.y = y
        des_vel.velocity.z = z
        return des_vel

    def _descent(self):
        rate = rospy.Rate(40)
        self.x_change = 1
        self.y_change = 1
        self.x_prev_error = 0
        self.y_prev_error = 0
        self.x_sum_error = 0
        self.y_sum_error = 0
        timeout = 30
        pid_clip = 1.0
        dcnt_h = 7.0

        while not rospy.is_shutdown():

            if self.tag_pt is not None:
                print("Keeping in center")

                err_x = self.tag_pt.x
                err_y = self.tag_pt.y
                err_z = self.tag_pt.z

                self.x_change += err_x * self.KP + (self.x_prev_error * self.KD) + (
                        self.x_sum_error * self.KI)  # chnage := change + err
                self.y_change += err_y * self.KP + (self.y_prev_error * self.KD) + (self.y_sum_error * self.KI)

                self.x_change = max(min(pid_clip * err_z / dcnt_h, self.x_change), -pid_clip * err_z / dcnt_h)
                self.y_change = max(min(pid_clip * err_z / dcnt_h, self.y_change), -pid_clip * err_z / dcnt_h)

                self.err[0][0] = self.x_change
                self.err[1][0] = self.y_change

                self.vel = np.matmul(self.tranformation_mat, self.err)

                des = self.get_descent(self.vel[0][0], self.vel[1][0], -0.3)

                self.vel_pub.publish(des)
                timeout -= 1
                rate.sleep()
                self.x_prev_error = err_x
                self.y_prev_error = err_y
                self.x_sum_error += err_x
                self.y_sum_error += err_y

                if timeout == 0 and self.curr.z > 0.7:
                    timeout = 30
                    self.x_change = 0
                    self.y_change = 0
                    self.x_sum_error = 0
                    self.y_sum_error = 0
                    self.x_prev_error = 0
                    self.y_prev_error = 0

                if self.curr.z < 2.0:
                    print("landing")
                    self._land_on_position()
                    break

        print('Exiting while in decent')

    def _follow_rover(self)
        self.correct_pose.pose.position.x = self.rovr_pose.x
        self.correct_pose.pose.position.y = self.rovr_pose.y
        self.correct_pose.pose.position.z = self.curr.z
        print(self.correct_pose.pose.position.x, self.correct_pose.pose.position.y, self.correct_pose.pose.position.z)
        self.local_pose_pub.publish(self.correct_pose)

    def _land_on_position(self):
        print("Started to land")
        local_pos_pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        arm_service = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        landing_position = PositionTarget()
        landing_position.position.x = self.curr.x  # self.rovr_pose.x
        landing_position.position.y = self.curr.y
        landing_position.position.z = -3.5
        landing_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        landing_position.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                     + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                     + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            local_pos_pub.publish(landing_position)
            rate.sleep()
            if self.curr.z < -3.2:
                print("Landed =========")
                arm_service(False)

    def _pose_cb(self, points):
        self.present_pose = points
        self.curr = self.present_pose.pose.position

    def _status_cb(self, data):
        self.drone_status = data

    def _update_state_cb(self, state):
        self.mode = state.data

    def _tag_detections(self, msgs):
        rate = rospy.Rate(30)
        if len(msgs.detections) > 0:
            msg = msgs.detections[0].pose
            self.tag_pt = msg.pose.pose.position
            self.tag_ori = msg.pose.pose.orientation
        else:
            self.tag_pt = None
            self.tag_ori = None

    def _rover_cb(self, msg):
        self.rovr_pose = msg.pose.position

    def launch(self):
        self.rate.sleep()
        self._setmode_offb()
        while not rospy.is_shutdown():
            self._goto_rover()
            self._descent()


def main():
    rospy.init_node('ProbeMission')
    rospy.loginfo("[ProbeMission] initialised")
    mission_instance = Mission()
    mission_instance.launch()
    rospy.spin()


if __name__ == '__main__':
    main()
