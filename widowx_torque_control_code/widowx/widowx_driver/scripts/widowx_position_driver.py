#!/usr/bin/env python

"""
Start ROS node to implement position driver of the widowx arm through the arbotix controller.
"""

import rospy, roslib
import operator
import time
from math import pi
import numpy as np
from arbotix_python.arbotix import ArbotiX
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import JointState
from servos_parameters import *
from widowx_driver.srv import *

class WidowxNode(ArbotiX):
    """Node to control in torque the dynamixel servos"""
    def __init__(self, serial_port, baud, robot_name):
        #Initialize arbotix comunications
        print"\nArbotix initialization for " + robot_name + ", wait 10 seconds..."
        print"Serial port = " + serial_port
        ArbotiX.__init__(self, port=serial_port, baud = baud)
        for x in xrange(1,11):
            time.sleep(0.5)
            print(str(x*0.5) + "/5s for " + robot_name)
            if rospy.is_shutdown():
                break
        print robot_name + " Done."

        #reset vel limit
        print"Reset max vels for " + robot_name
        max_rpm = 10.0
        max_rad_s = max_rpm * 2*pi/60
        max_speed_steps = int(max_rpm/MX_VEL_UNIT)
        ax_max_speed_steps = int(max_rpm/AX_VEL_UNIT)
        self.setSpeed(1, max_speed_steps)
        self.setSpeed(2, max_speed_steps)
        self.setSpeed(3, max_speed_steps)
        self.setSpeed(4, max_speed_steps)
        self.setSpeed(5, ax_max_speed_steps)
        # #Disable Torque control 
        self.disableTorqueControl(2)
        self.disableTorqueControl(3)
        self.disableTorqueControl(4)

        # #reset PID values
        internal_PID = [32, 0, 0]
        self.setPID(1, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(2, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(3, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(4, internal_PID[0], internal_PID[1], internal_PID[2])

    #     #Set inital torque limits
        print"Limiting torques for " + robot_name
        mx28_init_torque_limit = int(MX_TORQUE_STEPS/3)
        mx64_init_torque_limit = int(MX_TORQUE_STEPS/5)
        ax_init_torque_limit = int(AX_TORQUE_STEPS/4)
        max_torque_msg = [[1, mx28_init_torque_limit], [2, mx64_init_torque_limit], [3, mx64_init_torque_limit], [4, mx64_init_torque_limit], [5, ax_init_torque_limit], [6,ax_init_torque_limit]]
        #Setup position msg
        init_pos_1 = rospy.get_param(rospy.get_name() + "/init_pos_1", MX_POS_CENTER)
        init_pos_2 = rospy.get_param(rospy.get_name() + "/init_pos_2", 1710)
        init_pos_3 = rospy.get_param(rospy.get_name() + "/init_pos_3", 1577)
        init_pos_4 = rospy.get_param(rospy.get_name() + "/init_pos_4", 2170)
        init_pos_5 = rospy.get_param(rospy.get_name() + "/init_pos_5", 512)
        init_pos_6 = rospy.get_param(rospy.get_name() + "/init_pos_6", AX_POS_CENTER)
        pos_msg = [[1, int(init_pos_1)], [2, int(init_pos_2)], [3, int(init_pos_3)], [4, int(init_pos_4)], [5, int(init_pos_5)], [6,int(init_pos_6)]]
        #Move to inital position
        self.syncSetTorque(max_torque_msg, pos_msg)
        #Wait for movement
        time.sleep(3)

        mx28_init_torque_limit = int(MX_TORQUE_STEPS/2)
        mx64_init_torque_limit = int(MX_TORQUE_STEPS/2)
        ax_init_torque_limit = int(AX_TORQUE_STEPS/2)
        self.max_torque_msg = [[1, mx28_init_torque_limit], [2, mx64_init_torque_limit], [3, mx64_init_torque_limit], [4, mx64_init_torque_limit], [5, ax_init_torque_limit]]


    #     #Close gripper
    #     # print(robot_name + ": Closing grippers")
    #     # self.setPosition(int(6), 50)

        #Limit joints velocities
        # max_rpm = rospy.get_param(rospy.get_name() + "/max_rpm", 1.5)
        # max_rad_s = max_rpm * 2*pi/60
        # print(robot_name + ": Limiting joints velocities at: "+ str(max_rpm) +"rpm = "+ str(max_rad_s) +"rad/s")
        # max_speed_steps = int(max_rpm/MX_VEL_UNIT)
        # ax_max_speed_steps = int(max_rpm/AX_VEL_UNIT)
        # self.setSpeed(1, max_speed_steps)
        # self.setSpeed(2, max_speed_steps)
        # self.setSpeed(3, max_speed_steps)
        # self.setSpeed(4, max_speed_steps)
        # self.setSpeed(5, ax_max_speed_steps)


        #Setup PID parameter

        print robot_name + " ready, setting up ROS topics..."

        #Setup velocities and positions vectors and messages
        self.joints_poses = [0,0,0,0,0,0]
        self.joints_vels = [0,0,0,0,0]
        self.ee_closed = 0
        self.vels_to_pub = Float32MultiArray()
        self.poses_to_pub = Float32MultiArray()
        self.poses_layout = MultiArrayDimension('joints_poses', 6, 0)
        self.vels_layout = MultiArrayDimension('joints_vels', 5, 0)
        self.poses_to_pub.layout.dim = [self.poses_layout]
        self.poses_to_pub.layout.data_offset = 0
        self.vels_to_pub.layout.dim = [self.vels_layout]
        self.vels_to_pub.layout.data_offset = 0

        # Setup velocity filter for motor 4
        self.previous_velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.second_previous_velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_diff = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity_factor = 3

        # Setup positions and velocities as joint states msg
        self.joint_states = JointState()
        self.joint_states.header.seq = 1
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.header.frame_id = ''

        self.joint_states.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0]

        #ROS pubblisher for joint velocities and positions
        self.pos_pub = rospy.Publisher('/widowx' +'/joints_positions', Float32MultiArray, queue_size=1)
        self.vel_pub = rospy.Publisher('/widowx' +'/joints_vels', Float32MultiArray, queue_size=1)
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.pub_rate = rospy.Rate(100)

        #ROS listener for control torues
        self.torque_sub = rospy.Subscriber('/widowx'+'/desired_joint_positions', Float32MultiArray, self._position_callback, queue_size=1)
        # self.gripper_sub = rospy.Subscriber('widowx_'+ robot_name +'/gripper', Bool, self._gripper_callback, queue_size=1)

        #ROS service for security stop
        self.sec_stop_server = rospy.Service('widowx'+ '/security_stop', SecurityStop, self._sec_stop)


        print"\nWidowx" + " node created, waiting for messages in:"
        print"/widowx" +"/desired_joint_positions"
        print"Publishing joints' positions and velocities in:"
        print '/widowx' +'/joints_positions'
        print '/widowx' +'/joints_vels'
        print"Security stop server running: widowx" + "/security_stop"
        #Start publisher
        self.publish()

    def _position_callback(self, msg):
        """
        ROS callback for torque
        """
        desired_positions_rad = msg.data
        desired_positions_steps = [0, 0, 0, 0, 0]
        # Calculate the desired posi
        desired_positions_steps[0] = (desired_positions_rad[0]/MX_POS_UNIT) + MX_POS_CENTER
        desired_positions_steps[1] = (desired_positions_rad[1]/MX_POS_UNIT) + MX_POS_CENTER
        desired_positions_steps[2] = MX_POS_CENTER - (desired_positions_rad[2]/MX_POS_UNIT)
        desired_positions_steps[3] = MX_POS_CENTER - (desired_positions_rad[3]/MX_POS_UNIT)
        desired_positions_steps[4] = (desired_positions_rad[4]/AX_POS_UNIT) + AX_POS_CENTER

        #Setup position msg
        pos_msg = [[1, int(desired_positions_steps[0])], [2, int(desired_positions_steps[1])], [3, int(desired_positions_steps[2])], [4, int(desired_positions_steps[3])], [5, int(desired_positions_steps[4])]]

        #Move to inital position
        self.syncSetTorque(self.max_torque_msg, pos_msg)


    def publish(self):
        rad_mx_step = (pi/30) * MX_VEL_UNIT
        rad_ax_step = (pi/30) * AX_VEL_UNIT
        while not rospy.is_shutdown():
            #MX-* servos poses
            #self.joints_poses[0] = MX_POS_UNIT * (self.getPosition(1) - MX_POS_CENTER)

            present_positions = self.syncGetPos([1, 2, 3, 4, 5])
            present_vels = self.syncGetVel([1, 2, 3, 4, 5])

            #Check if got good values for position and vels otherwise repeat the reading
            if not -1 in present_vels and not -1 in present_positions:

                if present_positions[0] < MX_POS_STEPS + 1:
                    self.joints_poses[0] = MX_POS_UNIT * (present_positions[0] - MX_POS_CENTER)
                if present_positions[1] < MX_POS_STEPS + 1:
                    self.joints_poses[1] = MX_POS_UNIT * (present_positions[1] - MX_POS_CENTER)
                if present_positions[2] < MX_POS_STEPS + 1:
                    self.joints_poses[2] = MX_POS_UNIT * (MX_POS_CENTER - present_positions[2])
                # if self.joints_poses[2] > -0.45:
                #     rospy.logerr(robot_name + ": Joint 2 near jacobian singularity. Shutting Down. Actual position: %frad, singularity in: -0.325rad", self.joints_poses[2])
                #     rospy.signal_shutdown(robot_name + ": Joint 2 near jacobian singularity.")
                # elif self.joints_poses[2] > -0.55: #I'm near the Jacobian sigularity => send warning
                #     rospy.logwarn(robot_name + ": Joint 2 is approaching the jacobian singularity (actual position: %frad, singularity in: -0.325rad): Move away from here.", self.joints_poses[2])
                if present_positions[3] < MX_POS_STEPS + 1:
                    self.joints_poses[3] = MX_POS_UNIT * (MX_POS_CENTER - present_positions[3])
                #AX 12 servos poses
                if present_positions[4] < AX_POS_STEPS:
                    self.joints_poses[4] = AX_POS_UNIT * (present_positions[4] - AX_POS_CENTER)


                #self.joints_poses[5] = self.ee_closed

                #MX-* servos vels
                for j in xrange(0,4):
                    if present_vels[j] < MX_VEL_CENTER:
                        self.joints_vels[j] = rad_mx_step * present_vels[j]
                    else:
                        if present_vels[j] < MX_VEL_STEPS:
                            self.joints_vels[j] = rad_mx_step * (MX_VEL_CENTER - present_vels[j])
                    # if abs(self.joints_vels[j]) > 0.1:
                    #     print(j)
                    #     print(self.joints_vels[j])
                    #     print(present_vels[j])

                #Invert second joint velocity sign
                self.joints_vels[2] = -1*self.joints_vels[2]
                self.joints_vels[3] = -1*self.joints_vels[3]
                #AX 12 servos vels
                actualax_step_speed = present_vels[4]
                # print("present_vels[4] = ")
                # print(present_vels[4])
                if actualax_step_speed < AX_VEL_CENTER:
                    self.joints_vels[4] = rad_ax_step * actualax_step_speed
                else:
                    self.joints_vels[4] = rad_ax_step * (AX_VEL_CENTER - actualax_step_speed)

                # Apply filtering in the velocity values
                for k in xrange(0,5):
                	self.velocity_diff[k] = self.previous_velocity[k] - self.second_previous_velocity[k]
                	if abs(self.joints_vels[k] - self.previous_velocity[k]) > self.velocity_factor * self.velocity_diff[k]:
                		self.joints_vels[k] = self.previous_velocity[k]

                self.poses_to_pub.data = self.joints_poses
                self.vels_to_pub.data = self.joints_vels
                self.pos_pub.publish(self.poses_to_pub)
                self.vel_pub.publish(self.vels_to_pub)
                # for k in xrange(0,5):
            	self.second_previous_velocity = list(self.previous_velocity)
            	self.previous_velocity = list(self.joints_vels)
        		

                self.joint_states.header.seq += 1
                self.joint_states.header.stamp = rospy.Time.now()

                self.joint_states.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
                self.joint_states.position = list(self.joints_poses[0:5])
                self.joint_states.velocity = list(self.joints_vels)
                # self.joint_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0]


                self.joint_states_pub.publish(self.joint_states)



                self.pub_rate.sleep()
            else:
                rospy.logwarn(robot_name + ": Lost packet at %fs", rospy.get_rostime().to_sec()) # If getting lost packets check return delay of servos or reduce publish rate for torques and/or joints vels and poses
                # self.pub_rate.sleep()
    # Security stop serice in case of robot malfunctining
    def _sec_stop(self, req):
        rospy.logerr(req.reason)
        rospy.signal_shutdown(req.reason)

    def turn_off_arm(self):
        """
        Disable all servos.
        """
        print robot_name + ": Disabling servos please wait..."
        mx28_init_torque_limit = int(MX_TORQUE_STEPS/3)
        mx64_init_torque_limit = int(MX_TORQUE_STEPS/5)
        ax_init_torque_limit = int(AX_TORQUE_STEPS/4)
        max_torque_msg = [[1, mx28_init_torque_limit], [2, mx64_init_torque_limit], [3, mx64_init_torque_limit], [4, mx64_init_torque_limit], [5, ax_init_torque_limit], [6,ax_init_torque_limit]]
        pos_msg = [[1, int(MX_POS_CENTER)], [2, int(1210)], [3, int(1700)], [4, int(1500)], [5, int(AX_POS_CENTER)], [6, int(AX_POS_CENTER)]]
        self.syncSetTorque(max_torque_msg, pos_msg)

        print robot_name + ": Servos disabled. Driver node closed."


if __name__ == '__main__':
    #Iitialize the node
    rospy.init_node("widowx_torque_control_driver")
    robot_name = rospy.get_param(rospy.get_name() + "/robot_name", "widowx")
    serial_port = rospy.get_param(rospy.get_name() + "/serial_port", "/dev/ttyUSB0")
    baud_rate = rospy.get_param(rospy.get_name() + "/baud_rate", "1000000")

    #Create widowx arm object
    wn = WidowxNode(serial_port, baud_rate, robot_name)
    #Handle shutdown
    rospy.on_shutdown(wn.tourn_off_arm)
    rospy.spin()
