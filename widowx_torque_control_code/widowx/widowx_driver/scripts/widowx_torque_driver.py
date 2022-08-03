#!/usr/bin/env python

"""
Start ROS node to implement torque driver of the widowx arm through the arbotix controller.
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
        #Initialize arbotix comunications with motors
        print"\nArbotix initialization for " + robot_name + ", wait 10 seconds..."
        print"Serial port = " + serial_port
        ArbotiX.__init__(self, port=serial_port, baud = baud)
        for x in xrange(1,11):
            time.sleep(0.5)
            print(str(x*0.5) + "/5s for " + robot_name)
            if rospy.is_shutdown():
                break
        print robot_name + " Done."

        #reset velocity limit
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

        # #reset motor PID values
        internal_PID = [32, 0, 0]
        self.setPID(1, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(2, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(3, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(4, internal_PID[0], internal_PID[1], internal_PID[2])
        # #Disable Torque control 
        self.disableTorqueControl(2)
        self.disableTorqueControl(3)
        self.disableTorqueControl(4)

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
        #Move to inital position with low level function in arbotix
        self.syncSetTorque(max_torque_msg, pos_msg) 
        #Wait for movement
        time.sleep(3)

        #Setup PID parameters
        internal_P = rospy.get_param(rospy.get_name() + "/internal_P", 8)
        internal_I = rospy.get_param(rospy.get_name() + "/internal_I", 0)
        internal_D = rospy.get_param(rospy.get_name() + "/internal_D", 0)
        internal_PID = [internal_P, internal_I, internal_D]
        print(robot_name + ": Setting servos position PID as: ")
        print(internal_PID)
        self.setPID(1, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(2, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(3, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(4, internal_PID[0], internal_PID[1], internal_PID[2])
        self.setPID(5, internal_PID[0], internal_PID[1], internal_PID[2])

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
        self.torque_sub = rospy.Subscriber('/widowx'+'/desired_torques', Float32MultiArray, self._torque_callback, queue_size=1)
        # self.gripper_sub = rospy.Subscriber('widowx_'+ robot_name +'/gripper', Bool, self._gripper_callback, queue_size=1)

        #Topic for checks and plotting
        self.check_pub = rospy.Publisher('/torque_check', Float32MultiArray, queue_size=1)
        #Initialize check message
        self.check = Float32MultiArray()
        self.check_layout = MultiArrayDimension('torque_check', 6, 0)
        self.check.layout.dim = [self.check_layout]
        self.check.layout.data_offset = 0

        #ROS service for security stop
        self.sec_stop_server = rospy.Service('widowx'+ '/security_stop', SecurityStop, self._sec_stop)

        #Frequency estimation for written torques
        self.cycle_count = 1
        self.freq_sum = 0
        self.iter_time = rospy.get_rostime()
        self.old_time = self.iter_time
        self.first_torque = True

        print"\nWidowx" + " node created, waiting for messages in:"
        print"/widowx" +"/desired_torques"
        print"Publishing joints' positions and velocities in:"
        print '/widowx' +'/joints_positions'
        print '/widowx' +'/joints_vels'
        print"Security stop server running: widowx" + "/security_stop"
        #Start publisher
        self.publish()

    def _compute_currents(self, torques, directions):
        """
        Compute currents from torques. The motors are (1:mx-28, 2:mx-106, 3:mx-106, 4:mx-64, 5:ax12 (using stall torque ratio with mx28 with the same current))
        """

        currents = [0.8567 * torques[0],\
                    0.611 * torques[1],\
                    0.611 * torques[2],\
                    0.933358 * torques[3],\
                    0.6* 0.8567 * torques[4]]
        # np.sign(torques[0])*(-0.0202)*(torques[0]**2) + 0.8777*torques[0] + self._gain(np.sign(directions[0]),np.sign(torques[0]))*0.0203, \
        #             np.sign(torques[1])*0.0410*(torques[1]**2) + 0.4715*torques[1] + self._gain(np.sign(directions[1]),np.sign(torques[1]))*0.0323, \
        #             np.sign(torques[2])*0.0410*(torques[2]**2) + 0.4715*torques[2] + self._gain(np.sign(directions[2]),np.sign(torques[2]))*0.0323, \
        #             np.sign(torques[3])*(-0.0230)*(torques[3]**2) + 0.9534*torques[3] + self._gain(np.sign(directions[3]),np.sign(torques[3]))*0.0215, \
        #             0.6*(np.sign(torques[4])*(-0.0202)*(torques[4]**2) + 0.8777*torques[4] + self._gain(np.sign(directions[4]),np.sign(torques[4]))*0.0203)]
                    # np.sign(torques[4])*(-0.01212)*(torques[4]**2) + 0.52662*torques[4] + self._gain(np.sign(directions[4]),np.sign(torques[4]))*0.01218]
                    # np.sign(torques[0])*(-0.0202)*(torques[0]**2) + 0.8777*torques[0] + self._gain(np.sign(directions[0]),np.sign(torques[0]))*0.0203]

        return currents

    def _gain(self, direc, tau):
        """
        Compute the gain for the costant term in current computation, arguments must be the signs of direction and torque
        """
        if direc == tau:
            return direc
        else:
            return 2*direc

    def _torque_callback(self, msg):
        """
        ROS callback for torque
        """
        #Initialize torque values.
        if self.first_torque:
            old_time = rospy.get_rostime()
            self.first_torque = False

        goal_current = self._compute_currents(msg.data[0:5], msg.data[5:10])
        goal_current_steps = [0,0,0,0,0] #size 5
        direction = [0,0,0,0,0]

        #Setup torque steps
        max1 = MX_TORQUE_STEPS/1.5
        max2 = MX_TORQUE_STEPS/1.5
        max3 = MX_TORQUE_STEPS/1.5
        max4 = MX_TORQUE_STEPS/1.5
        max5 = AX_TORQUE_STEPS/1.5
        goal_current_steps[0] = min(int(MX28_CURRENT_UNIT * abs(goal_current[0])), int(max1))
        goal_current_steps[1] = min(int(MX106_CURRENT_UNIT * abs(goal_current[1])), int(max2))
        goal_current_steps[2] = min(int(MX106_CURRENT_UNIT * abs(goal_current[2])), int(max3))
        goal_current_steps[3] = min(int(MX64_CURRENT_UNIT * abs(goal_current[3])), int(max4))
        goal_current_steps[4] = min(int(AX_CURRENT_UNIT * abs(goal_current[4])), int(max5))


        if goal_current_steps[0] == int(max1) or goal_current_steps[1] == int(max2) or goal_current_steps[2] == int(max3) or goal_current_steps[3] == int(max4) or goal_current_steps[4] == int(max5):
            print("\nWARNING, "+ robot_name +" MAX TORQUE LIMIT REACHED FOR ID: ")
            if goal_current_steps[0] == int(max1):
                print("1")
            if goal_current_steps[1] == int(max2):
                print("2")
            if goal_current_steps[2] == int(max3):
                print("3")
            if goal_current_steps[3] == int(max4):
                print("4")
            if goal_current_steps[4] == int(max5):
                print("5")
            print("goal_current:")
            print(goal_current)
            print("goal_current_steps:")
            print(goal_current_steps)
            max_torque_msg = [[1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, int(AX_TORQUE_STEPS/4)]]
            pos_msg = [[1, int(MX_POS_CENTER)], [2, int(1710)], [3, int(1577)], [4, int(2170)], [5, int(AX_POS_CENTER)], [6, int(AX_POS_CENTER)]]
            self.syncSetTorque(max_torque_msg, pos_msg)
            rospy.signal_shutdown(robot_name + ": MAX TORQUE LIMIT REACHED.")

        #Setup directions------FOR ID 2 THE DIRECTION IS INVERTED!!!!!!
        #ID 3 and 4
        for j in xrange(0,5):
            if goal_current[j] >= 0:
                direction[j] = MX_POS_STEPS - 10 #CCW
            else:
                direction[j] = 10 #CW
        # ID 2
        if goal_current[2] >= 0:
            direction[2] = 10 #CCW
        else:
            direction[2] = MX_POS_STEPS - 10 #CW

        if goal_current[3] >= 0:
            direction[3] = 10 #CCW
        else:
            direction[3] = MX_POS_STEPS - 10 #CW

        if goal_current[4] >= 0:
            direction[4] = 10 #CCW
        else:
            direction[4] = AX_POS_STEPS - 10 #CW

        # direction = msg.data
        # direction_msg = [[1, int(direction[0])], [2, int(direction[1])], [3, int(direction[2])], [4, int(direction[3])], [5, int(direction[4])]]
        torque_msg = [[1, goal_current_steps[0]], [2, goal_current_steps[1]], [3, goal_current_steps[2]], [4, goal_current_steps[3]], [5, goal_current_steps[4]]]

        direction_msg = [[1, int(direction[0])], [2, int(direction[1])], [3, int(direction[2])], [4, int(direction[3])], [5, int(direction[4])]]
        self.syncSetTorque(torque_msg, direction_msg)

        #####read present loads and confront with applied torques: ##########
        # present_load = [0,0,0]
        # for ID in xrange(2,5):
        #     load = self.getLoad(ID)
        #     if load > 1023:
        #         load = load-1024
        #     present_load[ID-2] = load

        # self.check.data = [goal_current_steps[0]/MX64_CURRENT_UNIT, goal_current_steps[1]/MX64_CURRENT_UNIT, goal_current_steps[2]/MX28_CURRENT_UNIT, present_load[0]/MX64_CURRENT_UNIT, present_load[1]/MX64_CURRENT_UNIT, present_load[2]/MX28_CURRENT_UNIT]
        # self.check_pub.publish(self.check)

        ####################################################################

 
    # def _gripper_callback(self, msg):
    #     """
    #     ROS callback
    #     """
    #     if msg.data:
    #         self.setPosition(int(6), 50)
    #     else:
    #         self.setPosition(int(6), AX_POS_CENTER)


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

                if actualax_step_speed < AX_VEL_CENTER:
                    self.joints_vels[4] = rad_ax_step * actualax_step_speed
                else:
                    self.joints_vels[4] = rad_ax_step * (AX_VEL_CENTER - actualax_step_speed)

                # Apply filtering in the velocity values
                for k in xrange(0,5):
                	self.velocity_diff[k] = self.previous_velocity[k] - self.second_previous_velocity[k]
                	if abs(self.joints_vels[k] - self.previous_velocity[k]) > self.velocity_factor * self.velocity_diff[k]:
                		self.joints_vels[k] = self.previous_velocity[k]

                self.poses_to_pub.data = list(self.joints_poses)
                self.vels_to_pub.data = list(self.joints_vels)
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
        max_torque_msg = [[1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, int(AX_TORQUE_STEPS/4)]]
        pos_msg = [[1, int(MX_POS_CENTER)], [2, int(1710)], [3, int(1577)], [4, int(2170)], [5, int(AX_POS_CENTER)], [6, int(AX_POS_CENTER)]]
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
