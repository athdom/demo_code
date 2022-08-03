#!/usr/bin/env python3

"""
Start ROS node to implement Stiffness control by transforming forces and torques in the task space
into joint torques of the widowx arm through the arbotix controller.
"""

#Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from widowx_driver.srv import *
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import quaternion

#Arm parameters
#from widowx_arm import *
#widowx dynamics and kinematics class
from widowx_stiffness_matrices import WidowxStiffness
# from widowx_compute_dynamics import WidowxDynamics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class WidowxController():
    """Class to compute and pubblish joints torques"""
    def __init__(self):

        self.first_iter = True
        self.pos_valid = False
        self.vel_valid = False
        self.target_valid = False
        #Init widowx dynamics and kinematics handler
        self.wd = WidowxStiffness()

        #Control gains matrices
        #position gains P term
        self.K_Pt = np.matrix([[12, 0, 0],  [0, 4, 0], [0, 0, 25]]) #translation part
        self.K_Po = np.matrix([[1, 0, 0],  [0, 3, 0], [0, 0, 1]]) #orientation part
        # Scale with a specified factor

        trans_factor = 30
        self.K_Pt = trans_factor*self.K_Pt
        orien_factor = 2
        self.K_Po = orien_factor*self.K_Po


        #velocity gains D term
        self.K_D = np.matrix([[3, 0, 0, 0, 0, 0],  [0, 3, 0, 0, 0, 0], [0, 0, 3, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0]])
        vel_factor =0.5
        self.K_D = vel_factor*self.K_D

        # Declaration of desired pose
        self.desired_pose = PoseStamped()

        self.desired_pose.header.seq = 1
        self.desired_pose.header.stamp = rospy.Time.now()
        self.desired_pose.header.frame_id = "base_footprint"

        self.desired_pose.pose.position.x = 0.0
        self.desired_pose.pose.position.y = 0.0
        self.desired_pose.pose.position.z = 0.0

        self.desired_pose.pose.orientation.x = 0.0
        self.desired_pose.pose.orientation.y = 0.0
        self.desired_pose.pose.orientation.z = 0.0
        self.desired_pose.pose.orientation.w = 1.0

        # Define robot pose for visualization purposes
        self.robot_pose = PoseStamped()

        self.robot_pose.header.seq = 1
        self.robot_pose.header.stamp = rospy.Time.now()
        self.robot_pose.header.frame_id = "base_footprint"

        self.robot_pose.pose.position.x = 0.0
        self.robot_pose.pose.position.y = 0.0
        self.robot_pose.pose.position.z = 0.0

        self.robot_pose.pose.orientation.x = 0.0
        self.robot_pose.pose.orientation.y = 0.0
        self.robot_pose.pose.orientation.z = 0.0
        self.robot_pose.pose.orientation.w = 1.0

        #Initialize desired quaternion
        self.desired_rotQuat = np.quaternion(1,0,0,0)

        #Initialize desired rotation matrix
        self.desired_rotMat = quaternion.as_rotation_matrix(self.desired_rotQuat)

        #ROS SETUP
        #initialize pose, velocity listeners and torques publisher
        #Robot
        self.robot_position_sub = rospy.Subscriber('/widowx/joints_positions', Float32MultiArray, self._robot_position_callback, queue_size=1)
        self.robot_vel_sub = rospy.Subscriber('/widowx/joints_vels', Float32MultiArray, self._robot_vel_callback, queue_size=1)
        self.robot_torque_pub = rospy.Publisher('/widowx/desired_torques', Float32MultiArray, queue_size=1)
        self.robot_visPose_pub = rospy.Publisher('/widowx_pose', PoseStamped, queue_size=1)

        #Trajectory listener
        self.target_sub = rospy.Subscriber('/desired_pose', PoseStamped, self._target_callback, queue_size=1)
        #Signal check publisher
        # self.errors_pub = rospy.Publisher('/control_signals', Float32MultiArray, queue_size=1)
        #Publishing rate
        rate = 100 ##TO BE SET IN ROS PARAM
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        #Initialize torque message
        self.torques = Float32MultiArray()
        self.torques_layout = MultiArrayDimension('control_torques', 6, 0)
        self.torques.layout.dim = [self.torques_layout]
        self.torques.layout.data_offset = 0

        #Robot
        self.robot_joints_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_joints_vels =  [0.0, 0.0, 0.0, 0.0, 0.0]

        #Security signal services
        print("\nChecking security-stop service availability ... ...")
        rospy.wait_for_service('/widowx/security_stop')
        print("robot: security-stop ok ...")

        self.robot_sec_stop = rospy.ServiceProxy('/widowx/security_stop', SecurityStop)

        print("\nWidowX controller node created")
        print("\nWaiting for target Cartesian space pose...")

        self.compute_torques()

    #SENSING CALLBACKS
    def _robot_position_callback(self, msg):
        """
        ROS callback to get the joint poses
        """
        self.robot_joints_pos = msg.data
        self.pos_valid = True

    def _robot_vel_callback(self, msg):
        """
        ROS callback to get the joint velocities
        """
        self.robot_joints_vels = msg.data
        self.vel_valid = True

    def _target_callback(self, msg):
        """
        ROS callback to get the target pose
        """
        self.desired_pose = msg
        #Update desired numpy quaternion in class
        self.desired_rotQuat.w = self.desired_pose.pose.orientation.w
        self.desired_rotQuat.x = self.desired_pose.pose.orientation.x
        self.desired_rotQuat.y = self.desired_pose.pose.orientation.y
        self.desired_rotQuat.z = self.desired_pose.pose.orientation.z

        #Update desired rotation matrix in class
        self.desired_rotMat = quaternion.as_rotation_matrix(self.desired_rotQuat)
        self.target_valid = True

    #CONTROLLER
    def compute_torques(self):
        """
        Compute and pubblish torques values for all joints
        """

        while not rospy.is_shutdown():
            # Store joints state and desired position for the object
            robot_array_vels = np.asarray(self.robot_joints_vels)[np.newaxis].T
            robot_array_positions = np.asarray(self.robot_joints_pos)[np.newaxis].T

            # Compute ee position from joints_positions
            robot_ee_position = self.wd.compute_ee_pos(self.robot_joints_pos)

            # Compute ee orientation as rotation matrix Re
            robot_ee_rotMat = self.wd.compute_ee_orientation(self.robot_joints_pos)

            robot_ee_quat = quaternion.from_rotation_matrix(robot_ee_rotMat)

            # Compute Jacobian and ee velocities from joints_vels
            robot_J_e = self.wd.compute_jacobian(robot_array_positions)
            robot_ee_velocities = np.dot(robot_J_e, robot_array_vels)

            # If desired pose has set as input
            if self.target_valid:
                # Express ee desired position as numpy array
                desired_ee_position = np.array([[self.desired_pose.pose.position.x], [self.desired_pose.pose.position.y], [self.desired_pose.pose.position.z]])
            else:
                desired_ee_position = robot_ee_position
                self.desired_rotQuat = robot_ee_quat
                self.desired_rotMat = robot_ee_rotMat

            '''
            ###################################
            Define the operational space ERRORS
            ###################################
            '''
            # Translation error
            position_error = desired_ee_position - robot_ee_position
            # Orientation error
            orientation_error_mat = np.dot(np.transpose(robot_ee_rotMat) , self.desired_rotMat)
            orientation_error = quaternion.from_rotation_matrix(orientation_error_mat)

            print("position_error = ")
            print(position_error)
            print("orientation_error = ")
            print(orientation_error)

            '''
            ##########################
            Compute translation wrench
            ##########################
            '''
            # Initially compute gains for geometrically consistent approach
            K_Pt_prime = np.dot(np.dot(self.desired_rotMat,self.K_Pt), self.desired_rotMat.T)/2 + np.dot(np.dot(robot_ee_rotMat,self.K_Pt),robot_ee_rotMat.T)/2
            # Inline computation of Skew symmetric matrix
            Skew_Perr = np.matrix([[0, -position_error[2,0], position_error[1,0]],[position_error[2,0], 0, -position_error[0,0]],[-position_error[1,0], position_error[0,0], 0]])
            K_Pt_dprime = (Skew_Perr*self.desired_rotMat*self.K_Pt*self.desired_rotMat.T)/2

            # Translation force
            f_trans = np.dot(K_Pt_prime , position_error)
            # Translation torque
            t_trans = np.dot(K_Pt_dprime , position_error)

            # Translation wrench (concatenate f_trans with the first two elements of t_trans for 5 DOF robot)
            h_trans = np.array([[f_trans[0, 0]], [f_trans[1, 0]], [f_trans[2, 0]], [t_trans[0,0]], [t_trans[1,0]], [t_trans[2,0]]])
            # h_trans = np.array([[f_trans[0, 0]], [f_trans[1, 0]], [f_trans[2, 0]], [0.0], [0.0]])

            '''
            ##########################
            Compute Orientation wrench
            ##########################
            '''
            # Initially compute gains for geometrically consistent approach
            # For 5DOF robot make orientation error invariant of q0
            zero_joint_pos = [0.0, robot_array_positions[1,0], robot_array_positions[2,0], robot_array_positions[3,0], robot_array_positions[4,0]]
            zero_ee_rotMat = self.wd.compute_ee_orientation(zero_joint_pos)
            zero_robot_J_e = self.wd.compute_zero_jacobian(robot_array_positions)
            zero_robot_J_e[4,1] = 0.0
            # print("zero_robot_J_e = ")
            # print(zero_robot_J_e )


            oriError2base = np.dot(zero_ee_rotMat.T, np.array([[orientation_error.x],[orientation_error.y],[orientation_error.z]]))
            # print("oriError2base = ")
            # print(oriError2base)
            # Inline computation of Skew symmetric matrix
            Skew_Oerr = np.matrix([[0, -oriError2base[2, 0], oriError2base[1, 0]],[oriError2base[2, 0], 0, -oriError2base[0, 0]],[-oriError2base[1, 0], oriError2base[0, 0], 0]])
            Epsilon = orientation_error.w*(np.identity(3)-Skew_Oerr)
            K_Po_prime = 2*np.dot(np.dot(Epsilon.T,robot_ee_rotMat),np.dot(self.K_Po,robot_ee_rotMat.T))
            # Orientation force
            f_orien = np.array([[0],[0],[0]])
            # Orientation torque
            t_orien = np.dot(K_Po_prime, np.array([[oriError2base[0, 0]],[oriError2base[1, 0]],[oriError2base[2, 0]]]))
            # Orientation wrench (concatenate f_trans with the first two elements of t_trans for 5 DOF robot)
            h_orien = np.array([[f_orien[0, 0]], [f_orien[1, 0]], [f_orien[2, 0]], [t_orien[0,0]], [t_orien[1,0]], [0]]) #,t_orien[0,0]],[t_orien[1,0]]
            # print("h_orien = ")
            # print(h_orien)
            # Wrench caused by the D term of the controller
            h_vel = -np.dot(self.K_D, robot_ee_velocities)

            # Transform wrench to joint torques for 5DOF robot make orientation wrench invariant of q0
            u_total = np.dot(robot_J_e.T, h_trans) + np.dot(zero_robot_J_e.T, h_orien) + np.dot(robot_J_e.T, h_vel)

            # # Compute the gravity term in joint space
            u_gravity = self.wd.compute_g(robot_array_positions)
            #u_gravity=0
            control_torque = u_total + u_gravity
            direction = np.sign(control_torque)

            print("Control torque = ")
            print(control_torque)

            #Create ROS message
            # if  norm(control_torque) < 5:
                # Create ROS message
            self.torques.data = [control_torque[0,0], control_torque[1,0], control_torque[2,0], control_torque[3,0], control_torque[4,0], direction[0,0], direction[1,0], direction[2,0], direction[3,0], direction[4,0]]
            self.robot_torque_pub.publish(self.torques)
            # else:
            #     print("There's a problem with the torques")
            #     control_torque = u_gravity
            #     direction = np.sign(control_torque)
            #     self.torques.data = [control_torque[0,0], control_torque[1,0], control_torque[2,0], control_torque[3,0], control_torque[4,0], direction[0,0], direction[1,0], direction[2,0], direction[3,0], direction[4,0]]
            #     self.robot_torque_pub.publish(self.torques)

            # Publish robot end-effector pose for visualization purposes
            self.robot_pose.header.stamp = rospy.Time.now()
            self.robot_pose.pose.position.x = robot_ee_position[0]
            self.robot_pose.pose.position.y = robot_ee_position[1]
            self.robot_pose.pose.position.z = robot_ee_position[2]

            self.robot_pose.pose.orientation.x = robot_ee_quat.x
            self.robot_pose.pose.orientation.y = robot_ee_quat.y
            self.robot_pose.pose.orientation.z = robot_ee_quat.z
            self.robot_pose.pose.orientation.w = robot_ee_quat.w
            self.robot_visPose_pub.publish(self.robot_pose)

            self.pub_rate.sleep()




if __name__ == '__main__':
    #Iitialize the node
    rospy.init_node('widowx_stiffeness_controller')
    #Create widowx controller object
    wc = WidowxController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS WidowX controller node")
