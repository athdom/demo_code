#!/usr/bin/env python3

"""
Start ROS node to implement position control of the widowx arm through the arbotix controller.
"""

#Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped 
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
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

        self.group_name = "widowx_arm"
        self.ik_attempts = 100
        self.ik_timeout = 1.0
        self.avoid_collisions = False
        self.ik_link_name = "gripper_rail_link";
        self.ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.current_state = RobotState()
        # self.current_joint_state = JointState()
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
        #Initialize desired quaternion
        self.desired_rotQuat = np.quaternion(1,0,0,0)

        #Initialize desired rotation matrix
        self.desired_rotMat = quaternion.as_rotation_matrix(self.desired_rotQuat)
        self.robot_ee_rotMat = quaternion.as_rotation_matrix(self.desired_rotQuat)

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

        
        #ROS SETUP
        #initialize pose, velocity listeners and torques publisher
        #Robot
        self.robot_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        self.robot_position_sub = rospy.Subscriber('/widowx/joints_positions', Float32MultiArray, self._robot_position_callback, queue_size=1)
        self.robot_vel_sub = rospy.Subscriber('/widowx/joints_vels', Float32MultiArray, self._robot_vel_callback, queue_size=1)
        # self.robot_torque_pub = rospy.Publisher('/widowx/desired_torques', Float32MultiArray, queue_size=1)
        self.robot_visPose_pub = rospy.Publisher('/widowx_pose', PoseStamped, queue_size=1)
        self.desired_position_pub = rospy.Publisher('/widowx'+'/desired_joint_positions', Float32MultiArray, queue_size=1)

        #Trajectory listener
        self.target_sub = rospy.Subscriber('/desired_pose', PoseStamped, self._target_callback, queue_size=1)

        #Publishing rate
        rate = 100 ##TO BE SET IN ROS PARAM
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        #Robot
        self.robot_joints_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.robot_joints_vels =  [0.0, 0.0, 0.0, 0.0, 0.0]

        self.desired_position2pub = Float32MultiArray()
        self.desired_position_layout = MultiArrayDimension('desired_joints', 5, 0)
        self.desired_position2pub.layout.dim = [self.desired_position_layout]
        self.desired_position2pub.layout.data_offset = 0

        #Security signal services
        print("\nChecking security-stop service availability ... ...")
        rospy.wait_for_service('/widowx/security_stop')
        print("robot: security-stop ok ...")

        self.robot_sec_stop = rospy.ServiceProxy('/widowx/security_stop', SecurityStop)

        print("\nWidowX controller node created")
        print("\nWaiting for target Cartesian space pose...")

        self.publish()

    #SENSING CALLBACKS
    def joint_state_callback(self, msg):
        """
        ROS callback to get the joint_state
        """
        self.current_state.joint_state = msg


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
        self.desired_rotQuat.w = self.desired_pose.pose.orientation.w
        self.desired_rotQuat.x = self.desired_pose.pose.orientation.x
        self.desired_rotQuat.y = self.desired_pose.pose.orientation.y
        self.desired_rotQuat.z = self.desired_pose.pose.orientation.z
        


        euler = quaternion.as_euler_angles(self.desired_rotQuat)
        ik_response = self.get_ik()
        self.desired_position2pub.data = list(ik_response.solution.joint_state.position)
        self.desired_position2pub.data[4] = euler[2]
        # print("self.desired_position2pub.data = ")
        # print (self.desired_position2pub.data)
        self.desired_position_pub.publish(self.desired_position2pub)


        # #Update desired rotation matrix in class
        # self.desired_rotMat = quaternion.as_rotation_matrix(self.desired_rotQuat)
        self.target_valid = True
    
    def publish(self):
        while not rospy.is_shutdown():

            # Compute ee position from joints_positions
            robot_ee_position = self.wd.compute_ee_pos(self.robot_joints_pos)

            # Compute ee orientation as rotation matrix Re
            self.robot_ee_rotMat = self.wd.compute_ee_orientation(self.robot_joints_pos)

            robot_ee_quat = quaternion.from_rotation_matrix(self.robot_ee_rotMat)

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


    def get_ik(self, pose_stamped=None,
                ik_link_name=None,
               robot_state=None,
               group=None,
               ik_timeout=None,
               ik_attempts=None,
               avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """

        if pose_stamped is None:
            pose_stamped = self.desired_pose
        if group is None:
            group = self.group_name
        if robot_state is None:
            robot_state = self.current_state
        if ik_link_name is None:
            ik_link_name = self.ik_link_name

        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        
        #Correction to the pose to compensate the inverse kinematics errors
        pose_stamped.pose.position.x -= self.robot_ee_rotMat[0,2] * 0.043
        pose_stamped.pose.position.y -= self.robot_ee_rotMat[1,2] * 0.043
        pose_stamped.pose.position.z -= self.robot_ee_rotMat[2,2] * 0.043

        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.robot_state = robot_state
        req.ik_request.ik_link_name = ik_link_name

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            # rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp

    
if __name__ == '__main__':
    #Iitialize the node
    rospy.init_node('widowx_stiffeness_controller')
    #Create widowx controller object
    wc = WidowxController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS WidowX controller node")
