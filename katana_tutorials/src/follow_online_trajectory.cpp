/*
 * follow_online_trajectory.cpp
 *
 *  Created on: 07.2016
 *      Author: Athanasios Dometios
 */

#include <katana_tutorials/follow_online_trajectory.h>

namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    		traj_client_("/katana_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(1)
{
	//Initialize joint names used for kinematics
	joint_names_.push_back("katana_motor1_pan_joint");
	joint_names_.push_back("katana_motor2_lift_joint");
	joint_names_.push_back("katana_motor3_lift_joint");
	joint_names_.push_back("katana_motor4_lift_joint");
	joint_names_.push_back("katana_motor5_wrist_roll_joint");

	//Set up subscribers and service clients
	joint_state_sub_ = online_traj.subscribe("/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
	get_kin_info = online_traj.serviceClient<moveit_msgs::GetKinematicSolverInfo>("get_kinematic_solver_info");
	get_ik_client = online_traj.serviceClient<moveit_msgs::GetPositionIK>("get_ik");
	get_fk_client = online_traj.serviceClient<moveit_msgs::GetPositionFK>("get_fk");
	//online_pose_server = online_traj.advertiseService("follow_online_pose", &FollowJointTrajectoryClient::startTrajectory, this);

	//initialize target pose from yaml file and generate Pose msg
	online_traj.getParam("position_x", requested_pose.pose.position.x);
	online_traj.getParam("position_y", requested_pose.pose.position.y);
	online_traj.getParam("position_z", requested_pose.pose.position.z);

	online_traj.getParam("roll", roll);
	online_traj.getParam("pitch", pitch);
	online_traj.getParam("yaw", yaw);

	requested_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	requested_pose.header.frame_id = "katana_base_frame";
	requested_pose.header.stamp = ros::Time(0);
	requested_pose.header.seq =0;


	spinner_.start();

	// wait for action server to come up
	while (!traj_client_.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the follow_joint_trajectory server");
	}

	FollowJointTrajectoryClient::startTrajectory(requested_pose);
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

// Joint State callback. Ensure joint name ordering in input data.  
void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
	std::vector<double> ordered_js;

	ordered_js.resize(joint_names_.size());

	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		bool found = false;
		for (size_t j = 0; j < msg->name.size(); ++j)
		{
			if (joint_names_[i] == msg->name[j])
			{
				ordered_js[i] = msg->position[j];
				found = true;
				break;
			}
		}
		if (!found)
			return;
	}

	ROS_INFO_ONCE("Got joint state!");
	current_joint_state_ = ordered_js;
	got_joint_state_ = true; //Got_joint_state is important to ensure communication with the robots
}

//! Sends the command to start a given trajectory
bool FollowJointTrajectoryClient::startTrajectory(geometry_msgs::PoseStamped &req)
{
	// Generate current and goal poses
	goal = makeArmUpTrajectory(req);
	// When to start the trajectory: 1s from now
	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);
	traj_client_.sendGoal(goal);
	traj_client_.waitForResult(ros::Duration(10.0));

	std::cout << "Finished in state = " << traj_client_.getState().toString().c_str() << std::endl;
	std::cout << "Answer code: " << traj_client_.getResult()->error_code << std::endl;
	return true;
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory(geometry_msgs::PoseStamped &req)
{
	const size_t NUM_TRAJ_POINTS = 2;
	const size_t NUM_JOINTS = 5;

	std::vector<double> next_position(NUM_JOINTS);

	for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
	{
		ROS_DEBUG("waiting for joint state...");

		if (!ros::ok())
			exit(-1);
	}


	if(get_kin_info.call(kin_info.request, kin_info.response)){
		std::cout << "Got solver info" << std::endl;
	}

	// Calculate current pose from forware kinematics
	fk_srv.request.header.stamp = ros::Time(0);
	fk_srv.request.header.seq++;
	fk_srv.request.header.frame_id = "katana_base_frame";
	fk_srv.request.robot_state.joint_state.name.resize(kin_info.response.kinematic_solver_info.joint_names.size());
	fk_srv.request.robot_state.joint_state.name = kin_info.response.kinematic_solver_info.joint_names;
	fk_srv.request.fk_link_names.push_back("katana_gripper_tool_frame");
	fk_srv.request.robot_state.joint_state.position = current_joint_state_;

	if (get_fk_client.call(fk_srv)){
		std::cout <<"Current pose = " << fk_srv.response.pose_stamped[0] << std::endl;
	}else{
		std::cout <<"No service answer forward kinematics" << std::endl;
	}

	std::cout << "Current joint position = " << current_joint_state_[0] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[1] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[2] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[3] << std::endl;
	std::cout << "Current joint position = " << current_joint_state_[4] << std::endl;

	// Calculate joint values for desired pose using Inverse kinematics
	ik_srv.request.ik_request.robot_state.joint_state.position = current_joint_state_; // Provide current joint state to ik solver to accelerate solution calculation
	ik_srv.request.ik_request.robot_state.joint_state.name.resize(kin_info.response.kinematic_solver_info.joint_names.size());
	ik_srv.request.ik_request.robot_state.joint_state.name = kin_info.response.kinematic_solver_info.joint_names;
	ik_srv.request.ik_request.ik_link_name = "katana_gripper_tool_frame";
	ik_srv.request.ik_request.pose_stamped = req;

	std::cout << "Requested position = " << req.pose << std::endl;

	if (get_ik_client.call(ik_srv)){
		if(ik_srv.response.solution.joint_state.position.size()>0){
			std::cout << "Requested joint position = " << ik_srv.response.solution.joint_state.position[0] << std::endl;
			std::cout << "Requested joint position = " << ik_srv.response.solution.joint_state.position[1] << std::endl;
			std::cout << "Requested joint position = " << ik_srv.response.solution.joint_state.position[2] << std::endl;
			std::cout << "Requested joint position = " << ik_srv.response.solution.joint_state.position[3] << std::endl;
			std::cout << "Requested joint position = " << ik_srv.response.solution.joint_state.position[4] << std::endl;
		}else{
			std::cout << "Requested joint position vector is empty.. "  << std::endl;
		}

		next_position = ik_srv.response.solution.joint_state.position;
	}else{
		std::cout <<"No service answer inverse kinematics" << std::endl;
		std::cout <<"exit code" << ik_srv.response.error_code << std::endl;
	}

	trajectory_msgs::JointTrajectory trajectory;

	// Apply joint names to all waypoints
	trajectory.joint_names = joint_names_;
	trajectory.points.resize(NUM_TRAJ_POINTS);

	// trajectory point:
	int ind = 0;
	trajectory.points[ind].time_from_start = ros::Duration(ind);
	trajectory.points[ind].positions = current_joint_state_;

	// trajectory point
	ind++;
	trajectory.points[ind].time_from_start = ros::Duration(5*ind);
	trajectory.points[ind].positions = next_position;

	//  // all Velocities 0
	//  for (size_t i = 0; i < NUM_TRAJ_POINTS; ++i)
	//  {
	//    trajectory.points[i].velocities.resize(NUM_JOINTS);
	//    for (size_t j = 0; j < NUM_JOINTS; ++j)
	//    {
	//      trajectory.points[i].velocities[j] = 0.0;
	//    }
	//  }
	got_joint_state_ = false; //THIS MIGHT CAUSE PROBLEMS
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory = trajectory;
	return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
	return traj_client_.getState();
}

} /* namespace katana_tutorials */

int main(int argc, char** argv)
{

	// Init the ROS node
	ros::init(argc, argv, "follow_online_trajectory_server");

	katana_tutorials::FollowJointTrajectoryClient arm;
	// Start the trajectory
	ros::spinOnce();

	return 0;
}

