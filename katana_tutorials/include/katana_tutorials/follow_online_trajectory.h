/*
 * follow_online_trajectory.h
 *
 *  Created on: 07.2016
 *      Author: Athanasios Dometios
 */

#ifndef FOLLOW_JOINT_TRAJECTORY_CLIENT_H_
#define FOLLOW_JOINT_TRAJECTORY_CLIENT_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <image_trajectory/online_pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


namespace katana_tutorials
{

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class FollowJointTrajectoryClient
{
public:
  FollowJointTrajectoryClient();
  virtual ~FollowJointTrajectoryClient();
  ros::NodeHandle online_traj;

  bool startTrajectory(geometry_msgs::PoseStamped& req);
  control_msgs::FollowJointTrajectoryGoal makeArmUpTrajectory(geometry_msgs::PoseStamped &req);
  actionlib::SimpleClientGoalState getState();
  control_msgs::FollowJointTrajectoryGoal goal;

  ros::ServiceServer online_pose_server;

  ros::ServiceClient get_kin_info;
  ros::ServiceClient get_ik_client;
  ros::ServiceClient get_fk_client;
  moveit_msgs::GetKinematicSolverInfo kin_info;
  moveit_msgs::GetPositionIK ik_srv;
  moveit_msgs::GetPositionFK fk_srv;
  geometry_msgs::PoseStamped requested_pose;
  float roll, pitch, yaw;

private:

  TrajClient traj_client_;
  ros::Subscriber joint_state_sub_;
  std::vector<std::string> joint_names_;
  bool got_joint_state_;
  std::vector<double> current_joint_state_;
  ros::AsyncSpinner spinner_;

  void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
};

} /* namespace katana_tutorials */
#endif /* FOLLOW_JOINT_TRAJECTORY_CLIENT_H_ */
