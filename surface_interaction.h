#include <ros/ros.h>
#include <ros/spinner.h>

#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/UInt8MultiArray.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Array.h>

#include "reference_frames.h"
#include "image_frames.h"
#include "object_frames.h"
#include <tf/transform_listener.h>


#define PI 3.14159265359

class surface_interaction_Class{
public:
	ros::NodeHandle surface_interaction_node;
	ros::AsyncSpinner spinner;

	//Publishers and Transformations declarations
	//ros::ServiceClient online_pose_client, client;
	ros::Publisher pub, transformed_pub, marker_pub, object_pub, ROI_pub, poly_pub, target_position_pub, pose_pub, nf_pub, imagesp_pub;
	ros::Subscriber sub, segmentation_sub,segmentation_image_sub,  canonical_sub, current_position_sub, camera_info_sub;
  	//tf::TransformListener tf_listener_;
	//tf::StampedTransform base_to_kinect;
	tf::Transform base_to_camera, transform_to_publish, camera_to_virtual, virtual_to_surface, camera_to_surface;

	geometry_msgs::Point target_position_msg;
	image_geometry::PinholeCameraModel model_;

	//PointClouds required for visualization
	pcl::PointCloud<pcl::PointXYZRGB> traj_cloud, polygon_cloud, nf_cloud, imagesp_cloud;
	pcl::PointXYZRGB vertice;
	//The last valid PointCloud
	sensor_msgs::PointCloud2 last_cloud, tmp_cloud, transformed_cloud;
	sensor_msgs::CameraInfo camera_info;
	std_msgs::UInt8MultiArray last_segmentation, transformed_image, tmp_image;
	uint pixel_label;

	std::vector< Eigen::Vector3f > region_points;
	Eigen::Vector3f sample3d, image_point, object_point, virtual_camera_pos;
	Eigen::Vector2f sample;
	int reference_x, reference_y, window_size_y,  window_size_x, window_size_step , pixel_offset_x, pixel_offset_y;
  	Eigen::Vector3f point, ux, uy, uz, xz_projection, yz_projection, xy_projection, object_xz_projection, object_yz_projection, object_xy_projection, target_position, current_position;
  	double pitch, roll, yaw, object_pitch, object_roll, object_yaw;

  	//Required parameters for initialization
  	std::string input_camera_topic, input_camera_info, segmentation_topic, segmentation_image_topic, planar_reference_topic, reached_target_topic, robot_base_frame,
				camera_frame, robot_tip_frame, task_description, current_position_topic;
  	float distance_from_body, distance_from_object;
  	uint segmentation_counter, cloud_counter , maximum_counter;

  	//============================================================================================================================================//
  	//Image controller related declarations
 	Eigen::Vector2f goal, temp_goal, controller_output, scaled_extends, scaled_extends_inv, visualize_current;
 	Eigen::Vector2f output, time_projection, image_centre, roi_centre, tmp_rotated, grad_workspace, grad_ROI, grad_gamma, grad_phi, obstacle_total_grad;
  	std::vector<Eigen::Vector2f> grad_obstacle;
  	float dt, dt_time_proj, gamma_fun, workspace, ROI, sigma_time, sigma_phi, phi, time_grad_phi, epsilon;
  	std::vector<float> obstacle_beta;
  	float kappa, kappa_inv, exponent;
  	bool first_time;

  	//============================================================================================================================================//


  	std::vector< Eigen::Vector3f > valid_pixels, object_points;
  	ImagePlane region_of_interest;
  	ObjectFrame object_frame;
  	std::vector <ImagePlane> obstacles;

  	//Surface Circular Adaptation
  	Eigen::Vector3f image_point_previous, image_point_next;
  	Eigen::Vector3f surface_sample_previous, surface_sample_next, point_to_save, point_to_meet;
  	std::vector<Eigen::Vector3f> surface_samples;
  	float total_surf_distance, t, t_step, multiplier;
  	bool first_horizontal, first_vertical;


  	surface_interaction_Class();
  	~surface_interaction_Class();

  	void cloud_cb (const sensor_msgs::PointCloud2Ptr& cloud_msg);

  	void camera_info_cb (const sensor_msgs::CameraInfoConstPtr&);
  	void segmentation_cb (const std_msgs::UInt8MultiArray& segmentation);
  	void object_frame_cb (const sensor_msgs::Image & segmentation_image);
	void task_reference_cb (const geometry_msgs::Point& planar_point);

	void current_position_cb (const geometry_msgs::PointStamped& current_position_msg);

	void image_controller();
	void visualize_nf();
	void canonical_controller();
};
