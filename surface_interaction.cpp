/*
* Implementation of an interactive planning algorithm over a moving surface perceived with an RGB-D camera. 
* The PointCloud data of an RGB-D camera are transformed to a virtual camera that is assumed perpedicular to the moving surface. 
* Restricted areas and obstacles perceived on the surface are avoided with a Navigation function controller implemented on the image plane.

*  Created on: 11.2017
*  Author: Athanasios Dometios
*/


#include <surface_interaction/surface_interaction.h>
bool debug = true;
bool visualize = true;

surface_interaction_Class::surface_interaction_Class():
																																spinner(5) //Threads for each call back

{
	//Required initializations
	reference_x = 0, reference_y = 0, window_size_y = 6, window_size_x = 0, window_size_step = 1, pixel_offset_x = 0, pixel_offset_y = 0;
	pitch = 0, roll = 0, yaw = 0;
	object_pitch = 0, object_roll = 0, object_yaw = 0;
	multiplier = 2.0;
	
	//Image space controller initialization
	dt = 10, dt_time_proj = 4, workspace = 0.0, gamma_fun = 0.0, sigma_time = 0.0,
			sigma_phi = 0.0, phi = 0.0, time_grad_phi = 0.0, epsilon = 0.01, ROI = 0, kappa = 1, kappa_inv = 1/kappa, first_time = true;

	surface_interaction_node.getParam("window_size",window_size_y);
	surface_interaction_node.getParam("robot_base_frame",robot_base_frame);
	surface_interaction_node.getParam("robot_tip_frame",robot_tip_frame);
	surface_interaction_node.getParam("camera_frame",camera_frame);
	surface_interaction_node.getParam("distance_from_body",distance_from_body);
	surface_interaction_node.getParam("distance_from_object",distance_from_object);


	// Unit vectors
	ux.x() = 1; uy.x() = 0; uz.x() = 0;
	ux.y() = 0; uy.y() = 1; uz.y() = 0;
	ux.z() = 0; uy.z() = 0; uz.z() = 1;

	// Current camera to a virtual camera transformation that is perpendicular to the moving surface
	camera_to_virtual.setOrigin(tf::Vector3(0, 0, 0));

	//base_to_camera.setOrigin(tf::Vector3(1.05614, -0.100492, -0.872156));
	//tf::Quaternion q = tf::createQuaternionFromRPY(-2.49022, 0.295534, 0.753678);
	//base_to_camera.setRotation(q);

	// ROS publishers for data visualization
	pub = surface_interaction_node.advertise<sensor_msgs::PointCloud2> ("trajectory_cloud", 1);
	transformed_pub = surface_interaction_node.advertise<sensor_msgs::PointCloud2> ("transformed_cloud", 1);
	poly_pub = surface_interaction_node.advertise<sensor_msgs::PointCloud2> ("polygon_cloud", 1);
	nf_pub = surface_interaction_node.advertise<sensor_msgs::PointCloud2> ("nf_cloud", 1);
	imagesp_pub = surface_interaction_node.advertise<sensor_msgs::PointCloud2> ("image_space_cloud", 1);

	marker_pub = surface_interaction_node.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
	object_pub = surface_interaction_node.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
	ROI_pub = surface_interaction_node.advertise<visualization_msgs::MarkerArray>("roi_marker", 1);

	target_position_pub = surface_interaction_node.advertise<geometry_msgs::Point> ("target_position_topic", 1);
	pose_pub = surface_interaction_node.advertise<geometry_msgs::PoseStamped> ("pose_topic", 1);


	//Set the required subscribers
	surface_interaction_node.getParam("input_camera_topic",input_camera_topic);
	sub = surface_interaction_node.subscribe(input_camera_topic, 1, &surface_interaction_Class::cloud_cb, this);

	surface_interaction_node.getParam("input_camera_info",input_camera_info);
	camera_info_sub = surface_interaction_node.subscribe(input_camera_info, 1,&surface_interaction_Class::camera_info_cb, this);

	surface_interaction_node.getParam("segmentation_topic",segmentation_topic);
	segmentation_sub = surface_interaction_node.subscribe(segmentation_topic, 1, &surface_interaction_Class::segmentation_cb, this);

	surface_interaction_node.getParam("segmentation_image_topic",segmentation_image_topic);
	segmentation_image_sub = surface_interaction_node.subscribe(segmentation_image_topic, 1, &surface_interaction_Class::object_frame_cb, this);

	surface_interaction_node.getParam("planar_reference_topic",planar_reference_topic);
	canonical_sub = surface_interaction_node.subscribe(planar_reference_topic, 1, &surface_interaction_Class::task_reference_cb, this);

	//surface_interaction_node.getParam("current_position_topic", current_position_topic);
	//current_position_sub = surface_interaction_node.subscribe(current_position_topic, 1, &surface_interaction_Class::current_position_cb, this);

	surface_interaction_node.setParam("status", "running");

	spinner.start();
	ros::waitForShutdown();

}


surface_interaction_Class::~surface_interaction_Class(){

}


// Camera info for RGB-D camera
void surface_interaction_Class::camera_info_cb (const sensor_msgs::CameraInfoConstPtr& info_msg){
	camera_info = *info_msg;
}

// Point cloud data coming from RGB-D camera
void surface_interaction_Class::cloud_cb (const sensor_msgs::PointCloud2Ptr& cloud_msg){

	if (false){
		std::cout<<"Receiving cloud data" << std::endl;
	}

	if ((cloud_msg->data.size() < 1)||(last_segmentation.data.size() < 1)) {
		std::cout<<"Camera input PointCloud is empty or segmentation is empty" << std::endl;
	}else{


		//Update the 3D cloud information to the last cloud saved in the class
		last_cloud = *cloud_msg;

		polygon_cloud.header.frame_id = last_cloud.header.frame_id;
		polygon_cloud.header.seq++;
		polygon_cloud.header.stamp= ros::Time::now().toNSec()/1000ull;

		// Get X-Y-Z indices
		int x_idx = pcl::getFieldIndex (last_cloud, "x");
		int y_idx = pcl::getFieldIndex (last_cloud, "y");
		int z_idx = pcl::getFieldIndex (last_cloud, "z");

		int rgb_idx = pcl::getFieldIndex (last_cloud, "rgb");

		Eigen::Array4i xyz_offset (last_cloud.fields[x_idx].offset, last_cloud.fields[y_idx].offset, last_cloud.fields[z_idx].offset, 0);

		//Transform Point Cloud with respect to a virtual camera that is perpendicular to a moving surface
		//Initialize new transformed cloud
		tmp_cloud.header.frame_id = "virtual_camera";
		tmp_cloud.header.seq= cloud_msg->header.seq;
		tmp_cloud.header.stamp= cloud_msg->header.stamp;
		tmp_cloud.fields = last_cloud.fields;
		tmp_cloud.height = last_cloud.height;
		tmp_cloud.width = last_cloud.width;
		tmp_cloud.point_step = last_cloud.point_step;
		tmp_cloud.row_step = last_cloud.row_step;
		tmp_cloud.is_bigendian = last_cloud.is_bigendian;
		tmp_cloud.is_dense = last_cloud.is_dense;
		tmp_cloud.data.clear();
		tmp_cloud.data.resize(last_cloud.height*last_cloud.width*last_cloud.point_step, 0);

		// Compute the projection of the transformed Point Cloud on a new 2D image using camera info
		//Initialize new transformed image
		std_msgs::MultiArrayDimension dimension;

		dimension.label = "height";
		dimension.size = cloud_msg->height;
		dimension.stride = cloud_msg->height;
		tmp_image.layout.dim.push_back(dimension);

		dimension.label = "width";
		dimension.size = cloud_msg->width;
		dimension.stride = cloud_msg->width;
		tmp_image.layout.dim.push_back(dimension);

		tmp_image.data.resize(cloud_msg->height*cloud_msg->width, 0);

		//Use camera info to calculate the new image positions of each point in the PCL
		// Update camera model
		model_.fromCameraInfo(camera_info);

		cv::Point3d transformed_point;
		cv::Point2d projected_point;

		// Camera to virtual camera transformation. Virtual camera is always perpendicular to the region of interest.
		// Transformation is calculated in Sengentation image cb in line 333
		// Get the transformation
		Eigen::Matrix4f transform;
		if ((camera_to_virtual.getOrigin().x() == 0)&& (camera_to_virtual.getOrigin().y() == 0) && (camera_to_virtual.getOrigin().z() == 0)){
			std::cout<<"Camera to virtual camera transform is not yet calculated ..." << std::endl;
		}else{
			pcl_ros::transformAsMatrix(camera_to_virtual.inverse(), transform);

			//Iterators for accessing the PointCloud Data
			sensor_msgs::PointCloud2ConstIterator<float> iter_x(last_cloud, "x");
			sensor_msgs::PointCloud2ConstIterator<float> iter_h(last_cloud, "x");
			sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(last_cloud, "rgb");
			sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgbh(last_cloud, "rgb");

			uint index = 0, cloud_index = 0;
			for(uint row = 0; row < last_segmentation.layout.dim[0].size; row++) {
				index = row*last_segmentation.layout.dim[1].stride;

				for(uint col = 0; col < last_segmentation.layout.dim[1].size; col++){
					if(last_segmentation.data[index+col] == 1){
						iter_h = iter_x + row * last_cloud.width + col;
						iter_rgbh = iter_rgb + row * last_cloud.width + col;
						//read_point from iterator
						Eigen::Vector4f pt (iter_h[0], iter_h[1], iter_h[2], 1);
						Eigen::Vector4f pt_out;

						//Apply the transformation
						pt_out = transform * pt;

						//std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
						transformed_point = cv::Point3d(pt_out[0], pt_out[1], pt_out[2]);

						///WARNING!: (u,v) in rectified pixel coordinates!
						projected_point = model_.project3dToPixel(transformed_point);

						// If the projected point lies within the image plane
						if((projected_point.x < last_cloud.width) && (projected_point.y < last_cloud.height) && (projected_point.x >=0) && (projected_point.y >= 0)){
							//Calculate the new cloud index
							cloud_index = (int)projected_point.y*last_cloud.row_step + (int)projected_point.x*cloud_msg->point_step;

							//Update the information in new segmentation image
							tmp_image.data[(int)projected_point.y*last_cloud.width + (int)projected_point.x] = 1;

							image_point.x() = (float)projected_point.x;
							image_point.y() = (float)projected_point.y;
							image_point.z() = 0;
							valid_pixels.push_back(image_point); //collect the segmentation data in a vector

							if(debug){
								//Visualize new projected_point
								vertice.r = 255;  //Visualization color
								vertice.g = 255;
								vertice.b = 255;
								vertice.x = projected_point.x/1000;
								vertice.y = projected_point.y/1000;
								vertice.z = 0;
								polygon_cloud.push_back(vertice);
							}

							//Insert the transformed data into the new PointCloud
							memcpy (&tmp_cloud.data[cloud_index + xyz_offset[0]], &pt_out[0], sizeof (float));
							memcpy (&tmp_cloud.data[cloud_index + xyz_offset[1]], &pt_out[1], sizeof (float));
							memcpy (&tmp_cloud.data[cloud_index + xyz_offset[2]], &pt_out[2], sizeof (float));
							//Copy the rest data which regard the color of each point (rgb)
							memcpy (&tmp_cloud.data[cloud_index + last_cloud.fields[rgb_idx].offset], &iter_rgbh[0], sizeof (float));


						}
					}
				}
			}

			transformed_cloud = tmp_cloud;
			transformed_image = tmp_image;

			transformed_pub.publish(transformed_cloud);
		}
	}

	if (valid_pixels.size() < 1) {
		std::cout<<"Unable to calculate ROI" << std::endl;
	}
	else{
		//find the extends of the segmentation data on the virtual image plane
		ImagePlane current_region(valid_pixels);
		//Update the information in the class
		region_of_interest = current_region;
		if (debug){
			//Region of Interest Frame Visualization
			uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
			visualization_msgs::Marker arrows;
			visualization_msgs::MarkerArray array;
			arrows.header.frame_id = last_cloud.header.frame_id;
			arrows.header.stamp = transformed_cloud.header.stamp;
			arrows.type = shape_arrow;
			arrows.header.seq = 0;
			arrows.id = 0;

			arrows.scale.x = 0.01;
			arrows.scale.y = 0.02;
			arrows.scale.z = 0;
			geometry_msgs::Point base, tip, tip1, tip2;
			//-----------------------------------------------------------------------------------------------------//

			base.x = region_of_interest.p0.x()/1000;
			base.y = region_of_interest.p0.y()/1000;
			base.z = region_of_interest.p0.z()/1000;

			tip.x = base.x + region_of_interest.normal.x()*std::sqrt(0.01);
			tip.y = base.y + region_of_interest.normal.y()*std::sqrt(0.01);
			tip.z = base.z + region_of_interest.normal.z()*std::sqrt(0.01);

			//Set Arrow color
			arrows.color.r = 0.0f;
			arrows.color.g = 0.0f;
			arrows.color.b = 1.0f;
			arrows.color.a = 1.0;

			arrows.points.push_back(base);
			arrows.points.push_back(tip);
			arrows.header.seq ++;
			arrows.id++;

			array.markers.push_back(arrows);
			arrows.points.clear();
			//-----------------------------------------------------------------------------------------------------//
			tip1.x = base.x + region_of_interest.b1.x()*multiplier*std::sqrt(region_of_interest.height)/1000;
			tip1.y = base.y + region_of_interest.b1.y()*multiplier*std::sqrt(region_of_interest.height)/1000;
			tip1.z = base.z + region_of_interest.b1.z()*multiplier*std::sqrt(region_of_interest.height)/1000;

			//Set Arrow color
			arrows.color.r = 0.0f;
			arrows.color.g = 1.0f;
			arrows.color.b = 0.0f;
			arrows.color.a = 1.0;

			arrows.points.push_back(base);
			arrows.points.push_back(tip1);
			arrows.header.seq ++;
			arrows.id++;
			array.markers.push_back(arrows);
			arrows.points.clear();
			//-----------------------------------------------------------------------------------------------------//
			tip2.x = base.x + region_of_interest.b2.x()*multiplier*std::sqrt(region_of_interest.width)/1000;
			tip2.y = base.y + region_of_interest.b2.y()*multiplier*std::sqrt(region_of_interest.width)/1000;
			tip2.z = base.z + region_of_interest.b2.z()*multiplier*std::sqrt(region_of_interest.width)/1000;

			//Set Arrow color
			arrows.color.r = 1.0f;
			arrows.color.g = 0.0f;
			arrows.color.b = 0.0f;
			arrows.color.a = 1.0;

			arrows.points.push_back(base);
			arrows.points.push_back(tip2);
			arrows.header.seq ++;
			arrows.id++;

			array.markers.push_back(arrows);
			arrows.points.clear();

			ROI_pub.publish(array);
			array.markers.clear();
		}
	}
	if(debug){
		poly_pub.publish(polygon_cloud);
		polygon_cloud.clear();
	}

	valid_pixels.clear();

}

// Segmentation Mask indicating the Region of Interest
void surface_interaction_Class::object_frame_cb (const sensor_msgs::Image & segmentation_image){

	if (false) std::cout<<"Calculating object reference" << std::endl;

	if ((segmentation_image.data.size() < 1)||(last_cloud.data.size() < 1)) {
		std::cout<<"Segmentation image is empty" << std::endl;
	}else{

		//Iterators for accessing the PointCloud Data
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(last_cloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_h(last_cloud, "x");


		uint index = 0;
		// Collect the points of the that belong to ROI from the last available cloud in the Class
		for(uint row = 0; row < segmentation_image.height; row++) {
			index = row*segmentation_image.width;
			for(uint col = 0; col < segmentation_image.width; col++) {

				if(segmentation_image.data[index+col] > 0){
					iter_h = iter_x + row * last_cloud.width + col;
					if((std::isnan(iter_h[0])) || (std::isnan(iter_h[1])) || (std::isnan(iter_h[2]))) continue;
					if((iter_h[0] == 0) || (iter_h[1] == 0) || (iter_h[2] == 0)) continue;
					if(iter_h[2] >= 1.3) continue;
					if(iter_h[2] <= 0.5) continue;
					object_point.x() = iter_h[0];
					object_point.y() = iter_h[1];
					object_point.z() = iter_h[2];
					object_points.push_back(object_point); //collect the segmentation data in a vector

				}
			}
		}

		if (object_points.size() < 1) {
			std::cout<<"Unable to find Segmentation data in the message" << std::endl;
		}
		else{
			//find the ROI frame with PCA to define the perpendicular direction
			ObjectFrame object_frame_tmp(object_points);
			//Update the information in the class
			object_frame = object_frame_tmp;

			//Calculate the object orientation

			//calculate pitch (y rotation)
			object_xz_projection = object_frame.normal - object_frame.normal.dot(uy)*uy;
			object_pitch = std::acos(object_xz_projection.dot(uz)/object_xz_projection.norm());
			if (object_xz_projection.dot(-ux) > 0) object_pitch = -object_pitch;

			//calculate roll (x rotation)
			object_yz_projection = object_frame.normal - object_frame.normal.dot(ux)*ux;
			object_roll = std::acos(object_yz_projection.dot(uz)/object_yz_projection.norm());
			if (object_yz_projection.dot(uy) > 0) object_roll = -object_roll;

			//calculate yaw (z rotation)
			object_xy_projection = object_frame.b2 - object_frame.b2.dot(uz)*uz;
			object_yaw = std::acos(object_xy_projection.dot(ux)/object_xy_projection.norm());
			if (object_xy_projection.dot(-uy) > 0) object_yaw = -object_yaw;


			virtual_camera_pos = object_frame.p0 - distance_from_object*object_frame.normal;

			if (false){
				std::cout<< "======================================"<< std::endl;
				std::cout<< "Object position = " << virtual_camera_pos.x() << " , " << virtual_camera_pos.y() << " , " << virtual_camera_pos.z() << std::endl;
				std::cout<< "Object orientation = " << object_roll << " , "<< object_pitch << " , "<< object_yaw  << std::endl
						<<  "======================================" << std::endl;
			}

			camera_to_virtual.setOrigin(tf::Vector3( virtual_camera_pos.x(),  virtual_camera_pos.y(),  virtual_camera_pos.z()));
			tf::Quaternion q = tf::createQuaternionFromRPY(object_roll, object_pitch, object_yaw);
			camera_to_virtual.setRotation(q);


			if (debug){
				//ROI Frame Visualization

				uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
				visualization_msgs::Marker arrows;
				visualization_msgs::MarkerArray array;
				arrows.header.frame_id = robot_base_frame;
				arrows.header.stamp = last_cloud.header.stamp;
				arrows.type = shape_arrow;
				arrows.header.seq = 0;
				arrows.id = 0;

				arrows.scale.x = 0.02;
				arrows.scale.y = 0.02;
				arrows.scale.z = 0;
				geometry_msgs::Point base, tip, tip1, tip2;
				//-----------------------------------------------------------------------------------------------------//

				base.x = object_frame.p0.x();
				base.y = object_frame.p0.y();
				base.z = object_frame.p0.z();

				tip.x = base.x + object_frame.normal.x()*std::sqrt(0.01);
				tip.y = base.y + object_frame.normal.y()*std::sqrt(0.01);
				tip.z = base.z + object_frame.normal.z()*std::sqrt(0.01);

				//Set Arrow color
				arrows.color.r = 0.0f;
				arrows.color.g = 0.0f;
				arrows.color.b = 1.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip);
				arrows.header.seq ++;
				arrows.id++;

				array.markers.push_back(arrows);
				arrows.points.clear();
				//-----------------------------------------------------------------------------------------------------//
				tip1.x = base.x + object_frame.b1.x()*multiplier*std::sqrt(object_frame.egValues.y());
				tip1.y = base.y + object_frame.b1.y()*multiplier*std::sqrt(object_frame.egValues.y());
				tip1.z = base.z + object_frame.b1.z()*multiplier*std::sqrt(object_frame.egValues.y());

				//Set Arrow color
				arrows.color.r = 0.0f;
				arrows.color.g = 1.0f;
				arrows.color.b = 0.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip1);
				arrows.header.seq ++;
				arrows.id++;
				array.markers.push_back(arrows);
				arrows.points.clear();
				//-----------------------------------------------------------------------------------------------------//
				tip2.x = base.x + object_frame.b2.x()*multiplier*std::sqrt(object_frame.egValues.z());
				tip2.y = base.y + object_frame.b2.y()*multiplier*std::sqrt(object_frame.egValues.z());
				tip2.z = base.z + object_frame.b2.z()*multiplier*std::sqrt(object_frame.egValues.z());

				//Set Arrow color
				arrows.color.r = 1.0f;
				arrows.color.g = 0.0f;
				arrows.color.b = 0.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip2);
				arrows.header.seq ++;
				arrows.id++;

				array.markers.push_back(arrows);
				arrows.points.clear();

				object_pub.publish(array);
				array.markers.clear();
			}

			object_points.clear();
		}

	}
}

void surface_interaction_Class::segmentation_cb (const std_msgs::UInt8MultiArray& segmentation){

	if (false) std::cout<<"Segmentation data" << std::endl;

	if ((segmentation.data.size() < 1)) {
		std::cout<<"No Segmentation data received" << std::endl;
		//surface_interaction_node.setParam("status", "fail");
	}else{
		last_segmentation = segmentation;
	}
}

// As soon as 2D input point received, first scale to the image ROI and then project to 3D task space of the robot 
void surface_interaction_Class::task_reference_cb (const geometry_msgs::Point& planar_point){

	if(debug){
		std::cout<<"Receiving trajectory data" << std::endl;
	}

	// If no perception information received set output to zero
	if ((transformed_cloud.data.size() < 1)||((camera_to_virtual.getOrigin().x() == 0)&& (camera_to_virtual.getOrigin().y() == 0) && (camera_to_virtual.getOrigin().z() == 0))){
		target_position.x() = 0;
		target_position.y() = 0;
		target_position.z() = 0;
		roll = 0; pitch = 0; yaw = 0;

	}else{

		//traj_cloud.header.frame_id = robot_base_frame;
		//traj_cloud.header.seq= last_cloud.header.seq;
		//traj_cloud.header.stamp= last_cloud.header.stamp.toNSec()/1000ull;

		//Iterators for accessing the PointCloud Data
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_h(transformed_cloud, "x");



		float polar_r, polar_theta, xCircle, yCircle;
		//Transform input to Circle coordinates
		xCircle = planar_point.x * std::sqrt(1 - 0.5*std::pow(planar_point.y,2));
		yCircle = planar_point.y * std::sqrt(1 - 0.5*std::pow(planar_point.x,2));
		//Transform input to polar coordinates
		polar_r = std::sqrt(pow(xCircle,2) + pow(yCircle,2));
		polar_theta = std::atan2(yCircle, xCircle);


		if(debug){
			//Visualize sample in canonical space
			vertice.r = 0;  //Visualization color
			vertice.g = 0;
			vertice.b = 255;
			vertice.x = planar_point.x/2 + 0.5;
			vertice.y = 0;
			vertice.z = planar_point.y/2 - 0.5;
			imagesp_cloud.push_back(vertice);
		}

		//		if(debug){
		//			//Visualize sample in canonical space
		//			vertice.r = 0;  //Visualization color
		//			vertice.g = 255;
		//			vertice.b = 0;
		//			vertice.x = -0.5 +xCircle/2 ;
		//			vertice.y = 0;
		//			vertice.z = -0.5 + yCircle/2;
		//			traj_cloud.push_back(vertice);
		//		}



		std::cout<<"Canonical space square point = " << planar_point.x << " , " << planar_point.y << std::endl;
		std::cout<<"Canonical space circle point = " << xCircle << " , " << yCircle << std::endl;
		std::cout<<"Image space polar point (r,theta) = " << polar_r << " , " << polar_theta << std::endl;

		//Calculate the total distance on surface
		image_point_previous.x() = (int)region_of_interest.p0.x();
		image_point_previous.y() = (int)region_of_interest.p0.y();
		image_point_previous.z() = 0;

		image_point_next.x() = (int)region_of_interest.p0.x();
		image_point_next.y() = (int)region_of_interest.p0.y();
		image_point_next.z() = 0;

		// Scale 2D input point to the size of ROI on the image space
		point_to_meet.x() = multiplier*std::sqrt(region_of_interest.width)*std::cos(polar_theta) + region_of_interest.p0.x();
		point_to_meet.y() = multiplier*std::sqrt(region_of_interest.height)*std::sin(polar_theta) + region_of_interest.p0.y();
		point_to_meet.z() = 0;

		surface_samples.clear();
		t = 0;
		t_step = 0.02;
		total_surf_distance = 0;
		while (pow((image_point_next.x() - region_of_interest.p0.x())/(multiplier*std::sqrt(region_of_interest.width)), 2.0) + pow((image_point_next.y() - region_of_interest.p0.y())/(multiplier*std::sqrt(region_of_interest.height)), 2.0) < 1){

			//Get the second point from the PointCloud
			iter_h = iter_x + (int)image_point_next.y() * last_cloud.width + (int)image_point_next.x();
			if((std::isnan(iter_h[0])) || (std::isnan(iter_h[1])) || (std::isnan(iter_h[2]))|| (std::isnan(iter_h[3])||(iter_h[0] == 0) || (iter_h[1] == 0) || (iter_h[2] == 0))){
				//Follow the line to calculate the next point
				image_point_next.x() = (1 - t)*region_of_interest.p0.x() + t*point_to_meet.x();
				image_point_next.y() = (1 - t)*region_of_interest.p0.y() + t*point_to_meet.y();
				t += t_step;
				continue;
			}
			if(transformed_image.data[(int)image_point_next.y()*transformed_image.layout.dim[1].stride + (int)image_point_next.x()] == 0) {
				//Follow the line to calculate the next point
				image_point_next.x() = (1 - t)*region_of_interest.p0.x() + t*point_to_meet.x();
				image_point_next.y() = (1 - t)*region_of_interest.p0.y() + t*point_to_meet.y();
				t += t_step;
				continue;
			}
			surface_sample_next.x() = iter_h[0];
			surface_sample_next.y() = iter_h[1];
			surface_sample_next.z() = iter_h[2];

			//Get the first point from the PointCloud
			iter_h = iter_x + (int)image_point_previous.y() * last_cloud.width + (int)image_point_previous.x();
			if((std::isnan(iter_h[0])) || (std::isnan(iter_h[1])) || (std::isnan(iter_h[2]))|| (std::isnan(iter_h[3])||(iter_h[0] == 0) || (iter_h[1] == 0) || (iter_h[2] == 0))) {
				image_point_previous = image_point_next;
				break;
			}
			if(transformed_image.data[(int)image_point_previous.y()*transformed_image.layout.dim[1].stride + (int)image_point_previous.x()] == 0){
				image_point_previous = image_point_next;
				continue;
			}
			surface_sample_previous.x() = iter_h[0];
			surface_sample_previous.y() = iter_h[1];
			surface_sample_previous.z() = iter_h[2];

			//Accumulate the partial distances to the total distance
			total_surf_distance += (surface_sample_next - surface_sample_previous).norm();

			//Keep track of the calculated points
			point_to_save.x() = (int)image_point_next.x();
			point_to_save.y() = (int)image_point_next.y();
			point_to_save.z() = total_surf_distance;

			surface_samples.push_back(point_to_save);

			image_point_previous = image_point_next;

			//Follow the line to calculate the next point
			image_point_next.x() = (1 - t)*region_of_interest.p0.x() + t*point_to_meet.x();
			image_point_next.y() = (1 - t)*region_of_interest.p0.y() + t*point_to_meet.y();
			t += t_step;
		}

		for (uint k = 0; k < surface_samples.size(); k++){
			if (surface_samples[k].z() > polar_r*total_surf_distance){
				break;
			}

			goal.x() = surface_samples[k].x();
			goal.y() = surface_samples[k].y();
		}


		//Image space controller
		//Call image controller to calculate a valid goal position that avoids potential obstacles on the surface
		image_controller();


		//		reference_x = (int)output.x();
		//		reference_y = (int)output.y();

		//Do this assignment for adaptation without controller

		reference_x = (int)goal.x();
		reference_y = (int)goal.y();


		std::cout<<"Image space point (x,y) = " << goal.x() << " , " << goal.y() << std::endl;
		if(debug){
			//Visualize response in image space
			vertice.r = 0;  //Visualization color
			vertice.g = 255;
			vertice.b = 0;
			vertice.x = (float)reference_x/1000;
			vertice.y = (float)reference_y/1000;
			vertice.z = 0;
			imagesp_cloud.push_back(vertice);
		}


		if(debug){
			//Visualize border point to meet for the surface calculation
			vertice.r = 255;  //Visualization color
			vertice.g = 0;
			vertice.b = 0;
			vertice.x = point_to_meet.x()/1000;
			vertice.y = point_to_meet.y()/1000;
			vertice.z = 0;
			imagesp_cloud.push_back(vertice);
		}
		/*
			Project point in 3D Task space of the robot 
		*/

		//Collect the 3D points of ROI in window around the goal to calculate the local curvature of the surface
		window_size_x = (int)window_size_y/2;
		for(uint row = reference_y - window_size_y; row < reference_y + window_size_y + 1; row++) {

			for(uint col = reference_x - window_size_x; col < reference_x + window_size_x + 1; col++) {

				iter_h = iter_x + row * transformed_cloud.width + col;
				if((std::isnan(iter_h[0])) || (std::isnan(iter_h[1])) || (std::isnan(iter_h[2]))|| (std::isnan(iter_h[3]))) continue;
				if((iter_h[0] == 0) || (iter_h[1] == 0) || (iter_h[2] == 0)) continue;
				if(transformed_image.data[row*transformed_image.layout.dim[1].stride + col] == 0) continue;
				sample3d.x() = iter_h[0];
				sample3d.y() = iter_h[1];
				sample3d.z() = iter_h[2];
				region_points.push_back(sample3d);

			}

		}
		if(region_points.size() < 2) {
			std::cout<< "Current pixel region is empty.. Publishing zero for reseting.. " << std::endl;

			target_position_msg.x = 0;
			target_position_msg.y = 0;
			target_position_msg.z = 0;

		} else{
	
			
			//Calculate the principal axis
			PlanePolygon local_frame(region_points);


			target_position = local_frame.p0;  //the reference position equals to the centroid of ROI

			//--------------------------------------------------------------------------------------------------------------------//

			//calculate pitch (y rotation)
			xz_projection = local_frame.normal - local_frame.normal.dot(uy)*uy;
			pitch = std::acos(xz_projection.dot(uz)/xz_projection.norm());
			if (xz_projection.dot(ux) > 0) pitch = -pitch;

			//calculate roll (x rotation)
			xy_projection = local_frame.normal - local_frame.normal.dot(uz)*uz;
			roll = std::acos(xy_projection.dot(ux)/xy_projection.norm());
			if (xy_projection.dot(uy) > 0) roll = -roll;

			//calculate yaw (z rotation)
			xz_projection = local_frame.b2 - local_frame.b2.dot(uy)*uy;
			yaw = std::acos(xz_projection.dot(-uz)/xz_projection.norm());
			if (xz_projection.dot(uy) > 0) yaw = -yaw;


			std::cout<< "======================================"<< std::endl;
			std::cout<< "reference_pose position = " << target_position.x() << " , " << target_position.y() << " , " << target_position.z() << std::endl;
			std::cout<< "reference_pose orientation = " << roll << " , "<< pitch << " , "<< yaw << std::endl
					<<  "======================================" << std::endl;

			virtual_to_surface.setOrigin(tf::Vector3( target_position.x(),  target_position.y(),  target_position.z()));
			tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
			virtual_to_surface.setRotation(q);

			//Apply the transform w.r.t. the original camera frame
			camera_to_surface = camera_to_virtual* virtual_to_surface;

			//Transform local frame vectors for visualization purposes
			tf::Vector3 tranformed_normal, transformed_b1, transformed_b2;

			tf::Vector3 vector(local_frame.normal.x(), local_frame.normal.y(), local_frame.normal.z());
			tranformed_normal = camera_to_virtual.getBasis()*vector;

			vector.setX((double)local_frame.b1.x()); vector.setY((double)local_frame.b1.y()); vector.setZ((double)local_frame.b1.z());
			transformed_b1 = camera_to_virtual.getBasis()*vector;

			vector.setX((double)local_frame.b2.x()); vector.setY((double)local_frame.b2.y()); vector.setZ((double)local_frame.b2.z());
			transformed_b2 = camera_to_virtual.getBasis()*vector;

			// Visualization Clouds
			traj_cloud.header.frame_id = last_cloud.header.frame_id;
			traj_cloud.header.seq = 0;
			traj_cloud.header.stamp= ros::Time::now().toNSec()/1000ull;

			imagesp_cloud.header.frame_id = last_cloud.header.frame_id;
			imagesp_cloud.header.seq = 0;
			imagesp_cloud.header.stamp= ros::Time::now().toNSec()/1000ull;


			if(debug){
				//Visualize response in 3d space
				vertice.r = 0;  //Visualization color
				vertice.g = 255;
				vertice.b = 0;
				vertice.x = camera_to_surface.getOrigin().x();
				vertice.y = camera_to_surface.getOrigin().y();
				vertice.z = camera_to_surface.getOrigin().z();
				traj_cloud.push_back(vertice);
			}

			region_points.clear();

			//--------------------------------------------------------------------------------------------------------------------//
			if (debug){
				//Frame Visualization

				uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
				visualization_msgs::Marker arrows;
				visualization_msgs::MarkerArray array;
				arrows.header.frame_id = last_cloud.header.frame_id;
				arrows.header.stamp = transformed_cloud.header.stamp;
				arrows.type = shape_arrow;
				arrows.header.seq = 0;
				arrows.id = 0;

				arrows.scale.x = 0.01;
				arrows.scale.y = 0.02;
				arrows.scale.z = 0;
				geometry_msgs::Point base, tip, tip1, tip2, point;
				//
				//Visualize reference frame
				base.x = camera_to_surface.getOrigin().x();
				base.y = camera_to_surface.getOrigin().y();
				base.z = camera_to_surface.getOrigin().z();


				tip.x = base.x + tranformed_normal.x()*std::sqrt(local_frame.egValues.z()*40);
				tip.y = base.y + tranformed_normal.y()*std::sqrt(local_frame.egValues.z()*40);
				tip.z = base.z + tranformed_normal.z()*std::sqrt(local_frame.egValues.z()*40);

				//Set Arrow color
				arrows.color.r = 0.0f;
				arrows.color.g = 0.0f;
				arrows.color.b = 1.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip);
				arrows.header.seq ++;
				arrows.id++;

				array.markers.push_back(arrows);
				arrows.points.clear();
				//////////////////////////////////////////////////////////////////////////////////////////////////
				tip1.x = base.x + transformed_b1.x()*std::sqrt(local_frame.egValues.z()*40);
				tip1.y = base.y + transformed_b1.y()*std::sqrt(local_frame.egValues.z()*40);
				tip1.z = base.z + transformed_b1.z()*std::sqrt(local_frame.egValues.z()*40);

				//Set Arrow color
				arrows.color.r = 0.0f;
				arrows.color.g = 1.0f;
				arrows.color.b = 0.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip1);
				arrows.header.seq ++;
				arrows.id++;
				array.markers.push_back(arrows);
				arrows.points.clear();
				/////////////////////////////////////////////////////////////////////////////////////////////////
				tip2.x = base.x + transformed_b2.x()*std::sqrt(local_frame.egValues.z()*40);
				tip2.y = base.y + transformed_b2.y()*std::sqrt(local_frame.egValues.z()*40);
				tip2.z = base.z + transformed_b2.z()*std::sqrt(local_frame.egValues.z()*40);

				//Set Arrow color
				arrows.color.r = 1.0f;
				arrows.color.g = 0.0f;
				arrows.color.b = 0.0f;
				arrows.color.a = 1.0;

				arrows.points.push_back(base);
				arrows.points.push_back(tip2);
				arrows.header.seq ++;
				arrows.id++;

				array.markers.push_back(arrows);
				arrows.points.clear();

				//-------------------------------------------------------------------------------------------------------//

				marker_pub.publish(array);
				array.markers.clear();
			}
		}

		if(traj_cloud.size()>35){
			traj_cloud.erase(traj_cloud.begin());
		}

		if(imagesp_cloud.size()>10){
			imagesp_cloud.erase(imagesp_cloud.begin());
		}

		//--------------------------------------------------------------------------------------------------------------------//

		// Publish the data
		pub.publish(traj_cloud);

		imagesp_pub.publish(imagesp_cloud);


		//traj_cloud.clear();
	}
}


// Implementation of a Navigation function controller on the Image plane.
void surface_interaction_Class::image_controller(){


	if (first_time){
		output.x() = goal.x();
		output.y() = goal.y();
		time_projection = output;
		first_time = false;
	}else{
		//Workspace definition
		image_centre.x() = last_cloud.width/2;
		image_centre.y() = last_cloud.height/2;

		scaled_extends = multiplier*region_of_interest.extends;
		scaled_extends_inv = scaled_extends.cwiseInverse();

		//============================================================================================================================================//
		//Attractive field
		gamma_fun = (goal - output).cwiseProduct(scaled_extends_inv).norm();
		grad_gamma = 2*(goal - output).cwiseProduct(scaled_extends_inv);

		//Image Workspace
		exponent = 4;
		workspace = 1 - pow((output.x() - image_centre.x())/image_centre.x(),exponent) - pow((output.y() - image_centre.y())/image_centre.y(),exponent);
		grad_workspace.x() = -exponent*pow((output.x() - image_centre.x())/image_centre.x(),exponent-1); grad_workspace.y() = -exponent*pow((output.y() - image_centre.y())/image_centre.y(),exponent-1);

		exponent = 2;

		ROI = 1 - pow((region_of_interest.p0.x() - output.x())/(scaled_extends.x()),exponent)
								- pow((region_of_interest.p0.y() - output.y())/(scaled_extends.y()),exponent);

		grad_ROI.x() = -exponent*pow((region_of_interest.p0.x() - output.x())/(scaled_extends.x()),exponent-1);
		grad_ROI.y() = -exponent*pow((region_of_interest.p0.y() - output.y())/(scaled_extends.y()),exponent-1);


		exponent = 4;
		for(uint k = 0; k < obstacles.size(); k++){
			grad_obstacle.clear();
			obstacle_beta.clear();
			grad_obstacle.resize(obstacles.size());
			obstacle_beta.resize(obstacles.size(), 0);
			obstacle_beta[k] = -1 + pow((obstacles[k].p0.x() - output.x())/obstacles[k].extends.x(),exponent)
					+ pow((obstacles[k].p0.y() - output.y())/obstacles[k].extends.y(),exponent);

			grad_obstacle[k].x() = exponent*pow((obstacles[k].p0.x() - output.x())/obstacles[k].extends.x(),exponent-1);

			grad_obstacle[k].y() = exponent*pow((obstacles[k].p0.y() - output.y())/obstacles[k].extends.y(),exponent-1);
		}

		//Calculate the final value of the repulsive field
		//float total_obstacle = workspace*ROI;
		float total_obstacle = ROI;
		for(uint l = 0; l < obstacles.size(); l++){
			total_obstacle *=  obstacle_beta[l];
		}

		//Calculate the final value of the gradient repulsive field
		//obstacle_total_grad = (grad_workspace*ROI + grad_ROI*workspace)*total_obstacle;
		obstacle_total_grad = grad_ROI*total_obstacle/ROI;

		for(uint l = 0; l < obstacles.size(); l++){
			//obstacle_total_grad += workspace*ROI*grad_obstactle[l]*total_obstacle/obstacle_beta[l];
			obstacle_total_grad += grad_obstacle[l]*total_obstacle/obstacle_beta[l];
		}


		//total_obstacle *= workspace*ROI;

		// Phi and gradient Definition (f/g)' = (f'g-fg')/g^2
		//Navigation function value
		phi = gamma_fun/pow(pow(gamma_fun, kappa) + total_obstacle, kappa_inv);

		//Gradient of the Navigation Function
		grad_phi = (1/pow(pow(gamma_fun, kappa) + total_obstacle, 2*kappa_inv))*(grad_gamma*pow(pow(gamma_fun, kappa) + total_obstacle, kappa_inv)
		-(gamma_fun*kappa_inv)*pow(pow(gamma_fun, kappa) + total_obstacle, kappa_inv-1)*(kappa*pow(gamma_fun,kappa-1)*grad_gamma+obstacle_total_grad));


		if(debug){
			std::cout<< "======================================"<< std::endl;

			std::cout<< "gamma_fun = " << gamma_fun << std::endl;
			std::cout<< "grad_gamma (x,y) = " << grad_gamma.x() << " , " << grad_gamma.y() << std::endl;

			std::cout<< "workspace = " << workspace << std::endl;
			std::cout<< "grad_workspace (x,y) = " << grad_workspace.x() << " , " << grad_workspace.y() << std::endl;

			std::cout<< "ROI = " << ROI << std::endl;
			std::cout<< "grad_ROI (x,y) = " << grad_ROI.x() << " , " << grad_ROI.y() << std::endl;

			std::cout<< "total_obstacle = " << total_obstacle << std::endl;
			std::cout<< "obstacle_total_grad (x,y) = " << obstacle_total_grad.x() << " , " << obstacle_total_grad.y() << std::endl;

			std::cout<< "phi = " << phi << std::endl;
			std::cout<< "grad_phi (x,y) = " << grad_phi.x() << " , " << grad_phi.y() << std::endl;

			//			std::cout<< "obstacles size = " << obstacles.size() << std::endl;
			//std::cout<< "obstacles center (x,y) = " << obstacles[0].p0.x() << " , " << obstacles[0].p0.y() << std::endl;
			//			std::cout<< "obstacles (width ,height) = " << obstacles[0].width<< " , " << obstacles[0].height << std::endl;
			std::cout<< "======================================"<< std::endl;

		}
		output = output + dt*grad_phi;

		if(visualize){
			visualize_nf();
		}

	}

}

// Visualization of the calculated Navigation function
void surface_interaction_Class::visualize_nf(){

	uint offset = 20;

	nf_cloud.header.frame_id = last_cloud.header.frame_id;
	nf_cloud.header.seq = 0;
	nf_cloud.header.stamp= ros::Time::now().toNSec()/1000ull;

	for(uint row = 0 + offset; row < last_cloud.height - offset; row++) {

		for(uint col = 0 + offset; col < last_cloud.width - offset; col++){

			visualize_current.x() = (float)col;
			visualize_current.y() = (float)row;
			//============================================================================================================================================//
			//Attractive field
			gamma_fun = (goal - visualize_current).cwiseProduct(scaled_extends_inv).norm();
			grad_gamma = 2*(goal - visualize_current).cwiseProduct(scaled_extends_inv);

			//Image Workspace
			exponent = 4;
			workspace = 1 - pow((visualize_current.x() - image_centre.x())/image_centre.x(),exponent) - pow((visualize_current.y() - image_centre.y())/image_centre.y(),exponent);
			grad_workspace.x() = -exponent*pow((visualize_current.x() - image_centre.x())/image_centre.x(),exponent-1); grad_workspace.y() = -exponent*pow((visualize_current.y() - image_centre.y())/image_centre.y(),exponent-1);


			exponent = 2;

			ROI = 1 - pow((region_of_interest.p0.x() - visualize_current.x())/(scaled_extends.x()),exponent)
									- pow((region_of_interest.p0.y() - visualize_current.y())/(scaled_extends.y()),exponent);

			grad_ROI.x() = -exponent*pow((region_of_interest.p0.x() - visualize_current.x())/(scaled_extends.x()),exponent-1);
			grad_ROI.y() = -exponent*pow((region_of_interest.p0.y() - visualize_current.y())/(scaled_extends.y()),exponent-1);

			exponent = 4;
			for(uint k = 0; k < obstacles.size(); k++){
				grad_obstacle.clear();
				obstacle_beta.clear();
				grad_obstacle.resize(obstacles.size());
				obstacle_beta.resize(obstacles.size(), 0);
				obstacle_beta[k] = -1 + pow((obstacles[k].p0.x() - visualize_current.x())/obstacles[k].extends.x(),exponent)
						+ pow((obstacles[k].p0.y() - visualize_current.y())/obstacles[k].extends.y(),exponent);

				grad_obstacle[k].x() = exponent*pow((obstacles[k].p0.x() - visualize_current.x())/obstacles[k].extends.x(),exponent-1);

				grad_obstacle[k].y() = exponent*pow((obstacles[k].p0.y() - visualize_current.y())/obstacles[k].extends.y(),exponent-1);
			}

			//Calculate the final value of the repulsive field
			//float total_obstacle = workspace*ROI;
			float total_obstacle = ROI;
			for(uint l = 0; l < obstacles.size(); l++){
				total_obstacle *=  obstacle_beta[l];
			}

			//Calculate the final value of the gradient repulsive field
			//obstacle_total_grad = (grad_workspace*ROI + grad_ROI*workspace)*total_obstacle;
			obstacle_total_grad = grad_ROI*total_obstacle/ROI;

			for(uint l = 0; l < obstacles.size(); l++){
				//obstacle_total_grad += workspace*ROI*grad_obstactle[l]*total_obstacle/obstacle_beta[l];
				obstacle_total_grad += grad_obstacle[l]*total_obstacle/obstacle_beta[l];
			}


			//total_obstacle *= workspace*ROI;

			//Navigation function value
			phi = gamma_fun/pow(pow(gamma_fun, kappa) + total_obstacle, kappa_inv);

			if(debug){
				//Visualize new projected_point
				vertice.r = 255;  //Visualization color
				vertice.g = 0;
				vertice.b = 0;
				vertice.x = visualize_current.x()/1000;
				vertice.y = visualize_current.y()/1000;
				if(phi < 0 || phi > 1 || isnan(phi)){
					vertice.z = -0.25;
				}else{
					vertice.z = -phi/4;
				}

				nf_cloud.push_back(vertice);
			}
		}
	}
	nf_pub.publish(nf_cloud);
	nf_cloud.clear();

}



int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "surface_interaction");


	surface_interaction_Class cobj;


	return 0;
}
