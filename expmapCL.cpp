/*
 * Planning of Interactive trajectories on the Mesh structure of deformable objects
 *
 *  Created on: 07.2020
 *  Author: Athanasios Dometios
 */

// Mesh Parameterization implementation coming from:
// Ryan Schmidt - rms@unknownroad.com
// Copyright (c) 2006-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <shape_msgs/Mesh.h>
#include "tf/tf.h"
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/Marker.h> 
#include <nav_msgs/Path.h>

#include "MeshObject.h"
#include "ExtendedWmlCamera.h"
#include <rmsprofile.h>

#include "IMesh.h"

#define PI 3.141592
bool visualize = true;


class paramMesh{
public:
    paramMesh();
    ~paramMesh();

    //Input data callbacks
    void mesh_cb(const shape_msgs::Mesh & mesh_msg);
    void static_input_cb(const geometry_msgs::Point & inputPoint_msg);
    ros::Publisher static_visual_pub, visual_pub, desired_pose_pub, contact_pose_pub, normal_visual_pub, physical_path_pub, dynamic_path_pub;

    int vertices_number;
    //Declarations for input Mesh Data
    rms::VFTriangleMesh mesh;
    Wml::Vector3f fvec = Wml::Vector3f::ZERO;
    Wml::Vector3f nvec = Wml::Vector3f::ZERO;
    int id;
    bool firstCb, secondCb;
    //IMesh::VertexID vID;
    unsigned int vID;

    //Dynamic Canonical space
    rms::ExpMapGenerator expmapgen;
    rms::IMeshBVTree bvTree;
    //Static Canonical space
    rms::ExpMapGenerator static_expmapgen;
    rms::IMeshBVTree static_bvTree;

    //Planning Declarations
    float normal_offset;
    Wml::Vector2f static_input_point;
    Wml::Vector3f desired_normal;
    Wml::Vector3f desired_position, calculated_position;
    bool Status;
    geometry_msgs::PoseStamped desired_pose, contact_pose;

    //Projection to Dynamic Canonical space Declarations
    Wml::Vector2f dynamic_input_point;
    bool Dynamic_proj_Status;

    //Visualization declarations
    Wml::Vector3f vVertex, vNormal;
    std::vector<unsigned int> vIdx;
    std::vector<float> vU, vV;
    uint seed_vertex;

    visualization_msgs::Marker static_tri_vis, tri_vis, normalvec_vis;
    std_msgs::ColorRGBA vis_color;

    nav_msgs::Path physical_path, dynamic_path;
    geometry_msgs::PoseStamped dynamic_vis_pose;

    //Tf declarations for WidowX Robot corrections
    tf::Quaternion base_to_contact_q, contact_to_desired_q, desired_trans_q;

    tf::Vector3 base_to_contact_tra, contact_to_desired_tra;

    tf::Transform base_to_contact,contact_to_desired, desired_trans;
    bool start_publish;



};

paramMesh::paramMesh(){

    firstCb = true;
    secondCb = true;
    start_publish = false;
    vertices_number = 0;
    seed_vertex = 184; //Mesh vertex for parameterization
//    seed_vertex = 115;

    //static_input_point = Wml::Vector2f::ZERO;
    // Input in static canonical space
    static_input_point.X() = 0.0;
    static_input_point.Y() = 0.0;
    normal_offset = 0.0;
    Status = NULL;

    // Initializations for orientation and position vectors in the Task space
    desired_normal = Wml::Vector3f::ZERO;
    desired_position = Wml::Vector3f::ZERO;
    calculated_position = Wml::Vector3f::ZERO;


    //Mesh Visualization initializations
    tri_vis.header.frame_id = "base_footprint";
    tri_vis.header.seq = 0;
    tri_vis.header.stamp = ros::Time::now();

    tri_vis.type = visualization_msgs::Marker::TRIANGLE_LIST;
    tri_vis.id = 0;

    tri_vis.scale.x = 1;
    tri_vis.scale.y = 1;
    tri_vis.scale.z = 1;

    tri_vis.pose.position.x = -0.5;
    tri_vis.pose.position.y = 0.0;
    tri_vis.pose.position.z = 0.0;

    tri_vis.pose.orientation.x = 0.0;
    tri_vis.pose.orientation.y = 0.0;
    tri_vis.pose.orientation.z = 0.0;
    tri_vis.pose.orientation.w = 1.0;

    static_tri_vis.header.frame_id = "base_footprint";
    static_tri_vis.header.seq = 0;
    static_tri_vis.header.stamp = ros::Time::now();

    static_tri_vis.type = visualization_msgs::Marker::TRIANGLE_LIST;
    static_tri_vis.id = 0;
    static_tri_vis.text = "Static Canonical Space";

    static_tri_vis.scale.x = 1;
    static_tri_vis.scale.y = 1;
    static_tri_vis.scale.z = 1;

    static_tri_vis.pose.position.x = -0.5;
    static_tri_vis.pose.position.y = 0.5;
    static_tri_vis.pose.position.z = 0.0;

    static_tri_vis.pose.orientation.x = 0.0;
    static_tri_vis.pose.orientation.y = 0.0;
    static_tri_vis.pose.orientation.z = 0.0;
    static_tri_vis.pose.orientation.w = 1.0;

    //Initializations for path visualization
    physical_path.header.frame_id = "base_footprint";
    physical_path.header.seq = 0;
    physical_path.header.stamp = ros::Time::now();

    dynamic_path.header.frame_id = "base_footprint";
    dynamic_path.header.seq = 0;
    dynamic_path.header.stamp = ros::Time::now();

    dynamic_vis_pose.header.frame_id = "base_footprint";
    dynamic_vis_pose.header.seq = 0;


    //Initializations for pose corrections
    desired_pose.pose.position.x = 0.0;
    desired_pose.pose.position.y = 0.0;
    desired_pose.pose.position.z = 0.0;

    contact_pose.pose.position.x = 0.0;
    contact_pose.pose.position.y = 0.0;
    contact_pose.pose.position.z = 0.0;

    contact_to_desired.getOrigin()[0] = 0.045;
    contact_to_desired.getOrigin()[1] = 0.0;
    contact_to_desired.getOrigin()[2] = -0.0536;
    contact_to_desired.setRotation(tf::createQuaternionFromRPY(0, -PI/2, 0));

    //Normal Visualization initializations
    normalvec_vis.header.frame_id = "base_footprint";
    normalvec_vis.header.seq = 0;
    normalvec_vis.header.stamp = ros::Time::now();
    normalvec_vis.type = visualization_msgs::Marker::ARROW;
    normalvec_vis.id = 0;
    normalvec_vis.scale.x = 0.01;
    normalvec_vis.scale.y = 0.02;
    normalvec_vis.scale.z = 0;
    normalvec_vis.color.r = 0.0f;
    normalvec_vis.color.g = 0.0f;
    normalvec_vis.color.b = 1.0f;
    normalvec_vis.color.a = 1.0;

}

paramMesh::~paramMesh() {}

// Input 2D position + offset 
void paramMesh::static_input_cb(const geometry_msgs::Point & inputPoint_msg){

    static_input_point.X() = inputPoint_msg.x;
    static_input_point.Y() = inputPoint_msg.y;
    normal_offset = inputPoint_msg.z;
    start_publish = true;
}


void paramMesh::mesh_cb(const shape_msgs::Mesh & mesh_msg){

    std::cout << "In Mesh callback " << std::endl;

    //Assign the new vertices and normals
    //Half of the message includes the vertices and
    //the rest includes the correspondig normal vectors
    for (unsigned int i = 0; i < mesh_msg.vertices.size()/2; i++){ ///This is a trial version
        fvec[0] = mesh_msg.vertices[i].x;
        fvec[1] = mesh_msg.vertices[i].y;
        fvec[2] = mesh_msg.vertices[i].z;

        nvec[0] = mesh_msg.vertices[i + mesh_msg.vertices.size()/2].x;
        nvec[1] = mesh_msg.vertices[i + mesh_msg.vertices.size()/2].y;
        nvec[2] = mesh_msg.vertices[i + mesh_msg.vertices.size()/2].z;

        nvec.Normalize();

        if (firstCb) {
            id = mesh.AppendVertex(fvec);

            mesh.SetNormal(i, nvec);
        }else{
            mesh.SetVertex(i, fvec, &nvec);

        }
    }
    //Assign the new triangles only for the first time since the number and index of each triangle are not modified
    if (firstCb) {
        for (uint j = 0; j < mesh_msg.triangles.size(); j++){

            mesh.AppendTriangle(mesh_msg.triangles[j].vertex_indices[0]-1, mesh_msg.triangles[j].vertex_indices[1]-1, mesh_msg.triangles[j].vertex_indices[2]-1);

        }
    }
    if (firstCb)
    {
        //Initial parameterization to form the Static Canonical space
        static_bvTree.SetMesh(&mesh);
        static_expmapgen.SetSurface(&mesh, &static_bvTree);
        //static_expmapgen.SetUseNeighbourNormalSmoothing(true);
        //static_expmapgen.SetUseUpwindAveraging(true);

        // compute static expmap
        mesh.GetVertex(seed_vertex, vVertex, &vNormal); 
        rms::Frame3f vInitFrame(vVertex, vNormal);

        static_expmapgen.SetSurfaceDistances( vInitFrame, 0.10f, 185 );

        static_expmapgen.CopyBVTreeUV();

        // Visualize Static Canonical Mesh
        if(visualize){

            std::vector<unsigned int> sIdx;
            std::vector<float> sU, sV;

            static_expmapgen.GetVertexUVs(sIdx, sU, sV);

            size_t sCount = sIdx.size();
            int v_index1 = 0, v_index2 = 0, v_index3 = 0;
            geometry_msgs::Point visual_vertex;

            for ( unsigned int k = 0; k < mesh_msg.triangles.size(); k++ ) {
                if (k % 4 == 0)
                {
                        vis_color.r = 1.0;
                        vis_color.g = 0.0;
                }else if (k % 4 == 1){
                        vis_color.r = 0.0;
                        vis_color.g = 1.0;
                }else if (k % 4 == 2){
                        vis_color.r = 0.3;
                        vis_color.g = 0.7;
                }else{
                        vis_color.r = 0.7;
                        vis_color.g = 0.3;

                }

                vis_color.b = 0.0;
                vis_color.a = 1.0;
                for ( unsigned int j = 0; j < sCount; j++ ) {
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[0]) v_index1 = j;
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[1]) v_index2 = j;
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[2]) v_index3 = j;

                }

                visual_vertex.x = sU[v_index1];
                visual_vertex.y = sV[v_index1];
                visual_vertex.z = 0.0;
                static_tri_vis.points.push_back(visual_vertex);
                static_tri_vis.colors.push_back(vis_color);

                visual_vertex.x = sU[v_index2];
                visual_vertex.y = sV[v_index2];
                visual_vertex.z = 0.0;
                static_tri_vis.points.push_back(visual_vertex);
                static_tri_vis.colors.push_back(vis_color);

                visual_vertex.x = sU[v_index3];
                visual_vertex.y = sV[v_index3];
                visual_vertex.z = 0.0;
                static_tri_vis.points.push_back(visual_vertex);
                static_tri_vis.colors.push_back(vis_color);
                v_index1 = 0;
                v_index2 = 0;
                v_index3 = 0;
            }

            static_visual_pub.publish(static_tri_vis);
//            tri_vis.points.clear();
//            tri_vis.colors.clear();
        }


    }else{
        //At each time step renew the 3D mesh information also in Static canonical
        static_expmapgen.StaticUpdateMeshBV(&mesh, &static_bvTree);

        bvTree.SetMesh(&mesh);
        //expmapgen.SetUseNeighbourNormalSmoothing(true);
        //expmapgen.SetUseUpwindAveraging(true);
        expmapgen.SetSurface(&mesh, &bvTree);

        // compute Dynamic Canonical Space
        mesh.GetVertex(seed_vertex, vVertex, &vNormal);
        rms::Frame3f vSeedFrame(vVertex, vNormal);
        expmapgen.SetSurfaceDistances( vSeedFrame, 0.0f, 185 );
        expmapgen.CopyBVTreeUV();

        // Visualize Dynamic Canonical Mesh
        if(visualize){

            std::vector<unsigned int> sIdx;
            std::vector<float> sU, sV;
            static_expmapgen.GetVertexUVs(sIdx, sU, sV);
            size_t sCount = sIdx.size();
            int v_index1 = 0, v_index2 = 0, v_index3 = 0;
            geometry_msgs::Point visual_vertex;

            for ( unsigned int k = 0; k < mesh_msg.triangles.size(); k++ ) {

                //Modify the color of each triangle
                if (k % 4 == 0)
                {
                        vis_color.r = 1.0;
                        vis_color.g = 0.0;
                }else if (k % 4 == 1){
                        vis_color.r = 0.0;
                        vis_color.g = 1.0;
                }else if (k % 4 == 2){
                        vis_color.r = 0.3;
                        vis_color.g = 0.7;
                }else{
                        vis_color.r = 0.7;
                        vis_color.g = 0.3;

                }

                vis_color.b = 0.0;
                vis_color.a = 1.0;
                //Find the appropriate vertices index for each triangle
                for ( unsigned int j = 0; j < sCount; j++ ) {
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[0]) v_index1 = j;
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[1]) v_index2 = j;
                    if (sIdx[j]+1 == mesh_msg.triangles[k].vertex_indices[2]) v_index3 = j;

                }

                visual_vertex.x = sU[v_index1];
                visual_vertex.y = sV[v_index1];
                visual_vertex.z = 0.0;
                tri_vis.points.push_back(visual_vertex);
                tri_vis.colors.push_back(vis_color);

                visual_vertex.x = sU[v_index2];
                visual_vertex.y = sV[v_index2];
                visual_vertex.z = 0.0;
                tri_vis.points.push_back(visual_vertex);
                tri_vis.colors.push_back(vis_color);

                visual_vertex.x = sU[v_index3];
                visual_vertex.y = sV[v_index3];
                visual_vertex.z = 0.0;
                tri_vis.points.push_back(visual_vertex);
                tri_vis.colors.push_back(vis_color);

                v_index1 = 0;
                v_index2 = 0;
                v_index3 = 0;
            }

            visual_pub.publish(tri_vis);
            tri_vis.points.clear();
            tri_vis.colors.clear();
        }
        secondCb = false;

    }

    firstCb = false;

}


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "mesh_parametrization");
    ros::NodeHandle mesh_param;


    paramMesh param_obj;

    ros::Rate loop_rate(30);

    // mesh_param.getParam("vertices_number",param_obj.vertices_number);
    //mesh_param.getParam("activate_param",activate_param);


    /* initialize ROS subsrcibers and publishers */
    ros::Subscriber mesh_sub = mesh_param.subscribe("deformable_mesh", 1, &paramMesh::mesh_cb, &param_obj);
    ros::Subscriber static_input_sub = mesh_param.subscribe("static_input", 1, &paramMesh::static_input_cb, &param_obj);

    param_obj.visual_pub = mesh_param.advertise<visualization_msgs::Marker> ("param_mesh_topic", 1);
    param_obj.static_visual_pub = mesh_param.advertise<visualization_msgs::Marker> ("static_param_mesh_topic", 1);
    param_obj.normal_visual_pub = mesh_param.advertise<visualization_msgs::Marker> ("normal_vector_topic", 1);
    param_obj.desired_pose_pub = mesh_param.advertise<geometry_msgs::PoseStamped> ("desired_pose", 1);
    param_obj.contact_pose_pub = mesh_param.advertise<geometry_msgs::PoseStamped> ("contact_pose", 1);
    param_obj.physical_path_pub = mesh_param.advertise<nav_msgs::Path> ("physical_desired_path", 1);
    param_obj.dynamic_path_pub = mesh_param.advertise<nav_msgs::Path> ("dynamic_desired_path", 1);


    param_obj.desired_pose.header.frame_id = "base_footprint";
    param_obj.desired_pose.header.seq = 0;


    param_obj.contact_pose.header.frame_id = "base_footprint";
    param_obj.contact_pose.header.seq = 0;

    float roll = 0, pitch = 0, yaw = 0;
    Wml::Vector3f xz_projection = Wml::Vector3f::ZERO;
    Wml::Vector3f yz_projection = Wml::Vector3f::ZERO;

    Wml::Vector3f ux = Wml::Vector3f::UNIT_X;
    Wml::Vector3f uy = Wml::Vector3f::UNIT_Y;
    Wml::Vector3f uz = Wml::Vector3f::UNIT_Z;

    while(ros::ok()){

        if(!param_obj.firstCb){
// Project trajectories to the robot's Task space with Find3D function 
                //For Static Experiments use this projection function
            param_obj.calculated_position = param_obj.static_expmapgen.Find3D(param_obj.static_input_point, &param_obj.desired_normal, &param_obj.Status);
                std::cout << "--- Calculated_position  = " << param_obj.calculated_position << std::endl;

            if(!param_obj.secondCb){
                // For Dynamic Experiments use this projection function: First project to Dynamic Canonical and then to 3D Task space of the robot
               param_obj.dynamic_input_point = param_obj.static_expmapgen.ProjectToDynamic( param_obj.static_input_point, &param_obj.expmapgen.m_uvMesh, &param_obj.Dynamic_proj_Status );
               param_obj.calculated_position = param_obj.expmapgen.Find3D(param_obj.dynamic_input_point, &param_obj.desired_normal, &param_obj.Status);
               if(visualize){
                   param_obj.dynamic_vis_pose.header.stamp = ros::Time::now();
                   param_obj.dynamic_vis_pose.pose.orientation.x = 0.0;
                   param_obj.dynamic_vis_pose.pose.orientation.y = 1.0;
                   param_obj.dynamic_vis_pose.pose.orientation.z = 0.0;
                   param_obj.dynamic_vis_pose.pose.orientation.w = 0.0;

                   param_obj.dynamic_vis_pose.pose.position.x = param_obj.dynamic_input_point.X();
                   param_obj.dynamic_vis_pose.pose.position.y = param_obj.dynamic_input_point.Y();
                   param_obj.dynamic_vis_pose.pose.position.z = param_obj.normal_offset;

                   param_obj.dynamic_path.poses.push_back(param_obj.dynamic_vis_pose);
                   param_obj.dynamic_path_pub.publish(param_obj.dynamic_path);
               }

                // std::cout << "--- Dynamic projection position  = " << param_obj.dynamic_input_point << std::endl;
                // std::cout << "--- Calculated_position  = " << param_obj.calculated_position << std::endl;
            }
            if (param_obj.Status){

                // Generate Pose msg
                param_obj.desired_position = param_obj.calculated_position;
                param_obj.contact_pose.header.stamp = ros::Time::now();
                param_obj.contact_pose.pose.position.x = param_obj.desired_position.X() + param_obj.desired_normal.X()*param_obj.normal_offset;
                param_obj.contact_pose.pose.position.y = param_obj.desired_position.Y() + param_obj.desired_normal.Y()*param_obj.normal_offset;
                param_obj.contact_pose.pose.position.z = param_obj.desired_position.Z() + param_obj.desired_normal.Z()*param_obj.normal_offset;

                param_obj.desired_pose.header.stamp = param_obj.contact_pose.header.stamp;

                // Calculate roll and pitch values as part of the overall orientation provided to the robot
                xz_projection = param_obj.desired_normal - param_obj.desired_normal.Dot(uy)*uy;
                pitch = std::acos(xz_projection.Dot(uz)/xz_projection.Length());
                if (xz_projection.Dot(ux) < 0) pitch = -pitch;
                pitch = pitch + PI;

                yz_projection = param_obj.desired_normal - param_obj.desired_normal.Dot(ux)*ux;
                roll = std::acos(yz_projection.Dot(uz)/yz_projection.Length());
                if (yz_projection.Dot(uy) > 0) roll = -roll;
                roll = -roll;

                //Apply this correction to yaw orientation due to 5DOF robot
                if (param_obj.desired_pose.pose.position.x != 0){
                    yaw = std::atan2(param_obj.desired_pose.pose.position.y,param_obj.desired_pose.pose.position.x);
                }
                else{
                    yaw = 1.57079632679;
                }

                param_obj.contact_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( roll,  pitch,  yaw);
                param_obj.contact_pose_pub.publish(param_obj.contact_pose);
                if((visualize)&&(param_obj.start_publish)){
                    param_obj.physical_path.poses.push_back(param_obj.contact_pose);
                    param_obj.physical_path_pub.publish(param_obj.physical_path);
                }
                //Pose correction for WidowX_arm with contact gripper
                param_obj.base_to_contact.getOrigin()[0] = param_obj.contact_pose.pose.position.x;
                param_obj.base_to_contact.getOrigin()[1] = param_obj.contact_pose.pose.position.y;
                param_obj.base_to_contact.getOrigin()[2] = param_obj.contact_pose.pose.position.z;

                param_obj.base_to_contact.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));

                //Apply correction transformation for WidowX robot with contact gripper
                param_obj.desired_trans =  param_obj.base_to_contact *  param_obj.contact_to_desired;
                param_obj.desired_trans.getBasis().getRotation( param_obj.desired_trans_q);


                param_obj.desired_pose.pose.position.x = param_obj.desired_trans.getOrigin()[0];
                param_obj.desired_pose.pose.position.y = param_obj.desired_trans.getOrigin()[1];
                param_obj.desired_pose.pose.position.z = param_obj.desired_trans.getOrigin()[2];

                // /std::cout << "--- Calculated_pose  = " << roll << " , " << pitch << " , " << yaw  << std::endl;
                param_obj.desired_pose.pose.orientation.x = param_obj.desired_trans_q.x();
                param_obj.desired_pose.pose.orientation.y = param_obj.desired_trans_q.y();
                param_obj.desired_pose.pose.orientation.z = param_obj.desired_trans_q.z();
                param_obj.desired_pose.pose.orientation.w = param_obj.desired_trans_q.w();

                if (param_obj.start_publish)
                    param_obj.desired_pose_pub.publish(param_obj.desired_pose);

                if (false)
                {
                    geometry_msgs::Point base, tip;

                    base.x = param_obj.desired_position.X();
                    base.y = param_obj.desired_position.Y();
                    base.z = param_obj.desired_position.Z();

                    tip.x = base.x + param_obj.desired_normal.X()*std::sqrt(0.01);
                    tip.y = base.y + param_obj.desired_normal.Y()*std::sqrt(0.01);
                    tip.z = base.z + param_obj.desired_normal.Z()*std::sqrt(0.01);
                    param_obj.normalvec_vis.points.push_back(base);
                    param_obj.normalvec_vis.points.push_back(tip);

                    param_obj.normal_visual_pub.publish(param_obj.normalvec_vis);
                    param_obj.normalvec_vis.points.clear();

                }


            }else{
                std::cout << "--- Static input point is out of bounds ---" << std::endl;
                std::cout << "Publishing the last valid pose ..." << std::endl;
                if (param_obj.start_publish)
                    param_obj.desired_pose_pub.publish(param_obj.desired_pose);
            }
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

