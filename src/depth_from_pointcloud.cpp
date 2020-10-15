#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>

// BAD HEADERS
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#pragma GCC diagnostic pop
// END BAD HEADERS

#include <geometry_msgs/Vector3Stamped.h>

#define PI 3.14159265

const std::string point_cloud_topic = "/d435i/depth/color/points";
const std::string seg_point_cloud_topic = "/depth_estimation/point_cloud/segpointcloud";
const std::string pose_topic = "/depth_estimation/point_cloud/pose";
const std::string heading_angle_topic = "/depth_estimation/point_cloud/heading_angle";

const bool DEBUG = false;

geometry_msgs::Vector3Stamped Pose_center;
std_msgs::Float32 heading_angle;

sensor_msgs::PointCloud2 SEGCLOUD;

ros::Publisher Pose_pub_;
ros::Publisher Heading_angle;
ros::Publisher seg_point_cloud;

void initialize_pose_with_nan(geometry_msgs::Vector3Stamped& Pose_center) {
    Pose_center.vector.x = std::numeric_limits<float>::quiet_NaN();
    Pose_center.vector.y = std::numeric_limits<float>::quiet_NaN();
    Pose_center.vector.z = std::numeric_limits<float>::quiet_NaN();
}

void initialize_heading_angle_with_nan(std_msgs::Float32& ang) {
    ang.data = std::numeric_limits<float>::quiet_NaN();
}

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Code to segment out dominant plane

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(.0f, 70.0f, 70.0f);
    sor.filter(*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    // Set model and method
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(10);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() != 0) {
        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_seg);

        pcl::toROSMsg(*cloud_seg, SEGCLOUD);
        SEGCLOUD.header = msg->header;
        seg_point_cloud.publish(SEGCLOUD);

        pcl::PointXYZ p_centroid;
        pcl::computeCentroid(*cloud_filtered, inliers->indices, p_centroid);

        if (p_centroid.z > 1 && p_centroid.z < 10000) {
            Pose_center.header.stamp = msg->header.stamp;
            Pose_center.vector.x = round(p_centroid.x);
            Pose_center.vector.y = round(p_centroid.y);
            Pose_center.vector.z = round(p_centroid.z);

            float A = coefficients->values[0];
            float B = coefficients->values[1];
            float C = coefficients->values[2];
            float D = coefficients->values[3];

            ROS_DEBUG_STREAM_COND(
                DEBUG, "PlaneSeg: ABC: " << A << " " << B << " " << C
                                                << " " << D);

            heading_angle.data =
                acos(-C / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2)));

            Pose_pub_.publish(Pose_center);
            Heading_angle.publish(heading_angle);
        } else {
            Pose_center.header.stamp = msg->header.stamp;
            initialize_pose_with_nan(Pose_center);
            initialize_heading_angle_with_nan(heading_angle);
        }

    } else {
        ROS_WARN("PlaneSeg: Could not estimate a planar model");

        initialize_heading_angle_with_nan(heading_angle);
        initialize_pose_with_nan(Pose_center);
        Pose_center.header.stamp = msg->header.stamp;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_seg");
    ros::NodeHandle nhPriv("~");

    ROS_DEBUG_COND(DEBUG, "PlaneSeg: Node starting...");

    Pose_pub_ = nhPriv.advertise<geometry_msgs::Vector3Stamped>(pose_topic, 1);
    Heading_angle = nhPriv.advertise<std_msgs::Float32>(heading_angle_topic, 1);

    seg_point_cloud =
        nhPriv.advertise<sensor_msgs::PointCloud2>(seg_point_cloud_topic, 1);

    ros::Subscriber sub = nhPriv.subscribe(point_cloud_topic, 10, pointcloud_callback);

    ros::spin();
    return 0;
}
