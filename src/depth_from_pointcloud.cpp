#include <math.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>

// BAD HEADERS
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#pragma GCC diagnostic pop
// END BAD HEADERS

#include <geometry_msgs/Vector3Stamped.h>

#define PI 3.14159265

const std::string point_cloud_topic = "/d435i/depth/color/points";
const std::string filtered_point_cloud_topic =
    "/depth_estimation/point_cloud/filtered";
const std::string pose_topic = "/depth_estimation/point_cloud/pose";
const std::string heading_angle_topic =
    "/depth_estimation/point_cloud/heading_angle";

// z direction
double max_pc = 10.0;
double min_pc = -1.0;

// x depth out
double max_depth = 10.0;
double min_depth = 0.2;

// downsample amount
double downsample = 2.0;

bool DEBUG = false;

bool has_transform = false;

geometry_msgs::Vector3Stamped Pose_center;
std_msgs::Float32 heading_angle;
sensor_msgs::PointCloud2 filtered_msg;

ros::Publisher Pose_pub_;
ros::Publisher Heading_angle;
ros::Publisher seg_point_cloud;
ros::Publisher filtered_cloud;

tf::StampedTransform transform;

void initialize_pose_with_nan(geometry_msgs::Vector3Stamped& Pose_center) {
    Pose_center.vector.x = std::numeric_limits<float>::quiet_NaN();
    Pose_center.vector.y = std::numeric_limits<float>::quiet_NaN();
    Pose_center.vector.z = std::numeric_limits<float>::quiet_NaN();
}

void initialize_heading_angle_with_nan(std_msgs::Float32& ang) {
    ang.data = std::numeric_limits<float>::quiet_NaN();
}

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (has_transform) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        size_t num_points = cloud->size();
        int samples = (int)num_points / downsample;

        // transformed & filtered
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        // voxelized
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(
            new pcl::PointCloud<pcl::PointXYZ>);

        // downsample randomly
        pcl::RandomSample<pcl::PointXYZ> random_sample;
        random_sample.setInputCloud(cloud);
        random_sample.setSample(samples);
        random_sample.setSeed(rand());
        random_sample.filter(*cloud_filtered);

        pcl_ros::transformPointCloud(*cloud_filtered, *cloud_filtered,
                                     transform);

        // filter cloud in x (depth) and z (remove ground)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_pc, max_pc);
        pass.filter(*cloud_filtered);

        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_depth, max_depth);
        pass.filter(*cloud_filtered);

        // published filtered cloud for debug
        if (DEBUG) {
            pcl::toROSMsg(*cloud_filtered, filtered_msg);
            filtered_msg.header = msg->header;
            filtered_msg.header.frame_id = "base_link";
            filtered_cloud.publish(filtered_msg);
        }

        // voxelize cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setLeafSize(0.01, 0.01, 0.01);
        sor.filter(*cloud_voxel);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Set segmentation model and method
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(.1);
        seg.setInputCloud(cloud_voxel);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() != 0) {
            pcl::PointXYZ p_centroid;
            pcl::computeCentroid(*cloud_voxel, inliers->indices, p_centroid);

            // check still within limits
            if (p_centroid.x > min_depth && p_centroid.x < max_depth) {
                // depth is x component but everything else expects
                // depth in z field, so hack it in
                Pose_center.header.stamp = msg->header.stamp;
                Pose_center.vector.x = p_centroid.x;
                Pose_center.vector.y = p_centroid.y;
                Pose_center.vector.z = p_centroid.x;  // HACK

                float A = coefficients->values[0];
                float B = coefficients->values[1];

                // find heading
                heading_angle.data = atan2(A, B);

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
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_seg");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ROS_DEBUG_COND(DEBUG, "PlaneSeg: Node starting...");

    Pose_pub_ = nhPriv.advertise<geometry_msgs::Vector3Stamped>(pose_topic, 1);
    Heading_angle = nhPriv.advertise<std_msgs::Float32>(heading_angle_topic, 1);

    nhPriv.getParam("downsample", downsample);

    nhPriv.getParam("max_z", max_pc);
    nhPriv.getParam("min_z", min_pc);

    nhPriv.getParam("max_depth", max_depth);
    nhPriv.getParam("min_depth", min_depth);

    nhPriv.getParam("debug", DEBUG);

    ROS_INFO_STREAM("Downsample: " << downsample << " Max Z: " << max_pc
                                   << " Min Z: " << min_pc << " Max Depth: "
                                   << max_depth << " Min Depth: " << min_depth);

    if (DEBUG) {
        filtered_cloud = nhPriv.advertise<sensor_msgs::PointCloud2>(
            filtered_point_cloud_topic, 1);
    }

    ros::Subscriber sub =
        nhPriv.subscribe(point_cloud_topic, 10, pointcloud_callback);

    tf::TransformListener listener;
    ros::Rate rate(5.0);

    // static transform, no need to always get it
    while (nhPriv.ok() && !has_transform) {
        try {
            listener.lookupTransform("base_link", "d435i_depth_optical_frame",
                                     ros::Time(), transform);
            has_transform = true;
            ROS_INFO_ONCE("DepthEstimationPointCloud: Has transform");
        } catch (tf::TransformException ex) {
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
