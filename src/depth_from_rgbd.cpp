
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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// BAD HEADERS
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#pragma GCC diagnostic pop
// END BAD HEADERS

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>



#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection2D.h>

#define PI 3.14159265

using namespace message_filters;
using namespace sensor_msgs;
using namespace vision_msgs;
using namespace geometry_msgs;
using namespace std;
using namespace cv;


std::string depth_image_topic;
std::string depth_camera_info_topic;
const std::string point_cloud_topic = "point_cloud/pointcloud";
const std::string seg_point_cloud_topic = "point_cloud/segpointcloud";
const std::string pose_topic = "point_cloud/pose";
const std::string heading_angle_topic = "point_cloud/heading_angle";


Vector3Stamped Pose_center;
std_msgs::Float32 heading_angle;
sensor_msgs::PointCloud2 CLOUD;
sensor_msgs::PointCloud2 SEGCLOUD;
ros::Publisher Pose_pub_;
ros::Publisher Heading_angle;
ros::Publisher point_cloud;
ros::Publisher seg_point_cloud;


void MatType(cv::Mat inputMat) {
    int inttype = inputMat.type();

    string r, a;
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);
    switch (depth) {
        case CV_8U:
            r = "8U";
            a = "Mat.at<uchar>(y,x)";
            break;
        case CV_8S:
            r = "8S";
            a = "Mat.at<schar>(y,x)";
            break;
        case CV_16U:
            r = "16U";
            a = "Mat.at<ushort>(y,x)";
            break;
        case CV_16S:
            r = "16S";
            a = "Mat.at<short>(y,x)";
            break;
        case CV_32S:
            r = "32S";
            a = "Mat.at<int>(y,x)";
            break;
        case CV_32F:
            r = "32F";
            a = "Mat.at<float>(y,x)";
            break;
        case CV_64F:
            r = "64F";
            a = "Mat.at<double>(y,x)";
            break;
        default:
            r = "User";
            a = "Mat.at<UKNOWN>(y,x)";
            break;
    }
    r += "C";
    r += (chans + '0');
}

void initialize_pose_with_nan(Vector3Stamped& Pose_center) {
    Pose_center.vector.x = numeric_limits<float>::quiet_NaN();
    Pose_center.vector.y = numeric_limits<float>::quiet_NaN();
    Pose_center.vector.z = numeric_limits<float>::quiet_NaN();
}

void initialize_heading_angle_with_nan(std_msgs::Float32& ang) {
    ang.data = numeric_limits<float>::quiet_NaN();
}

void construct_point_cloud(const ImageConstPtr& depth, const CameraInfoConstPtr& cam_info) {   

    
    const double focal_length = (cam_info->K[0]+ cam_info->K[4])/2; //350.2664184570312;
    std::cerr<<"Focal length"<<focal_length;

    double Q[4][4] = {
    {1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, focal_length * 0.078, 0}, {0, 0, 0, 1}};

    // TODO: Read focal length from camera info
    cv::Mat Q2 = cv::Mat(4, 4, CV_64F, Q);
    cv_bridge::CvImagePtr depth_ptr;

    // Constuct new ros type for seamless encoding from 16UC1-> 8UC1(MONO8)
    if (depth->encoding == "16UC1") {
        sensor_msgs::Image img;
        img.header = depth->header;
        img.height = depth->height;
        img.width = depth->width;
        img.is_bigendian = depth->is_bigendian;
        img.step = depth->step;
        img.data = depth->data;
        img.encoding = "mono16";

        depth_ptr = cv_bridge::toCvCopy(img, image_encodings::MONO8);
    }

    CV_Assert(!depth_ptr->image.empty());
    cv::Mat XYZ(depth_ptr->image.size(), CV_32FC3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    reprojectImageTo3D(depth_ptr->image, XYZ, Q2, false, CV_32F);

    // Check data type for precision
    // MatType(color_ptr->image);
    // MatType(depth_ptr->image);
    // MatType(XYZ);

    // Reconstruct PointCloud with the depthmap points
    pcl::PointXYZ p;

    std::cerr<<"Creating point cloud"<<endl;
    for (int i = 0; i < depth_ptr->image.rows; ++i) {
        for (int j = 0; j < depth_ptr->image.cols; ++j) 
        {
            
            // The coordinate of the point is taken from the depth map

            if ((depth_ptr->image.at<float>(i, j)) > 0) {
                cv::Vec3f pixeldepth = XYZ.at<cv::Vec3f>(i, j);
                p.x = pixeldepth[0];
                p.y = -pixeldepth[1];
                p.z = pixeldepth[2];
                // ROS_INFO("pixel depth %f",pixeldepth[2]);

                  // Coloring the point with the corrispondent point in the
                // rectified image
                // Enable for PointXYZRGB type
                // cv::Vec3b pixel = color_ptr->image.at<cv::Vec3b>(i, j);
                // p.r = static_cast<uint8_t>(pixel[2]);
                // p.g = static_cast<uint8_t>(pixel[1]);
                // p.b = static_cast<uint8_t>(pixel[0]);
                cloud->points.push_back(p);

            } 
            // else {
            //     // ROS_INFO("%f",depth_ptr->image.at<float>(i,j));
            //     p.x = float(0);
            //     p.y = float(0);
            //     p.z = float(0);

            //     // Enable for PointXYZRGB type
            //     p.r = static_cast<uint8_t>(0);
            //     p.g = static_cast<uint8_t>(0);
            //     p.b = static_cast<uint8_t>(0);
            // }
       
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    // Code to segment out dominant plane

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (70.0f, 70.0f, 70.0f);
    sor.filter (*cloud_filtered);


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
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.9);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    pcl::toROSMsg(*cloud_filtered,CLOUD);
    CLOUD.header.frame_id = depth_ptr->header.frame_id;
    CLOUD.header.stamp = depth_ptr->header.stamp;
    point_cloud.publish(CLOUD);
    
    if (inliers->indices.size() != 0) {
        // Extract the inliers
        // extract.setInputCloud(cloud);
        // extract.setIndices(inliers);
        // extract.setNegative(false);
        // extract.filter(*cloud_p);

        // pcl::toROSMsg(*cloud_p,SEGCLOUD);
        // SEGCLOUD.header.frame_id = depth_ptr->header.frame_id;
        // SEGCLOUD.header.stamp = depth_ptr->header.stamp;
        // seg_point_cloud.publish(SEGCLOUD);
        
        pcl::PointXYZ p_centroid;
        pcl::computeCentroid(*cloud_filtered, inliers->indices, p_centroid);

        if (p_centroid.z > 0 && p_centroid.z < 1000) {
            Pose_center.header.stamp = depth_ptr->header.stamp;
            Pose_center.vector.x = round(p_centroid.x);
            Pose_center.vector.y = round(p_centroid.y);
            Pose_center.vector.z = round(p_centroid.z);

            float A = coefficients->values[0];
            float B = coefficients->values[1];
            float C = coefficients->values[2];
            float D = coefficients->values[3];

            std::cerr<<A<<" "<<B<<" "<<C<<" "<<D<<endl;

            heading_angle.data =
                acos(-C / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2)));

            Pose_pub_.publish(Pose_center);
            Heading_angle.publish(heading_angle);
        } else {
            Pose_center.header.stamp = depth_ptr->header.stamp;
            initialize_pose_with_nan(Pose_center);
            initialize_heading_angle_with_nan(heading_angle);
        }

    } else {
        ROS_WARN("DepthFromRGBD: Could not estimate a planar model");

        initialize_heading_angle_with_nan(heading_angle);
        initialize_pose_with_nan(Pose_center);
        Pose_center.header.stamp = depth_ptr->header.stamp;
    }
}

int main(int argc, char** argv) {

    std::cerr<<"Hello";
    ros::init(argc, argv, "depth_from_rgbd");
    ros::NodeHandle nhPriv("~");
 
    Pose_pub_ = nhPriv.advertise<geometry_msgs::Vector3Stamped>(pose_topic, 1);
    Heading_angle = nhPriv.advertise<std_msgs::Float32>(heading_angle_topic, 1);
    point_cloud = nhPriv.advertise<sensor_msgs::PointCloud2>(point_cloud_topic,1);
    seg_point_cloud = nhPriv.advertise<sensor_msgs::PointCloud2>(seg_point_cloud_topic,1);


    nhPriv.getParam("depth_image_topic", depth_image_topic);
    nhPriv.getParam("depth_camera_info_topic",depth_camera_info_topic);

    message_filters::Subscriber<Image> depth_sub(nhPriv, depth_image_topic , 1);
    message_filters::Subscriber<CameraInfo> depth_cam_info_sub(nhPriv, depth_camera_info_topic, 1);
    typedef sync_policies::ExactTime<Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  depth_sub, depth_cam_info_sub);
    sync.registerCallback(boost::bind(&construct_point_cloud, _1, _2));

    ros::spin();
    return 0;
}
