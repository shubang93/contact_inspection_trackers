
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
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
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

#include <iostream>
#include <Eigen/Dense>

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

const bool DEBUG = false;

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

void construct_point_cloud(const ImageConstPtr& depth,
                           const CameraInfoConstPtr& cam_info) {


    // ROS_DEBUG_STREAM_COND(DEBUG,
    //                       "DepthEstimation: Focal Length " << focal_length);
    cv_bridge::CvImagePtr depth_ptr;
    std::string encoding;

    // Constuct new ros type for seamless encoding from 16UC1-> 8UC1(MONO8)
    if (depth->encoding == "16UC1") {
        // sensor_msgs::Image img;
        // img.header = depth->header;
        // img.height = depth->height;
        // img.width = depth->width;
        // img.is_bigendian = depth->is_bigendian;
        // img.step = depth->step;
        // img.data = depth->data;
        // img.encoding = "mono16";

        depth_ptr = cv_bridge::toCvCopy(depth, encoding = "32FC1");
    } 

    CV_Assert(!depth_ptr->image.empty());
    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imwrite( "/home/sanjana/window.jpg", depth_ptr->image ); 
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);


    // Reconstruct PointCloud with the depthmap points
    pcl::PointXYZ p;

    ROS_DEBUG_STREAM_COND(DEBUG, "DepthEstimation: Creating point cloud");

    // #pragma omp parallel for
    for (int i = 0; i < depth_ptr->image.rows; ++i) {
        for (int j = 0; j < depth_ptr->image.cols; ++j) {
            // The coordinate of the point is taken from the depth map

                // std::cerr<<depth_ptr->image.at<float>(i,j);
                p.z = (depth_ptr->image.at<float>(i,j))/1000;

                double cx = cam_info->P[2];
                double cy = cam_info->P[5];
                double fx = cam_info->P[0];
                double fy = cam_info->P[6];
                double x = (i - cx) * p.z / fx;
                double y = (j - cy) * p.z / fy;
                p.x = x;
                p.y = y;
                // std::cerr<<" x:"<<p.x<<" y: "<<p.y<<" z: "<<p.z<<" i: "<<i<<" j: "<<j<<" fx: "<<fx<<" fy: "<<fy<<endl;
                if (p.z>0.1 && p.z<10){
                cloud->points.push_back(p);}
            }
    }
    

    cloud->width = cloud->points.size();
    cloud->height = 1;

    // Down Sample point cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, CLOUD);
    CLOUD.header.frame_id = depth_ptr->header.frame_id;
    CLOUD.header.stamp = depth_ptr->header.stamp;
    point_cloud.publish(CLOUD);

    //______________________________________________________________________________________________
    // Code to segment out dominant plane
    //______________________________________________________________________________________________

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    // Set model and method
    
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    seg.setAxis(axis);
    seg.setEpsAngle(0.26); 
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() != 0) {
        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        pcl::toROSMsg(*cloud_p,SEGCLOUD);
        SEGCLOUD.header.frame_id = depth_ptr->header.frame_id;
        SEGCLOUD.header.stamp = depth_ptr->header.stamp;
        seg_point_cloud.publish(SEGCLOUD);

        pcl::PointXYZ p_centroid;
        pcl::computeCentroid(*cloud_filtered, inliers->indices, p_centroid);

        if (p_centroid.z > 0.1 && p_centroid.z < 10) {
            Pose_center.header.stamp = depth_ptr->header.stamp;
            Pose_center.vector.x = p_centroid.x;
            Pose_center.vector.y = p_centroid.y;
            Pose_center.vector.z = p_centroid.z;

            float A = coefficients->values[0];
            float B = coefficients->values[1];
            float C = coefficients->values[2];
            float D = coefficients->values[3];

            ROS_DEBUG_STREAM_COND(
                DEBUG, "DepthEstimation: ABC: " << A << " " << B << " " << C
                                                << " " << D);
            
            float temp = (acos(C / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2)))*180/PI);
            if(temp<=90)
            {heading_angle.data=temp;}
            else
            {heading_angle.data= -(180.0-temp);}
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
    ros::init(argc, argv, "depth_from_rgbd");
    ros::NodeHandle nhPriv("~");

    ROS_DEBUG_COND(DEBUG, "DepthEstimation: Node starting...");

    Pose_pub_ = nhPriv.advertise<geometry_msgs::Vector3Stamped>(pose_topic, 1);
    Heading_angle = nhPriv.advertise<std_msgs::Float32>(heading_angle_topic, 1);
    
    point_cloud =
        nhPriv.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);

    seg_point_cloud =
       nhPriv.advertise<sensor_msgs::PointCloud2>(seg_point_cloud_topic, 1);

    nhPriv.getParam("depth_image_topic", depth_image_topic);
    nhPriv.getParam("depth_camera_info_topic", depth_camera_info_topic);

    message_filters::Subscriber<Image> depth_sub(nhPriv, depth_image_topic, 1);
    message_filters::Subscriber<CameraInfo> depth_cam_info_sub(
        nhPriv, depth_camera_info_topic, 1);

    typedef sync_policies::ExactTime<Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub,
                                    depth_cam_info_sub);
    sync.registerCallback(boost::bind(&construct_point_cloud, _1, _2));

    ros::spin();
    return 0;
}
