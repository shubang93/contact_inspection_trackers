
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>


#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


using namespace message_filters;
using namespace sensor_msgs;
using namespace std;
using namespace cv;


const std::string color_image_topic = "/front_depth_camera/color/image_raw";
const std::string depth_image_topic = "/front_depth_camera/aligned_depth_to_color/image_raw";
double focal_length = 619.2664184570312; 
double Q[4][4] = {{1,0,0,0},{0,-1,0,0},{0,0,-focal_length*0.03,0}, {0,0,0,1}};
// TODO: ReAD FOCAL lENGTH FROM CAmERa INFO
cv::Mat Q2 = cv::Mat(4, 4, CV_64F, Q);

void MatType( cv::Mat inputMat )
{
    int inttype = inputMat.type();

    string r, a;
    uchar depth = inttype & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (inttype >> CV_CN_SHIFT);
    switch ( depth ) {
        case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;  
        case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;  
        case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break; 
        case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break; 
        case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break; 
        case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break; 
        case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break; 
        default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break; 
    }   
    r += "C";
    r += (chans+'0');
    std::cout << "Mat is of type " << r << " and should be accessed with " << a << endl;

}

void construct_point_cloud(const ImageConstPtr& color,const ImageConstPtr& depth)
        {
            ROS_INFO_STREAM("Hello callback");

            cv_bridge::CvImagePtr color_ptr;
            cv_bridge::CvImagePtr depth_ptr;
            color_ptr = cv_bridge::toCvCopy(color, image_encodings::BGR8);
            if (depth->encoding == "16UC1"){
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
            MatType(color_ptr->image);
            MatType(depth_ptr->image);

                
            CV_Assert(!depth_ptr->image.empty());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            cv::Mat XYZ(depth_ptr->image.size(),CV_32FC3);
            // reprojectImageTo3D(disparity8U, XYZ, Q, false, CV_32F);
            reprojectImageTo3D(depth_ptr->image, XYZ, Q2, false, CV_32F );

            //Reconstruct PointCloud with the depthmap points
            for (int i = 0; i < depth_ptr->image.rows; ++i)
            {
                for (int j = 0; j < depth_ptr->image.cols; ++j)
                {
                    pcl::PointXYZRGB p;

                    //The coordinate of the point is taken from the depth map
                    //Y and Z  taken negative to immediately visualize the cloud in the right way

                    if ((depth_ptr->image.at<float>(i,j))>0){
                        ROS_INFO("%d, %d, %f",i,j,depth_ptr->image.at<float>(i,j) );
                        p.x = i;
                        p.y = j;
                        p.z = XYZ.at<float>(i,j);
                        cv::Vec3b pixel = color_ptr->image.at<cv::Vec3b>(i,j);
                        //Coloring the point with the corrispondent point in the rectified image
                        p.r = static_cast<uint8_t>(pixel[2]);
                        p.g = static_cast<uint8_t>(pixel[1]);
                        p.b = static_cast<uint8_t>(pixel[0]);
                    }
                    
                    

                    // Insert point in the cloud, cutting the points that are too distant
                    // if(( abs( p.x ) < 500 )&&( abs( p.y ) < 200 )&&( abs( p.z ) < 500 ))
                    cloud->points.push_back(p);
                }
            }
            cloud->width = depth_ptr->image.cols;
            cloud->height = depth_ptr->image.rows;
            pcl::io::savePLYFileBinary("/home/sanjana/trackers/src/contact_inspection_trackers/output2.ply", *cloud);
        }
       

int main(int argc, char** argv)
{   ROS_INFO("Hello main");
    ros::init(argc, argv, "point_cloud_save");

    ros::NodeHandle nh_;
    message_filters::Subscriber<Image> color_sub(nh_, color_image_topic, 1);
    message_filters::Subscriber<Image> depth_sub(nh_, depth_image_topic, 1);
    
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), color_sub, depth_sub);
    sync.registerCallback(boost::bind(&construct_point_cloud, _1, _2));
    
    ros::Subscriber bbox_coords;
    
    ros::spin();
    return 0;
}
