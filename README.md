# contact_inspection_trackers

## 1. Requirements
    Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic. [ROS install](http://wiki.ros.org/ROS/Installation)
    PCL 1.2 or higher
    OpenCV  
  
## 2. Build 
    cd ~/catkin_ws/src
    git clone https://github.com/sanjanamallya/contact_inspection_trackers.git
    cd ../
    catkin build
    source ~/catkin_ws/devel/setup.bash
## 3. Run 
    To run GUI free tracking that just latches to a ROI around the center of the Image:
    roslaunch 2020_trackers perception_roi_center.launch tracker:="1"

    To utilize GUI and a tracking algorithm:
    roslaunch 2020_trackers perception.launch tracker:="args"    
    
    args options for trackers:
    "1": CSRT
    "2": KCF
    "3": Boosting
    "4": MIL
    "5": TLD
    "6": MedianFlow
    "7": MOSSE
## 4. Data 
(https://drive.google.com/drive/u/0/folders/1cx-L1gHehPKWpD6fyoQyGOGPGdAsmsHt)

## 5. Topics Information 

    Currently for depth estimation topics subscribed to from bag file:
    Depth Image msgs: "/d435i/depth/image_rect_raw/"
    Depth Camera Info msgs: "/d435i/depth/camera_info"

    Modify perception_roi_center.launch if these topics are different

    Subscribe to:
    "/depth_estimation/point_cloud/pose": For pose information of the centroid of the segmented dominant plane
    "/depth_estimation/point_cloud/heading_angle": For the angle between the segmented dominant plane and the XY plane 
    "/depth_estimation/point_cloud/pose" : Point cloud of the scene 
    "/depth_estimation/point_cloud/segpointcloud" : Point cloud of the segmented dominant plane

    
    If using tracker:
    "/perception/tracker/bboxImage": For image annoated with tracked bbox
    "/perception/tracker/status": For status of tracker. To check if tracking was successful 


## TODO
    1. Improve tracker performance
    
    
