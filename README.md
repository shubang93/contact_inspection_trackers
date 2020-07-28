# contact_inspection_trackers

## 1 Requirements
  Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic. [ROS install](http://wiki.ros.org/ROS/Installation)
  PCL 1.2 or higher
  OpenCV  
  
## 2 Build 
    cd ~/catkin_ws/src
    git clone https://github.com/sanjanamallya/contact_inspection_trackers.git
    cd ../
    catkin build
    source ~/catkin_ws/devel/setup.bash
## 3 Run 
    roslaunch 2020_trackers perception.launch tracker:="args"    
    
    args options for trackers:
    "1": CSRT
    "2": KCF
    "3": Boosting
    "4": MIL
    "5": TLD
    "6": MedianFlow
    "7": MOSSE
## 4 Data 
https://drive.google.com/drive/u/0/folders/1cx-L1gHehPKWpD6fyoQyGOGPGdAsmsHt

## Topics Information 

Topics subscribed to from bag file:
Color Image msgs: "/front_depth_camera/color/image_raw"
Depth Image msgs: "/front_depth_camera/aligned_depth_to_color/image_raw"

Modify perception.launch if these topics are different

Subscribe to:
"point_cloud/pose": For pose information of the plane segmente from poi generated from ROI
"/perception/tracker/bboxOut": For bbox info from tracker
"/perception/tracker/bboxImage": For image annoated with tracked bbox
"/perception/tracker/status": For status of tracker. To check if tracking was successful 


## TODO
1. Improve tracker performance
2. Heading estimation 
3. point cloud filtering
    
