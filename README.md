# contact_inspection_trackers

## 1 Ubuntu and ROS
  Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic. [ROS install]http://wiki.ros.org/ROS/Installation
  
## 2 Build 
    cd ~/catkin_ws/src
    git clone 
    cd ../
    catkin build
    source ~/catkin_ws/devel/setup.bash
## 3 Run 
    roslaunch 2020_trackers perception.launch tracker:="args"    
    args options:
    "1": CSRT
    "2": KCF
    "3": Boosting
    "4": MIL
    "5": TLD
    "6": cMedianFlow
    "7": MOSSE
## 4 Data 

## TODO
1. OpenCV Goturn
2. Other deep-learning based trackers 
    
