ROS package for [**OpenPose**](https://github.com/CMU-Perceptual-Computing-Lab/openpose)

============================

## Usage
 - [**Build openpose**](https://github.com/CMU-Perceptual-Computing-Lab/openpose#installation-reinstallation-and-uninstallation)
 - Catkin b [**openpose_ros_msgs**](https://github.com/bajsk/openpose_ros_msgs)
 - Catkin b openpose_ros (The package needs catkin_simple, gflags, glog_catkin)
   + git clone https://github.com/ethz-asl/glog_catkin.git
   + git clone https://github.com/davetcoleman/gflags.git
   + git clone https://github.com/catkin/catkin_simple.git
   
 - Run launch file with `roslaunch openpose_ros openpose_ros.launch`
 - Output topic
   + Debug Image: /openpose_ros/debug_img
   <p align="center">
    <img src="doc/media/openpose.jpg", width="480">
   </p>
   
   + Skeleton Information: /openpose_ros/skeleton
