# Realtime_Segmentation_RealSense_Camera
Tutorial on setting up realtime segmentation using a RealSense Camera

Outcome
Initial Goal: Segmenting out point clouds in real time using a Realsense camera. I used Intel Realsense Lidar Camera, model L515. I used a Linux operating system installed on a Windows computer.
Realized Outcome: Success
Steps to achieve outcome
Using Bash. This tutorial involves using several terminals to run multiple ROS commands. In each new terminal you create, setup your bash within the root of your catkin workspace:
cd ~/catkin_ws
source devel/setup.bash


Setup
Installing ROS wrapper for Intel Realsense Devices — use ROS wrapper for Intel Realsense Devices as a reference. Run:
 sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

Installing Intel Realsense SDK 2.0 — use Intel realsense SDK 2.0 installation as a reference. Run:
sudo apt-key adv --keyserver keys.gnupg.net --recv-key     F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb <https://librealsense.intel.com/Debian/apt-repo> $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
modinfo uvcvideo | grep "version:"
sudo apt-get update
sudo apt-get upgrade

Build your catkin workspace to build the realsense package, then navigate to the launch folder in realsense-ros/realsense2_camera/: 
cd ~catkin_ws
catkin_make

...

cd ~/catkin_ws/src/realsense-ros/realsense2_camera/launch

Create a launch file for the L515, and change the default value of the arg name=”device_type” parameter to “L515”.
Download the segmentation file from the PCL-ROS-cluster-Segmentation repository, and build your workspace:
cd ~/catkin_ws/src
git clone <https://github.com/CircuitLaunch/PCL-ROS-cluster-Segmentation.git>
cd ..
catkin_make

Using Rviz to display camera feed and perform segmentation
Run your launch file with an additional filter as an argument. My launch file for the Realsense camera was called "rs_lidar_L515.launch".
 roslaunch realsense2_camera rs_lidar_L515.launch filters:=pointcloud

In a separate terminal, run rviz and display the camera/depth/color/points/PointCloud2 topic. Make sure to change the fixed frame dragdown under global options from map → camera_link


Run your segmentation file. You should see a list of centroid information displayed to the terminal, labeled as "c_<number>_getter".
rosrun obj_recognition obj_recognition_segmentation



Add visualizations of the clusters to rviz by adding topics within the GUI. After clicking add, navigate to the list of topics being broadcasted to ROS. Under object recognition, you should see a list of topics titled "dummy_cluster_<number>/PointCloud2 — add these to visualize different numbers of point clouds. You can also visualize the plane that is filtered out to allow for segmentation by adding the plane_cluster/PointCloud2 topic.

You should see a sequence of objects being detected by the camera as point cloud clusters -- feel free to mess around with parameters inside your segmentation file to find the right fit. If you decide to do so, make sure to rebuild your workspace with catkin_make each time.

 
Adding additional clouds for visualization. Currently, segmentation.cpp publishes a finite number of clusters to ROS; there might be more clusters than indicated by the feed. You can modify segmentation.cpp in order to visualize these potential additional clusters. This was the process for adding and publishing the dummy_cluster topic:
Under the segmentation class definition, under explicit segmentation, add the publisher object for dummy_cluster:
dummy_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster",1);

Under private, add:
ros::Publisher dummy_pub;
Within the segmentation::cloud_cb function, add:
sensor_msgs::PointCloud2 output;
output.header = cloud_msg->header;
pcl::PCLPointCloud2 outputPCL;
// Implementing centroid
pcl::CentroidPoint<pcl::PointXYZ> output_centroid;
pcl::PointXYZRGB c_getter;
pcl::PointXYZ centroid_adder;

Within the inner portion of the for loop structure (contained within the segmentation::cloud_cb function), add:
pcl::copyPoint(xyzpoint, centroid_adder);
output_centroid.add (centroid_adder);

Within the outer portion of the for loop structure (contained within the segmentation::cloud_cb function), add:
// convert to pcl::PCLPointCloud2
pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
// Convert to ROS data type
pcl_conversions::fromPCL(outputPCL, output);
// Reset Headers
output.header = cloud_msg->header;

// Running the publisher, and displaying the centroid to the terminal
dummy_pub.publish(output);
output_centroid.get(c_getter);
std::cout << "c_getter: " << c_getter << std::endl;

Notes:
The segmentation works — if a plane exists in the camera feed, the code will remove it from the overall point cloud, and then publish a portion of the available clusters that the realsense camera detects. However, currently parameters need to be tested to make sure specific object can be detected.
Depending on the location of the camera, it might detect objects outside of the removed plane.
I achieved best results by having the realsense camera focus on the floor, ie some flat surface that is relatively uniform for a large space.
Within segmentation.cpp, there is some commented out dummy code that was the starting implementation for object detection. Feel free to work on it.
References:
ROS wrapper for intel realsense devices -- https://github.com/IntelRealSense/realsense-ros
Intel realsense SDK 2.0 installation -- https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
CirctuitLaunch/PCL-ROS-Segmentation github page — https://github.com/CircuitLaunch/PCL-ROS-cluster-Segmentation

