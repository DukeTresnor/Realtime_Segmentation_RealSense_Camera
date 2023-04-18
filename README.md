# Realtime_Segmentation_RealSense_Camera
Tutorial on setting up realtime segmentation using a RealSense Camera

## Outcome
Initial Goal: Segmenting out point clouds in real time using a Realsense camera. I used Intel Realsense Lidar Camera, model L515. I used a Linux operating system installed on a Windows computer.
Realized Outcome: Success
## Steps to achieve outcome
1. Using Bash. This tutorial involves using several terminals to run multiple ROS commands. In each new terminal you create, setup your bash within the root of your catkin workspace:
`cd ~/catkin_ws
source devel/setup.bash`

2. Setup
a. Installing ROS wrapper for Intel Realsense Devices — use ROS wrapper for Intel Realsense Devices as a reference. Run:
`sudo apt-get install ros-$ROS_DISTRO-realsense2-camera`
b. Installing Intel Realsense SDK 2.0 — use Intel realsense SDK 2.0 installation as a reference. Run:
`sudo apt-key adv --keyserver keys.gnupg.net --recv-key     F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb <https://librealsense.intel.com/Debian/apt-repo> $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
modinfo uvcvideo | grep "version:"
sudo apt-get update
sudo apt-get upgrade`

c. Build your catkin workspace to build the realsense package, then navigate to the launch folder in realsense-ros/realsense2_camera/: 
`cd ~catkin_ws
catkin_make

...

cd ~/catkin_ws/src/realsense-ros/realsense2_camera/launch`

d. Create a launch file for the L515, and change the default value of the arg name=”device_type” parameter to “L515”.
e. Download the segmentation file from the PCL-ROS-cluster-Segmentation repository, and build your workspace:
`cd ~/catkin_ws/src
git clone <https://github.com/CircuitLaunch/PCL-ROS-cluster-Segmentation.git>
cd ..
catkin_make`

3. Using Rviz to display camera feed and perform segmentation
a. Run your launch file with an additional filter as an argument. My launch file for the Realsense camera was called "rs_lidar_L515.launch".
`roslaunch realsense2_camera rs_lidar_L515.launch filters:=pointcloud`

b. In a separate terminal, run rviz and display the camera/depth/color/points/PointCloud2 topic. Make sure to change the fixed frame dragdown under global options from map → camera_link



![image1](https://user-images.githubusercontent.com/11999157/232882797-3e9a36fb-8793-4fc8-9110-7f381e70dcc3.png)





![image2](https://user-images.githubusercontent.com/11999157/232882827-1f100882-3f91-4e34-a5a7-ccdec3fe2c93.png)



c. Run your segmentation file. You should see a list of centroid information displayed to the terminal, labeled as "c_<number>_getter".
`rosrun obj_recognition obj_recognition_segmentation`



![image3](https://user-images.githubusercontent.com/11999157/232882847-b920fd91-c426-4fcd-ac6b-4fd3faf2ce43.png)


d. Add visualizations of the clusters to rviz by adding topics within the GUI. After clicking add, navigate to the list of topics being broadcasted to ROS. Under object recognition, you should see a list of topics titled "dummy_cluster_<number>/PointCloud2 — add these to visualize different numbers of point clouds. You can also visualize the plane that is filtered out to allow for segmentation by adding the plane_cluster/PointCloud2 topic.




![image4](https://user-images.githubusercontent.com/11999157/232882874-cc582cff-4bb1-486b-8014-a849062d4e7a.png)


e. You should see a sequence of objects being detected by the camera as point cloud clusters -- feel free to mess around with parameters inside your segmentation file to find the right fit. If you decide to do so, make sure to rebuild your workspace with catkin_make each time.




![image5](https://user-images.githubusercontent.com/11999157/232882897-81ee764a-16e6-4563-b06f-2087c7c9bb87.png)

4. Adding additional clouds for visualization. Currently, segmentation.cpp publishes a finite number of clusters to ROS; there might be more clusters than indicated by the feed. You can modify segmentation.cpp in order to visualize these potential additional clusters. This was the process for adding and publishing the dummy_cluster topic:
  a. Under the segmentation class definition, under explicit segmentation, add the publisher object for dummy_cluster:
  `dummy_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/dummy_cluster",1);`
 
  b. Under private, add:
`ros::Publisher dummy_pub;`
c. Within the segmentation::cloud_cb function, add:
`sensor_msgs::PointCloud2 output;
output.header = cloud_msg->header;
pcl::PCLPointCloud2 outputPCL;
// Implementing centroid
pcl::CentroidPoint<pcl::PointXYZ> output_centroid;
pcl::PointXYZRGB c_getter;
pcl::PointXYZ centroid_adder;`

d. Within the inner portion of the for loop structure (contained within the segmentation::cloud_cb function), add:
`pcl::copyPoint(xyzpoint, centroid_adder);
output_centroid.add (centroid_adder);`
e. Within the outer portion of the for loop structure (contained within the segmentation::cloud_cb function), add:
`// convert to pcl::PCLPointCloud2
pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
// Convert to ROS data type
pcl_conversions::fromPCL(outputPCL, output);
// Reset Headers
output.header = cloud_msg->header;

// Running the publisher, and displaying the centroid to the terminal
dummy_pub.publish(output);
output_centroid.get(c_getter);
std::cout << "c_getter: " << c_getter << std::endl;`

## Notes:
1. The segmentation works — if a plane exists in the camera feed, the code will remove it from the overall point cloud, and then publish a portion of the available clusters that the realsense camera detects. However, currently parameters need to be tested to make sure specific object can be detected.
2. Depending on the location of the camera, it might detect objects outside of the removed plane.
3. I achieved best results by having the realsense camera focus on the floor, ie some flat surface that is relatively uniform for a large space.
4. Within segmentation.cpp, there is some commented out dummy code that was the starting implementation for object detection. Feel free to work on it.

## References:
1. ROS wrapper for intel realsense devices -- https://github.com/IntelRealSense/realsense-ros
2. Intel realsense SDK 2.0 installation -- https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md
3. CirctuitLaunch/PCL-ROS-Segmentation github page — https://github.com/CircuitLaunch/PCL-ROS-cluster-Segmentation
