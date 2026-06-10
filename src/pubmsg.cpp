#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr Ptr;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pubmsg");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100);
  nav_msgs::Odometry odom_msg;

  // wait for subscribers to connect
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    Ptr cloud(new pcl::PointCloud<PointT>);
    // 对文件夹下的所有pcd文件进行读取并发布
    std::string cloud_path = "/mnt/e/SX/dafentech/catkin_2SCPGO_ws/src/bridgePGO/all_pcd_body";
    // 获取该路径下pcd_poses.txt文件中的位姿信息并发布
    std::string pose_path = cloud_path + "/lidar_poses.txt";
    // 对txt文件中每一行位姿（tum格式）进行读取 
    std::ifstream pose_file(pose_path);
    if (!pose_file.is_open()) {
      ROS_ERROR_STREAM("Could not open pose file: " << pose_path);
      break;
    }
    std::string line;
    int count = 0;
    ROS_INFO("Start publishing dataset: %s", cloud_path.c_str());

    // 逐行读取位姿并发布
    while (std::getline(pose_file, line) && ros::ok())
    {
      std::istringstream iss(line);
      double timestamp, tx, ty, tz, qx, qy, qz, qw;
      //行首时间戳为该行位姿对应的点云文件名，解析并发布点云
      if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) { continue; }
      
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6) << timestamp;
      std::string cloud_file = cloud_path + "/" + ss.str() + ".pcd";
      if (pcl::io::loadPCDFile(cloud_file, *cloud) == -1) {
        ROS_WARN_STREAM("Could not load PCD file: " << cloud_file);
        continue;
      }

      // 使用文件里的原始时间戳进行发布
      ros::Time msg_time(timestamp);

      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header.stamp = msg_time;
      cloud_msg.header.frame_id = "base_link";
      pub2.publish(cloud_msg);

      // 发布位姿: base_link 在 map 坐标系下的位姿
      odom_msg.header.stamp = msg_time;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose.position.x = tx;
      odom_msg.pose.pose.position.y = ty;
      odom_msg.pose.pose.position.z = tz;
      odom_msg.pose.pose.orientation.x = qx;
      odom_msg.pose.pose.orientation.y = qy;
      odom_msg.pose.pose.orientation.z = qz;
      odom_msg.pose.pose.orientation.w = qw;
      pub.publish(odom_msg);

      count++;
      if (count % 50 == 0) ROS_INFO("Published %d frames...", count);

      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Finished publishing all %d frames.", count);
    break; // 如果只需要播放一遍，在此跳出外层 while(ros::ok())
  }
  
  // 播放完成后保持节点运行，以便查看 RViz 中的静止结果
  ros::spin(); 
  return 0;
}