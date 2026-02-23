#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <vector>

using namespace std;
string file_name;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node("~");

  ros::Publisher cloud_vis_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/visual_cloud", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  pcl::PointCloud<pcl::PointXYZ> cloud_temp, cloud_visual;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud_temp);
  ROS_WARN_STREAM("PCD File Name: " << file_name);

  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }

  ROS_INFO("SUCCESS LOAD PCD FILE");
  ROS_INFO("PCD Map point size = %d", (int)cloud_temp.points.size());

  cloud_visual.clear();
  for (int i = 0; i < (int)cloud_temp.size(); i++) {
    float z = cloud_temp.points[i].z;
    if (z > -1.0 && z < 5.5)
    {
      cloud_visual.points.push_back(cloud_temp.points[i]);
    }
  }
  cloud_visual.width = cloud_visual.points.size();
  cloud_visual.height = 1;
  cloud_visual.is_dense = true;

  sensor_msgs::PointCloud2 msg_vis;
  pcl::toROSMsg(cloud_visual, msg_vis);
  msg_vis.header.frame_id = "world";

  int count = 0;
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();
    cloud_vis_pub.publish(msg_vis);
    ++count;
    if (count > 10)
    {
      break;
    }
  }
  cout << "finish publish map." << endl;

  return 0;
}