#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv){
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle n;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ros::Publisher map_pub = n.advertise<PointCloud>("map", 10);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/mahua/map.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
  }else{
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    std::cout << "frame_id :"
              << cloud -> header.frame_id 
              << std::endl;
              /*
    for (const auto& point: *cloud)
      std::cout << "    " << point.x
                << " "    << point.y
                << " "    << point.z << std::endl;
                */
  }
  cloud->header.frame_id = "tf_frame";

  ros::Rate loop_rate(1);




  pcl::PCLPointCloud2::Ptr cloud_original (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("/home/mahua/map.pcd", *cloud_original); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud_original->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud_original) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_original);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  cloud_original->header.frame_id = "world";


  while(ros::ok()){
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    //map_pub.publish(cloud);
    map_pub.publish(cloud_original);
    ros::spinOnce();
    loop_rate.sleep();
  }




  return (0);
}
