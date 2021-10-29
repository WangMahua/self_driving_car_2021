#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <mutex>
#include <queue>
#include "tf/transform_listener.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>


typedef pcl::PointXYZ PointType;
typedef std::pair<sensor_msgs::PointCloud2ConstPtr ,geometry_msgs::PoseStampedConstPtr> CombinedData;

ros::Publisher pub_icp,pub_localization;
ros::Publisher pub_pointBefore, pub_pointAfter;
sensor_msgs::PointCloud2 pointBefore,pointAfter,temp_cloud ;
geometry_msgs::PointStamped now_gps;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_before(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_after(new pcl::PointCloud<pcl::PointXYZ>());

std::mutex m_buf, com_buf, process_buf;
bool initial = false;
bool first_icp = false ; 
int frame_count = 0;

void laser_callback(const sensor_msgs::PointCloud2ConstPtr &laserCloud){
    m_buf.lock();
    tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ>::Ptr n_pcl_before(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr n_pcl_after(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_ros::transformPointCloud("base_link",*laserCloud,temp_cloud,tf_listener);
    if(initial == false){ 
        pointAfter = temp_cloud;
        pointBefore = pointAfter;
        initial = true;
    }else{
        pointBefore = pointAfter; 
        pointAfter = temp_cloud;
    }
    // 將ros message格式轉成pcl
    pcl::fromROSMsg(pointBefore, *n_pcl_before);
    pcl::fromROSMsg(pointAfter, *n_pcl_after);

    // downsample
    pcl::VoxelGrid<pcl::PointXYZ> sor1,sor2;
    sor1.setInputCloud (n_pcl_before);
    sor1.setLeafSize (0.01f, 0.01f, 0.01f);
    sor1.filter (*pcl_before);
    sor1.setInputCloud (n_pcl_before);
    sor1.setLeafSize (0.01f, 0.01f, 0.01f);
    sor1.filter (*pcl_before);

    m_buf.unlock();
}

void gps_callback(const geometry_msgs::PointStampedConstPtr &gps){
    m_buf.lock();
    now_gps = *gps;
    m_buf.unlock();
}

void icpmatch(pcl::PointCloud<PointType>::Ptr &i_frame, pcl::PointCloud<PointType>::Ptr &j_frame){

    std::vector<int> indices1,indices2;
    pcl::removeNaNFromPointCloud(*i_frame, *i_frame, indices1);
    pcl::removeNaNFromPointCloud(*j_frame, *j_frame, indices2);
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    pcl::PointCloud<PointType>::Ptr Final(new pcl::PointCloud<PointType>());

    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);
    icp.setRANSACIterations(0);

    icp.setInputSource(j_frame);
    icp.setInputTarget(i_frame);

    if(first_icp == false){
        icp.align(*Final);
    }else{ //first time 
        Eigen::Quaterniond temp_Q;
        Eigen::Vector3d t;

        Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(-2.2370340344819, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d rotationMatrix = q.matrix();

        t(0) = now_gps.point.x;
        t(1) = now_gps.point.y;
        t(2) = now_gps.point.z;

        Eigen::Matrix4f Init = Eigen::Matrix4f::Identity();
        Init.block<3,3>(0,0) = rotationMatrix.transpose().cast<float>();
        //Init.block<3,3>(0,0) = xxx.transpose().cast<float>();
        Init.block<3,1>(0,3) = t.cast<float>();
        icp.align(*Final,Init);
    }
    
    if (icp.hasConverged()){
        ROS_WARN_STREAM("finish");
    }

}

void process(){

    icpmatch(pcl_before, pcl_after);
}


int main (int argc, char** argv){
    ros::init (argc, argv, "icp_location");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 100, laser_callback);
    ros::Subscriber opti_sub = nh.subscribe<geometry_msgs::PointStamped>("/gps", 100, gps_callback);
    
    pub_localization = nh.advertise<nav_msgs::Odometry>("/my_localization",100);
    pub_pointBefore = nh.advertise<sensor_msgs::PointCloud2>("/PointBefore",100);
    pub_pointAfter = nh.advertise<sensor_msgs::PointCloud2>("/PointTarget",100);

    while(ros::ok){
        if(initial==true){
            process_buf.lock();
            process();
            process_buf.unlock();
        }
    }
 return (0);
}