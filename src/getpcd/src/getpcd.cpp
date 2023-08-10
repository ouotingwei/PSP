#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread.hpp>
#include <Eigen/Geometry> 
#include<iostream>
#include <chrono>

using namespace std;

void Callback_cam1(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    string scanfile_dir = "./scan/camera1/";
    string filename = "cam1_3.pcd";
    pcl::io::savePCDFileASCII(scanfile_dir + filename, *cloud);
    ROS_INFO_STREAM("Saved " << cloud->points.size() << " data points to " << filename);
}

void Callback_cam2(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    string scanfile_dir = "./scan/camera2/";
    string filename = "cam2_3.pcd";
    pcl::io::savePCDFileASCII(scanfile_dir + filename, *cloud);
    ROS_INFO_STREAM("Saved " << cloud->points.size() << " data points to " << filename);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getpcd");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    ros::Subscriber sub_cam1 = nh.subscribe<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", 1, Callback_cam1);
    ros::Subscriber sub_cam2 = nh.subscribe<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", 1, Callback_cam2);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}