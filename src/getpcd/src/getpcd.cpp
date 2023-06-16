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

#define SCANTIMES 3
#define FILTER 0.01f

using namespace std;

int cnt = 1;

void Callback_cam1(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    string str_cnt = to_string(cnt);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    string scanfile_dir = "./scan/camera1/";
    string filename = ".pcd";
    pcl::io::savePCDFileASCII(scanfile_dir + str_cnt + filename, *cloud);
    ROS_INFO_STREAM("Saved " << cloud->points.size() << " data points to " << cnt << filename);
    cnt++;
}

int main(int argc, char **argv)
{
    auto total_start = std::chrono::high_resolution_clock::now();

    ros::init(argc, argv, "getpcd");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    ros::Subscriber sub_cam1 = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, Callback_cam1);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merge(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr D_sample(new pcl::PointCloud<pcl::PointXYZRGBA>);

    while (ros::ok() && cnt <= SCANTIMES)
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    for(int i = 1; i <= SCANTIMES; i++){

        if(i == 1){
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/camera1/1.pcd", *merge) != 0){
                cout << "[error!]" <<" .pcd load error" << endl;
                return -1;
            }

        }else{
            string str_cnt = to_string(i);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/camera1/" + str_cnt + ".pcd", *scan) != 0){
                cout << "[error!]" <<" .pcd load error" << endl;
                return -1;
            }
            *merge = (*merge) + (*scan);
        } 
    }

    pcl::VoxelGrid<pcl::PointXYZRGBA> downSizeFilter;
    downSizeFilter.setInputCloud(merge);
    downSizeFilter.setLeafSize (FILTER, FILTER, FILTER);
    downSizeFilter.filter(*D_sample);

    pcl::io::savePCDFileASCII("./scan/merge/camera1.pcd", *D_sample);

    auto merge_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> merge_diff = merge_end - total_start;
    std::cout << "Time to run the code: " << merge_diff.count() << " s\n";

    return 0;
}

