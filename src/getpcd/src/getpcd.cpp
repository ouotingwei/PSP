#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <getpcd/getpcd.h>

using namespace std;

ros::Subscriber sub_cam1;
ros::Subscriber sub_cam2;

void Callback_cam1(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    string scanfile_dir = "/home/honglang/PSP/scan/camera1/";
    string filename = "cam1.pcd";
    pcl::io::savePCDFileASCII(scanfile_dir + filename, *cloud);
    ROS_INFO_STREAM("Saved " << cloud->points.size() << " data points to " << filename);
}

void Callback_cam2(const sensor_msgs::PointCloud2ConstPtr& msg)
{   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);
    string scanfile_dir = "/home/honglang/PSP/scan/camera2/";
    string filename = "cam2.pcd";
    pcl::io::savePCDFileASCII(scanfile_dir + filename, *cloud);
    ROS_INFO_STREAM("Saved " << cloud->points.size() << " data points to " << filename);
}

bool server_callback(getpcd::getpcd::Request &req, getpcd::getpcd::Response &res){
    ros::NodeHandle nh;
    
    if(req.REQU_PCD == true){
        sub_cam1 = nh.subscribe<sensor_msgs::PointCloud2>("/cam_1/depth/color/points", 1, Callback_cam1);
        sub_cam2 = nh.subscribe<sensor_msgs::PointCloud2>("/cam_2/depth/color/points", 1, Callback_cam2);
        res.RESP_PCD = true;
    }else{
        res.RESP_PCD = false;
    }
    //sleep(1); // 1s
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getpcd");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    ros::ServiceServer service = nh.advertiseService("getpcd", server_callback);

    while(ros::ok()){
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
