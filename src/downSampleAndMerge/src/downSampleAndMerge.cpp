#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h> 
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include<ctime>

#define PI 3.141592653589793
#define ANGLE 180
#define SAM_ANG 39
#define SHIFT_X 0.05
#define SHIFT_Y -0.71
#define SHIFT_Z 0.0
#define HIGH_THRESHOLD 0.497

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downSampleAndMerge");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    auto total_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merge(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/1.pcd", *cloud1) == -1)
    {
        cout << "Failed to load cloud1" << endl;
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/2.pcd", *cloud2) == -1)
    {
        cout << "Failed to load cloud2" << endl;
        return -1;
    }

    Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
    rotation.rotate(Eigen::AngleAxisf(SAM_ANG * PI / 180, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1_spl(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2_spl(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud1, *cloud1_spl, rotation);
    pcl::transformPointCloud(*cloud2, *cloud2_spl, rotation);

    //rotate cloud1_spl
    Eigen::Affine3f rotation_matrix = Eigen::Affine3f::Identity();
    rotation_matrix.rotate(Eigen::AngleAxisf(ANGLE * PI / 180, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud1_spl, *transformed_cloud, rotation_matrix);

    //shift cloud1_spl
    Eigen::Affine3f translation_cloud1 = Eigen::Affine3f::Identity();
    translation_cloud1.translation() << SHIFT_X, SHIFT_Y, SHIFT_Z;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shifted_cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*transformed_cloud, *shifted_cloud1, translation_cloud1);

    //turn cloud2 into red 
    for (auto& point : shifted_cloud1->points) {
        point.r = 255;   
        point.g = 0;     
        point.b = 0;     
    }

    //merge two clouds
    *merge = *shifted_cloud1 + *cloud2_spl;

    //Remove ground
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    for (const auto& point : merge->points) {
        if (point.z <= HIGH_THRESHOLD) {
            filtered_cloud->points.push_back(point);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    //save files
    pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/merged_cloud.pcd", *filtered_cloud);
    
    auto merge_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> merge_diff = merge_end - total_start;
    std::cout << "Time to run the code: " << merge_diff.count() << " s\n";

    ros::spinOnce();

    return 0;
}
