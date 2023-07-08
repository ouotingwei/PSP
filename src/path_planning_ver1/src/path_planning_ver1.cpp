#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <open3d/Open3D.h>

void find_boundary(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    open3d::geometry::PointCloud o3d_cloud;
    for (const auto& point : cloud->points)
    {
        o3d_cloud.points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    open3d::geometry::PointCloud o3d_normals;
    for (const auto& normal : normals->points)
    {
        o3d_normals.points_.push_back(Eigen::Vector3d(normal.normal_x, normal.normal_y, normal.normal_z));
    }

    open3d::geometry::PointCloud o3d_oriented_cloud;
    o3d_oriented_cloud.points_ = o3d_cloud.points_;
    o3d_oriented_cloud.normals_ = o3d_normals.points_;
    o3d_oriented_cloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, 10000.0));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr oriented_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (const auto& point : o3d_oriented_cloud.points_)
    {
        pcl::PointXYZRGBA pcl_point;
        pcl_point.x = static_cast<float>(point.x());
        pcl_point.y = static_cast<float>(point.y());
        pcl_point.z = static_cast<float>(point.z());
        oriented_cloud->points.push_back(pcl_point);
    }

    pcl::io::savePCDFile("output_cloud.pcd", *oriented_cloud);
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/1.pcd", *cloud) == -1)
    {
        std::cout << "Failed to load point cloud." << std::endl;
        return -1;
    }

    find_boundary(cloud);

    return 0;
}
