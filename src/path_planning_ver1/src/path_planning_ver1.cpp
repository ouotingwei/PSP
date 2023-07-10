#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <open3d/Open3D.h>

using namespace std;

void estimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    // down sample
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.01, 0.01, 0.01);
    voxelGrid.filter(*downsampledCloud);

    // normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(downsampledCloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // Convert to Open3D's PointCloud format
    open3d::geometry::PointCloud o3dCloud;
    for (const auto& point : downsampledCloud->points){
        o3dCloud.points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // Adjusting the normal directions
    open3d::geometry::PointCloud o3dNormals;
    for (const auto& normal : normals->points){
        o3dNormals.points_.push_back(Eigen::Vector3d(normal.normal_x, normal.normal_y, normal.normal_z));
    }

    o3dCloud.normals_ = o3dNormals.points_;
    o3dCloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, -10000.0));

    vector<shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(make_shared<const open3d::geometry::PointCloud>(o3dCloud));

    open3d::visualization::DrawGeometries(geometries, "result", 1920, 1080, 50, 50, true);
    for (const auto& normal : o3dCloud.normals_) {
        cout << "Normal: " << normal << endl;
    }

}

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/1.pcd", *cloud) == -1){
        cout << "Failed to load point cloud." << endl;
        return -1;
    }

    estimateNormals(cloud);

    return 0;
}
