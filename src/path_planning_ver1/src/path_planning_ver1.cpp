#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <open3d/Open3D.h>

// define sample size
#define WINDOW_SIZE 0.1
#define DOWN_SAMPLE_SIZE 0.01

//Filter out the workspace
#define SEARCH_RANGE_BX 0.2
#define SEARCH_RANGE_SX -0.2
#define SEARCH_RANGE_BY -0.2
#define SEARCH_RANGE_SY -0.45
#define SEARCH_HEIGHT 4.5

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZRGBA>);
    *cloud_smoothed = *cloud_in; 

    int half_window = WINDOW_SIZE / 2;
    for(size_t i = half_window; i < cloud_in->size() - half_window; ++i){
        pcl::PointXYZRGBA avg_point;
        avg_point.x = 0;
        avg_point.y = 0;
        avg_point.z = 0;
        avg_point.r = 0;
        avg_point.g = 0;
        avg_point.b = 0;
        avg_point.a = 0;

        for(int j = -half_window; j <= half_window; ++j){
            avg_point.x += (*cloud_in)[i + j].x;
            avg_point.y += (*cloud_in)[i + j].y;
            avg_point.z += (*cloud_in)[i + j].z;
            avg_point.r += (*cloud_in)[i + j].r;
            avg_point.g += (*cloud_in)[i + j].g;
            avg_point.b += (*cloud_in)[i + j].b;
            avg_point.a += (*cloud_in)[i + j].a;
        }

        avg_point.x /= WINDOW_SIZE;
        avg_point.y /= WINDOW_SIZE;
        avg_point.z /= WINDOW_SIZE;
        avg_point.r /= WINDOW_SIZE;
        avg_point.g /= WINDOW_SIZE;
        avg_point.b /= WINDOW_SIZE;
        avg_point.a /= WINDOW_SIZE;

        (*cloud_smoothed)[i] = avg_point;
    }

    return cloud_smoothed;
}

void estimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    // down sample
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE);
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

    /*
    for (const auto& normal : o3dCloud.normals_) {
        cout << "Normal: " << normal << endl;
    }
    */
}

void printPointCloudRange(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const pcl::PointXYZRGBA& point : cloud->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
    }

    std::cout << "Point Cloud Range:" << std::endl;
    std::cout << "X: " << min_x << " to " << max_x << std::endl;
    std::cout << "Y: " << min_y << " to " << max_y << std::endl;
    std::cout << "Z: " << min_z << " to " << max_z << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchAndFilterItems(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (const pcl::PointXYZRGBA& point : cloud->points) {
        if (point.x >= SEARCH_RANGE_SX && point.x <= SEARCH_RANGE_BX && point.y >= SEARCH_RANGE_SY && point.y <= SEARCH_RANGE_BY && point.z < SEARCH_HEIGHT){
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;

    return filteredCloud;
}

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/merged_cloud.pcd", *cloud) == -1){
        cout << "Failed to load point cloud." << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothedCloud = smoothPointCloud(cloud);
    printPointCloudRange(smoothedCloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud = searchAndFilterItems(cloud);
    estimateNormals(filteredCloud);

    if (pcl::io::savePCDFileBinary("./scan/merge/filtered_cloud.pcd", *filteredCloud) == -1) {
        cout << "Failed to save filtered point cloud." << endl;
        return -1;
    }

    return 0;
}
