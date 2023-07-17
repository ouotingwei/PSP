#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <open3d/Open3D.h>

// define sample size
#define WINDOW_SIZE 0.1

//Filter out the workspace
#define SEARCH_RANGE_BX 0.2
#define SEARCH_RANGE_SX -0.25
#define SEARCH_RANGE_BY 0.5
#define SEARCH_RANGE_SY 0.1
#define SEARCH_HEIGHT -0.485
#define PROXIMITY_THRESHOLD 0.01
  
using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FlipPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flipped_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    for (const pcl::PointXYZRGBA& point : *cloud_in) {
        pcl::PointXYZRGBA flipped_point;

        flipped_point.x = -point.x;
        flipped_point.y = -point.y;
        flipped_point.z = -point.z;
        flipped_point.r = point.r;
        flipped_point.g = point.g;
        flipped_point.b = point.b;
        flipped_point.a = point.a;
        
        flipped_cloud->push_back(flipped_point);
    }
    
    return flipped_cloud;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::map<std::pair<float, float>, float> max_heights;

    for (const pcl::PointXYZRGBA& point : cloud_in->points) {

        std::pair<float, float> position(point.x, point.y);
        float height = point.z;

        if (max_heights.find(position) == max_heights.end() || height > max_heights[position]) {
            max_heights[position] = height;
        }
    }


    for (const pcl::PointXYZRGBA& point : cloud_in->points) {
        std::pair<float, float> position(point.x, point.y);
        float height = point.z;

        bool is_similar_position = false;
        for (const auto& max_height : max_heights) {
            if (std::abs(max_height.first.first - position.first) <= PROXIMITY_THRESHOLD &&
                std::abs(max_height.first.second - position.second) <= PROXIMITY_THRESHOLD) {
                is_similar_position = true;
                break;
            }
        }

        if (is_similar_position && height >= max_heights[position]) {
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;

    return filteredCloud;
}

vector<vector<double>> estimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // Convert to Open3D's PointCloud format
    open3d::geometry::PointCloud o3dCloud;
    for (const auto& point : cloud->points) {
        o3dCloud.points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // Adjusting the normal directions
    open3d::geometry::PointCloud o3dNormals;
    for (const auto& normal : normals->points) {
        o3dNormals.points_.push_back(Eigen::Vector3d(normal.normal_x, normal.normal_y, normal.normal_z));
    }

    o3dCloud.normals_ = o3dNormals.points_;
    o3dCloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, -10000.0));

    // Convert to (x, y, z, a, b, c) vector format
    vector<vector<double>> vectors;
    for (size_t i = 0; i < o3dCloud.points_.size(); ++i) {
        const auto& point = o3dCloud.points_[i];
        const auto& normal = o3dCloud.normals_[i];
        vector<double> vector{point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z()};
        vectors.push_back(vector);
    }

    return vectors;
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
        if (point.x >= SEARCH_RANGE_SX && point.x <= SEARCH_RANGE_BX && point.y >= SEARCH_RANGE_SY && point.y <= SEARCH_RANGE_BY && point.z >= SEARCH_HEIGHT){
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;

    return filteredCloud;
}

vector<vector<double>> OriginCorrectionPointCloud(vector<vector<double>> cloud){
    float center_x, center_y, low_z, max_x, min_x, max_y, min_y = 0;

    max_x = cloud[0][0];  
    for (int i = 0; i < cloud.size(); i++) {
        if (cloud[i][0] > max_x) {
            max_x = cloud[i][0];  
        }
    }

    min_x = cloud[0][0];  
    for (int i = 0; i < cloud.size(); i++) {
        if (cloud[i][0] < min_x) {
            min_x = cloud[i][0];  
        }
    }

    max_y = cloud[0][1];  
    for (int i = 0; i < cloud.size(); i++) {
        if (cloud[i][1] > max_y) {
            max_y = cloud[i][1];  
        }
    }

    min_y = cloud[0][1];  
    for (int i = 0; i < cloud.size(); i++) {
        if (cloud[i][1] < min_y) {
            min_y = cloud[i][1];  
        }
    }

    low_z = cloud[0][2];  
    for (int i = 0; i < cloud.size(); i++) {
        if (cloud[i][2] < low_z) {
            low_z = cloud[i][2];  
        }
    }

    //cout << max_x << " " << min_x << " " << max_y << " " << min_y << " " << low_z << endl;
    center_x = (max_x + min_x) / 2;
    center_y = (max_y + min_y) / 2;

    for(int i = 0; i < cloud.size(); i++){
        cloud[i][0] = cloud[i][0] - center_x;
        cloud[i][1] = cloud[i][1] - center_y;
        cloud[i][2] = cloud[i][2] - low_z;
    }

    return cloud;

}

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/merged_cloud.pcd", *cloud) == -1){
        cout << "Failed to load point cloud." << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flipCloud = FlipPointCloud(cloud);
    //pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/flip_cloud.pcd", *flipCloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smoothedCloud = smoothPointCloud(flipCloud);
    //pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/smooth_cloud.pcd", *smoothedCloud);
    //printPointCloudRange(smoothedCloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud = searchAndFilterItems(smoothedCloud);
    printPointCloudRange(filteredCloud);
    pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/filted_cloud.pcd", *filteredCloud);
    vector<vector<double>> vectors = estimateNormals(filteredCloud);
    vector<vector<double>> ok_cloud = OriginCorrectionPointCloud(vectors);

    
    for (size_t i = 0; i < ok_cloud.size(); ++i) {
        for (size_t j = 0; j < ok_cloud[i].size(); ++j) {
            cout << ok_cloud[i][j] << " ";
        }
        cout << endl;
    }
    

    return 0;
}
