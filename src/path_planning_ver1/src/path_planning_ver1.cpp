#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <open3d/Open3D.h>
#include "workingSpaceTF.cpp"
#include "json.hpp"

// // define sample size
// #define WINDOW_SIZE 0.1
// // Filter out the workspace
// #define SEARCH_RANGE_BX 0.2
// #define SEARCH_RANGE_SX -0.25
// #define SEARCH_RANGE_BY 0.5
// #define SEARCH_RANGE_SY 0.1
// #define SEARCH_HEIGHT -0.485
// #define PROXIMITY_THRESHOLD 0.01
// #define DOWN_SAMPLE_SIZE 0.005
// // path planning
// #define CLOUD_SEARCHING_RANGE 0.0022
// #define PLASMA_DIA 0.03

// define sample size
double WINDOW_SIZE = 0.1;
// Filter out the workspace
double SEARCH_RANGE_BX = 0.2;
double SEARCH_RANGE_SX = -0.25;
double SEARCH_RANGE_BY = 0.5;
double SEARCH_RANGE_SY = 0.1;
double SEARCH_HEIGHT = -0.485;
double PROXIMITY_THRESHOLD = 0.01;
double DOWN_SAMPLE_SIZE = 0.005;
// path planning
double CLOUD_SEARCHING_RANGE = 0.0022;
double PLASMA_DIA = 0.03;
double TF_Z_BIAS=0;
float removeBounceGate = 0.1;

using namespace std;
using json = nlohmann::json;
vector<vector<double>> edge_contour;

int readParameters()
{
    const char* homeDir = getenv("HOME");
    if (homeDir == nullptr) {
        std::cerr << "Failed to get the home directory." << std::endl;
        return 1;
    }

    std::string filePath = std::string(homeDir) + "/PSP/src/path_planning_ver1/src/parameters.json";
    cout<<filePath<<endl;
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        std::cerr << "Error opening parameters.json" << std::endl;
        return 0;
    }

    json parameters;
    file >> parameters;
    file.close();

    WINDOW_SIZE = parameters["WINDOW_SIZE"];
    SEARCH_RANGE_BX = parameters["SEARCH_RANGE_BX"];
    SEARCH_RANGE_SX = parameters["SEARCH_RANGE_SX"];
    SEARCH_RANGE_BY = parameters["SEARCH_RANGE_BY"];
    SEARCH_RANGE_SY = parameters["SEARCH_RANGE_SY"];
    SEARCH_HEIGHT = parameters["SEARCH_HEIGHT"];
    PROXIMITY_THRESHOLD = parameters["PROXIMITY_THRESHOLD"];
    DOWN_SAMPLE_SIZE = parameters["DOWN_SAMPLE_SIZE"];
    CLOUD_SEARCHING_RANGE = parameters["CLOUD_SEARCHING_RANGE"];
    PLASMA_DIA = parameters["PLASMA_DIA"];
    TF_Z_BIAS = parameters["TF_Z_BIAS"];
    removeBounceGate=parameters["removeBounceGate"];
    return 1;
}

bool isNearEdge(vector<double> point, double &refer_height)
{
    for (auto edge_point : edge_contour)
    {
        if (abs(edge_point[0] - point[0]) + abs(edge_point[1] - point[1]) < 0.009)
        {
            refer_height = edge_point[2];
            return true;
        }
    }
    return false;
}

vector<vector<double>> smoothEdgePointCloud(vector<vector<double>> point_cloud)
{
    vector<vector<double>> return_cloud = point_cloud;
    double refer_height=0;
    for (auto &point : return_cloud)
    {
        if (isNearEdge(point,refer_height))
        {
            point[2]=refer_height;
        }
    }
    return return_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FlipPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flipped_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (const pcl::PointXYZRGBA &point : *cloud_in)
    {
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

vector<vector<double>> estimateNormals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.1);
    ne.compute(*normals);

    // Convert to Open3D's PointCloud format
    open3d::geometry::PointCloud o3dCloud;
    for (const auto &point : cloud->points)
    {
        o3dCloud.points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // Adjusting the normal directions
    open3d::geometry::PointCloud o3dNormals;
    for (const auto &normal : normals->points)
    {
        o3dNormals.points_.push_back(Eigen::Vector3d(normal.normal_x, normal.normal_y, normal.normal_z));
    }

    o3dCloud.normals_ = o3dNormals.points_;
    o3dCloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0.0, 0.0, -10000.0));

    // Convert to (x, y, z, a, b, c) vector format
    vector<vector<double>> vectors;
    for (size_t i = 0; i < o3dCloud.points_.size(); ++i)
    {
        const auto &point = o3dCloud.points_[i];
        const auto &normal = o3dCloud.normals_[i];
        vector<double> vector{point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z()};
        vectors.push_back(vector);
    }

    return vectors;
}

void printPointCloudRange(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const pcl::PointXYZRGBA &point : cloud->points)
    {
        if (point.x < min_x)
            min_x = point.x;
        if (point.x > max_x)
            max_x = point.x;
        if (point.y < min_y)
            min_y = point.y;
        if (point.y > max_y)
            max_y = point.y;
        if (point.z < min_z)
            min_z = point.z;
        if (point.z > max_z)
            max_z = point.z;
    }

    std::cout << "Point Cloud Range:" << std::endl;
    std::cout << "X: " << min_x << " to " << max_x << std::endl;
    std::cout << "Y: " << min_y << " to " << max_y << std::endl;
    std::cout << "Z: " << min_z << " to " << max_z << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr searchAndFilterItems(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (const pcl::PointXYZRGBA &point : cloud->points)
    {
        if (point.x >= SEARCH_RANGE_SX && point.x <= SEARCH_RANGE_BX && point.y >= SEARCH_RANGE_SY && point.y <= SEARCH_RANGE_BY && point.z >= SEARCH_HEIGHT)
        {
            filteredCloud->points.push_back(point);
        }
    }

    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;

    return filteredCloud;
}

vector<vector<double>> OriginCorrectionPointCloud(vector<vector<double>> cloud)
{
    float center_x, center_y, low_z, max_x, min_x, max_y, min_y = 0;

    max_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] > max_x)
        {
            max_x = cloud[i][0];
        }
    }

    min_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] < min_x)
        {
            min_x = cloud[i][0];
        }
    }

    max_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] > max_y)
        {
            max_y = cloud[i][1];
        }
    }

    min_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] < min_y)
        {
            min_y = cloud[i][1];
        }
    }

    low_z = cloud[0][2];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][2] < low_z)
        {
            low_z = cloud[i][2];
        }
    }

    cout << max_x << " " << min_x << " " << max_y << " " << min_y << " " << low_z << endl;

    center_x = (max_x + min_x) / 2;
    center_y = (max_y + min_y) / 2;

    for (int i = 0; i < cloud.size(); i++)
    {
        cloud[i][0] = cloud[i][0] - center_x;
        cloud[i][1] = cloud[i][1] - center_y;
        cloud[i][2] = cloud[i][2] - low_z;
    }

    return cloud;
}

bool SortYaxisBigToSmall(vector<double> a, vector<double> b)
{
    return a[1] > b[1];
}

bool SortYaxisSmallToBig(vector<double> a, vector<double> b)
{
    return a[1] < b[1];
}

vector<vector<double>> removeBouncePoints(vector<vector<double>> cloud)
{

    float temp_z = cloud[0][2];
    for(int i = 1; i < cloud.size(); i++)
    {
        if(abs(cloud[i][2] - temp_z) > removeBounceGate)
        {
            if(cloud[i][2] < temp_z)
            {
                cloud[i][2] = temp_z;
            }

            temp_z = cloud[i][2];
        }
    }

    return cloud;
}

vector<vector<double>> PathCloudFilter(vector<vector<double>> cloud)
{
    int rounds = 12;
    vector<vector<double>> ok_cloud_1;
    vector<vector<double>> ok_cloud_2;
    vector<vector<double>> ok_cloud_3;

    float max_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] > max_x)
        {
            max_x = cloud[i][0];
        }
    }

    float min_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] < min_x)
        {
            min_x = cloud[i][0];
        }
    }

    float max_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] > max_y)
        {
            max_y = cloud[i][1];
        }
    }

    float min_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] < min_y)
        {
            min_y = cloud[i][1];
        }
    }

    float shift_distance = (max_x - min_x) / rounds;

    vector<double> startPoint = {0, 0, 0.1, 0, 0, 0};
    ok_cloud_1.push_back(startPoint);

    for (int i = 0; i <= rounds; i++)
    {
        float x = max_x - (shift_distance * i);
        float up_x = x + CLOUD_SEARCHING_RANGE;
        float low_x = x - CLOUD_SEARCHING_RANGE;

        vector<double> ap_max_y = {x, max_y + PLASMA_DIA, 0, 0, 0, 0};
        vector<double> ap_min_y = {x, min_y - PLASMA_DIA, 0, 0, 0, 0};

        vector<vector<double>> tmp_cloud;
        for (int j = 0; j < cloud.size(); j++)
        {
            if (cloud[j][0] > low_x && cloud[j][0] < up_x)
            {
                tmp_cloud.push_back(cloud[j]);
            }
        }
        if (i % 2 == 0)
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisBigToSmall);
            edge_contour.push_back(tmp_cloud.front());
            edge_contour.push_back(tmp_cloud.back());
            ok_cloud_1.push_back(ap_max_y);

            for (auto c : tmp_cloud)
                ok_cloud_1.push_back(c);
            ok_cloud_1.push_back(ap_min_y);
        }
        else
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisSmallToBig);
            edge_contour.push_back(tmp_cloud.front());
            edge_contour.push_back(tmp_cloud.back());
            ok_cloud_1.push_back(ap_min_y);

            for (auto c : tmp_cloud)
                ok_cloud_1.push_back(c);
            ok_cloud_1.push_back(ap_max_y);
        }
    }
    for (auto v1 : ok_cloud_1)
    {
        for (auto v2 : v1)
        {
            cout << v2 << ", ";
        }
        cout << endl;
    }

    ok_cloud_1.push_back(startPoint);
    ok_cloud_2 = removeBouncePoints(ok_cloud_1);
    ok_cloud_3 = smoothEdgePointCloud(ok_cloud_2);

    return ok_cloud_3;
}

vector<vector<double>> PathPlanning(vector<vector<double>> cloud)
{

    // cout << "[ PathPlanning ] before filtered_cloud " << cloud.size() << endl;
    vector<vector<double>> filtered_cloud = PathCloudFilter(cloud);
    // cout << "[ PathPlanning ] after filtered_cloud " << filtered_cloud.size() << endl;

    // Convert input cloud to Open3D format
    open3d::geometry::PointCloud open3d_cloud;
    for (const auto &point : filtered_cloud)
    {
        open3d_cloud.points_.push_back(Eigen::Vector3d(point[0], point[1], point[2]));
    }

    std::cout << "[ PathPlanning ] after filtered_cloud " << open3d_cloud.points_.size() << std::endl;

    // Filter the point cloud using Open3D functions
    open3d::geometry::PointCloud filtered_open3d_cloud = open3d_cloud; // Perform your filtering operation here

    // Visualize the filtered point cloud
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Point Cloud", 1920, 1080);

    std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>(filtered_open3d_cloud);
    visualizer.AddGeometry(filtered_geometry_ptr);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return filtered_cloud;
}

int main(int argc, char **argv)
{

    readParameters();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("./scan/merge/merged_cloud.pcd", *cloud) == -1)
    {
        cout << "Failed to load point cloud." << endl;
        return -1;
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE);
    voxelGrid.filter(*downsampledCloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(downsampledCloud);
    sor.setMeanK(500);
    sor.setStddevMulThresh(0.001);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smooth(new pcl::PointCloud<pcl::PointXYZRGBA>);
    sor.filter(*smooth);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flipCloud = FlipPointCloud(smooth);
    //pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/flip_cloud.pcd", *flipCloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud = searchAndFilterItems(flipCloud);
    printPointCloudRange(filteredCloud);
    //pcl::io::savePCDFile<pcl::PointXYZRGBA>("./scan/merge/filtered_cloud.pcd", *filteredCloud);
    vector<vector<double>> vectors = estimateNormals(filteredCloud);
    vector<vector<double>> ok_cloud = OriginCorrectionPointCloud(vectors);
    // vector<vector<double>> to_ls_cloud = PathPlanning(ok_cloud);

    // jylong edit
    std::vector<std::vector<double>> point_cloud = PathPlanning(ok_cloud);
    for (auto &point : point_cloud)
    {
        point[0] = point[0] * 1000;
        point[1] = point[1] * 1000;
        point[2] = point[2] * 1000;
    }

    std::vector<Waypoint> waypoints;
    double theta = 0;
    workingSpaceTF(point_cloud, waypoints, theta,TF_Z_BIAS);

    // // Print waypoints
    // for (int i = 0; i < waypoints.size(); i++)
    // {
    //     printf("Waypoint %d:\n", i);
    //     printf("x: %lf\n", waypoints[i].x);
    //     printf("y: %lf\n", waypoints[i].y);
    //     printf("z: %lf\n", waypoints[i].z);
    //     printf("W: %lf\n", waypoints[i].W);
    //     printf("P: %lf\n", waypoints[i].P);
    //     printf("R: %lf\n", waypoints[i].R);
    //     printf("V: %lf\n", waypoints[i].V);
    //     printf("C: %s\n", waypoints[i].C.c_str());
    //     printf("\n");
    // }

    const std::string file_path = "S001.LS";
    if (writeLsFile(file_path, waypoints))
        printf("Write LS error !!!\n");
    else
        printf("Sucess!!!\n");

    // for (size_t i = 0; i < vectors.size(); ++i)
    // {
    //     for (size_t j = 0; j < vectors[i].size(); ++j)
    //     {
    //         cout << vectors[i][j] << " ";
    //     }
    //     cout << endl;
    // }

    return 0;
}
