#include <iostream>
#include <fstream>
#include <math.h>
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
#include <queue>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <ros/ros.h>
#include <path_planning_ver1/path_planning_ver1.h>

#define PI 3.14159
#define REI_B 0.04
#define REI_H 0.01

using namespace std;
using json = nlohmann::json;

// Filter out the workspacef
double DOWN_SAMPLE_SIZE = 0.005;

// path planning
double CLOUD_SEARCHING_RANGE = 0.002;
double PLASMA_DIA = 0.05;
double TF_Z_BIAS = 0;
double nearby_distance = 0.022;
float removeBounceGate = 0.1;
int rounds=10;
double height=0.03;
double velocity = 300;
double camera_x = 0;
double camera_y = 0;
double camera_z = -100;
vector<vector<double>> edge_contour;

int readParameters ()
{
    const char *homeDir = getenv ( "HOME" );
    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string filePath = std::string ( homeDir ) + "/PSP/src/path_planning_ver1/src/parameters.json";
    std::ifstream file ( filePath );
    if ( !file.is_open() )
    {
        std::cerr << "Error opening parameters.json" << std::endl;
        return 0;
    }

    json parameters;
    file >> parameters;
    file.close();

    PLASMA_DIA = parameters[ "PLASMA_DIA" ];
    TF_Z_BIAS = parameters[ "TF_Z_BIAS" ];
    removeBounceGate = parameters[ "removeBounceGate" ];
    nearby_distance = parameters[ "nearby_distance" ];
    velocity = parameters[ "velocity" ];
    rounds = parameters[ "rounds" ];
    height = parameters[ "height" ];
    
    return 1;
}

double midHighestHeightOfShoe ( vector<vector<double>> point_cloud )
{
    priority_queue<double> pq;

    for ( auto point : point_cloud )
    {
        pq.push( point[ 2 ] );
    }

    for ( int i = 0; i < point_cloud.size() / 2; i++ )
    { 
         pq.pop();
    }
       
    return pq.top();
}

bool isNearEdge ( vector<double> point, double &refer_height )
{
    for ( auto edge_point : edge_contour )
    {
        if ( abs( edge_point[ 0 ] - point[ 0 ] ) + abs( edge_point[ 1 ] - point[ 1 ] ) < nearby_distance ) 
        {
            return true;
        }
    }

    return false;
}

vector<vector<double>> smoothEdgePointCloud ( vector<vector<double>> point_cloud )
{
    double refer_height = midHighestHeightOfShoe( point_cloud );
    vector<vector<double>> return_cloud = point_cloud;
    for ( auto &point : return_cloud )
    {
        if ( isNearEdge( point, refer_height ) )
        {
            point[ 2 ] = refer_height;
        }
    }

    return return_cloud;
}

vector<vector<double>> estimateNormals ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud( cloud );
    ne.setRadiusSearch( 0.1 );
    ne.compute( *normals );

    // Convert to Open3D's PointCloud format
    open3d::geometry::PointCloud o3dCloud;
    for ( const auto &point : cloud->points )
    {
        o3dCloud.points_.push_back( Eigen::Vector3d( point.x, point.y, point.z ) );
    }

    // Adjusting the normal directions
    open3d::geometry::PointCloud o3dNormals;
    for ( const auto &normal : normals->points )
    {
        o3dNormals.points_.push_back( Eigen::Vector3d( normal.normal_x, normal.normal_y, normal.normal_z ) );
    }

    o3dCloud.normals_ = o3dNormals.points_;
    // o3dCloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
    o3dCloud.OrientNormalsTowardsCameraLocation( Eigen::Vector3d( camera_x, camera_y, camera_z ) );

    // Convert to (x, y, z, a, b, c) vector format
    vector<vector<double>> vectors;
    for ( size_t i = 0; i < o3dCloud.points_.size(); ++i )
    {
        const auto &point = o3dCloud.points_[ i ];
        const auto &normal = o3dCloud.normals_[ i ];
        vector<double> vector{ point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z() };
        vectors.push_back( vector );
    }

    vector<shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back( make_shared<const open3d::geometry::PointCloud>( o3dCloud ) );

    //open3d::visualization::DrawGeometries(geometries, "result", 800, 800, 50, 50, true);

    return vectors;
}

void printPointCloudRange ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for ( const pcl::PointXYZRGBA &point : cloud->points )
    {
        if ( point.x < min_x )
        {
            min_x = point.x;
        }

        if ( point.x > max_x ) 
        {
            max_x = point.x;
        }

        if ( point.y < min_y )
        {
            min_y = point.y;
        }
            
        if ( point.y > max_y )
        {
            max_y = point.y;
        }
            
        if ( point.z < min_z )
        {
            min_z = point.z;
        }
            
        if ( point.z > max_z )
        {
            max_z = point.z;
        }  
    }
}

vector<vector<double>> OriginCorrectionPointCloud ( vector<vector<double>> cloud )
{
    float center_x, center_y, low_z, max_x, min_x, max_y, min_y = 0;
    max_x = cloud[ 0 ][ 0 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] > max_x )
        {
            max_x = cloud[ i ][ 0 ];
        }
    }

    min_x = cloud[ 0 ][ 0 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] < min_x )
        {
            min_x = cloud[ i ][ 0 ];
        }
    }

    max_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] > max_y )
        {
            max_y = cloud[ i ][ 1 ];
        }
    }

    min_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] < min_y )
        {
            min_y = cloud[ i ][ 1 ];
        }
    }

    low_z = cloud[ 0 ][ 2 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 2 ] < low_z )
        {
            low_z = cloud[ i ][ 2 ];
        }
    }

    center_x = ( max_x + min_x ) / 2;
    center_y = ( max_y + min_y ) / 2;

    for ( int i = 0; i < cloud.size(); i++ )
    {
        cloud[ i ][ 0 ] = cloud[ i ][ 0 ] - center_x;
        cloud[ i ][ 1 ] = cloud[ i ][ 1 ] - center_y;
        cloud[ i ][ 2 ] = cloud[ i ][ 2 ] - low_z;
    }

    return cloud;
}

bool SortYaxisBigToSmall ( vector<double> a, vector<double> b )
{
    return a[ 1 ] > b[ 1 ];
}

bool SortYaxisSmallToBig ( vector<double> a, vector<double> b )
{
    return a[ 1 ] < b[ 1 ];
}

double polar_angle ( vector<double> center, vector<double> p )
{
    return atan2( p[ 1 ] - center[ 1 ], p[ 0 ] - center[ 0 ] );
}

vector<vector<double>> removeBouncePoints ( vector<vector<double>> cloud )
{

    float temp_z = cloud[ 0 ][ 2 ];
    for ( int i = 1; i < cloud.size(); i++ )
    {
        if ( abs( cloud[ i ][ 2 ] - temp_z ) > removeBounceGate )
        {
            if ( cloud[ i ][ 2 ] < temp_z )
            {
                cloud[ i ][ 2 ] = temp_z;
            }

            temp_z = cloud[ i ][ 2 ];
        }
    }

    return cloud;
}

bool customCompare ( const vector<double> &a, const vector<double> &b )
{
    return a[ 7 ] > b[ 7 ];
}

vector<vector<double>> BorderReinforcement ( vector<vector<double>> cloud )
{
    open3d::geometry::PointCloud pcd;
    vector<vector<double>> square_path;
    vector<vector<double>> contour = edge_contour;

    double center_x = ( contour[ contour.size()-1 ][ 0 ] + contour[ contour.size() - 2 ][ 0 ]) / 2;
    double center_y = ( contour[ contour.size()-1] [ 1 ] + contour[ contour.size() - 2 ][ 1 ]) / 2;
    vector<double> center{ center_x, center_y };

    sort( contour.begin(), contour.end(), [&](const vector<double> &a, const vector<double> &b )
    {
        return polar_angle ( center, a ) < polar_angle( center, b ); 
    });

    for( auto& point: contour ) 
    {
        point[ 2 ]+=0.03;
        pcd.points_.push_back( { point[ 0 ], point[ 1 ], point[ 2 ] } );
    }

    open3d::visualization::DrawGeometries( { make_shared<open3d::geometry::PointCloud>( pcd ) } );

    return contour;
}

vector<vector<double>> PathCloudFilter ( vector<vector<double>> input_cloud )
{
    // int rounds = 6;
    vector<vector<double>> ok_cloud_1;
    // vector<vector<double>> ok_cloud_2;
    // vector<vector<double>> ok_cloud_3;
    vector<vector<double>> cloud=input_cloud;
    for(auto& point:cloud){
        double tmp=point[0];
        point[0]=point[1];
        point[1]=tmp;

    }
    float max_x = cloud[ 0 ][ 0 ];
    // find the pointcloud range
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] > max_x )
        {
            max_x = cloud[ i ][ 0 ];
        }
    }

    float min_x = cloud[ 0 ][ 0 ];
    for ( int i = 0; i < cloud.size(); i++ ) 
    {
        if ( cloud[ i ][ 0 ] < min_x ) 
        {
            min_x = cloud[ i ][ 0 ];
        }
    }

    float max_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] > max_y )
        {
            max_y = cloud[ i ][ 1 ];
        }
    }

    float min_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] < min_y )
        {
            min_y = cloud[ i ][ 1 ];
        }
    }

    float shift_distance = ( max_x - min_x ) / rounds;

    vector<double> startPoint = { 0, 0, -0.3, 0, 0, 0 };
    
    // ok_cloud_1.push_back( startPoint );

    for ( int i = 0; i <= rounds; i++ )
    {
        float x = max_x - ( shift_distance * i );
        float up_x = x + CLOUD_SEARCHING_RANGE;
        float low_x = x - CLOUD_SEARCHING_RANGE;

        vector<vector<double>> tmp_cloud;
        for ( int j = 0; j < cloud.size(); j++ )
        {
            if ( cloud[ j ][ 0 ] > low_x && cloud[ j ][ 0 ] < up_x )
            {
                tmp_cloud.push_back( cloud[ j ] );
            }
        }

        if ( i % 2 == 0 )
        {
            std::sort( tmp_cloud.begin(), tmp_cloud.end(), SortYaxisBigToSmall );
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.05, tmp_cloud[ 0 ][ 2 ] + 0.02, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.05, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ] + 0.02, 0, 0, 0 };
            edge_contour.push_back( tmp_cloud.front() );
            edge_contour.push_back( tmp_cloud.back() );
            ok_cloud_1.push_back( ap_max_y );

            for ( auto c : tmp_cloud )
            {
                ok_cloud_1.push_back( c );
            }
                
            ok_cloud_1.push_back( ap_min_y );

        }
        else
        {
            std::sort( tmp_cloud.begin(), tmp_cloud.end(), SortYaxisSmallToBig );
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.05, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ]+ 0.02, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.05, tmp_cloud[ 0 ][ 2 ]+ 0.02, 0, 0, 0 };
            edge_contour.push_back( tmp_cloud.back() );
            edge_contour.push_back( tmp_cloud.front() );
            ok_cloud_1.push_back( ap_min_y );

            for ( auto c : tmp_cloud )
            {
                ok_cloud_1.push_back( c );
            }
                
            ok_cloud_1.push_back( ap_max_y );
        }
    }

    // ok_cloud_1.push_back( startPoint );

    return ok_cloud_1;
}

vector<vector<double>> PathPlanning ( vector<vector<double>> cloud )
{
    vector<vector<double>> filtered_cloud = PathCloudFilter( cloud );

    // Convert input cloud to Open3D format
    open3d::geometry::PointCloud open3d_cloud;
    for ( const auto &point : filtered_cloud )
    {
        open3d_cloud.points_.push_back( Eigen::Vector3d ( point[ 0 ], point[ 1 ], point[ 2 ] ) );
    }

    // Filter the point cloud using Open3D functions
    open3d::geometry::PointCloud filtered_open3d_cloud = open3d_cloud; // Perform your filtering operation here
    
    // Visualize the filtered point cloud
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow( "Open3D Point Cloud", 800, 800 );

    std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>( filtered_open3d_cloud );
    visualizer.AddGeometry( filtered_geometry_ptr );
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return filtered_cloud;
}

void the_origin_main_function ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );

    const char* homeDir = getenv( "HOME" );
    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string pointCloudPath = std::string( homeDir ) + "/PSP/files/point_cloud.pcd";

    if ( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( pointCloudPath, *cloud ) == -1 )
    {
        //do nothing
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGrid;
    voxelGrid.setInputCloud( cloud );
    voxelGrid.setLeafSize( DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE );
    voxelGrid.filter( *downsampledCloud );

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud( downsampledCloud );
    sor.setMeanK( 500 );
    sor.setStddevMulThresh( 0.001 );

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smooth( new pcl::PointCloud<pcl::PointXYZRGBA> );
    // sor.filter( *smooth );

    vector<vector<double>> vectors = estimateNormals( downsampledCloud );
    vector<vector<double>> ok_cloud = OriginCorrectionPointCloud( vectors );

    // jylong edit
    std::vector<std::vector<double>> point_cloud = PathPlanning( ok_cloud );

    for ( auto &point : point_cloud )
    {
        point[ 0 ] = point[ 0 ] * 1000;
        point[ 1 ] = point[ 1 ] * 1000;
        point[ 2 ] = (point[ 2 ]+height )* 1000;
    }

    std::vector<Waypoint> waypoints;
    double theta = 0;
    vector2Angle( point_cloud );
    workingSpaceTF( point_cloud, waypoints, theta, TF_Z_BIAS, velocity );

    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string absfile_path = std::string( homeDir ) + "/PSP/files/H001.LS";

    const std::string file_path = "H001.LS";

    // for(auto& point:waypoints){
    //     double tmp=point.x;
    //     point.x=point.y;
    //     point.y=tmp;
    // }

    if ( writeLsFile( absfile_path,file_path ,waypoints ) )
    {
        printf( "Write LS error !!!\n" );
    }
    else
    {
        printf( "Sucess!!!\n" );
    }
}

bool server_callback ( path_planning_ver1::path_planning_ver1::Request &req, 
                       path_planning_ver1::path_planning_ver1::Response &res )
{
    if ( req.REQU_PP == true )
    {
        the_origin_main_function();
        res.RESP_PP = true;
    }
    else
    {
        res.RESP_PP = false;
    }

    return true;
}

int main ( int argc, char **argv )
{
    readParameters();
    the_origin_main_function();

    ros::init( argc, argv, "path_planning_ver1" );
    ros::NodeHandle nh;
    ros::Rate loop_rate( 30 );
    ros::ServiceServer service = nh.advertiseService( "path_planning_ver1", server_callback );

    while( ros::ok() ) 
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}