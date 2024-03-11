#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <fstream>
#include <array>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

struct Waypoint
{
    double x;
    double y;
    double z;
    double W;
    double P;
    double R;
    double V;
    std::string C;
};

void initializeWaypoints(Waypoint *waypoint,double V)
{
    waypoint->x = 0.000;
    waypoint->y = 0.000;
    waypoint->z = 0.000;
    waypoint->W = 0.000;
    waypoint->P = 0.000;
    waypoint->R = 0.000;
    waypoint->V = V;
    waypoint->C = "CNT100";
}

void vector2Angle(std::vector<std::vector<double>>& points)
{
    for (int i = 0; i < points.size(); i++) {
        //double row, pich, yow;
        double row;
        if(!points[i][4] && !points[i][5])
            row = 0;
        else
            row = std::atan2(-points[i][4], -points[i][5]);
        //pich = std::atan2(points[i][3], points[i][5]);
        //yow = std::atan2(points[i][3], points[i][4]);
        row = row*180.0/M_PI;
        //pich = pich*180.0/M_PI;
        //yow = yow*180.0/M_PI;
        points[i][3] = row;
        points[i][4] = 0.0;
        points[i][5] = 0.0;
    }
}

void workingSpaceTF(const std::vector<std::vector<double>>& points, std::vector<Waypoint>& waypoints, double theta,double TF_Z_BIAS,double vel) 
{
    // Transform robot base to camera
    double transition_rtc[3] = {500.000, 0.000, 400.000};
    Eigen::MatrixXd tf_robot_to_camera(4, 4);
    tf_robot_to_camera << 0.0, 1.0, 0.0, transition_rtc[1],
                          1.0, 0.0, 0.0, -transition_rtc[0],
                          0.0, 0.0, -1.0, transition_rtc[2],
                          0.0, 0.0, 0.0, 1.0;
    std::cout << "tf_robot_to_camera:" << std::endl << tf_robot_to_camera << std::endl;
    
    // Transform camera to workpiece
    std::string file_path = "/home/honglang/PSP/files/TF.txt"; 
    std::ifstream input_file(file_path);
    if (!input_file.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return;  
    }

    std::vector<double> camera_to_piece;
    std::string line;
    while (std::getline(input_file, line)) {
        camera_to_piece.push_back(std::stod(line));
    }
    input_file.close();

    Eigen::MatrixXd tf_camera_to_workpiece(4, 4);
    tf_camera_to_workpiece << camera_to_piece[0], camera_to_piece[1], camera_to_piece[2], -camera_to_piece[9]*1000,
                          camera_to_piece[3], camera_to_piece[4], camera_to_piece[5], -camera_to_piece[10]*1000,
                          camera_to_piece[6], camera_to_piece[7], camera_to_piece[8], camera_to_piece[11]*1000,
                          0.0, 0.0, 0.0, 1.0;
    std::cout << "tf_camera_to_workpiece:" << std::endl << tf_camera_to_workpiece << std::endl;

    // Transform robot base to workspace
    // ! need add camera to plasma bias 
    // double transition_p[3] = {420.000, 0.000, -325.827+TF_Z_BIAS};
    double transition_p[3] = {0.000, 0.000, -325.827+TF_Z_BIAS};
    double transition_v[3] = {-180, 0, 0};

    theta = theta * (3.1415 / 180);
    Eigen::MatrixXd tf_robot_workspace(4, 4);
    tf_robot_workspace << cos(theta), -sin(theta), 0, transition_p[0],
                          sin(theta), cos(theta), 0, transition_p[1],
                          0.0, 0.0, 1.0, transition_p[2],
                          0.0, 0.0, 0.0, 1.0;
    std::cout << "tf_robot_workspace:" << std::endl << tf_robot_workspace << std::endl;

    std::cout << "tf_robot_to_workpiece:" << std::endl << tf_robot_to_camera*tf_camera_to_workpiece << std::endl;
    // std::cout << "tf_robot_to_workpiece inverse:" << std::endl << tf_robot_to_workpiece.inverse() << std::endl;
    // std::cout << "all :" << std::endl << tf_robot_workspace*tf_robot_to_workpiece.inverse() << std::endl;

    for (int i = 0; i < points.size(); i++) {
        
        Eigen::Vector4d point_matrix(points[i][0], points[i][1], points[i][2], 1.0);
        Eigen::MatrixXd tf_robot_to_workpiece = tf_robot_to_camera*tf_camera_to_workpiece;
        
        // Eigen::MatrixXd position_tf = tf_robot_to_camera.inverse()*point_matrix;     // V2
        // Eigen::MatrixXd position_tf = tf_robot_to_workpiece.inverse()*point_matrix;     // V3
        // Eigen::MatrixXd position_tf = tf_robot_workspace*tf_robot_to_workpiece.inverse()*point_matrix;  // V4
        Eigen::MatrixXd position_tf = tf_robot_workspace*tf_robot_to_workpiece.inverse()*point_matrix;
        // Eigen::MatrixXd position_tf = tf_robot_workspace*point_matrix;  // V1
        // std::cout << "inverse matrix:" << std::endl << tf_robot_workspace*tf_camera_to_workpiece.inverse() << std::endl;
        // std::cout << "position_tf matrix:" << std::endl << position_tf << std::endl;

        // Transform vectors to workspace
        double vector_tf[3];
        for (int j = 0; j < 3; j++) {
            vector_tf[j] = points[i][j + 3] + transition_v[j];
        }

        // Output waypoints
        Waypoint newPoint;
        initializeWaypoints(&newPoint,vel);
        newPoint.x = position_tf(0, 0);
        newPoint.y = position_tf(1, 0);
        newPoint.z = position_tf(2, 0);
        newPoint.W = vector_tf[0];
        newPoint.P = vector_tf[1];
        newPoint.R = vector_tf[2];
        waypoints.push_back(newPoint);
    }
}

int writeLsFile(const std::string& absfile, const std::string& file, const std::vector<Waypoint>& waypoints) 
{
    std::ofstream f(absfile);
    if (!f.is_open()) {
        std::cout << "Error opening file: " << file << std::endl;
        return -1;
    }

    f << "/PROG  " << file << "\n";
    f << "/ATTR\n";
    f << "OWNER       = MNEDITOR;\n";
    f << "COMMENT     = \"\";\n";
    f << "PROG_SIZE   = 636;\n";
    f << "CREATE      = DATE 23-01-07  TIME 11:59:14;\n";
    f << "MODIFIED    = DATE 23-01-07  TIME 12:02:18;\n";
    f << "FILE_NAME   = ;\n";
    f << "VERSION     = 0;\n";
    f << "LINE_COUNT  = 4;\n";
    f << "MEMORY_SIZE = 992;\n";
    f << "PROTECT     = READ_WRITE;\n";
    f << "TCD:  STACK_SIZE    = 0,\n";
    f << "      TASK_PRIORITY = 50,\n";
    f << "      TIME_SLICE    = 0,\n";
    f << "      BUSY_LAMP_OFF = 0,\n";
    f << "      ABORT_REQUEST = 0,\n";
    f << "      PAUSE_REQUEST = 0;\n";
    f << "DEFAULT_GROUP    = 1,*,*,*,*;\n";
    f << "CONTROL_CODE     = 00000000 00000000;\n";

    f << "/MN\n";
    f << "   1:J P[1] 100"<<"%"<<" FINE    ;\n";
    for (size_t i = 2; i <= waypoints.size(); ++i) {
        f << "   " << i << ":L P[" << i << "] " << waypoints[i - 1].V << "mm/sec " << waypoints[i - 1].C << "    ;\n";
    }

    f << "/POS\n";
    for (size_t i = 1; i <= waypoints.size(); ++i) {
        f << "P[" << i << "]{\n";
        f << "   GP1:\n";
        f << "    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',\n";
        f << "    X =  " << std::fixed << std::setprecision(3) << waypoints[i - 1].x << "  mm,    Y =   " << waypoints[i - 1].y << "  mm,    Z =   " << waypoints[i - 1].z << "  mm,\n";
        f << "    W =  " << waypoints[i - 1].W << " deg,    P =   " << waypoints[i - 1].P << " deg,    R =   " << waypoints[i - 1].R << " deg\n";
        f << "};\n";
    }

    f << "/END\n";
    f.close();

    return 0;
}