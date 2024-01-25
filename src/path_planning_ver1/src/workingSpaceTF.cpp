#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <fstream>
#include <array>
#include <iostream>
#include <iomanip>

struct Waypoint {
    double x;
    double y;
    double z;
    double W;
    double P;
    double R;
    double V;
    std::string C;
};

void initializeWaypoints(Waypoint *waypoint,double V) {
    waypoint->x = 0.000;
    waypoint->y = 0.000;
    waypoint->z = 0.000;
    waypoint->W = 0.000;
    waypoint->P = 0.000;
    waypoint->R = 0.000;
    waypoint->V = V;
    waypoint->C = "CNT100";
}

std::vector<std::vector<double>> matrixMultiplication(const std::vector<std::vector<double>>& matrix1, const std::vector<std::vector<double>>& matrix2) {
    int rows1 = matrix1.size();
    int cols1 = matrix1[0].size();
    int cols2 = matrix2[0].size();

    std::vector<std::vector<double>> result(rows1, std::vector<double>(cols2, 0));

    for (int i = 0; i < rows1; ++i) {
        for (int j = 0; j < cols2; ++j) {
            for (int k = 0; k < cols1; ++k) {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }

    return result;
}

void vector2Angle(std::vector<std::vector<double>>& points){
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
        //points[i][4] = pich;
        //points[i][5] = yow;
    }
}

void workingSpaceTF(const std::vector<std::vector<double>>& points, std::vector<Waypoint>& waypoints, double theta,double TF_Z_BIAS,double vel) {
    double transition_p[3] = {420.000, 0.000, -325.827+TF_Z_BIAS};
    double transition_v[3] = {-180, 0, 0};

    theta = theta * (3.1415 / 180);
    std::vector<std::vector<double>> rotation_matrix = {
        {cos(theta), -sin(theta), 0},
        {sin(theta), cos(theta), 0},
        {0, 0, 1}
    };

    for (int i = 0; i < points.size(); i++) {
        // Transform points to workspace
        std::vector<std::vector<double>> matrix_tmp = {{points[i][0], points[i][1], points[i][2]}};
        std::vector<std::vector<double>> position_tf = matrixMultiplication(matrix_tmp, rotation_matrix);
        position_tf[0][0] += transition_p[0];
        position_tf[0][1] += transition_p[1];
        position_tf[0][2] += transition_p[2];

        // Transform vectors to workspace
        double vector_tf[3];
        for (int j = 0; j < 3; j++) {
            vector_tf[j] = points[i][j + 3] + transition_v[j];
        }

        // Output waypoints
        Waypoint newPoint;
        initializeWaypoints(&newPoint,vel);
        newPoint.x = position_tf[0][0];
        newPoint.y = position_tf[0][1];
        newPoint.z = position_tf[0][2];
        newPoint.W = vector_tf[0];
        newPoint.P = vector_tf[1];
        newPoint.R = vector_tf[2];
        waypoints.push_back(newPoint);
    }
}

int writeLsFile(const std::string& absfile, const std::string& file, const std::vector<Waypoint>& waypoints) {
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

// int main() {
//     std::vector<std::vector<double>> point_cloud = {
//         {1, 2, 3, 0, 0, 0},
//         {4, 5, 6, 0, 0, 0},
//         {7, 8, 9, 0, 0, 0}
//     }; 

//     std::vector<Waypoint> waypoints;
//     double theta = 45.0;
//     vector2Angle(point_cloud);
//     workingSpaceTF(point_cloud, waypoints, theta);

//     // Print waypoints
//     for (int i = 0; i < waypoints.size(); i++) {
//         printf("Waypoint %d:\n", i);
//         printf("x: %lf\n", waypoints[i].x);
//         printf("y: %lf\n", waypoints[i].y);
//         printf("z: %lf\n", waypoints[i].z);
//         printf("W: %lf\n", waypoints[i].W);
//         printf("P: %lf\n", waypoints[i].P);
//         printf("R: %lf\n", waypoints[i].R);
//         printf("V: %lf\n", waypoints[i].V);
//         printf("C: %s\n", waypoints[i].C.c_str());
//         printf("\n");
//     }

//     const std::string file_path = "B0001.LS";
//     if(writeLsFile(file_path, waypoints))
//         printf("Write LS error !!!\n");
//     else
//         printf("Sucess!!!\n");
    
//     return 0;
// }
