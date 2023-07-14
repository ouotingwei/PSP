#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <array>
#include <iostream>

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

void initializeWaypoints(Waypoint *waypoint) {
    waypoint->x = 0.000;
    waypoint->y = 0.000;
    waypoint->z = 0.000;
    waypoint->W = 0.000;
    waypoint->P = 0.000;
    waypoint->R = 0.000;
    waypoint->V = 100;
    waypoint->C = "CNT100";
}

std::vector<std::vector<int>> matrixMultiplication(const std::vector<std::vector<int>>& matrix1, const std::vector<std::vector<int>>& matrix2) {
    int rows1 = matrix1.size();
    int cols1 = matrix1[0].size();
    int cols2 = matrix2[0].size();

    std::vector<std::vector<int>> result(rows1, std::vector<int>(cols2, 0));

    for (int i = 0; i < rows1; ++i) {
        for (int j = 0; j < cols2; ++j) {
            for (int k = 0; k < cols1; ++k) {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }

    return result;
}

void workingSpaceTF(const std::vector<std::vector<int>>& points, std::vector<Waypoint>& waypoints, double theta) {
    double transition_p[3] = {350.000, 0.000, -325.827};
    double transition_v[3] = {-180, 0, 0};

    theta = theta * (3.1415 / 180);
    std::vector<std::vector<double>> rotation_matrix = {
        {cos(theta), -sin(theta), 0},
        {sin(theta), cos(theta), 0},
        {0, 0, 1}
    };

    // Initial position
    Waypoint initialPoint;
    initializeWaypoints(&initialPoint);
    initialPoint.z = transition_p[2] + 100;
    waypoints.push_back(initialPoint);

    for (int i = 0; i < points.size() - 2; i++) {
        // Transform points to workspace
        std::vector<std::vector<int>> matrix_tmp = {{points[i][0], points[i][1], points[i][2]}};
        std::vector<std::vector<int>> position_tf = matrixMultiplication(matrix_tmp, rotation_matrix);
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
        initializeWaypoints(&newPoint);
        newPoint.x = position_tf[0][0];
        newPoint.y = position_tf[0][1];
        newPoint.z = position_tf[0][2];
        newPoint.W = vector_tf[0];
        newPoint.P = vector_tf[1];
        newPoint.R = vector_tf[2];
        waypoints.push_back(newPoint);
    }

    // End position
    Waypoint endPoint;
    initializeWaypoints(&endPoint);
    endPoint.z = transition_p[2] + 100;
    waypoints.push_back(endPoint);
}

int main() {
    std::vector<std::vector<int>> point_cloud = {
        {1, 2, 3, 1, 2, 3},
        {4, 5, 6, 4, 5, 6},
        {7, 8, 9, 7, 8, 9}
    }; 

    std::vector<Waypoint> waypoints;
    double theta = 45.0;
    workingSpaceTF(point_cloud, waypoints, theta);

    // Print waypoints
    for (int i = 0; i < waypoints.size(); i++) {
        printf("Waypoint %d:\n", i);
        printf("x: %lf\n", waypoints[i].x);
        printf("y: %lf\n", waypoints[i].y);
        printf("z: %lf\n", waypoints[i].z);
        printf("W: %lf\n", waypoints[i].W);
        printf("P: %lf\n", waypoints[i].P);
        printf("R: %lf\n", waypoints[i].R);
        printf("V: %lf\n", waypoints[i].V);
        printf("C: %s\n", waypoints[i].C.c_str());
        printf("\n");
    }
    return 0;
}
