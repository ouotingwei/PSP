#include <ros/ros.h>
#include <iostream>
#include <time.h>

#include <getpcd/getpcd.h>

using namespace std;

bool get_pcd(){
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<getpcd::getpcd>("get_pcd");

    getpcd::getpcd srv;
    srv.REQU_PCD = true;

    if (client.call(srv)) {
        if (srv.response.RESP_PCD) {
            ROS_INFO("Received RESP_PCD: true");
        }else {
            ROS_INFO("Received RESP_PCD: false");
            return false;
        }
    }else {
        ROS_ERROR("Failed to call 'getpcd' service");
        return false;
    }

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "main_3d");
    
    bool system_flag = true;

    while(ros::ok() && system_flag == true){
        // detection

        // get pcd
        system_flag = get_pcd();

        // down_sample_and_merge

        // path_plnning

        // fanuc communication

        ros::spinOnce();
    }

    return 0;
}