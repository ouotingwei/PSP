#include <ros/ros.h>
#include <iostream>
#include <time.h>

#include <getpcd/getpcd.h>
#include <down_sample_and_merge/downSampleAndMerge.h>
#include <path_planning_ver1/path_planning_ver1.h>

using namespace std;

bool get_pcd()
{
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

bool down_sample_and_merge()
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<down_sample_and_merge::downSampleAndMerge>("downSampleAndMerge");

    down_sample_and_merge::downSampleAndMerge srv;

    srv.REQU_DSAM = true;

    if(client.call(srv)){
        if(srv.response.RESP_DSAM){
            ROS_INFO("Received RESP_DSAM: true");
        }else{
            ROS_INFO("Received RESP_DSAM: false");
            return false;
        }
    }else{
        ROS_ERROR("Failed to call 'downSampleAndMerge' service");
        return false;
    }
    
    return true;
}

bool path_planning_ver1()
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<path_planning_ver1::path_planning_ver1>("path_planning_ver1");

    path_planning_ver1::path_planning_ver1 srv;

    sev.REQU_PP = true;

    if(client.call(srv)){
        if(srv.response.RESP_PP){
            ROS_INFO("Received RESP_PP: true");
        }else{
            ROS_INFO("Received RESP_PP: false");
            return false;
        }
    }else{
        ROS_ERROR("Failed to call 'path_planning_ver1' service");
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_3d");
    
    bool system_flag = true;

    while(ros::ok() && system_flag == true){
        // [][][][]
        // >>>>>>>
        // the logic seem a little bit strange ?!?
        // >>>>>>>
        // [][][][]

        // detection

        // get pcd
        system_flag = get_pcd();

        // down_sample_and_merge
        system_flag = down_sample_and_merge();

        // path_plnning
        system_flag = path_planning_ver1();

        // fanuc communication

        ros::spinOnce();
    }

    return 0;
}