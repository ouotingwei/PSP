#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <errno.h>
#include <unistd.h>
#include "ros/ros.h"
#include "communication/ModbusRobot.h" 

/* FUNUC ROBOT */
#define SERVER_ADDRESS "192.168.255.200" 
#define SERVER_PORT 502 
#define SLAVE_ID 1       
#define QUANTITY 4
#define START_ADDRESS 0

modbus_t* ctx; 

void Set_val(modbus_t* ctx, uint16_t address, uint16_t val) {
    uint16_t output_data[QUANTITY] = { 0 };
    output_data[0] = val;
    int rc = modbus_write_registers(ctx, address, QUANTITY, output_data);
    if (rc == -1) {
        fprintf(stderr, "modbus_write_registers error: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        exit(1);
    }
}

int Read_val(modbus_t* ctx, uint16_t address) {
    uint16_t read_data[QUANTITY] = { 0 };
    int rc = modbus_read_input_registers(ctx, address, QUANTITY, read_data);
    if (rc == -1) {
        fprintf(stderr, "modbus_read_registers error: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        exit(1);
    }

    printf("Register Address : %d, Value: %d\n", address, read_data[0]);
    return read_data[0];
}

bool modbusControlCallback(communication::ModbusRobot::Request &req,
                           communication::ModbusRobot::Response &res) {
    if (req.start) {
        /* execute new tp program */
        while(Read_val(ctx, START_ADDRESS) == 0){
            printf("set DI on\n");
            Set_val(ctx, START_ADDRESS, 1);
            usleep(500);
        }

        /* wait manipulate set DO[1] to ON */
        usleep(3000);
        Set_val(ctx, START_ADDRESS, 0);

        /* wait manipulater finish tp program and set DI[1] to off */
        while(1){
            // read registers
            if (Read_val(ctx, START_ADDRESS) == 0) {
                // Set_val(ctx, START_ADDRESS, 0);
                break;
            }

            //scan frequency
            usleep(500); 
        }

        res.success = true;
    }
    else {
        res.success = false;
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "modbus_robot_control_node");
    ros::NodeHandle nh;

    /* set modbus tcp */
    ctx = modbus_new_tcp(SERVER_ADDRESS, SERVER_PORT);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp error\n");
        exit(1);
    }

    modbus_set_slave(ctx, SLAVE_ID);

    // Connect to manipulator
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "modbus_connect error\n");
        modbus_free(ctx);
        exit(1);
    }

    // Create a ROS service for modbus control
    ros::ServiceServer service = nh.advertiseService("modbus_robot_control", modbusControlCallback);
    ROS_INFO("Ready to control manipulator via Modbus.");

    // Spin ROS node
    ros::spin();

    // Close modbus connection and free context
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}


