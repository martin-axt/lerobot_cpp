/**
 * @file SO101_Record.cpp
 * @brief Manual movement and position logging for SO101 6-DOF robot
 * 
 * @details
 * This example demonstrates how to disable torque on the SO101 robot joints, 
 * allowing manual movement, and continuously logs the current joint angles 
 * in radians. This is useful for teaching or recording waypoints.
 * 
 * Hardware Requirements:
 * - 6x Feetech STS3215 protocol servos (IDs: 1, 2, 3, 4, 5, 6)
 * - SO101 robot arm assembly
 * - Serial connection at 1Mbps
 */

#include <iostream>
#include <array>
#include <unistd.h>
#include <iomanip>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/SO101.h>
#include "ExampleUtils.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port> [baud_rate]" << std::endl;
        std::cout << "Default baud rate: 1000000" << std::endl;
        return 0;
    }

    const char* serialPort = argv[1];
    int baudRate = (argc > 2) ? std::stoi(argv[2]) : 1000000;

    STS3215 sm_st;
    if (!sm_st.begin(baudRate, serialPort)) {
        std::cerr << "Error: Failed to initialize serial port " << serialPort << std::endl;
        return 1;
    }

    // Create SO101 robot instance
    SO101 robot(sm_st);

    // Initialize with default IDs (1-6)
    if (!robot.init()) {
        std::cerr << "Error: Failed to initialize SO101 robot. Check servo IDs and power." << std::endl;
        return 1;
    }

    // Disable torque to allow manual movement
    std::cout << "Disabling torque on all joints. You can move the robot manually now." << std::endl;
    robot.enableTorque(false);

    std::cout << "Logging joint positions (rad). Press [ENTER] to stop." << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "  J1 \t  J2 \t  J3 \t  J4 \t  J5 \t  J6" << std::endl;

    while (!ExampleUtils::isEnterPressed()) {
        std::array<float, 6> positions{};
        bool read_error = false;

        for (int i = 0; i < 6; ++i) {
            float angle = robot.getJointAngle(i);
            positions[i] = angle;
            if (std::isnan(angle)) {
                read_error = true;
            }
        }

    	std::cout << "\r" << std::fixed << std::setprecision(3);
        if (!read_error) {
            for (size_t i = 0; i < 6; ++i) {
                std::cout << positions[i] << (i == 5 ? "" : "\t");
            }
        } else {
            std::cout << "Error reading joint positions! Check connections.";
        }
        std::cout << std::flush;
    }

    std::cout << std::endl << "Recording stopped." << std::endl;
    sm_st.end();
    return 0;
}
