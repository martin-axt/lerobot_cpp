/**
 * @file SO101_Home.cpp
 * @brief Home position control for SO101 6-DOF robot
 * 
 * @details
 * This example demonstrates how to use the SO101 robot class to move
 * the manipulator to its home position (midpoint of all joint limits).
 * 
 * Hardware Requirements:
 * - 6x Feetech STS3215 protocol servos (IDs: 1, 2, 3, 4, 5, 6)
 * - SO101 robot arm assembly
 * - Serial connection at 1Mbps
 * 
 * Usage:
 * @code
 * ./lerobot_cpp_example_SO101_Home /dev/ttyUSB0
 * @endcode
 */

#include <iostream>
#include <array>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/SO101.h>

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
        std::cerr << "Failed to initialize serial port: " << serialPort << std::endl;
        return 1;
    }

    // Create SO101 robot instance
    SO101 robot(sm_st);

    // Initialize with default IDs (1, 2, 3, 4, 5, 6)
    std::cout << "Initializing SO101 robot..." << std::endl;
    if (!robot.init()) {
        std::cerr << "Failed to initialize SO101 robot. Check servo IDs and connection." << std::endl;
        return 1;
    }

    // Enable torque for all joints
    std::cout << "Enabling torque..." << std::endl;
    robot.enableTorque(true);

    // Home position (all joints at 0 radians - now relative to midpoint of limits)
    std::cout << "Moving to Home Position (Midpoint: 0, 0, 0, 0, 0, 0)..." << std::endl;
    std::array<float, 6> homePos;
    homePos.fill(0.0f);
    robot.setAllJointAngles(homePos);
    robot.waitMovementFinished();

    sm_st.end();
    std::cout << "Example finished." << std::endl;
    return 0;
}
