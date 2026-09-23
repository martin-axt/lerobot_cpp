/**
 * @file SO101_Home.cpp
 * @brief Home position control for robot manipulator
 * 
 * @details
 * This example demonstrates how to use the Robot class to move
 * the manipulator to its home position (midpoint of all joint limits).
 * 
 * Hardware Requirements:
 * - Feetech STS3215 protocol servos (IDs: 1 to N)
 * - Robot arm assembly (e.g. SO101 6-DOF)
 * - Serial connection at 1Mbps
 * 
 * Usage:
 * @code
 * ./lerobot_cpp_example_SO101_Home /dev/ttyUSB0 [num_joints] [baud_rate]
 * @endcode
 */

#include <iostream>
#include <array>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/Robot.h>
#include <lerobot_cpp/robots/SO101.h>
#include "ExampleUtils.h"

template <size_t N>
int runHome(Robot<N>& robot) {
    // Initialize with default IDs (1 to N)
    std::cout << "Initializing robot (" << N << " joints)..." << std::endl;
    if (!robot.init()) {
        std::cerr << "Failed to initialize robot. Check servo IDs and connection." << std::endl;
        return 1;
    }

    // Enable torque for all joints
    std::cout << "Enabling torque..." << std::endl;
    robot.enableTorque(true);

    // Home position (all joints at 0 radians - midpoint of limits)
    std::cout << "Moving to Home Position (0 radians for all joints)..." << std::endl;
    std::array<float, N> homePos{};
    homePos.fill(0.0f);
    robot.setAllJointAngles(homePos);
    robot.waitMovementFinished();

    std::cout << "Example finished." << std::endl;
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port> [num_joints] [baud_rate]" << std::endl;
        std::cout << "Default number of joints: 6 (SO101 configuration)" << std::endl;
        std::cout << "Default baud rate: 1000000" << std::endl;
        return 0;
    }

    const char* serialPort = argv[1];
    size_t numJoints = 6;
    int baudRate = 1000000;

    if (argc > 2) {
        numJoints = static_cast<size_t>(std::stoi(argv[2]));
    }
    if (argc > 3) {
        baudRate = std::stoi(argv[3]);
    }

    STS3215 sm_st;
    if (!sm_st.begin(baudRate, serialPort)) {
        std::cerr << "Failed to initialize serial port: " << serialPort << std::endl;
        return 1;
    }

    int result = ExampleUtils::runWithRobot(numJoints, sm_st, [](auto& robot) {
        return runHome(robot);
    });

    sm_st.end();
    return result;
}
