/**
 * @file SO101_Record.cpp
 * @brief Manual movement and position logging for robot manipulator
 * 
 * @details
 * This example demonstrates how to disable torque on the robot joints, 
 * allowing manual movement, and continuously logs the current joint angles 
 * in radians. This is useful for teaching or recording waypoints.
 * 
 * Hardware Requirements:
 * - Feetech STS3215 protocol servos (IDs: 1 to N)
 * - Robot arm assembly (e.g. SO101 6-DOF)
 * - Serial connection at 1Mbps
 */

#include <iostream>
#include <array>
#include <unistd.h>
#include <iomanip>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/Robot.h>
#include <lerobot_cpp/robots/SO101.h>
#include "ExampleUtils.h"

template <size_t N>
int runRecord(Robot<N>& robot) {
    // Initialize with default IDs (1 to N)
    std::cout << "Initializing robot (" << N << " joints)..." << std::endl;
    if (!robot.init()) {
        std::cerr << "Error: Failed to initialize robot. Check servo IDs and power." << std::endl;
        return 1;
    }

    // Disable torque to allow manual movement
    std::cout << "Disabling torque on all joints. You can move the robot manually now." << std::endl;
    robot.enableTorque(false);

    std::cout << "Logging joint positions (rad). Press [ENTER] to stop." << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    for (size_t i = 0; i < N; ++i) {
        std::cout << "  J" << (i + 1) << (i == N - 1 ? "" : "\t");
    }
    std::cout << std::endl;

    while (!ExampleUtils::isEnterPressed()) {
        std::array<float, N> positions{};
        bool read_error = false;

        for (size_t i = 0; i < N; ++i) {
            float angle = robot.getJointAngle(static_cast<u8>(i));
            positions[i] = angle;
            if (std::isnan(angle)) {
                read_error = true;
            }
        }

        std::cout << "\r" << std::fixed << std::setprecision(3);
        if (!read_error) {
            for (size_t i = 0; i < N; ++i) {
                std::cout << positions[i] << (i == N - 1 ? "" : "\t");
            }
        } else {
            std::cout << "Error reading joint positions! Check connections.";
        }
        std::cout << std::flush;
    }

    std::cout << std::endl << "Recording stopped." << std::endl;
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
        std::cerr << "Error: Failed to initialize serial port " << serialPort << std::endl;
        return 1;
    }

    int result = ExampleUtils::runWithRobot(numJoints, sm_st, [](auto& robot) {
        return runRecord(robot);
    });

    sm_st.end();
    return result;
}
