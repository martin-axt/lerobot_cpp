/**
 * @file SO101_Wave.cpp
 * @brief Wave movement for SO101 6-DOF robot
 * 
 * @details
 * This example demonstrates a soft waving movement with the SO101 robot.
 * 
 * Hardware Requirements:
 * - 6x Feetech STS3215 protocol servos (IDs: 1, 2, 3, 4, 5, 6)
 * - SO101 robot arm assembly
 * - Serial connection at 1Mbps
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
    if (!robot.init()) {
        std::cerr << "Failed to initialize SO101 robot." << std::endl;
        return 1;
    }

    // Enable torque
    robot.enableTorque(true);

    // Home position
    std::array<float, 6> homePos;
    homePos.fill(0.0f);
    
    // Wave parameters
    std::array<float, 6> readyPos = {-1.517f, -1.657f, 0.193f, -0.018f, -0.247f, -1.135f};
    std::array<float, 6> softSpeeds;
    softSpeeds.fill(0.5f); // 0.5 rad/s
    std::array<float, 6> waveSpeeds;
    waveSpeeds.fill(1.0f); // 1 rad/s
    std::array<float, 6> softAccs;
    softAccs.fill(2.0f);   // 2.0 rad/s^2

    // Hello position (Raising arm)
    std::cout << "Raising arm to 'ready' position..." << std::endl;
    robot.setAllJointAngles(readyPos, softSpeeds, softAccs);
    robot.waitMovementFinished();

    // Wave movement (J5 wrist side-to-side)
    std::cout << "Performing soft 'hello' wave..." << std::endl;
    std::array<float, 6> wavePos1 = {-1.519f, -1.793f, 0.762f, 0.379f, -0.247f, -0.009f};
    std::array<float, 6> wavePos2 = {-1.517f, -1.594f, -0.239f, -0.084f, -0.247f, -0.686f};

    for (int i = 0; i < 3; ++i) {
        std::cout << "Wave " << i+1 << "..." << std::endl;
        robot.setAllJointAngles(wavePos1, waveSpeeds, softAccs);
        robot.waitMovementFinished();
        robot.setAllJointAngles(wavePos2, waveSpeeds, softAccs);
        robot.waitMovementFinished();
    }

    // Return to ready
    std::cout << "Returning to ready position..." << std::endl;
    robot.setAllJointAngles(readyPos, softSpeeds, softAccs);
    robot.waitMovementFinished();

    // Return to home
    std::cout << "Returning to home position..." << std::endl;
    robot.setAllJointAngles(homePos, softSpeeds, softAccs);
    robot.waitMovementFinished();

    sm_st.end();
    std::cout << "Example finished." << std::endl;
    return 0;
}
