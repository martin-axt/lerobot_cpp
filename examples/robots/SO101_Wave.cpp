/**
 * @file SO101_Wave.cpp
 * @brief Wave movement for robot manipulator
 * 
 * @details
 * This example demonstrates a soft waving movement with the robot.
 * 
 * Hardware Requirements:
 * - Feetech STS3215 protocol servos (IDs: 1 to N)
 * - Robot arm assembly (e.g. SO101 6-DOF)
 * - Serial connection at 1Mbps
 */

#include <iostream>
#include <array>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/Robot.h>
#include <lerobot_cpp/robots/SO101.h>
#include "ExampleUtils.h"

template <size_t N>
int runWave(Robot<N>& robot) {
    // Initialize with default IDs (1 to N)
    std::cout << "Initializing robot (" << N << " joints)..." << std::endl;
    if (!robot.init()) {
        std::cerr << "Failed to initialize robot." << std::endl;
        return 1;
    }

    // Enable torque
    robot.enableTorque(true);

    // Home position
    std::array<float, N> homePos{};
    homePos.fill(0.0f);
    
    // Wave parameters
    std::array<float, N> softSpeeds{};
    softSpeeds.fill(0.5f); // 0.5 rad/s
    std::array<float, N> waveSpeeds{};
    waveSpeeds.fill(1.0f); // 1 rad/s
    std::array<float, N> softAccs{};
    softAccs.fill(2.0f);   // 2.0 rad/s^2

    std::array<float, N> readyPos{};
    readyPos.fill(0.0f);
    std::array<float, N> wavePos1{};
    wavePos1.fill(0.0f);
    std::array<float, N> wavePos2{};
    wavePos2.fill(0.0f);

    constexpr float so101_ready[6] = {-1.517f, -1.657f, 0.193f, -0.018f, -0.247f, -1.135f};
    constexpr float so101_wave1[6] = {-1.519f, -1.793f, 0.762f, 0.379f, -0.247f, -0.009f};
    constexpr float so101_wave2[6] = {-1.517f, -1.594f, -0.239f, -0.084f, -0.247f, -0.686f};

    for (size_t i = 0; i < N && i < 6; ++i) {
        readyPos[i] = so101_ready[i];
        wavePos1[i] = so101_wave1[i];
        wavePos2[i] = so101_wave2[i];
    }
    if (N < 5) {
        wavePos1[N - 1] = 0.5f;
        wavePos2[N - 1] = -0.5f;
    }

    // Hello position (Raising arm)
    std::cout << "Raising arm to 'ready' position..." << std::endl;
    robot.setAllJointAngles(readyPos, softSpeeds, softAccs);
    robot.waitMovementFinished();

    // Wave movement
    std::cout << "Performing soft 'hello' wave..." << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << "Wave " << i + 1 << "..." << std::endl;
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
        return runWave(robot);
    });

    sm_st.end();
    return result;
}
