/**
 * @file SO101_Calibration.cpp
 * @brief Calibration tool for SO101 6-DOF robot
 * 
 * @details
 * This example provides an interactive procedure to calibrate the SO101 robot:
 * 1. Disable torque to allow manual movement.
 * 2. User aligns all joints to physical 0 (Home).
 * 3. Save Home (sets current positions as 0 in servos) via CalibrationOfs.
 * 4. User moves joints to find their mechanical limits (min/max).
 * 5. Save limits to servos' EEPROM.
 */

#include <iostream>
#include <array>
#include <vector>
#include <limits>
#include <algorithm>
#include <unistd.h>
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
        std::cerr << "Failed to initialize serial port: " << serialPort << std::endl;
        return 1;
    }

    SO101 robot(sm_st);
    std::array<u8, 6> ids = {1, 2, 3, 4, 5, 6};
    if (!robot.init(ids)) {
        std::cerr << "Failed to initialize SO101 robot." << std::endl;
        return 1;
    }

    // Step 1: Manual positioning for Home
    std::cout << "\n--- STEP 1: SET HOME POSITION ---" << std::endl;
    std::cout << "Disabling torque to allow manual positioning..." << std::endl;
    robot.enableTorque(false);
    std::cout << "Please manually move all robot joints to their ZERO/HOME positions." << std::endl;
    std::cout << "Press ENTER when finished to save Home positions..." << std::endl;
    ExampleUtils::waitForEnter();

    std::cout << "Saving Home positions (setting current physical position as logical 0)..." << std::endl;
    for (u8 id : ids) {
        if (sm_st.CalibrationOfs(id) == -1) {
            std::cerr << "Warning: Failed to calibrate offset for Servo ID " << (int)id << std::endl;
        }
    }
    std::cout << "Home positions saved." << std::endl;

    // Step 2: Record Limits
    std::cout << "\n--- STEP 2: RECORD JOINT LIMITS ---" << std::endl;
    std::cout << "Now move the robot arm through its full range of motion for each joint." << std::endl;
    std::cout << "The program will track the minimum and maximum positions reached." << std::endl;
    std::cout << "Press ENTER when you are done recording limits..." << std::endl;

    std::array<int, 6> minPos;
    minPos.fill(4096);
    std::array<int, 6> maxPos;
    maxPos.fill(-4096);

    std::cout << "Recording... (Press ENTER to stop)" << std::endl;
    while (!ExampleUtils::isEnterPressed())
    {
            // No input, poll servos
            for (size_t i = 0; i < ids.size(); ++i) {
                int pos = sm_st.ReadPos(ids[i]);
                if (pos != -1) {
                    if (pos < minPos[i]) minPos[i] = pos;
                    if (pos > maxPos[i]) maxPos[i] = pos;
                }
            }
            
            std::cout << "\rCurrent Limits [ID: min/max]: ";
            for (size_t i = 0; i < ids.size(); ++i) {
                std::cout << (int)ids[i] << ": " << minPos[i] << "/" << maxPos[i] << "  ";
            }
            std::cout << std::flush;
    }
    std::cout << "\nRecording stopped." << std::endl;

    // Step 3: Save Limits to EEPROM
    std::cout << "\n--- STEP 3: SAVE LIMITS TO EEPROM ---" << std::endl;
    std::cout << "Saving recorded limits to servos..." << std::endl;

    if (robot.storeLimits(minPos, maxPos)) {
        std::cout << "\nCalibration complete! Limits saved to EEPROM." << std::endl;
    } else {
        std::cerr << "\nError: Failed to save limits to EEPROM." << std::endl;
    }
    
    std::cout << "Enabling torque and moving to Home (Midpoint)..." << std::endl;
    robot.enableTorque(true);
    std::array<float, 6> home;
    home.fill(0.0f);
    robot.setAllJointAngles(home);
    robot.waitMovementFinished();

    sm_st.end();
    return 0;
}
