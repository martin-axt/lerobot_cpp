/**
 * @file SO101.h
 * @brief Robot model for SO101 6-axis manipulator using Feetech STS3215 servos
 * 
 * @details
 * This class provides a high-level abstraction for the SO101 robot, representing 
 * its 6 servos as kinematic joints. All physical measurements are handled in 
 * SI units (radians and millimeters) for ease of integration with inverse 
 * kinematics and higher-level planners.
 * 
 * **Servo Configuration (ID range 1-6):**
 * - Joint 1 (Base Rotation): ID 1
 * - Joint 2 (Shoulder): ID 2
 * - Joint 3 (Elbow): ID 3
 * - Joint 4 (Wrist Pitch): ID 4
 * - Joint 5 (Wrist Roll): ID 5
 * - Joint 6 (Gripper/End Effector): ID 6
 * 
 * **Conversion Factors:**
 * - 4096 steps = 2π radians (~360 degrees)
 * - 1 step ≈ 0.001534 radians
 * - Acceleration units: 100 steps/s²
 * 
 * **Hardware Requirements:**
 * - 6x Feetech STS3215 protocol servos
 * - Serial communication at 1Mbps recommended
 * 
 * @see STS3215 for the underlying communication layer
 */

#ifndef _SO101_H
#define _SO101_H

#include <array>
#include <vector>
#include <cmath>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/RobotUtils.h>

/**
 * @class SO101
 * @brief Manipulator controller for SO101 robot with 6-DOF
 */
class SO101 {
public:
    /**
     * @brief Initialize SO101 robot controller
     * @param servoInstance Reference to an initialized STS3215 instance
     */
    SO101(STS3215& servoInstance);

    /**
     * @brief Configure joint limits and IDs (default IDs: 1 to 6)
     * @param ids Array of 6 servo IDs
     * @return true if successful
     */
    bool init(const std::array<u8, 6>& ids = {1, 2, 3, 4, 5, 6});

    /**
     * @brief Reload joint limits from servos
     * @return true if successful
     */
    bool reloadLimits();

    /**
     * @brief Save joint limits to servos' EEPROM and reload internal cache
     * @param minPositions Array of 6 minimum joint positions (steps, 0-4095)
     * @param maxPositions Array of 6 maximum joint positions (steps, 0-4095)
     * @return true if successful
     */
    bool storeLimits(const std::array<int, 6>& minPositions, const std::array<int, 6>& maxPositions);

    /**
     * @brief Move a specific joint to an angle in radians
     * @param jointIndex Joint index (0-5)
     * @param angleRad Target angle in radians
     * @param speedRadPerS Angular velocity in radians/second (default 1.0)
     * @param accRadPerS2 Angular acceleration in radians/second^2 (default 0.5)
     * @return 1 on success, 0 on failure
     */
    int setJointAngle(u8 jointIndex, float angleRad, float speedRadPerS = 1.0f, float accRadPerS2 = 0.5f);

    /**
     * @brief Move all joints simultaneously to target angles
     * @param anglesRad Array of 6 target angles in radians
     * @param speedsRadPerS Array of 6 angular velocities (optional)
     * @param accsRadPerS2 Array of 6 angular accelerations (optional)
     */
    void setAllJointAngles(const std::array<float, 6>& anglesRad,
                           const std::array<float, 6>& speedsRadPerS = {},
                           const std::array<float, 6>& accsRadPerS2 = {});

    /**
     * @brief Get current joint angle in radians
     * @param jointIndex Joint index (0-5)
     * @return Angle in radians, or NaN on error
     */
    float getJointAngle(u8 jointIndex);

    /**
     * @brief Check if any joint of the robot is moving
     * @return true if moving, false otherwise
     */
    bool isMoving();

    /**
     * @brief Wait until all joints have finished their current movement
     * @param pollIntervalMs Interval between checks in milliseconds (default 20ms)
     * @param timeoutMs Maximum wait time in milliseconds (0 for no timeout, default 0)
     * @return true if all joints stopped, false on timeout
     */
    bool waitMovementFinished(int pollIntervalMs = 20, int timeoutMs = 0);

    /**
     * @brief Enable/disable torque for all joints
     * @param enable true to enable, false to disable
     */
    void enableTorque(bool enable);

private:
    STS3215& sm_st;
    std::array<u8, 6> servoIDs{};
    std::array<s16, 6> minLimits{};
    std::array<s16, 6> maxLimits{};
};

#endif // _SO101_H
