/**
 * @file RobotBase.h
 * @brief Non-templated base class for robot manipulators using Feetech STS3215 servos
 * 
 * @details
 * This class provides a common base for manipulator robots, abstracting servo
 * communication and joint control without requiring compile-time template parameters.
 * 
 * @see STS3215 for the underlying communication layer
 * @see Robot for the templated derived class
 */

#ifndef _ROBOT_BASE_H
#define _ROBOT_BASE_H

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Circular constant for calculations when not in math.h
#endif

/**
 * @class RobotBase
 * @brief Non-templated base controller for generic manipulator robots
 */
class RobotBase {
public:
    /**
     * @brief Factory method to create a Robot instance with the specified number of joints
     * @param servoInstance Reference to an initialized STS3215 instance
     * @param numJoints Number of joints (degrees of freedom)
     * @return std::unique_ptr<RobotBase> pointing to Robot<N>, or nullptr if unsupported
     */
    static std::unique_ptr<RobotBase> createRobot(STS3215& servoInstance, int numJoints);

    /**
     * @brief Initialize robot base with servo communication instance
     * @param servoInstance Reference to an initialized STS3215 instance
     */
    explicit RobotBase(STS3215& servoInstance);

    virtual ~RobotBase() = default;

    /**
     * @brief Initialize motors using configured/default servo IDs
     * @return true if successful
     */
    virtual bool init() = 0;

    /**
     * @brief Move a specific joint to an angle in radians
     * @param jointIndex Joint index (0 to numJoints-1)
     * @param angleRad Target angle in radians
     * @param speedRadPerS Angular velocity in radians/second (default 1.0)
     * @param accRadPerS2 Angular acceleration in radians/second^2 (default 0.5)
     * @return 1 on success, 0 on failure
     */
    virtual int setJointAngle(u8 jointIndex, float angleRad, float speedRadPerS = 1.0f, float accRadPerS2 = 0.5f) = 0;

    /**
     * @brief Get current joint angle in radians
     * @param jointIndex Joint index (0 to numJoints-1)
     * @return Angle in radians, or NaN on error
     */
    virtual float getJointAngle(u8 jointIndex) = 0;

    /**
     * @brief Get current joint speed in radians per second
     * @param jointIndex Joint index (0 to numJoints-1)
     * @return Speed in rad/s, or NaN on error
     */
    virtual float getJointSpeed(u8 jointIndex) = 0;

    /**
     * @brief Check if any joint of the robot is moving
     * @return true if moving, false otherwise
     */
    virtual bool isMoving() = 0;

    /**
     * @brief Wait until all joints have finished their current movement
     * @param pollIntervalMs Interval between checks in milliseconds (default 20ms)
     * @param timeoutMs Maximum wait time in milliseconds (0 for no timeout, default 0)
     * @return true if all joints stopped, false on timeout
     */
    virtual bool waitMovementFinished(int pollIntervalMs = 20, int timeoutMs = 0);

    /**
     * @brief Enable/disable torque for all joints
     * @param enable true to enable, false to disable
     */
    virtual void enableTorque(bool enable) = 0;

    /**
     * @brief Set limits for a specific joint
     * @param jointIndex Joint index (0 to numJoints-1)
     * @param minRad Minimum angle in radians
     * @param maxRad Maximum angle in radians
     */
    virtual void setJointLimit(u8 jointIndex, float minRad, float maxRad) = 0;

    /**
     * @brief Get limits for a specific joint
     * @param jointIndex Joint index
     * @return Pair of min/max angles in radians, or {NAN, NAN} if index invalid
     */
    virtual std::pair<float, float> getJointLimit(u8 jointIndex) const = 0;

    /**
     * @brief Get number of joints
     * @return Number of joints
     */
    virtual size_t getNumJoints() const = 0;

    /**
     * @brief Get underlying STS3215 instance
     * @return Reference to STS3215
     */
    STS3215& getServoInstance();

protected:
    STS3215& sm_st;
};

#endif // _ROBOT_BASE_H
