/**
 * @file RobotUtils.h
 * @brief Common utility functions and constants for robot models
 */

#ifndef _ROBOT_UTILS_H
#define _ROBOT_UTILS_H

#include <cmath>
#include <lerobot_cpp/INST.h>

namespace RobotUtils {

    // Constants for STS3215 (4096 steps = 2π)
    static constexpr float STEPS_PER_REV = 4096.0f;
    static constexpr float RAD_PER_STEP = (2.0f * M_PI) / STEPS_PER_REV;

    /**
     * @brief Convert radians to raw servo steps
     * @param rad Angle in radians
     * @return Raw servo steps (0-4095)
     */
    s16 radToSteps(float rad);

    /**
     * @brief Convert raw servo steps to radians
     * @param steps Raw servo steps
     * @return Angle in radians
     */
    float stepsToRad(s16 steps);

    /**
     * @brief Convert angular velocity (rad/s) to raw servo speed units (steps/s)
     * @param radPerS Angular velocity in radians per second
     * @return Raw servo speed units
     */
    u16 radPerSToStepsPerS(float radPerS);

    /**
     * @brief Convert angular acceleration (rad/s^2) to raw servo acceleration units
     * @param radPerS2 Angular acceleration in radians per second squared
     * @return Raw servo acceleration units (1 unit = 100 steps/s^2)
     */
    u8 radPerS2ToAccUnits(float radPerS2);

} // namespace RobotUtils

#endif // _ROBOT_UTILS_H
