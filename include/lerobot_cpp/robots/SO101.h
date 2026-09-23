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

#include <lerobot_cpp/robots/Robot.h>

/**
 * @class SO101
 * @brief Manipulator controller for SO101 robot with 6-DOF
 */
class SO101 : public Robot<6> {
public:

	// Nominal limits from `so101_new_calib.urdf` (radians), used for logging and safe clamping.
	// URDF mentions some 5° Offset in the joint between lower and upper arms (joint 3)
	// URDF and joint limits taken from https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
	// Commit 385e8d7
	static constexpr std::array<std::pair<float, float>, 6> JOINT_LIMITS = {{
		{-1.91986f, 1.91986f},
		{-1.74533f, 1.74533f},
		{-1.69f, 1.69f}, //Joint 3
		{-1.65806f, 1.65806f},
		{-2.74385f, 2.84121f},
		{-0.174533f, 1.74533f}
	}};

    /**
     * @brief Initialize SO101 robot controller
     * @param servoInstance Reference to an initialized STS3215 instance
     */
    SO101(STS3215& servoInstance);
};

#endif // _SO101_H
