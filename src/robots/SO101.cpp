#include <lerobot_cpp/robots/SO101.h>
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>
#include <unistd.h>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Circular constant for calculations when not in math.h
#endif

SO101::SO101(STS3215& servoInstance) : sm_st(servoInstance) {
}

bool SO101::init(const std::array<u8, 6>& ids) {
    servoIDs = ids;

    for (u8 id : servoIDs) {
        // Initialize motor to servo mode (0)
        if (sm_st.InitMotor(id, 0, 1) == 0) {
            // Check if motor responds (Ping) as fallback
            if (sm_st.Ping(id) == -1) {
                return false;
            }
        }
    }
    return true;
}

int SO101::setJointAngle(u8 jointIndex, float angleRad, float speedRadPerS, float accRadPerS2) {

    if (jointIndex >= servoIDs.size()) return 0;
	// Clamping to given URDF angles to avoid invalid movements
	std::pair<float, float> jointLimits = JOINT_LIMITS[jointIndex];
	angleRad = std::clamp(angleRad, jointLimits.first, jointLimits.second);
	s16 steps = RobotUtils::radToSteps(angleRad);
    u16 speedSteps = RobotUtils::radPerSToStepsPerS(speedRadPerS);
    u8 accUnits = RobotUtils::radPerS2ToAccUnits(accRadPerS2);
    
    return sm_st.WritePosEx(servoIDs[jointIndex], steps, speedSteps, accUnits);
}

void SO101::setAllJointAngles(const std::array<float, 6>& anglesRad,
                             const std::array<float, 6>& speedsRadPerS,
                             const std::array<float, 6>& accsRadPerS2) {
    std::array<s16, 6> positions{};
    std::array<u16, 6> speeds{};
    std::array<u8, 6> accs{};
    
    // Default values if empty arrays are provided (not really possible with std::array unless we check values)
    // But here we can just use the provided values. 
    // Wait, the default parameters in header were {}, which for std::array means zero-initialized.
    // Let's check if speeds/accs are "default" (e.g. 0) and use defaults in that case if needed, 
    // or just assume the caller provides what they want.
    // In the vector version, it checked sizes.
    
    for (size_t i = 0; i < 6; ++i) {
    	// Clamping to given URDF angles to avoid invalid movements
    	std::pair<float, float> jointLimits = JOINT_LIMITS[i];
    	positions[i] = RobotUtils::radToSteps(std::clamp(anglesRad[i], jointLimits.first, jointLimits.second));

        // If speed is 0, we might want to use a default speed, 
        // but let's just convert what's provided.
        speeds[i] = RobotUtils::radPerSToStepsPerS(speedsRadPerS[i]);
        if (speeds[i] == 0 && speedsRadPerS[i] == 0.0f) speeds[i] = 1000; // Default if not specified

        accs[i] = RobotUtils::radPerS2ToAccUnits(accsRadPerS2[i]);
        if (accs[i] == 0 && accsRadPerS2[i] == 0.0f) accs[i] = 10; // Default if not specified
    }
    
    sm_st.SyncWritePosEx(servoIDs.data(), 6, positions.data(), speeds.data(), accs.data());
}

float SO101::getJointAngle(u8 jointIndex) {
    if (jointIndex >= servoIDs.size()) return NAN;
    
    int pos = sm_st.ReadPos(servoIDs[jointIndex]);
    if (pos == -1) return NAN;

	return RobotUtils::stepsToRad((s16)pos);
}

bool SO101::isMoving() {
    for (u8 id : servoIDs) {
        if (sm_st.ReadMove(id) == 1) {
            return true;
        }
    }
    return false;
}

bool SO101::waitMovementFinished(int pollIntervalMs, int timeoutMs) {
    auto start = std::chrono::steady_clock::now();
    while (isMoving()) {
        if (timeoutMs > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsed >= timeoutMs) {
                return false;
            }
        }
        usleep(pollIntervalMs * 1000);
    }
    return true;
}

void SO101::enableTorque(bool enable) {
    for (u8 id : servoIDs) {
        sm_st.EnableTorque(id, enable ? 1 : 0);
    }
}