/**
 * @file Robot.h
 * @brief Generic robot model for N-axis manipulators using Feetech STS3215 servos
 * 
 * @details
 * This class provides a high-level abstraction for an N-DOF manipulator robot,
 * representing its N servos as kinematic joints. All physical measurements are
 * handled in SI units (radians and millimeters).
 * 
 * @see STS3215 for the underlying communication layer
 * @see RobotBase for the non-templated base class
 */

#ifndef _ROBOT_H
#define _ROBOT_H

#include <array>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <optional>
#include <vector>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/RobotBase.h>
#include <lerobot_cpp/robots/RobotUtils.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Circular constant for calculations when not in math.h
#endif

/**
 * @class Robot
 * @brief Manipulator controller for generic robot with N degrees of freedom
 * @tparam N Number of servos / joints
 */
template <size_t N>
class Robot : public RobotBase {
public:
    static constexpr size_t NUM_JOINTS = N;

    /**
     * @brief Generate default servo IDs (1 to N)
     * @return Array of servo IDs [1, 2, ..., N]
     */
    static constexpr std::array<u8, N> defaultServoIDs() {
        std::array<u8, N> ids{};
        for (size_t i = 0; i < N; ++i) {
            ids[i] = static_cast<u8>(i + 1);
        }
        return ids;
    }

    /**
     * @brief Generate default joint limits [-pi, pi] for all joints
     * @return Array of default min/max joint limits in radians
     */
    static constexpr std::array<std::pair<float, float>, N> defaultJointLimits() {
        std::array<std::pair<float, float>, N> limits{};
        for (size_t i = 0; i < N; ++i) {
            limits[i] = {-3.14159265f, 3.14159265f};
        }
        return limits;
    }

    /**
     * @brief Initialize generic robot controller
     * @param servoInstance Reference to an initialized STS3215 instance
     * @param limits Optional joint limits array (defaults to [-pi, pi] per joint)
     */
    Robot(STS3215& servoInstance, const std::array<std::pair<float, float>, N>& limits = defaultJointLimits())
        : RobotBase(servoInstance), servoIDs(defaultServoIDs()), jointLimits(limits) {
    }

    virtual ~Robot() = default;

    /**
     * @brief Configure servo IDs and initialize motors (default IDs: 1 to N)
     * @param ids Array of N servo IDs
     * @return true if successful
     */
    bool init(const std::array<u8, N>& ids) {
        servoIDs = ids;

        for (u8 id : servoIDs) {
            // Initialize motor to servo mode (0)
            if (this->sm_st.InitMotor(id, 0, 1) == 0) {
                // Check if motor responds (Ping) as fallback
                if (this->sm_st.Ping(id) == -1) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Initialize motors using currently configured servo IDs
     * @return true if successful
     */
    bool init() override {
        return init(servoIDs);
    }

    /**
     * @brief Move a specific joint to an angle in radians
     * @param jointIndex Joint index (0 to N-1)
     * @param angleRad Target angle in radians
     * @param speedRadPerS Angular velocity in radians/second (default 1.0)
     * @param accRadPerS2 Angular acceleration in radians/second^2 (default 0.5)
     * @return 1 on success, 0 on failure
     */
    int setJointAngle(u8 jointIndex, float angleRad, float speedRadPerS = 1.0f, float accRadPerS2 = 0.5f) override {
        if (jointIndex >= N) return 0;
        // Clamping to joint limits to avoid invalid movements
        std::pair<float, float> limit = jointLimits[jointIndex];
        angleRad = std::clamp(angleRad, limit.first, limit.second);
        s16 steps = RobotUtils::radToSteps(angleRad);
        u16 speedSteps = RobotUtils::radPerSToStepsPerS(speedRadPerS);
        u8 accUnits = RobotUtils::radPerS2ToAccUnits(accRadPerS2);

        return this->sm_st.WritePosEx(servoIDs[jointIndex], steps, speedSteps, accUnits);
    }

    /**
     * @brief Move all joints simultaneously to target angles
     * @param anglesRad Array of N target angles in radians
     * @param speedsRadPerS Array of N angular velocities (optional)
     * @param accsRadPerS2 Array of N angular accelerations (optional)
     */
    void setAllJointAngles(const std::array<float, N>& anglesRad,
                           const std::array<float, N>& speedsRadPerS = {},
                           const std::array<float, N>& accsRadPerS2 = {}) {
        std::array<s16, N> positions{};
        std::array<u16, N> speeds{};
        std::array<u8, N> accs{};

        for (size_t i = 0; i < N; ++i) {
            // Clamping to joint limits to avoid invalid movements
            std::pair<float, float> limit = jointLimits[i];
            positions[i] = RobotUtils::radToSteps(std::clamp(anglesRad[i], limit.first, limit.second));

            speeds[i] = RobotUtils::radPerSToStepsPerS(speedsRadPerS[i]);
            if (speeds[i] == 0 && speedsRadPerS[i] == 0.0f) speeds[i] = 1000; // Default if not specified

            accs[i] = RobotUtils::radPerS2ToAccUnits(accsRadPerS2[i]);
            if (accs[i] == 0 && accsRadPerS2[i] == 0.0f) accs[i] = 10; // Default if not specified
        }

        this->sm_st.SyncWritePosEx(servoIDs.data(), static_cast<u8>(N), positions.data(), speeds.data(), accs.data());
    }

    /**
     * @brief Get current joint angle in radians
     * @param jointIndex Joint index (0 to N-1)
     * @return Angle in radians, or NaN on error
     */
    float getJointAngle(u8 jointIndex) override {
        if (jointIndex >= N) return NAN;

        int pos = this->sm_st.ReadPos(servoIDs[jointIndex]);
        if (pos == -1) return NAN;

        return RobotUtils::stepsToRad((s16)pos);
    }

    /**
     * @brief Get all joint angles via synchronized bus read
     * @return Array of N angles in rad, or nullopt if sync read failed
     */
    std::optional<std::array<float, N>> getAllJointAngles() {
        std::array<s16, N> positions{};
        if (this->sm_st.SyncReadPos(servoIDs.data(), static_cast<u8>(N), positions.data()) == 0)
            return std::nullopt;

        std::array<float, N> anglesRad{};
        for (size_t i = 0; i < N; i++)
            anglesRad[i] = RobotUtils::stepsToRad(positions[i]);

        return anglesRad;
    }

    /**
     * @brief Get current joint speed in radians per second
     * @param jointIndex Joint index (0 to N-1)
     * @return Speed in rad/s, or NaN on error
     */
    float getJointSpeed(u8 jointIndex) override {
        if (jointIndex >= N) return NAN;

        int speedSteps = this->sm_st.ReadSpeed(servoIDs[jointIndex]);
        if (speedSteps == -1) return NAN;

        return RobotUtils::stepsPerSToRadPerS((s16)speedSteps);
    }

    /**
     * @brief Get all joint speeds via synchronized bus read
     * @return Array of N speeds in rad/s, or nullopt if sync read failed
     */
    std::optional<std::array<float, N>> getAllJointSpeeds() {
        std::array<s16, N> speedsSteps{};
        if (this->sm_st.SyncReadSpeed(servoIDs.data(), static_cast<u8>(N), speedsSteps.data()) == 0)
            return std::nullopt;

        std::array<float, N> speedsRad{};
        for (size_t i = 0; i < N; i++)
            speedsRad[i] = RobotUtils::stepsPerSToRadPerS(speedsSteps[i]);

        return speedsRad;
    }

    /**
     * @brief Check if any joint of the robot is moving
     * @return true if moving, false otherwise
     */
    bool isMoving() override {
        for (u8 id : servoIDs) {
            if (this->sm_st.ReadMove(id) == 1) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Enable/disable torque for all joints
     * @param enable true to enable, false to disable
     */
    void enableTorque(bool enable) override {
        for (u8 id : servoIDs) {
            this->sm_st.EnableTorque(id, enable ? 1 : 0);
        }
    }

    /**
     * @brief Set limits for all joints
     * @param limits Array of min/max angle pairs in radians
     */
    void setJointLimits(const std::array<std::pair<float, float>, N>& limits) {
        jointLimits = limits;
    }

    /**
     * @brief Set limits for a specific joint
     * @param jointIndex Joint index (0 to N-1)
     * @param minRad Minimum angle in radians
     * @param maxRad Maximum angle in radians
     */
    void setJointLimit(u8 jointIndex, float minRad, float maxRad) override {
        if (jointIndex < N) {
            jointLimits[jointIndex] = {minRad, maxRad};
        }
    }

    /**
     * @brief Get current joint limits as std::array
     * @return Array of min/max angle pairs in radians
     */
    const std::array<std::pair<float, float>, N>& getJointLimits() const {
        return jointLimits;
    }

    /**
     * @brief Get limits for a specific joint
     * @param jointIndex Joint index
     * @return Pair of min/max angles in radians, or {NAN, NAN} if index invalid
     */
    std::pair<float, float> getJointLimit(u8 jointIndex) const override {
        if (jointIndex < N) {
            return jointLimits[jointIndex];
        }
        return {NAN, NAN};
    }

    /**
     * @brief Get configured servo IDs as std::array
     * @return Array of N servo IDs
     */
    const std::array<u8, N>& getServoIDs() const {
        return servoIDs;
    }

    /**
     * @brief Get number of joints
     * @return Number of joints
     */
    size_t getNumJoints() const override {
        return N;
    }

protected:
    std::array<u8, N> servoIDs;
    std::array<std::pair<float, float>, N> jointLimits;
};

#endif // _ROBOT_H
