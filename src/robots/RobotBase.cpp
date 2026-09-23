#include <lerobot_cpp/robots/RobotBase.h>
#include <lerobot_cpp/robots/Robot.h>

RobotBase::RobotBase(STS3215& servoInstance)
    : sm_st(servoInstance) {
}

bool RobotBase::waitMovementFinished(int pollIntervalMs, int timeoutMs) {
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

STS3215& RobotBase::getServoInstance() {
    return sm_st;
}

std::unique_ptr<RobotBase> RobotBase::createRobot(STS3215& servoInstance, int numJoints) {
    switch (numJoints) {
        case 1: return std::make_unique<Robot<1>>(servoInstance);
        case 2: return std::make_unique<Robot<2>>(servoInstance);
        case 3: return std::make_unique<Robot<3>>(servoInstance);
        case 4: return std::make_unique<Robot<4>>(servoInstance);
        case 5: return std::make_unique<Robot<5>>(servoInstance);
        case 6: return std::make_unique<Robot<6>>(servoInstance);
        case 7: return std::make_unique<Robot<7>>(servoInstance);
        case 8: return std::make_unique<Robot<8>>(servoInstance);
        case 9: return std::make_unique<Robot<9>>(servoInstance);
        case 10: return std::make_unique<Robot<10>>(servoInstance);
        case 11: return std::make_unique<Robot<11>>(servoInstance);
        case 12: return std::make_unique<Robot<12>>(servoInstance);
        case 13: return std::make_unique<Robot<13>>(servoInstance);
        case 14: return std::make_unique<Robot<14>>(servoInstance);
        case 15: return std::make_unique<Robot<15>>(servoInstance);
        case 16: return std::make_unique<Robot<16>>(servoInstance);
        default: return nullptr;
    }
}
