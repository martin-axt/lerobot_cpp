#include <lerobot_cpp/robots/SO101.h>

SO101::SO101(STS3215& servoInstance) : Robot<6>(servoInstance, JOINT_LIMITS) {
}