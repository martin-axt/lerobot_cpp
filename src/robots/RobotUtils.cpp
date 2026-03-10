#include <lerobot_cpp/robots/RobotUtils.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Circular constant for calculations when not in math.h
#endif

namespace RobotUtils {

    s16 radToSteps(float rad) {
        float steps = rad * (STEPS_PER_REV / (2.0f * M_PI));
        return (s16)round(steps);
    }

    float stepsToRad(s16 steps) {
        return (float)steps * RAD_PER_STEP;
    }

    u16 radPerSToStepsPerS(float radPerS) {
        float stepsPerS = radPerS / RAD_PER_STEP;
        return (u16)abs(round(stepsPerS));
    }

    u8 radPerS2ToAccUnits(float radPerS2) {
        float stepsPerS2 = radPerS2 / RAD_PER_STEP;
        float accUnits = stepsPerS2 / 100.0f;
        return (u8)abs(round(accUnits));
    }

} // namespace RobotUtils
