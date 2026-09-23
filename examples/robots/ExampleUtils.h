/**
 * @file ExampleUtils.h
 * @brief Common utilities for robot examples
 */

#ifndef _EXAMPLE_UTILS_H
#define _EXAMPLE_UTILS_H

#include <iostream>
#include <limits>
#include <sys/select.h>
#include <unistd.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/robots/Robot.h>
#include <lerobot_cpp/robots/SO101.h>

/**
 * @namespace ExampleUtils
 * @brief Shared helper functions for lerobot_cpp examples
 */
namespace ExampleUtils {

/**
 * @brief Check if the ENTER key has been pressed without blocking indefinitely.
 * @details Uses the select() system call on STDIN to check for available data.
 * @param timeoutUs Timeout in microseconds to wait for input (default 0).
 * @return true if data is available on STDIN (usually indicating ENTER was pressed).
 */
inline bool isEnterPressed(long timeoutUs = 100000) {
    fd_set rfds;
    struct timeval tv;
    
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = timeoutUs;

    int retval = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    
    if (retval > 0) {
        // Data available, consume the input and return true
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return true;
    }
    
    return false;
}

/**
 * @brief Simple blocking wait for ENTER key.
 */
inline void waitForEnter() {
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

/**
 * @brief Run a function with a Robot instance initialized with the specified number of joints.
 * Defaults to SO101 configuration when 6 joints are specified.
 * @tparam Func Callable taking Robot<N>&
 * @param numJoints Number of joints (degrees of freedom)
 * @param sm_st Reference to initialized STS3215 instance
 * @param func Function to execute with the robot instance
 * @return Exit code from func
 */
template <typename Func>
int runWithRobot(size_t numJoints, STS3215& sm_st, Func&& func) {
    switch (numJoints) {
        case 1: { Robot<1> r(sm_st); return func(r); }
        case 2: { Robot<2> r(sm_st); return func(r); }
        case 3: { Robot<3> r(sm_st); return func(r); }
        case 4: { Robot<4> r(sm_st); return func(r); }
        case 5: { Robot<5> r(sm_st); return func(r); }
        case 6: { SO101 r(sm_st); return func(r); } // Default SO101 configuration
        default:
            std::cerr << "Unsupported number of joints: " << numJoints << " (supported: 1-6)" << std::endl;
            return 1;
    }
}

} // namespace ExampleUtils

#endif // _EXAMPLE_UTILS_H
