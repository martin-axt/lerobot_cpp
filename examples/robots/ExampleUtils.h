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

} // namespace ExampleUtils

#endif // _EXAMPLE_UTILS_H
