# lerobot_cpp

![Project Status](https://img.shields.io/badge/Status-Active-green)
[![Repository](https://img.shields.io/badge/Github-martin--axt%2Flerobot__cpp-purple?style=flat&logo=github&link=https%3A%2F%2Fgithub.com%2Fmartin-axt%2Flerobot_cpp)](https://github.com/martin-axt/lerobot_cpp)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/martin-axt/lerobot_cpp?label=License)

Linux C++ SDK for LeRobot SO-100/101 and other robots using Feetech's STS3215 Motors.

> **📌 About This Fork:** This repository is a fork of [SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) limited to STS3215 with high-level robot classes and examples. For low-level protocol details and other motor series, please refer to the original repository.

## Features

- **High-Level Control**: Dedicated `SO101` class for 6-DOF robot control.
- **Physical Units**: Work directly with **radians** and **millimeters**.
- **Smooth Motion**: Integrated support for velocity, acceleration, and synchronized movement.
- **Calibration**: Interactive tools for setting home positions and joint limits.
- **Optimized for Linux**: Native C++17 implementation for x86_64 and ARM64.

## Quick Start

### 1. Build the library

```bash
sudo apt-get update && sudo apt-get install build-essential cmake git -y
git clone https://github.com/martin-axt/lerobot_cpp.git
cd lerobot_cpp
mkdir build && cd build
cmake .. && make -j4
```

### 2. Calibrate the robot (Optional)
The joint limits and home positions can be set interactively using the calibration tool.
```bash
./lerobot_cpp_calibration /dev/ttyACM0
```


### 3. Run the Wave Example

To build and run the provided wave example:

```bash
# Check which port your robot is connected to and ensure it's writable
sudo chmod 666 /dev/ttyACM0

# From the build directory
./lerobot_cpp_example_SO101_Wave /dev/ttyACM0
```

## Robot Examples

| Example | Description |
|---------|-------------|
| `SO101_Home` | Moves the robot to the calibrated 0 radian (midpoint) position. |
| `SO101_Wave` | Performs a soft "hello" waving animation. |
| `SO101_Record` | Logs current joint positions in real-time for manual teaching. |
| `SO101_Calibration` | Interactive tool to set EEPROM offsets and mechanical limits. |

## Documentation

- **High-Level API**: See `include/lerobot_cpp/robots/SO101.h` for the manipulator interface.
- **Low-Level Protocol**: This SDK is focused on the **STS3215** series. For other models (SCS series, etc.), see the [original SCServo_Linux repository](https://github.com/adityakamath/SCServo_Linux).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
