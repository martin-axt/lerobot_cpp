cmake_minimum_required(VERSION 3.10)
set(project "lerobot_cpp")
project(${project} LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

option(BUILD_EXAMPLES "Builds the examples" ON)

set(Headers
        include/lerobot_cpp/STS3215.h
        include/lerobot_cpp/SCSerial.h
        include/lerobot_cpp/SCS.h
        include/lerobot_cpp/INST.h
        include/lerobot_cpp/ServoUtils.h
        include/lerobot_cpp/SyncWriteBuffer.h
        include/lerobot_cpp/ServoErrors.h

        # Robots
        include/lerobot_cpp/robots/SO101.h
        include/lerobot_cpp/robots/RobotUtils.h
)

set(Source
        src/STS3215.cpp
        src/SCSerial.cpp
        src/SCS.cpp

        # Robots
        src/robots/SO101.cpp
        src/robots/RobotUtils.cpp
)


add_library(${project} STATIC ${Headers} ${Source})

target_include_directories(${project} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

if (${BUILD_EXAMPLES})
    include(examples/examples.cmake)
endif ()