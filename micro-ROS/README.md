---
title: "<img src='/config/assets/images/micro-ROS_logo.png' alt='Logo' style='height: 12px; vertical-align: bottom; transform: translateY(0px);'> micro-ROS"
time: 2024-06-19
---

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/Micro-ROS/banner-light-theme.png" style="background: transparent;" >
</div>

# Micro-ROS

<span style="color:#47c7ef">**Micro-ROS**</span> is an extension of <span style="color:#4762a6">**ROS 2 (Robot Operating System 2)**</span> specifically designed for resource-constrained embedded systems such as microcontrollers. <span style="color:#47c7ef">**Micro-ROS**</span> aims to extend the <span style="color:#4762a6">**ROS 2**</span> ecosystem to embedded devices, enabling easier and more consistent integration into robotic architectures.

Here is a simple representation of the interface between a host computer and a microcontroller using <span style="color:#47c7ef">**Micro-ROS**</span>:

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/Micro-ROS/Micro-ROS_OneAgent_diagram.png" style="background: transparent;" width="60%" >
</div>

<span style="color:#47c7ef">**Micro-ROS**</span> does not limit interfacing to a single microcontroller; multiple microcontrollers can be used as shown below:

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/Micro-ROS/Micro-ROS_MultipleAgent_diagram.png" style="background: transparent;" width="60%" >
</div>

This tutorial for ROS2 Humble will guide you through configuring <span style="color:#47c7ef">**Micro-ROS**</span> on both the host computer and a microcontroller and specifically for the <span style="color:#c41f4c">**Raspberry Pi Pico**</span>. Finally, we'll explore using custom messages with <span style="color:#47c7ef">**Micro-ROS**</span> via cross-compilation.

## Host Computer Side (micro-ROS agent)
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

To run the <span style="color:#47c7ef">**Micro-ROS**</span> agent from the main workspace, you need to include the necessary packages. I recommend to organize them into a folder named `uros` for clarity. Two packages are required:

- **`micro-ROS-Agent`**:
  ```bash
  git clone -b humble git@github.com:micro-ROS/micro-ROS-Agent.git
  ```
- **`micro_ros_msgs`**: Required by micro-ROS-Agent
  ```bash
  git clone -b humble git@github.com:micro-ROS/micro_ros_msgs.git
  ```

Here's how your main workspace structure should look:
```
ros2_ws
└── src
    └── uros
        ├── micro-ROS-Agent
        └── micro_ros_msgs
```

After running
```
colcon build
```
Use this command to launch the agent for the device `/dev/ttyACM0`:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/Micro-ROS/Micros-ROS_agent.gif" >
</div>

## Microcontroller Side (micro-ROS node on Raspberry Pi Pico)
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

For developing micro-ROS nodes on the microcontroller side, create a dedicated working folder, e.g., `pico`. Here, you'll work with two libraries:

- **`pico-sdk`**: <br>
  Libraries and build system necessary for programming on the RP2040.
  ```bash
  git clone git@github.com:raspberrypi/pico-sdk.git
  ```
  Make this library accessible to other files by adding the following environment variable to your `.bashrc`:
  ```bash
  export PICO_SDK_PATH=$HOME/pico/pico-sdk
  ```

- **`micro_ros_raspberrypi_pico_sdk`**: <br>
  Precompiled micro-ROS library for Raspberry Pi Pico.
  ```bash
  git clone -b humble git@github.com:raspberrypi/pico-sdk.git
  ```
  Copy `libmicroros`, `pico_uart_transports.h`, and `pico_uart_transports.cpp` from this folder to your micro-ROS node project folder.

<br>
Your workspace structure for creating micro-ROS nodes should look like this:

```
pico
├── pico-sdk
├── micro_ros_raspberrypi_pico_sdk
│   ├── pico_uart_transports.h (copy directly to your Pico project folder)
│   ├── pico_uart_transports.cpp (copy directly to your Pico project folder)
│   └── libmicroros
│       ├── include
│       └── libmicroros.a
└── my_uros_node_folder
    ├── libmicroros
    │   ├── include
    │   └── libmicroros.a
    └── pico_uart_transport
        ├── pico_uart_transports.h
        └── pico_uart_transports.cpp
```

Once set up, develop your micro-ROS node `.cpp` and its associated `CMakeLists`. I won't describe the creation of the `.cpp` node and the `CMakeLists` here, I'll leave that to the different dedicated templates. Assuming your `CMakeLists` is configured and your `.cpp` node is ready, generate the `.elf` file for your node:

```bash
cd my_uros_node
mkdir build
cd build
cmake ..
make
```

Your `.elf` file for the node will be in the `build` folder:
```
pico
└── my_uros_node
    └── build
        └── main.elf
```

Copy this file to your <span style="color:#c41f4c">**Raspberry Pi Pico**</span> to deploy your <span style="color:#47c7ef">**Micro-ROS**</span>. Congratulations on using <span style="color:#47c7ef">**Micro-ROS**</span>!.

<br>

## Custom message on micro-ROS (on Raspberry Pi Pico)
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

Using custom ROS messages for communication between nodes can be very useful, and <span style="color:#47c7ef">**Micro-ROS**</span> nodes are no exception. Here’s how to integrate your custom message into <span style="color:#47c7ef">**Micro-ROS**</span>:

To avoid cluttering the main ROS workspace, I recommend to work in a dedicated <span style="color:#4762a6">**ROS 2**</span> workspace, here named `uros_ws`.

For cross-compilation, you'll need:

- **`micro_ros_setup`**: <br>
  Tools and utilities for micro-ROS cross-compilation.
  ```bash
  git clone -b humble git@github.com:micro-ROS/micro_ros_setup.git
  ```

- **my_custom_message**: <br>
  Your custom message type. Follow the official ROS 2 tutorial [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) to create it.

<br>

- **`my_colcon.meta`**: <br>
  Specific compilation parameters file.
  <details markdown="1">
  ```json
    {
        "names": {
            "tracetools": {
                "cmake-args": [
                    "-DTRACETOOLS_DISABLED=ON",
                    "-DTRACETOOLS_STATUS_CHECKING_TOOL=OFF"
                ]
            },
            "rosidl_typesupport": {
                "cmake-args": [
                    "-DROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT=ON"
                ]
            },
            "rcl": {
                "cmake-args": [
                    "-DBUILD_TESTING=OFF",
                    "-DRCL_COMMAND_LINE_ENABLED=OFF",
                    "-DRCL_LOGGING_ENABLED=OFF"
                ]
            }, 
            "rcutils": {
                "cmake-args": [
                    "-DENABLE_TESTING=OFF",
                    "-DRCUTILS_NO_FILESYSTEM=ON",
                    "-DRCUTILS_NO_THREAD_SUPPORT=ON",
                    "-DRCUTILS_NO_64_ATOMIC=ON",
                    "-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=ON"
                ]
            },
            "microxrcedds_client": {
                "cmake-args": [
                    "-DUCLIENT_PIC=OFF",
                    "-DUCLIENT_PROFILE_UDP=OFF",
                    "-DUCLIENT_PROFILE_TCP=OFF",
                    "-DUCLIENT_PROFILE_DISCOVERY=OFF",
                    "-DUCLIENT_PROFILE_SERIAL=OFF",
                    "-UCLIENT_PROFILE_STREAM_FRAMING=ON",
                    "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON"
                ]
            },
            "rmw_microxrcedds": {
                "cmake-args": [
                    "-DRMW_UXRCE_MAX_NODES=1",
                    "-DRMW_UXRCE_MAX_PUBLISHERS=10",
                    "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                    "-DRMW_UXRCE_MAX_SERVICES=1",
                    "-DRMW_UXRCE_MAX_CLIENTS=1",
                    "-DRMW_UXRCE_MAX_HISTORY=4",
                    "-DRMW_UXRCE_TRANSPORT=custom"
                ]
            },
            "action_led_interfaces": {
                "cmake-args": [
                    "-DROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT=ON"
                ]
            }
        }
    }
  ```
  </details>

<br>

- **`my_toolchain.cmake`**: <br>
  Project-specific cross-compilation configuration for an ARM Cortex-M0+ microcontroller architecture.
  <details markdown="1">
  ```cmake
    include($ENV{PICO_SDK_PATH}/cmake/preload/toolchains/find_compiler.cmake)
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_CROSSCOMPILING 1)
    set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

    if (NOT PICO_GCC_TRIPLE)
        if (DEFINED ENV{PICO_GCC_TRIPLE})
            set(PICO_GCC_TRIPLE $ENV{PICO_GCC_TRIPLE})
            message("PICO_GCC_TRIPLE set from environment: $ENV{PICO_GCC_TRIPLE}")
        else()
            set(PICO_GCC_TRIPLE arm-none-eabi)
            message("PICO_GCC_TRIPLE defaulted to arm-none-eabi")
        endif()
    endif()

    pico_find_compiler(PICO_COMPILER_CC ${PICO_GCC_TRIPLE}-gcc)
    pico_find_compiler(PICO_COMPILER_CXX ${PICO_GCC_TRIPLE}-g++)
    set(CMAKE_C_COMPILER ${PICO_COMPILER_CC} CACHE FILEPATH "C compiler")
    set(CMAKE_CXX_COMPILER ${PICO_COMPILER_CXX} CACHE FILEPATH "C++ compiler")

    SET(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
    SET(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

    set(FLAGS "-O2 -march=armv6-m -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -fno-exceptions -nostdlib -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

    set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
    set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
  ```
  </details>

Here's how your workspace structure should look for cross-compilation:
```
uros_ws
├── src
│   ├── micro_ros_setup
│   └── my_custom_message
├── my_colcon.meta
└── my_toolchain.cmake
```

Run the following commands to build and generate the necessary files:
```bash
colcon build
```

```bash
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
```

This will create a `firmware` directory at the root of your workspace:
```
uros_ws
└── firmware
```

Next, run:
```bash
ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_toolchain.cmake $(pwd)/my_colcon.meta
```

This will generate the `libmicroros.a` file and an `include` directory within the `firmware/build` folder:
```
uros_ws
└── firmware
    └── build
        ├── include
        └── libmicroros.a
```

Copy the `include` directory and `libmicroros.a` file to your Pico project folder. Now, proceed to generate the `.elf` file for your micro-ROS node as described earlier.

<br>

## References
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

This project was inspired by the following resources:

- [micro.ros.org](https://micro.ros.org/docs/tutorials/core/first_application_linux/) - Official Micro-ROS documentation.
- [MicroROS Custom Msg on Raspberry PI Pico](https://drjonea.co.uk/2023/06/09/microros-custom-msg-on-raspberry-pi-pico/) - Dr. Jon Durrant work.
- [Micro ROS and Robot Operating System on Raspberry PI Pico](https://drjonea.co.uk/2023/05/31/micro-ros-and-robot-operating-system-on-raspberry-pi-pico/) - Dr. Jon Durrant work.