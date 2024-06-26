---
title: Micro-ROS Pico Fixed-Rate Publisher Template
time: 2024-06-27
---

<br>

# <span style="color:#47c7ef">**Micro-ROS**</span> <span style="color:#c41f4c">**Raspberry Pi Pico**</span> Fixed-Rate Publisher Template

<br>

## Content
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Publisher Node Code](#publisher-node-code)
- [Code Review](#code-review)
- [CMakeLists.txt](#cmakeliststxt)
- [Deployment Results](#deployment-results)
- [Ten Seconds to Remember It All](#ten-seconds-to-remember-it-all)
- [Conclusion](#conclusion)
  
<br>

## Introduction
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

In this tutorial, we will extend the previous template [Micro-ROS Raspberry Pi Pico Publisher Template](https://robotcopper.github.io/micro-ROS/pico_publisher_template.html). Indeed, instead of publishing at the maximum possible frequency (processor frequency subject to system constraints) we will specify a publication frequency.

<br>

## Prerequisites
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

- Pico SDK installed.
- Basic knowledge of <span style="color:#4762a6">**ROS 2**</span> and understanding of nodes and messages.
- Familiar with the [Micro-ROS Raspberry Pi Pico Publisher Template](https://robotcopper.github.io/micro-ROS/pico_publisher_template.html)

<br>

## Fixed-Rate Publisher Node Code
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

Here is the complete code for the <span style="color:#4762a6">**ROS 2**</span> node using <span style="color:#47c7ef">**Micro-ROS**</span> on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> to publish at fixed-rate a messages via custom serial communication.

```cpp
#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

#include "std_msgs/msg/string.h" // Standard String message for ROS 2

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <string> // Include the standard C++ string library

constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_publisher_t publisher; // Declare the ROS 2 publisher
std_msgs__msg__String publisher_msg; // Declare the ROS 2 message

bool message_send = false; // Flag for message sending

const char * publisher_topic_name = "pico_publisher_topic";
const char * node_name = "pico_node";
const int frec = 50; //publication frequency in Hz

// Define the states
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_node_t node; // Declare the ROS 2 node
rcl_allocator_t allocator; // Declare the memory allocator
rclc_support_t support; // Declare the ROS 2 support
rcl_timer_t timer; // Declare the ROS 2 timer
rclc_executor_t executor; // Declare the ROS 2 executor

#define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling

void publisher_content(rcl_timer_t *timer, int64_t last_call_time) {

    publisher_msg.data.data = const_cast<char *>("Hello World from F.Jousselin!"); // Directly assign the C string
    publisher_msg.data.size = strlen(publisher_msg.data.data); // Set the size of the string
    publisher_msg.data.capacity = publisher_msg.data.size + 1; // Set the capacity of the string
    
    rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL); // Publish the message
    CHECK_RET(ret); // Check and handle the return value

    message_send = true; // Set the flag indicating the message was sent
    gpio_put(LED_PIN, 1); // Turn on the LED
}

bool pingAgent() {
    const int timeout_ms = 100; // Timeout of 100ms
    const uint8_t attempts = 1; // Number of attempts

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts); // Ping the micro-ROS agent
    return (ret == RCL_RET_OK); // Return true if ping succeeded, false otherwise
}

void createEntities() {
    allocator = rcl_get_default_allocator(); // Get the default memory allocator

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator); // Initialize the support
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_node_init_default(&node, node_name, "", &support); // Initialize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            publisher_topic_name); // Initialize the publisher
    CHECK_RET(ret); // Check and handle the return value
    
    int period_ms = 1000 / frec;
    rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(period_ms),
		publisher_content);
    CHECK_RET(ret); // Check and handle the return value
    
    const rosidl_message_type_support_t * type_support =
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
	    
    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
    CHECK_RET(ret); // Check and handle the return value
    ret = rclc_executor_add_timer(&executor, &timer);
    CHECK_RET(ret); // Check and handle the return value
}

void destroyEntities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context); // Get the RMW context
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0); // Set the destruction timeout
    
    rcl_ret_t ret;

    ret = rcl_publisher_fini(&publisher, &node); // Finalize the publisher
    CHECK_RET(ret); // Check and handle the return value

    ret = rcl_node_fini(&node); // Finalize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_support_fini(&support); // Finalize the support
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rcl_timer_fini(&timer); // Finalize the timer
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_fini(&executor); // Finalize the executor
    CHECK_RET(ret); // Check and handle the return value
}

void handle_state_waiting_agent() {
    state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT; // If ping successful, go to AGENT_AVAILABLE, otherwise stay in WAITING_AGENT
}

void handle_state_agent_available() {
    createEntities(); // Create ROS 2 entities
    state = AGENT_CONNECTED; // Go to AGENT_CONNECTED state
}

void handle_state_agent_connected() {
    if (pingAgent()) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Send content if connected
    } else {
        state = AGENT_DISCONNECTED; // If ping fails, go to AGENT_DISCONNECTED
    }
}

void handle_state_agent_disconnected() {
    destroyEntities(); // Destroy ROS 2 entities
    state = WAITING_AGENT; // Return to WAITING_AGENT state
}

void state_machine() {
    switch (state) {
        case WAITING_AGENT:
            handle_state_waiting_agent(); // Handle WAITING_AGENT state
            break;
        case AGENT_AVAILABLE:
            handle_state_agent_available(); // Handle AGENT_AVAILABLE state
            break;
        case AGENT_CONNECTED:
            handle_state_agent_connected(); // Handle AGENT_CONNECTED state
            break;
        case AGENT_DISCONNECTED:
            handle_state_agent_disconnected(); // Handle AGENT_DISCONNECTED state
            break;
        default:
            break;
    }
}

int main() {
    stdio_init_all(); // Initialize standard I/O

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    ); // Set the custom serial transport for micro-ROS

    gpio_init(LED_PIN); // Initialize the LED pin
    gpio_set_dir(LED_PIN, GPIO_OUT); // Set the LED pin direction to output

    allocator = rcl_get_default_allocator(); // Get the default memory allocator
    state = WAITING_AGENT; // Initialize the state to WAITING_AGENT

    while (true) {
        state_machine(); // Handle the state machine

        if (message_send) {
            message_send = false; // Reset the flag
        } else {
            gpio_put(LED_PIN, 0); // Turn off the LED if no new message was sent
        }
    }
    
    return 0; // End of the program
}
```

<br>

## Code Review
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

This code modifies and includes several essential parts of the previous template:

- **Initialization and Configuration**: <br>
Adds frequency, timer and executor declaration

    ```cpp
    const int frec = 50; //publication frequency in Hz
    ```

    ```cpp
    rcl_timer_t timer; // Declare the ROS 2 timer
    rclc_executor_t executor; // Declare the ROS 2 executor
    ```

- **Publisher Content Function**: <br>
Adds parameters to the `publisher_content()` method to manage periodic publisher calls 

    ```cpp
    void publisher_content(rcl_timer_t *timer, int64_t last_call_time)
    ```

- **Agent Connection State Management**: 
    - In the `createEntities()` method are to initialise the timer and its executor:
    
        ```cpp
        int period_ms = 1000 / frec;
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(period_ms),
            publisher_content);
        CHECK_RET(ret); // Check and handle the return value

        const rosidl_message_type_support_t * type_support =
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
            
        ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
        CHECK_RET(ret); // Check and handle the return value
        ret = rclc_executor_add_timer(&executor, &timer);
        CHECK_RET(ret); // Check and handle the return value
        ```

    - The timer and executor are destroyed in the `destroyEntities()` method:
    
        ```cpp
        ret = rcl_timer_fini(&timer); // Finalize the timer
        CHECK_RET(ret); // Check and handle the return value
        
        ret = rclc_executor_fini(&executor); // Finalize the executor
        CHECK_RET(ret); // Check and handle the return value
        ```

    - When the agent is activated in the `handle_state_agent_connected()` method, the executor spin is executed:
    
        ```cpp
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Send content if connected
        ```

<br>

## CMakeLists.txt
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

To compile this program, ensure you edit `CMakeLists.txt` as follows:

```cmake
# Set the minimum required version of CMake for this project
cmake_minimum_required(VERSION 3.13)

# Include the CMake file to import the Raspberry Pi Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Declare the project with its name and the languages used
project(main C CXX ASM)
set(CMAKE_C_STANDARD 11) # Set the C standard to use
set(CMAKE_CXX_STANDARD 17) # Set the C++ standard to use

# Initialize the Pico SDK
pico_sdk_init()

# Add the directory for libraries to search
link_directories(libmicroros)

# Add an executable named "main" including the specified source files
add_executable(main
        publisher.cpp
        pico_uart_transport/pico_uart_transport.c
        )

# Link the necessary libraries to the "main" executable
target_link_libraries(main
    pico_stdlib # Standard library for Pico
    microros # Micro-ROS library
)

# Specify the include directories for the "main" executable
target_include_directories(main PUBLIC
    libmicroros/include # Include headers for Micro-ROS
    pico_uart_transport # Include headers specific to Pico UART transport
)

# Add extra outputs for the "main" executable (binary files, UF2, etc.)
pico_add_extra_outputs(main)

# Set compilation flags to optimize the binary size
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Enable USB output and disable UART output for standard I/O
pico_enable_stdio_usb(main 1) # Enable USB output, necessary for using picotool for loading
pico_enable_stdio_uart(main 0) # Disable UART output

# Add compilation definitions to configure carriage return and line feed (CRLF) support
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0) # Disable CRLF support for UART
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0) # Disable CRLF support for stdio
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0) # Set the default CRLF conversion to 0 (no conversion)

# Add extra outputs again to ensure all configurations are accounted for
pico_add_extra_outputs(main)

```

<br>

## Deployment Results
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/Micro-ROS/pico_fixed-rate-publisher_template_deployment.webp" style="background: transparent;" >
</div>

<br>

## Ten Seconds to Remember It All
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

The aim of this section is to provide a quick overview of the critical parts involved in publishing messages.<br>
In this code a few lines have been added and modified from the [Micro-ROS Raspberry Pi Pico Publisher Template](https://robotcopper.github.io/micro-ROS/pico_publisher_template.html) to integrate a fixed-rate publication and the modified lines are identified in the [Code Review](#code-review) section.

<br>

```note
This publisher uses a reliable Quality of Service (QoS) setting that prioritizes guaranteeing message reception over maximizing the number of sent messages. To use a best-effort QoS, replace rclc_publisher_init_default with rclc_publisher_init_best_effort in the createEntities() method.
```

<br>

## Conclusion
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

In this tutorial, we explored how to extend the [Micro-ROS Raspberry Pi Pico Publisher Template](https://robotcopper.github.io/micro-ROS/pico_publisher_template.html) to publish at  fixed-rate. <br>
You can expand this project by adding more complex behaviours, exploring different message types, or integrating more deeply with existing <span style="color:#4762a6">**ROS 2**</span> components in your robotic network by following one of my other templates. Enjoy exploring <span style="color:#4762a6">**ROS 2**</span> on embedded platforms!

<br>
<br>

```warning
All the codes provided in this template are distributed under the BSD 3-Clause licence.
```