---
title: Micro-ROS Pico Subscriber Template
time: 2024-06-26
---

<br>

# <span style="color:#47c7ef">**Micro-ROS**</span> <span style="color:#c41f4c">**Raspberry Pi Pico**</span> Subscriber Template

<br>

## Content
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Subscriber Node Code](#subscriber-node-code)
- [Code Review](#code-review)
- [CMakeLists.txt](#cmakeliststxt)
- [Deployment Results](#deployment-results)
- [Ten Seconds to Remember It All](#ten-seconds-to-remember-it-all)
- [Conclusion](#conclusion)
  
<br>

## Introduction
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

In this tutorial, we will create a <span style="color:#4762a6">**ROS 2**</span> node on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> using <span style="color:#47c7ef">**Micro-ROS**</span> to subscribe to a messages via a custom serial communication. <span style="color:#47c7ef">**Micro-ROS**</span> enables connectivity between microcontrollers like <span style="color:#c41f4c">**Raspberry Pi Pico**</span> and <span style="color:#4762a6">**ROS 2**</span> middleware, facilitating robust and flexible communication between embedded systems and more powerful computers.

<br>

## Prerequisites
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

- Pico SDK installed.
- Basic knowledge of <span style="color:#4762a6">**ROS 2**</span> and understanding of nodes and messages.

<br>

## Subscriber Node Code
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

Here is the complete code for the <span style="color:#4762a6">**ROS 2**</span> node using <span style="color:#47c7ef">**Micro-ROS**</span> on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> to subscribe to a messages via custom serial communication.

```cpp
#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

#include "std_msgs/msg/int8.h" // Standard Int8 message for ROS 2

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <string> // Include the standard C++ string library

constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_subscription_t subscriber; // Declare the ROS 2 subscriber
std_msgs__msg__Int8 subscriber_msg; // Declare the ROS 2 message

bool message_send = false; // Flag for message sending

const char * subscriber_topic_name = "pico_subscriber_topic";
const char * node_name = "pico_node";

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
rclc_executor_t executor; // Declare the ROS 2 executor

#define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling

void subscription_callback(const void * msgin){
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
    
    for (int8_t i = 0; i < msg->data; ++i) {
        gpio_put(LED_PIN, 1); // Turn on the LED
        sleep_ms(500); // Wait 500 ms
        gpio_put(LED_PIN, 0); // Turn off the LED
        sleep_ms(500); // Wait 500 ms
    }
    
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

    ret = rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
            subscriber_topic_name); // Initialize the subscriber
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_add_subscription(&executor,
			&subscriber,
			&subscriber_msg,
			&subscription_callback,
			ON_NEW_DATA);
    CHECK_RET(ret); // Check and handle the return value
}

void destroyEntities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context); // Get the RMW context
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0); // Set the destruction timeout
    
    rcl_ret_t ret;

    ret = rcl_subscription_fini(&subscriber, &node); // Finalize the subscriber
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rclc_executor_fini(&executor); // Finalize the executor
    CHECK_RET(ret); // Check and handle the return value
    
    ret = rcl_node_fini(&node); // Finalize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_support_fini(&support); // Finalize the support
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Execute ROS 2 executor to process messages every 100 millisecond
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

This code includes several essential parts:

- **Initialization and Configuration**: <br>
Includes necessary libraries for <span style="color:#47c7ef">**Micro-ROS**</span>, <span style="color:#4762a6">**ROS 2**</span>, Pico SDK, and declarations of required variables.

    ```cpp
    #include <stdio.h>
    #include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

    extern "C" {
    #include <rcl/rcl.h> // Main ROS 2 client library
    #include <rcl/error_handling.h> // Error handling for ROS 2
    #include <rclc/rclc.h> // C library for ROS 2
    #include <rclc/executor.h> // Executor for ROS 2
    #include <rmw_microros/rmw_microros.h> // Middleware for micro-ROS

    #include "std_msgs/msg/int8.h" // Standard Int8 message for ROS 2

    #include "pico_uart_transports.h" // UART transport specific for Pico
    }

    #include <string> // Include the standard C++ string library

    constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

    rcl_subscription_t subscriber; // Declare the ROS 2 subscriber
    std_msgs__msg__Int8 subscriber_msg; // Declare the ROS 2 message

    bool message_send = false; // Flag for message sending

    const char * subscriber_topic_name = "pico_subscriber_topic";
    const char * node_name = "pico_node";

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
    rclc_executor_t executor;

    #define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling
    ```

- **Subscriber Callback Function**: <br>
`subscription_callback(const void * msgin)` initializes and subscribe a <span style="color:#4762a6">**ROS 2**</span> message.

    ```cpp
    void subscription_callback(const void * msgin){
        const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
        
        for (int8_t i = 0; i < msg->data; ++i) {
            gpio_put(LED_PIN, 1); // Turn on the LED
            sleep_ms(500); // Wait 500 ms
            gpio_put(LED_PIN, 0); // Turn off the LED
            sleep_ms(500); // Wait 500 ms
        }
        
    }
    ```

- **Agent Connection State Management**: <br>
Functions (`pingAgent()`, `createEntities()`, `destroyEntities()`, etc.) manage <span style="color:#4762a6">**ROS 2**</span> entity initialization, publishing, and destruction based on <span style="color:#47c7ef">**Micro-ROS**</span> agent connection state.

- **State Machine**: <br>
`state_machine()` controls the workflow based on the current connection state.

    ```cpp
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
    ```

- **Main Loop**: <br>
The main `main` loop manages the state machine and the initialisation of GPIOs.

    ```cpp
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
        subscriber.cpp
        pico_uart_transport/pico_uart_transport.c
        )

# Link the necessary libraries to the "main" executable
target_link_libraries(main
    pico_stdlib # Standard library for Pico
    microros # micro-ROS library
)

# Specify the include directories for the "main" executable
target_include_directories(main PUBLIC
    libmicroros/include # Include headers for micro-ROS
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
    <img src="/config/assets/images/Micro-ROS/pico_subscriber_template_deployment.webp" style="background: transparent;" >
</div>



<br>

## Ten Seconds to Remember It All
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

The aim of this section is to provide a quick overview of the critical parts involved in subscribing to a messages.<br>
In this code, the lines dedicated to subscribing to a specific message type and content are as follows:

- The inclusion of the dedicated message header file: <br>
`#include "std_msgs/msg/int8.h"`
- The declaration of the ROS message: <br>
`std_msgs__msg__Int8 subscriber_msg;`
- The declaration of the ROS topic name: <br>
`const char * subscriber_topic_name = "pico_subscriber_topic";`
- The declaration of the ROS node name: <br>
`const char * node_name = "pico_node";`
- The content of the `subscription_callback(const void * msgin)` method.
- In the `createEntities()` method, the use of `ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8)` during the subscriber initialization.
- In the `handle_state_agent_connected()` method, the use of `rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));` for the call of the subscriber callback.

<br>

## Conclusion
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

In this tutorial, we explored how to create a <span style="color:#4762a6">**ROS 2**</span> node on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> using <span style="color:#47c7ef">**Micro-ROS**</span> to subscribe a int8 on a topic to make the <span style="color:#c41f4c">**Raspberry Pi Pico**</span> LED blink the number of time it has been sent. We configured custom serial communication, initialized <span style="color:#4762a6">**ROS 2**</span> entities like a node and subscriber, and implemented a state machine to manage the connection to a <span style="color:#47c7ef">**Micro-ROS**</span> agent. This tutorial gets you started with <span style="color:#47c7ef">**Micro-ROS**</span> on microcontrollers like <span style="color:#c41f4c">**Raspberry Pi Pico**</span>, paving the way for deeper integration with ROS. <br>
You can expand this project by adding more complex behaviours, exploring different message types, or integrating more deeply with existing <span style="color:#4762a6">**ROS 2**</span> components in your robotic network by following one of my other templates. Enjoy exploring <span style="color:#4762a6">**ROS 2**</span> on embedded platforms!

<br>
<br>

```warning
All the codes provided in this template are distributed under the BSD 3-Clause licence.
```