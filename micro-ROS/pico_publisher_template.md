---
title: Micro-ROS Pico Publisher Template
time: 2024-06-20
---

<br>

# <span style="color:#47c7ef">**Micro-ROS**</span> <span style="color:#c41f4c">**Raspberry Pi Pico**</span> Publisher Template

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

In this tutorial, we will create a <span style="color:#4762a6">**ROS 2**</span> node on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> using <span style="color:#47c7ef">**Micro-ROS**</span> to publish messages via a custom serial communication. <span style="color:#47c7ef">**Micro-ROS**</span> enables connectivity between microcontrollers like <span style="color:#c41f4c">**Raspberry Pi Pico**</span> and <span style="color:#4762a6">**ROS 2**</span> middleware, facilitating robust and flexible communication between embedded systems and more powerful computers.

<br>

## Prerequisites
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

- Pico SDK installed.
- Basic knowledge of <span style="color:#4762a6">**ROS 2**</span> and understanding of nodes and messages.

<br>

### Publisher Node Code
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

Here is the complete code for the <span style="color:#4762a6">**ROS 2**</span> node using <span style="color:#47c7ef">**Micro-ROS**</span> on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> to publish messages via custom serial communication.

```cpp
#include <stdio.h>
#include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

extern "C" {
#include <rcl/rcl.h> // Main ROS 2 client library
#include <rcl/error_handling.h> // Error handling for ROS 2
#include <rclc/rclc.h> // C library for ROS 2
#include <rclc/executor.h> // Executor for ROS 2
#include <rmw_microros/rmw_microros.h> // Middleware for Micro-ROS

#include "std_msgs/msg/string.h" // Standard String message for ROS 2

#include "pico_uart_transports.h" // UART transport specific for Pico
}

#include <string> // Include the standard C++ string library

constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

rcl_publisher_t publisher; // Declare the ROS 2 publisher
std_msgs__msg__String publisher_msg; // Declare the ROS 2 message

bool message_send = false; // Flag for message sending

// Define the states
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_timer_t timer; // Declare the ROS 2 timer
rcl_node_t node; // Declare the ROS 2 node
rcl_allocator_t allocator; // Declare the memory allocator
rclc_support_t support; // Declare the ROS 2 support

#define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling

void publisher_content() {

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

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts); // Ping the Micro-ROS agent
    return (ret == RCL_RET_OK); // Return true if ping succeeded, false otherwise
}

void createEntities() {
    allocator = rcl_get_default_allocator(); // Get the default memory allocator

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator); // Initialize the support
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_node_init_default(&node, "pico_node", "", &support); // Initialize the node
    CHECK_RET(ret); // Check and handle the return value

    ret = rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "pico_publisher_topic"); // Initialize the publisher
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

    rclc_support_fini(&support); // Finalize the support
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
        publisher_content(); // Send content if connected
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
    ); // Set the custom serial transport for Micro-ROS

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

- **Initialization and Configuration**: Includes necessary libraries for <span style="color:#47c7ef">**Micro-ROS**</span>, <span style="color:#4762a6">**ROS 2**</span>, Pico SDK, and declarations of required variables.

    ```cpp
    #include <stdio.h>
    #include "pico/stdlib.h" // Include the standard library for Raspberry Pi Pico

    extern "C" {
    #include <rcl/rcl.h> // Main ROS 2 client library
    #include <rcl/error_handling.h> // Error handling for ROS 2
    #include <rclc/rclc.h> // C library for ROS 2
    #include <rclc/executor.h> // Executor for ROS 2
    #include <rmw_microros/rmw_microros.h> // Middleware for Micro-ROS

    #include "std_msgs/msg/string.h" // Standard String message for ROS 2

    #include "pico_uart_transports.h" // UART transport specific for Pico
    }

    #include <string> // Include the standard C++ string library

    constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN; // Define the LED pin number

    rcl_publisher_t publisher; // Declare the ROS 2 publisher
    std_msgs__msg__String publisher_msg; // Declare the ROS 2 message

    bool message_send = false; // Flag for message sending

    // Define the states
    enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
    } state;

    rcl_timer_t timer; // Declare the ROS 2 timer
    rcl_node_t node; // Declare the ROS 2 node
    rcl_allocator_t allocator; // Declare the memory allocator
    rclc_support_t support; // Declare the ROS 2 support

    #define CHECK_RET(ret) if (ret != RCL_RET_OK) { rcl_reset_error(); } // Macro for silent error handling
    ```

- **Publisher Content Function**: `publisher_content()` initializes and publishes a <span style="color:#4762a6">**ROS 2**</span> message. 

    ```cpp
    void publisher_content() {

        publisher_msg.data.data = const_cast<char *>("Hello World from F.Jousselin!"); // Directly assign the C string
        publisher_msg.data.size = strlen(publisher_msg.data.data); // Set the size of the string
        publisher_msg.data.capacity = publisher_msg.data.size + 1; // Set the capacity of the string
        
        rcl_ret_t ret = rcl_publish(&publisher, &publisher_msg, NULL); // Publish the message
        CHECK_RET(ret); // Check and handle the return value

        message_send = true; // Set the flag indicating the message was sent
        gpio_put(LED_PIN, 1); // Turn on the LED
    }
    ```

- **Agent Connection State Management**: Functions (`pingAgent()`, `createEntities()`, `destroyEntities()`, etc.) manage <span style="color:#4762a6">**ROS 2**</span> entity initialization, publishing, and destruction based on <span style="color:#47c7ef">**Micro-ROS**</span> agent connection state.

- **State Machine**: `state_machine()` controls the workflow based on the current connection state.

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

- **Main Loop**: The main `main` loop manages the state machine and the initialisation of GPIOs.

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
        ); // Set the custom serial transport for Micro-ROS

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
    <img src="/config/assets/images/Micro-ROS/pico_publisher_template_deployment.gif" style="background: transparent;" >
</div>

<br>

## Ten Seconds to Remember It All
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

The aim of this section is to provide a quick overview of the critical parts involved in publishing messages.<br>
In this code, the lines dedicated to publishing a specific message type and content are as follows:

- The inclusion of the dedicated message header file: `#include "std_msgs/msg/string.h"`
- The declaration of the ROS message: `std_msgs__msg__String publisher_msg;`
- The content of the `publisher_content()` method
- In the `createEntities()` method, the use of `ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String)` during the publisher initialization

<br>

## Conclusion
<hr style="height: 1px; background-color: #dfe2e5; border: none;">

In this tutorial, we explored how to create a <span style="color:#4762a6">**ROS 2**</span> node on <span style="color:#c41f4c">**Raspberry Pi Pico**</span> using <span style="color:#47c7ef">**Micro-ROS**</span>. We configured custom serial communication, initialized <span style="color:#4762a6">**ROS 2**</span> entities like a node and publisher, and implemented a state machine to manage the connection to a <span style="color:#47c7ef">**Micro-ROS**</span> agent. This tutorial gets you started with <span style="color:#47c7ef">**Micro-ROS**</span> on microcontrollers like <span style="color:#c41f4c">**Raspberry Pi Pico**</span>, paving the way for deeper integration with ROS. <br>
You can expand this project by adding subscriptions, exploring different message types, or integrating more deeply with existing <span style="color:#4762a6">**ROS 2**</span> components in your robotic network by following one of my other templates. Enjoy exploring <span style="color:#4762a6">**ROS 2**</span> on embedded platforms!