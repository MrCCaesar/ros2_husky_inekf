// STL
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <iostream>

// Internal Libraries
#include "utils/husky_data.hpp"
#include "communication/husky_comms.h"
#include "system/husky_system.hpp"

#include "core/InEKF.h"
#include "core/NoiseParams.h"

// External Libraries
#include "rclcpp/rclcpp.hpp" // ROS2

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include <boost/timer/timer.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = rclcpp::Node::make_shared("husky_estimator");

    // Threading
    husky_inekf::husky_data_t husky_data_buffer;

    // Set noise parameters
    inekf::NoiseParams params;

    std::cout << "main node namespace: " << node->get_namespace() << std::endl;

    // Initialize HuskyComms (assuming it accepts rclcpp::Node::SharedPtr)
    HuskyComms husky_comms(node, &husky_data_buffer);

    // Initialize HuskySystem
    HuskySystem system(node, &husky_data_buffer);

    // Create executor for spinning
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Main loop
    while (rclcpp::ok())
    {
        // boost::timer::auto_cpu_timer t;
        system.step();

        // Process ROS2 callbacks (non-blocking)
        executor.spin_some(std::chrono::milliseconds(1));
    }

    // Cleanup
    rclcpp::shutdown();
    return 0;
}