#include "state_estimator/state_estimator_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <memory>
#include <thread>
#include <chrono>


void mySigintHandler(int sig);

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, mySigintHandler);

  auto node = std::make_shared<state_estimator::state_estimator_node>();
  
  RCLCPP_INFO(node->get_logger(), "Starting state estimator");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (!g_request_shutdown && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
    
  node->shutdown();
  rclcpp::shutdown();
  
  return 0;
}

void mySigintHandler(int sig) {
    g_request_shutdown = 1;
}

