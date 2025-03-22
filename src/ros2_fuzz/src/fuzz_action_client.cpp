#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  rclcpp::init(0, nullptr);
  using namespace std::chrono_literals;
  auto node = rclcpp::Node::make_shared("fuzz_action_client");
  (void)data;
  (void)size;
  // Stub example - normally you'd need an action definition
  rclcpp::shutdown();
  return 0;
}