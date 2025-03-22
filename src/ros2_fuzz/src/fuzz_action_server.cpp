#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("fuzz_action_server");
  (void)data;
  (void)size;
  // Stub example - normally you'd need to implement handle_goal, handle_result
  rclcpp::shutdown();
  return 0;
}