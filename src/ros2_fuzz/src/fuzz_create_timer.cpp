#include <rclcpp/rclcpp.hpp>
#include <chrono>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  if (size < 1) return 0;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("fuzz_node");
  auto period_ms = static_cast<size_t>(data[0]) * 10;
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    []() {});
  rclcpp::shutdown();
  return 0;
}