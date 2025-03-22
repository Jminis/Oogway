#include <rclcpp/rclcpp.hpp>
#include <string>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("fuzz_node");
  std::string key(reinterpret_cast<const char*>(data), size);
  rclcpp::Parameter param(key, 42);
  node->set_parameter(param);
  rclcpp::shutdown();
  return 0;
}