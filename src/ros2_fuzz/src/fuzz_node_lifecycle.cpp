#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fuzz_lifecycle");
  (void)data;
  (void)size;
  node->configure();
  node->activate();
  node->deactivate();
  node->cleanup();
  rclcpp::shutdown();
  return 0;
}