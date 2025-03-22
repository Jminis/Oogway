#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
  if (Size < 1) return 0;

  auto context = std::make_shared<rclcpp::Context>();
  try {
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<rclcpp::Node>("fuzz_node", options);

    std::string topic(reinterpret_cast<const char*>(Data), Size);
    auto pub = node->create_publisher<std_msgs::msg::String>(topic, 10);
    (void)pub;
  } catch (...) {}

  rclcpp::shutdown(context);
  return 0;
}
