#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
  if (Size < 1) return 0;

  auto context = std::make_shared<rclcpp::Context>();
  try {
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<rclcpp::Node>("fuzz_node", options);

    std::string name(reinterpret_cast<const char*>(Data), Size);
    auto client = node->create_client<std_srvs::srv::Trigger>(name);
    (void)client;

  } catch (...) {}

  rclcpp::shutdown(context);
  return 0;
}
