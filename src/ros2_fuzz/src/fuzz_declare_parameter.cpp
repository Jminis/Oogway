#include <rclcpp/rclcpp.hpp>
#include <string>
#include <cstdint>
#include <cstring>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
  if (Size < 5) return 0;

  auto context = std::make_shared<rclcpp::Context>();
  try {
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<rclcpp::Node>("fuzz_node", options);

    uint8_t name_len = Data[0] % 20 + 1;
    if (Size < 1 + name_len + sizeof(int)) return 0;

    std::string param_name(reinterpret_cast<const char*>(&Data[1]), name_len);
    int val;
    std::memcpy(&val, &Data[1 + name_len], sizeof(int));

    node->declare_parameter<int>(param_name, val);
    int out;
    node->get_parameter(param_name, out);
  } catch (...) {}

  rclcpp::shutdown(context);
  return 0;
}
