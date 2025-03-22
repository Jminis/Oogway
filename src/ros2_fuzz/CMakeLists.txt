cmake_minimum_required(VERSION 3.5)
project(ros2_fuzz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 경고 플래그
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 퍼징 기능 활성화 여부
option(USE_LIBFUZZER "Enable libFuzzer and ASan" ON)

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 퍼징 관련 sanitizer 플래그
set(FUZZ_SAN_COMPILE_FLAGS
  -fsanitize=address,fuzzer
  -O1 -g
  -fno-omit-frame-pointer
)

set(FUZZ_SAN_LINK_FLAGS
  -fsanitize=address,fuzzer
)

# 반복되는 빌드 매크로
macro(build_fuzzer target_name source_file)
  add_executable(${target_name} src/${source_file})
  ament_target_dependencies(${target_name}
    rclcpp
    std_msgs
    std_srvs
    rclcpp_action
    rclcpp_lifecycle
  )
  if(USE_LIBFUZZER)
    target_compile_options(${target_name} PRIVATE ${FUZZ_SAN_COMPILE_FLAGS})
    target_link_options(${target_name} PRIVATE ${FUZZ_SAN_LINK_FLAGS})
  endif()
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endmacro()

# === 개별 퍼징 타겟 빌드 ===
build_fuzzer(fuzz_create_publisher     fuzz_create_publisher.cpp)
build_fuzzer(fuzz_create_subscription  fuzz_create_subscription.cpp)
build_fuzzer(fuzz_declare_parameter    fuzz_declare_parameter.cpp)
build_fuzzer(fuzz_create_client        fuzz_create_client.cpp)
build_fuzzer(fuzz_set_parameter        fuzz_set_parameter.cpp)
build_fuzzer(fuzz_create_timer         fuzz_create_timer.cpp)
build_fuzzer(fuzz_action_client        fuzz_action_client.cpp)
build_fuzzer(fuzz_action_server        fuzz_action_server.cpp)
build_fuzzer(fuzz_node_lifecycle       fuzz_node_lifecycle.cpp)
build_fuzzer(fuzz_create_service       fuzz_create_service.cpp)

ament_package()
