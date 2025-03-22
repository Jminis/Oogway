FROM ros:humble-ros-base

RUN apt update && apt install -y \
  clang lld libc++-dev libc++abi-dev \
  cmake git python3-colcon-common-extensions \
  libstd-msgs-dev libstd-srvs-dev \
  && rm -rf /var/lib/apt/lists/*

ENV CC=clang
ENV CXX=clang++

WORKDIR /workspace
COPY src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ros2_fuzz --cmake-args -DCMAKE_CXX_COMPILER=clang++ -DUSE_LIBFUZZER=ON

