FROM ubuntu:latest

RUN apt-get update && apt-get install -y lsb-release gnupg wget

RUN echo "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-12 main" >> /etc/apt/sources.list

RUN echo "deb-src http://apt.llvm.org/focal/ llvm-toolchain-focal-12 main" >> /etc/apt/sources.list

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key| apt-key add -

RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata keyboard-configuration

RUN apt-get update && \
    apt-get install -y clang-12 clang-tools-12 clang-12-doc libclang-common-12-dev libclang-12-dev libclang1-12 clang-format-12 clangd-12

RUN update-alternatives --install /usr/bin/c++ c++ $(command -v clang++-12) 1000
RUN update-alternatives --install /usr/bin/cc  cc  $(command -v clang-12)   1000

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y ros-noetic-desktop-full ros-noetic-ackermann-msgs && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN rosdep init && rosdep update