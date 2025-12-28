FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-22.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

# Optionally install the cmake for vcpkg
COPY ./reinstall-cmake.sh /tmp/

# 第一部分：微软原本的逻辑（处理 CMake）
RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

# 第二部分：你自己的逻辑（安装 Eigen 和其他工具）
# 1. apt-get update: 更新列表
# 2. export ...: 防止安装过程中弹窗问你时区设置
# 3. install: 安装 eigen 和 build-essential
# 4. rm -rf ...: 清理安装垃圾，减小镜像体积
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    libeigen3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*