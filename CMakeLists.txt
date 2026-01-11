cmake_minimum_required(VERSION 3.10)
project(GNC_Learning_Repo)

set(CMAKE_CXX_STANDARD 14)

# 1. 设置输出路径（把所有可执行文件都扔到 build/bin 里，方便找）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 2. 包含公共库的头文件路径（让所有子目录都能找到 common）
# include_directories("${PROJECT_SOURCE_DIR}/common/include")

# 3. 寻找第三方库 (Eigen) - 全局配置，子目录继承
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 4. 添加子目录（每开一个新章节，就在这里加一行）
# add_subdirectory(01_InventendPendulum/inverted_pendulum)

add_subdirectory(02_UAV/RotationR&R)
