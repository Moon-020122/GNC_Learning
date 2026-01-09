# 01_/inverted_pendulum/CMakeLists.txt

# 1. 自动查找 src 目录下所有的 .cpp 文件，存入变量 SOURCES 中
# file(GLOB SOURCES "src/*.cpp")

# file(GLOB HEADERS "include/*.hpp")

add_executable(pendulum_sim 
    src/main.cpp
    src/pendulum_dynamics.cpp
    src/logger.cpp
    src/pendulum_ekf_model.cpp
    src/ekf.cpp



    include/pendulum_dynamics.hpp
    include/pendulum_params.hpp
    include/saturations.hpp
    include/integrators.hpp
    include/controllers.hpp
    include/logger.hpp
    include/linear_discrete_model.hpp
    include/plant_linear.hpp
    include/pid.hpp
    include/rate_limiter.hpp
    include/cascade_pid.hpp
    include/observers.hpp
    include/pendulum_ekf_model.hpp
    include/ekf.hpp
)


# 2. 定义可执行文件
#    名字叫 pendulum_sim (你可以随便改)
#    源文件使用上面找到的 SOURCES

# add_executable(pendulum_sim ${SOURCES} ${HEADERS})

# 3. 指定头文件路径
#    PRIVATE include: 让编译器去当前的 include/ 文件夹找 .hpp 文件
target_include_directories(pendulum_sim PRIVATE include)

# 4. 链接 Eigen 库
#    虽然 Eigen 是纯头文件库，但这样写能保证路径正确传递
target_link_libraries(pendulum_sim Eigen3::Eigen)