cmake_minimum_required(VERSION 3.10)
project(TestTensorRT)

# 设置 CUDA 路径
set(CUDA_INCLUDE_DIRS /usr/local/cuda/include)
set(CUDA_LIBRARIES /usr/local/cuda/lib64)

# 包含 TensorRT 和 CUDA 头文件
include_directories(${CUDA_INCLUDE_DIRS} /usr/include/aarch64-linux-gnu)

# 链接 TensorRT 和 CUDA 库
link_directories(${CUDA_LIBRARIES} /usr/lib/aarch64-linux-gnu)

# 添加可执行文件
add_executable(test_tensorRT test_tensorRT.cpp)

# 链接库
target_link_libraries(test_tensorRT nvinfer cudart)
