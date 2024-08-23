cmake_minimum_required(VERSION 3.10)

# 项目信息
project(CudaOpenCVTest)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找CUDA
find_package(CUDA REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加包含目录
include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

# 添加可执行文件
add_executable(CudaOpenCVTest test_opencv.cpp)

# 链接OpenCV和CUDA库
target_link_libraries(CudaOpenCVTest ${OpenCV_LIBS} ${CUDA_LIBRARIES})

# 如果需要支持CUDA编译，可以添加以下设置
set_target_properties(CudaOpenCVTest PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# 打印CUDA设备信息（可选）
message(STATUS "CUDA Version: ${CUDA_VERSION}")
message(STATUS "CUDA Library: ${CUDA_LIBRARIES}")
