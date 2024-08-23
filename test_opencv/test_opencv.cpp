#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>  // 添加 cudaarithm 头文件
#include <iostream>

int main() {
    // 检查CUDA是否可用
    int cuda_device_count = cv::cuda::getCudaEnabledDeviceCount();
    if (cuda_device_count == 0) {
        std::cout << "No CUDA devices found or CUDA is not enabled in OpenCV." << std::endl;
        return -1;
    }

    std::cout << "CUDA is available. Number of CUDA devices: " << cuda_device_count << std::endl;

    // 设置CUDA设备
    cv::cuda::setDevice(0);

    // 创建一个简单的Mat对象（CPU上）
    cv::Mat img = cv::Mat::ones(256, 256, CV_8UC1) * 50; // 创建一个灰度图像
    cv::Mat img2 = cv::Mat::ones(256, 256, CV_8UC1) * 50; // 创建第二个灰度图像

    // 将Mat对象上传到GPU上
    cv::cuda::GpuMat d_img, d_img2, d_result;
    d_img.upload(img);
    d_img2.upload(img2);

    // 在GPU上进行矩阵加法操作
    cv::cuda::addWeighted(d_img, 1.0, d_img2, 1.0, 0.0, d_result);

    // 将结果从GPU下载回CPU
    cv::Mat result;
    d_result.download(result);

    // 输出结果的前几个像素值作为验证
    std::cout << "Result (top-left corner): " << std::endl;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            std::cout << static_cast<int>(result.at<uchar>(i, j)) << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "CUDA matrix addition applied successfully." << std::endl;

    return 0;
}
