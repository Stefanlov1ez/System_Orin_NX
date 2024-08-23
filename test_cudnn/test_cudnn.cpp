#include <iostream>
#include <cudnn.h>

int main() {
    cudnnHandle_t cudnn;
    cudnnStatus_t status = cudnnCreate(&cudnn);

    if (status == CUDNN_STATUS_SUCCESS) {
        std::cout << "cuDNN is working correctly!" << std::endl;
    } else {
        std::cerr << "cuDNN initialization failed: " << cudnnGetErrorString(status) << std::endl;
    }

    cudnnDestroy(cudnn);
    return 0;
}
