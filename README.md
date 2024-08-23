
# System_Orin_NX

NVIDIA Jetson Orin NX 上配置环境的步骤，包括安装 ROS 2、JetPack SDK、OpenCV 4.9，以及 `autonomy_stack_diablo_setup` 和 `facefollow` 功能模块的测试。

## NOTE

1. 以下安装省略了部分基本操作，包括但不限于**编译/安装/sudo权限/查看端口/ROS2**的一些基本命令

2. cuda/cudnn/opencv(with cuda)/tensorRT均提供了测试用例，可分别进入每个文件夹进行编译，并运行测试

3. 如若遇到编译/运行错误，可自行核对 `system_deps.txt` 中的提供的依赖版本

## 第一步：安装 ROS 2（Humble 版本）

   ```bash
   wget http://fishros.com/install -O fishros && . fishros
   ```

## 第二步：通过 SDK Manager 安装 JetPack 6.0

1. 打开 NVIDIA SDK Manager，选择 JetPack 6.0，并安装以下组件：
   - CUDA
   - cuDNN
   - CUDA Toolkit
   - TensorRT (可选)

2. 选择离线下载 `.deb` 包，无 ` Host Machine` 模式，请将其转移到 Jetson Orin NX 上，并参考以下命令进行安装：

   ```bash
   sudo dpkg -i *.deb
   sudo apt-get update
   sudo apt-get -f install
   ```

3. 安装完成后，在对应的测试文件夹中编译并运行代码，以验证各个组件是否正确安装。例如，测试 CUDA：

   ```bash
   cd /usr/local/cuda/samples/1_Utilities/deviceQuery
   sudo make
   ./deviceQuery
   ```

   如果显示 CUDA 设备信息，则表示安装成功。

## 第三步：安装 OpenCV 4.9.0 (with CUDA)

1. 首先，确保系统上没有安装 OpenCV（包括 OpenCV-Python）：

   ```bash
   sudo apt-get remove --purge '*opencv*'
   sudo apt-get autoremove
   ```

2. 运行 `install_opencv4.9.0_Jetson.sh` 脚本来编译并安装 OpenCV 4.9.0：

   ```bash
   chmod +x install_opencv4.9.0_Jetson.sh
   ./install_opencv4.9.0_Jetson.sh
   ```

3. 编译过程可能需要一些时间，安装完成后，可以通过以下命令验证安装是否成功：

   ```bash
   python3 -c "import cv2; print(cv2.__version__)"
   ```

   如果显示 `4.9.0`，则表示安装成功。

## 第四步：配置 UVC 摄像头规则

1. 将 `99-uvc-camera.rules` 文件复制到 `/etc/udev/rules.d/` 目录下：

   ```bash
   sudo cp 99-uvc-camera.rules /etc/udev/rules.d/
   ```

2. 重新加载 udev 规则：

   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

## 第五步：编译并安装 `cv_bridge`

1. 进入 `autonomy_stack_diablo_setup/src/utilities/cv_bridge` 目录：

   ```bash
   cd autonomy_stack_diablo_setup/src/utilities/cv_bridge
   ```

2. 编译并安装 `cv_bridge`：

   ```bash
   mkdir build && cd build
   make && sudo make install
   ```

   确保编译和安装过程没有报错。

## 第六步：编译并测试 `autonomy_stack_diablo_setup`

- 按照 [autonomy_stack_diablo_setup](https://github.com/jizhang-cmu/autonomy_stack_diablo_setup) 中的步骤进行编译和测试。

## 第七步：测试 `facefollow` 功能模块

1. 进入 `autonomy_stack_diablo_setup` 目录：

   ```bash
   cd autonomy_stack_diablo_setup
   ```

2. 运行以下命令来加载 ROS 2 环境并启动 `face_detection` 模块：

   ```bash
   source install/setup.bash
   ros2 launch face_detection face_detection.launch.py
   ```

3. 打开另一个终端，加载 ROS 2 环境并运行 `facefollow` 控制模块：

   ```bash
   source install/setup.bash
   ros2 launch face_detection facefollow_ctrl.launch.py
   ```

