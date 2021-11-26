# camera_driver

ROS driver for multiple cameras

## 安装
 - 使用这个包需要计算机拥有多个USB Bus，运行时应将不同的相机连接在不同的USB Bus上，查看USB Bus信息
   ```Shell
   lsusb
   ```
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/camera_driver.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`camera_driver/launch/camera_driver.launch`
   ```Shell
   <rosparam param="cameras_soft_id" > [0, 1, 2, 3] </rosparam>
   <rosparam param="cameras_hard_id" > [1, 2, 3, 4] </rosparam>
   ```
    - `cameras_soft_id`指明不同相机在`ls /dev/video*`下的编号。
    - `cameras_hard_id`指明不同相机的标定文件名。
 - 修改`camera_driver/conf`目录下的各标定文件，标定文件以`数字.yaml`命名，格式如下
   ```Shell
   %YAML:1.0
   ---
   CameraMat: !!opencv-matrix
      rows: 3
      cols: 3
      dt: d
      data: [433.9740099516489, 0, 326.126084593681, 0, 434.4705611820618, 232.211589751021, 0, 0, 1]
   DistCoeff: !!opencv-matrix
      rows: 5
      cols: 1
      dt: d
      data: [-0.3782688310073667, 0.1377794575242172, 1.771495495902854e-06, 0.001848560332994272, 0]
   ImageSize: [640, 480]
   Reprojection Error: 0.0
   DistModel: plumb_bob
   ```

## 运行
 - 启动`camera_driver`
   ```Shell
   roslaunch camera_driver camera_driver.launch
   ```
 - 查看图像
   ```Shell
   cd camera_driver
   ./multiple_view.sh
   ```
   
   
   
