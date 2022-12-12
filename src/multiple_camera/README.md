# multiple_camera

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
   git clone https://github.com/shangjie-li/multiple_camera.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`multiple_camera/launch/usb_cam-test.launch`
   ```Shell
   ...
   <param name="video_device" value="/dev/video0" />
   <param name="image_width" value="1280" />
   <param name="image_height" value="720" />
   <param name="framerate" value="30" />
   <param name="pixel_format" value="mjpeg" />
   <param name="camera_frame_id" value="cam_1" />
   <param name="camera_name" value="cam_1" />
   ...
   <param name="video_device" value="/dev/video1" />
   <param name="image_width" value="1280" />
   <param name="image_height" value="720" />
   <param name="framerate" value="30" />
   <param name="pixel_format" value="mjpeg" />
   <param name="camera_frame_id" value="cam_2" />
   <param name="camera_name" value="cam_2" />
   ...
   <param name="video_device" value="/dev/video2" />
   <param name="image_width" value="1280" />
   <param name="image_height" value="720" />
   <param name="framerate" value="30" />
   <param name="pixel_format" value="mjpeg" />
   <param name="camera_frame_id" value="cam_3" />
   <param name="camera_name" value="cam_3" />
   ...
   <param name="video_device" value="/dev/video3" />
   <param name="image_width" value="1280" />
   <param name="image_height" value="720" />
   <param name="framerate" value="30" />
   <param name="pixel_format" value="mjpeg" />
   <param name="camera_frame_id" value="cam_4" />
   <param name="camera_name" value="cam_4" />
   ...
   ```
    - `video_device`的值为`ls /dev/video*`下的编号。
 - 编写各相机标定文件，存放在`~/.ros/camera_info`目录下，以`camera_name`命名，例如`cam_1.yaml`
   ```Shell
   image_width: 1280
   image_height: 720
   camera_name: cam_1
   camera_matrix:
     rows: 3
     cols: 3
     data: [600.8084424381476, 0, 614.2845413844701, 0, 602.1021176450085, 334.8654850271977, 0, 0, 1]
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [0.03041809356381303, -0.04509464307944619, -0.003105665562312764, -0.001037639416656771, 0]
   rectification_matrix:
     rows: 3
     cols: 3
     data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
   projection_matrix:
     rows: 3
     cols: 4
     data: [581.921142578125, 0, 605.0637343471462, 0, 0, 604.7725830078125, 332.6973828462578, 0, 0, 0, 1, 0]
   ```
   
## 运行
 - 启动`multiple_camera`
   ```Shell
   roslaunch multiple_camera usb_cam-test.launch
   ```
 - 查看图像
   ```Shell
   cd multiple_camera
   ./multiple_view.sh
   ```
   
