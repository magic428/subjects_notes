# realsense d435i 标定 imu 与 camera  

**使用到的工具:**

- Kalibr;
- imu_tools;

**假设你已经具备了以下条件:**  

- Ubuntu 1804 - melodic ros package(其他平台和版本也可以);
- librealsense sdk 安装;  
- realsense ros wrapper.  

## 一. 标定目的  

realsense d435i 包含两个红外相机、红外发射器、RGB 相机和 IMU 四个模块, 显然四个传感器的空间位置是不同的, 我们在处理图像和 IMU 数据时需要将这些数据都放在统一的坐标系中, 因此需要标定 IMU 设备相对 RGB 相机的空间位置 (包括旋转和位移) . 另外, 相机固有参数比如焦距、畸变参数等以及 IMU 的零偏和 scale 系数等都需要提前知道. 前者我们称为外参, 后者称为内参, 在运行程序前我们需要标定它们, 不论程序是否有自标定功能, 毕竟好的初始标定值对于自标定来说也是有利的.   


## 二. 标定准备  

1\) 安装 realsense-sdk2.0, 包括 d435i 的驱动等, 直到可以运行 realsense-viewer, 可以看到图像和深度图;   
2\) 安装 realsense-ros-wrapper, 这个包可以直接读取 d435i 的数据流, 并发布各个 topic 供标定程序订阅; 
3\) 安装 imu_utils, 前提要安装 code_utils, 这个用于标定 IMU 的噪音密度和随机游走系数;   
4\) 安装 Kalibr. 这个软件包可以同时标定多个相机的外参和内参 (提供不同的相机的模型) , 另外可以标定相机和 IMU 的外参.  

关于 realsense-sdk2.0 和 realsense-ros-wrapper 可以参考别的文章. 这里重点介绍 imu_utils 和 Kalibr 的安装.  

### 2.1 安装 imu_tools   

```bash
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/code_utils.git

# 打开 ~/catkin_ws/src/code_utils/src/sumpixel_test.cpp 文件
# 修改 #include "backward.hpp" 为 #include “code_utils/backward.hpp”

cd ..
catkin_make
```

这里需要修改 ~/catkin_ws/src/code_utils/src/sumpixel_test.cpp 文件中的 #include "backward.hpp" 语句, 否者会提示文件找不到而引发编译错误 `code_utils-master/src/sumpixel_test.cpp:2:24: fatal error: backward.hpp:No such file or directory`.  

```bash
cd ~/catkin_ws/src
git clone https://github.com/gaowenliang/imu_utils.git
cd ..
catkin_make
```

至此, 安装好了 imu_tools. 

### 2.2 安装 Kalibr  

```bash
cd ~/catkin_ws/src
git clone https://github.com/etHz-asl/Kalibr.git

cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```

至此, 安装好了 kalibr. 

在标定 (如kalibr_calibrate_cameras) 前, 需设置启动 ros 软件包的环境变量:  

```bash
source ~/catkin_ws/devel/setup.bash
```

## 三. IMU 标定 - 获取内参 

在 imu_utils/launch/ 目录下新建 d435i_imu.launch, 或复制一份 A3.launch, 写入以下内容.  

```xml
<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "/camera/imu"/>
        <param name="imu_name" type="string" value= "d435i"/>
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "120"/>
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>
```

其中,   

- imu_topic 为 IMU 设备发布的 topic 名字;  
- imu_name 为 IMU 设备名称;  
- max_time_min 为标定的数据时长, 单位为: 分钟.  


**设置 rs_camera.launch**:  

realsense 的 ROS launch 文件中有将 accel 和 gyro 两个整成一个 imu topic 的配置, 设置一下就好, 对应为:  

```xml
<arg name="enable_sync"              default="true"/>
<arg name="unite_imu_method"         value="linear_interpolation"/>     【原来值为空: value= “ ”/】
```

其中, linear_interpolation 可以换成 copy.  

**运行相机**  

```bash
roslaunch realsense2_camera rs_camera.launch
```

**录制 imu 数据包**

```bash
rosbag record -O imu_calibration /camera/imu
```

录制 2 小时按 CTRL+C 完成.  

**运行校准程序**  

```bash
roslaunch imu_utils d435i_imu.launch
```

**回放数据包(200 倍回放)**  

```bash
rosbag play -r 200 imu_calibration.bag
```

经过这些标定会生成一个 yaml 文件和很多 txt 文件, 主要是 yaml 文件, 如下所示给出了加速度计和陀螺仪三轴的 noise_density 和 random_walk,  同时计算出了平均值, 后面 IMU+Camera 联合标定的时候需要这些均值.   

```bash
%YAML:1.0
---
type: IMU
name: d435i
Gyr:
   unit: " rad/s"
   avg-axis:
      gyr_n: 4.6252299865200278e-03
      gyr_w: 6.5747864795302093e-05
   x-axis:
      gyr_n: 2.6645160389095852e-03
      gyr_w: 1.0139677196221221e-05
   y-axis:
      gyr_n: 4.8116489932123977e-03
      gyr_w: 1.9114728943547871e-05
   z-axis:
      gyr_n: 6.3995249274381022e-03
      gyr_w: 1.6798918824613716e-04
Acc:
   unit: " m/s^2"
   avg-axis:
      acc_n: 2.7972858634203440e-02
      acc_w: 4.9164474715115763e-04
   x-axis:
      acc_n: 2.9423415791454164e-02
      acc_w: 4.5193315084899861e-04
   y-axis:
      acc_n: 2.2288625305226104e-02
      acc_w: 5.5760059333782625e-04
   z-axis:
      acc_n: 3.2206534805930054e-02
      acc_w: 4.6540049726664803e-04
```

根据标定结果修改 IMU.yaml, 后面会用到, 其文件内容为:  

```yaml
rostopic: /camera/imu     #the IMU ROS topic
update_rate: 200.0        #Hz (for discretization of the val

accelerometer_noise_density: 2.7972858634203440e-02 #Noise density (continuous-time)
accelerometer_random_walk: 4.9164474715115763e-04 #Bias random walk
gyroscope_noise_density: 4.6252299865200278e-03 #Noise density (continuous-time)
gyroscope_random_walk: 6.5747864795302093e-05 #Bias random walk
```

## 四. 使用 Kalibr 标定 realsense D435i Camera   

**1\) 将棋盘格参数设置保存到 checkboard.yaml 文件**  

checkboard 的参数配置 yaml 文件在下载链接: https://github.com/etHz-asl/kalibr/wiki/downloads, 下载后修改以下参数 (根据自己的实际标定板参数进行修改, 这里是 9x12 的标定棋盘格) :   

```yaml
target_type: 'checkerboard'  #gridtype
targetCols: 8                #number of internal chessboard corners, 注意是内角点数目
targetRows: 11               #number of internal chessboard corners, 注意是内角点数目
rowSpacingMeters: 0.025      #size of one chessboard square [m]
colSpacingMeters: 0.025      #size of one chessboard square [m]
```

这里也可以使用 april 标定板, 例如:

```yaml
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.02           #size of apriltag, edge to edge [m]
tagSpacing: 0.3        #ratio of space between tags to tagSize
```

使用 kalibr_create_target_pdf 命令可以自动生成你想要的任何 apriltag 尺寸, 命令格式为:  

```bash
kalibr_create_target_pdf --type apriltag --nx [NUM_COLS] --ny [NUM_ROWS] --tsize [TAG_WIDTH_M] --tspace [TAG_SPACING_PERCENT]]
```

使用下面的命令生成 tagSize=0.02, tagSpacing=0.3, 6x6 的 apriltag pdf 文件.  

```bash
sudo apt-get install python-pyx
kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.02 --tspace 0.3
```

常用 aprilgrid 标定板下载请访问: https://github.com/ethz-asl/kalibr/wiki/downloads.  

**2\) 使用 rosbag 工具录制 camera 标定数据**  

为了方便查看, 先打开可视化窗口:   

打开终端, 输入: rviz, 选 Global Options | Fixed Frame | camera_link, 点击左侧导航栏下侧的 Add 按钮, By topic | camera | color | image_raw | Image.  

然后启动相机节点:  

```bash
roslaunch realsense2_camera rs_camera.launch 
```

将图像频率降低为 4Hz, 这里可以用 throttle 方法, 不会出错, 并发布新的 topic, 不会修改原 topic: 

```bash
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /color
```

利用 throttle 工具降低录制的 RGB 图像频率, 降至 4Hz, 新发布的 topic 名字是 /color

```bash
rosbag record -O camd435i /color
```

其中 -O 表示可以保存成 camd435i.bag (设置 bag 名称),  /color 为录制的话题名称. Ctrl-C 结束录制, bag 文件会自动保存到录制时命令行所在的目录, 我这里是 ~/data/realsense/calibration) 

打开可视化窗口, 固定相机, 移动标定板 (效果应稍微好一些). 大部分时间应保证目标 (标定板) 占视野一半以上, 尽可能多角度和多位置 (上下左右等) 甚至到摄像头捕捉图像的边缘, 这样移动目标 1min 左右即可.  
 
**3\) 使用 Kalibr 标定单个 camera**  

Note: 需要先启动 kalibr_workspace 的 setup.bash 脚本.  

```bash
kalibr_calibrate_cameras --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml --bag ~/data/realsense/calibration/camd435i.bag --bag-from-to 5 50  --models pinhole-radtan --topics /color --show-extraction
```

其中
- --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml 指定定位 apriltag yaml 参数文件路径;  
- --bag ~/data/realsense/calibration/camd435i.bag 指定录制好的 bag 文件所在路径;  
- --bag-from-to 5 50 表示用 5~50s (根据录制数据的质量决定时间的取舍) 之间的数据进行计算;  
- --models pinhole-radtan 为相机模型;   
- --topics /color,  bag 录制的话题为 topics. 

`--show-extraction` 用于解决卡在角点上的情况, 具体参考: https://github.com/etHz-asl/kalibr/issues/164.  

根据标定结果修改 camd435i.yaml 文件, 其文件内容如下. 保存后会在后面使用. 

```yaml
cam0:
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.14636059487716624, -0.32114732321540473, -0.002691777523318871,
    0.005917920127500349]
  distortion_model: radtan
  intrinsics: [601.6198872736225, 603.9854373056774, 332.822350543945, 229.09221740717032]
  resolution: [640, 480]
  rostopic: /color
```

此外, 还可以对多个相机进行标定, 如两个相机:   

```bash
kalibr_calibrate_cameras --target ~/data/realsense/calibration/checkerboard.yaml --bag ~/data/realsense/calibration/bag/fisheye_2019-07-15-21-52-12.bag --models pinhole-radtan pinhole-radtan --topics /image_raw1_th /image_raw2_th
```

**错误集合**:  

错误 1: No module named ceres  
解决: 参考 https://github.com/ceres-solver/ceres-solver, 安装 ceres 求解器.  

错误 2: ImportError: No module named igraph  
解决: sudo apt install python-igraph.  

错误 3: error: No module named Image  
解决: 将 ` import Image` 改为 `from PIL import Image`.  

## 五. 使用 Kalibr 标定 Camera-IMU  

Camera-IMU 联合标定的目的: 为了得到 IMU 和 Camera 坐标系的相对位姿矩阵 T 和相对时间延时 t_shift(t_imu = t_cam + t_shift).  

### 5.1 需要的文件  

**1\)** april_6x6_20x20cm.yaml: 标定目标板的参数;  
**2\)** IMU.yaml: 包含 IMU 的噪声密度、随机游走;
**3\)** camd435i.yaml: 包含相机的内参、畸变参数的文件, 如果是双目的话, 还包含两个相机的位置转换矩阵;  
**4\)** .bag: 包含有图片信息和 IMU 数据的 ROS 包.

我们之前已经获取了 1), 2) 和 3) 三个文件, 只差 bag 包了. 将相关文件准备好, 放到 ~/data/realsense/calibration 目录下.  

### 5.2 录制数据包  
 
官方推荐 camera 20 Hz, imu 200 Hz. 利用 throttle 工具降低录制的 topic 的频率后重新发布新的 topic:   

```bash
rosrun topic_tools throttle messages /camera/color/image_raw 20.0 /color
rosrun topic_tools throttle messages /camera/imu 200.0 /imu
```

RGB 图像频率降至 20 Hz, 新发布的 topic 名字是 /color; 
IMU 频率降低为 200 Hz, 新发布的 topic 名字是 /imu.  

**运动原则:** 固定标定板目标, 确保摄像头能够提取特征前提下充分调整 d435i 的姿势和位置. 先面对标定目标, 然后俯仰、偏航和横滚三个角度分别面向目标运动, 然后是前后左右和上下运动, 充分运动起来, 推荐时间在 2 min 以上. 录制数据包. 准确的标定动作可参照官网的视频(需要翻墙观看): https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration, 基本原则是需要激活 6 个自由度的运动 (xyz 旋转与平移).  

使用下面的命令录制 imu_camera_calibration.bag 数据 (同时录制 /imu 与 /color 话题):  

```bash
rosbag record -O imu_camera_calibration /color /imu
```

完成后得到第 4 个需要的文件: imu_camera_calibration.bag.  

### 5.3 运行标定  

标定方法基本与 camera 标定相同.  

```bash
kalibr_calibrate_imu_camera --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml --cam ~/data/realsense/calibration/camchain.yaml --imu ~/data/realsense/calibration/imu.yaml --bag ~/data/realsense/calibration/imu_camera_calibration.bag --bag-from-to 10 100 --show-extraction
```

**注意:**  

- --bag-from-to 10 100 选择 10-100s 之间的数据;  
- --show-extraction 展示特征提取情况.

问题: ImportError: No module named scipy.optimize
解决 pip install scipy –user

### 5.4 标定输出结果  

T_ic: (cam0 to imu0):  

```latex
[[ 0.99960066 0.01259205 -0.02529763 -0.0054386 ]
[-0.01271351 0.99990839 -0.00464588 0.01025557]
[ 0.02523681 0.00496565 0.99966917 0.00876445]
[ 0. 0. 0. 1. ]]
```

timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)   

```latex
-0.0451287115686
```

根据 IMU 和 Camera 坐标系的相对位姿矩阵 T 计算重投影误差 (或者像素误差, Pixel Error), 即在 x 和 y 方向上以像素为单位的重投影误差的标准差. 根据优化的准则我们知道重投影误差越小, 就说相机标定的精度越高.   

## 标定结果使用实例   

以 VINS-Mono 为例, 其 VINS-Mono/config/realsense/realsense_color_config.yaml 文件中将会使用这些标定参数.   

**相机标定参数:**  

```yaml
#camera calibration 
model_type: PINHOLE
camera_name: camera
image_width: 640
image_height: 480
distortion_parameters:
   k1: 9.2615504465028850e-02
   k2: -1.8082438825995681e-01
   p1: -6.5484100374765971e-04
   p2: -3.5829351558557421e-04
projection_parameters:
   fx: 6.133822021484375e+02
   fy: 6.132175903320312e+02
   cx: 3.22447509765625e+02
   cy: 2.3254379272460938e+02
```

**IMU - Camera 标定参数:**  

```yaml
# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 0   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                        # 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. Do some rotation movement at beginning.                        
#If you choose 0 or 1, you should write down the following matrix.
#Rotation from camera frame to imu frame, imu^R_cam
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 0.99964621,  0.01105994,  0.02418954,
           -0.01088975,  0.9999151,  -0.00715601, 
           -0.02426663,  0.00689006,  0.99968178]
#Translation from camera frame to imu frame, imu^T_cam
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.07494282, -0.01077138, -0.00641822]
```

**IMU 标定参数:**  

```yaml
#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.2          # accelerometer measurement noise standard deviation. #0.2
gyr_n: 0.05         # gyroscope measurement noise standard deviation.     #0.05
acc_w: 0.02         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 4.0e-5       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.81       # gravity magnitude

...
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.000                           # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
```

## 附录: realsense D435i IMU 标定 (官网方法)  

该方法便于商用, 但仅采用了六个位置静止放置, 标定数据太少, 标定误差会较大, 不利于高精度测量和导航应用, 可能并不适合 SLAM. 

**1\. 源码编译安装 pyrealsense2**

```bash
cd build
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
make -j4
sudo make install
# update your PYTHONPATH environment variable to add the path to the pyrealsense library
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
```

Note: To force compilation with a specific version on a system with both Python 2 and Python 3 installed, add the following flag to CMake command: -DPYTHON_EXECUTABLE=[full path to the exact python executable]

Alternatively, copy the build output (librealsense2.so and pyrealsense2.so) next to your script.
Note: Python 3 module filenames may contain additional information, e.g. pyrealsense2.cpython-35m-arm-linux-gnueabihf.so)

**2\. 开始标定**  

```bash
cd librealsense/tools/rs-imu-calibration
python3 rs-imu-calibration.py
```

然后按照 [RealSense_Depth_D435i_IMU_Calib 标定教程(白皮书)](https://www.intel.com/content/www/us/en/support/articles/000032303/emerging-technologies/intel-realsense-technology.html), 将相机位置摆放为提示的 6 个位置完成后开始标定.  

## 踩坑记录  

1. rosbag play 和 python rosbag read 的时间戳不匹配.  

So I have hunted down the issue a bit and have a better understanding.

When using global timestamps the timestamp returned is the device hardware timestamp synchronized with the host time. In order to do this a linear regression model is created that takes samples. These samples are (x,y) pairs, where x is the device time and y is the host time. A linear regression is performed on the samples to generate a baisc line mode (y=mx+b). This sampling process takes place in:
librealsense/src/global_timestamp_reader.cpp

Line 144 in 2decb32

 double sample_hw_time = _device->get_device_time_ms(); 
Notice the get_device_time_ms(). This function is an api call that seems to actually send a command (using a class called hwmon) to return back the internal clock of the device. This is always successful for my case.

However this call is used to gather data to generate the model. To use the model an input timestamp from the frame of interest is sent. This is done here:
librealsense/src/global_timestamp_reader.cpp

Line 216 in 2decb32

 double frame_time = _device_timestamp_reader->get_frame_timestamp(mode, fo); 

This is where the issue comes in. My accelerator and gyro for my patched kernel seem to be working and give back good HID timestamps. These frame timestamps are very similar to the ones the hwmon api call gives. I have checked this with lots of logging. However my RGB and depth camera seem to have issues and I get the errror:
13/11 15:22:16,663 WARNING [139792792680192] (ds5-timestamp.cpp:64) UVC metadata payloads not available. Please refer to the installation chapter for details.
When this happens a backup reader is used:
librealsense/src/ds5/ds5-timestamp.cpp

Line 67 in 2decb32

 return _backup_timestamp_reader->get_frame_timestamp(mode, fo); 

Turns out this backup reader actually just gives the host time. But its expecting the device time! The host time and device time are very different from eachother. The result is the host frame timestamp is sent to the regressed model (trained on input device timestamps for x) leading to incredibly bad predictions for y, a synchonized global time.
So two things come to my mind. Realsense knows that the video timestamp is unavailable and is using the backup reader (host time). If it knows that it should not allow global time synchronization for that camera. Second I need to figure out how to fix the driver so that UVC metadata is working...


```
 14/01 16:26:38,361 WARNING [140061760808704] (sensor.cpp:1154) HID timestamp not found, switching to Host timestamps.
[ INFO] [1578990398.392431496]: RealSense Node Is Up!
 14/01 16:26:38,444 WARNING [140061735630592] (ds5-timestamp.cpp:64) UVC metadata payloads not available. Please refer to the installation chapter for details.
 14/01 16:26:38,791 WARNING [140061752416000] (ds5-timestamp.cpp:64) UVC metadata payloads not available. Please refer to the installation chapter for details.
```

**The problem should be fixed with firmware 5.12.1.0. Could you please check it out?**

## 参考资料  

[1] realsense d435i标定imu与camera: https://www.okcode.net/article/93623  
[2] Realsense D435I标定: https://blog.csdn.net/weixin_40628128/article/details/95945945  


## 录制 rosbag 使用的命令集合 

```bash
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /color

rosbag record -O camd435i /color

kalibr_calibrate_cameras --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml --bag camd435i.bag --bag-from-to 5 50  --models pinhole-radtan --topics /color --show-extraction
```

大部分时间应保证目标 (标定板) 占视野一半以上, 尽可能多角度和多位置 (上下左右等) 甚至到摄像头捕捉图像的边缘, 这样移动目标 1min 左右即可. 

```bash
rosrun topic_tools throttle messages /camera/color/image_raw 20.0 /color
rosrun topic_tools throttle messages /camera/imu 200.0 /imu

rosbag record -O imu_camera_calibration /color /imu

kalibr_calibrate_imu_camera --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml --cam ~/data/realsense/calibration/camd435i.yaml --imu ~/data/realsense/calibration/imu.yaml --bag ~/data/realsense/calibration/imu_camera_calibration.bag --bag-from-to 5 100 --show-extraction
```
