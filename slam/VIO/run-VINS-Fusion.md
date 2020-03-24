# 使用 Realsense D435i 运行 VINS-Mono  



```bash
rosbag record -O imu_camera_test /camera/color/image_raw /camera/imu
roslaunch realsense2_camera rs_imu_camera.launch
roslaunch vins_estimator realsense_color.launch 
roslaunch vins_estimator vins_rviz.launch
```


kalibr_calibrate_cameras --target ~/data/realsense/calibration/april_6x6_20x20cm.yaml --bag imu_camera_test.bag --bag-from-to 5 50  --models pinhole-radtan --topics /camera/color/image_raw --show-extraction


echo “source /home/q/Projects/catkin_ws_vins-fusion/devel/setup.sh” >> ~/.bashrc
source ~/.bashrc
gedit ~/.bashrc

启动D435i
roslaunch realsense2_camera rs_camera.launch

rosrun vins vins_node /home/q/Projects/catkin_ws_vins-fusion/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

rosrun loop_fusion loop_fusion_node /home/q/Projects/catkin_ws_vins-fusion/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

roslaunch vins vins_rviz.launch

## Run VIN-Fusion with Realsense D435i

--------------------------------

### First shot


```bash
roslaunch vins_estimator realsense_color.launch
```

启动 vins_estimator 后一直打印 `wait for imu ...` 消息:  

```bash
[ WARN] [1556005316.073774275]: waiting for image and imu...
wait for imu ... 
wait for imu ... 
```


Copy the .launch file in package VINS-Fusion to the directory of realsense2_cameara/launch

```bash
cd ~/catkin_ws

cp src/VINS-Fusion/config/realsense_d435i/rs_camera.launch src/realsense/realsense2_camera/launch/rs_imu_camera.launch

cp src/realsense/realsense2_camera/launch/rs_camera.launch src/realsense/realsense2_camera/launch/rs_imu_camera.launch
```

修改 


当unite_imu_method_str 是 "copy"或者“linear_interpolation”就行了！
而这又是读取了launch文件中的unite_imu_method，所以在rs_camera.launch中直接修改：


```bash
magic@G7:~/catkin_ws$ rostopic list
/camera/accel/imu_info
/camera/accel/sample
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_infra1
/camera/extrinsics/depth_to_infra2
/camera/gyro/imu_info
/camera/gyro/sample
/camera/infra1/camera_info
/camera/infra1/image_rect_raw
/camera/infra2/camera_info
/camera/infra2/image_rect_raw
/camera/motion_module/parameter_descriptions
/camera/motion_module/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/auto_exposure_roi/parameter_descriptions
/camera/rgb_camera/auto_exposure_roi/parameter_updates
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/camera/stereo_module/auto_exposure_roi/parameter_descriptions
/camera/stereo_module/auto_exposure_roi/parameter_updates
/camera/stereo_module/parameter_descriptions
/camera/stereo_module/parameter_updates
/diagnostics
/rosout
/rosout_agg
/tf
/tf_static
```

```bash
magic@G7:~/catkin_ws$ rostopic list
/camera/accel/imu_info
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_infra1
/camera/extrinsics/depth_to_infra2
/camera/gyro/imu_info
/camera/imu
/camera/infra1/camera_info
/camera/infra1/image_rect_raw
/camera/infra2/camera_info
/camera/infra2/image_rect_raw
/camera/motion_module/parameter_descriptions
/camera/motion_module/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/auto_exposure_roi/parameter_descriptions
/camera/rgb_camera/auto_exposure_roi/parameter_updates
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/camera/stereo_module/auto_exposure_roi/parameter_descriptions
/camera/stereo_module/auto_exposure_roi/parameter_updates
/camera/stereo_module/parameter_descriptions
/camera/stereo_module/parameter_updates
/diagnostics
/rosout
/rosout_agg
/tf
/tf_static
```

此时可以看到发布的topic变成了"/camera/imu".

### 运行 VINS

这里还需要修改一下配置文件：(在 realsense_color_config.yaml 基础上)  

1、订阅的topic

```yaml
#common parameters
imu_topic: "/camera/imu"
image_topic: "/camera/color/image_raw"
```

2、相机的内参，通过读取 camera_info 得到或者自己标定，采用以下命令可以读取厂家的 camera_info，但与实际可能存在差距。

```bash
rostopic echo /camera/color/camera_info
```

3、IMU 到相机的变换矩阵

# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 0   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                        # 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. Do some rotation movement at beginning.                        
#If you choose 0 or 1, you should write down the following matrix.
1
2
3
4
5
这里IMU和camera之间的外参矩阵建议使用Kalibr工具进行离线标定，也可以改成1或者2让估计器自己标定和优化。

4、IMU参数，这个需要对IMU的噪声和Bias进行标定，同时重力加速度对结果有影响。

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.2          # accelerometer measurement noise standard deviation. #0.2
gyr_n: 0.05         # gyroscope measurement noise standard deviation.     #0.05
acc_w: 0.02         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 4.0e-5       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.81       # gravity magnitude
1
2
3
4
5
6
5、realsense d435i说是已经做好了硬件同步所以不需要在线估计同步时差（但是用kalibr标定出来和在线估计出来都存在大概-0.06的时间差）

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.000                           # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)
1
2
3
6、相机曝光方式应为全局曝光

#rolling shutter parameters
rolling_shutter: 0                      # 0: global shutter camera, 1: rolling shutter camera
rolling_shutter_tr: 0               # unit: s. rolling shutter read out time per frame (from data sheet). 
1
2
3
然后就可以运行了
————————————————
版权声明：本文为CSDN博主「Manii」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_41839222/article/details/86552367



```bash
roslaunch realsense2_camera rs_camera.launch 
roslaunch vins_estimator realsense_color.launch 
roslaunch vins_estimator vins_rviz.launch
```



roslaunch vins vins_rviz.launch
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
(optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
roslaunch realsense2_camera rs_imu_camera.launch
then error occurred

[ERROR] [1555901595.625850921]: Hardware Notification:Right MIPI error,1.5559e+12,Error,Hardware Error
change the USB and Type-C hardware interface to another one because sometimes someone might don't work.

rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
[ INFO] [1555916680.872819604]: init begins
config_file: /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
USE_IMU: 1
IMU_TOPIC: /camera/imu
result path /home/william/Documents/output/vinsFusion//vio.csv
[ WARN] [1555916726.571724172]:  Optimize extrinsic param around initial guess!
camera number 2
[ INFO] [1555916726.571853778]: Unsynchronized sensors, online estimate time offset, initial td: 0
[ INFO] [1555916726.571872262]: ROW: 480 COL: 640 
 exitrinsic cam 0
-0.00575863 -0.00404633    0.999975
  -0.999983 -0.00102246 -0.00576281
 0.00104575   -0.999991 -0.00404037
 0.0203293 0.00793252 0.00285598
 exitrinsic cam 1
-0.00100218 0.000363135    0.999999
  -0.999992 -0.00383034 -0.00100078
 0.00382998   -0.999993 0.000366971
 0.0152858 -0.0524358 0.00869313
set g     0     0 9.805
[ INFO] [1555916726.572012229]: reading paramerter of camera /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/left.yaml
[ INFO] [1555916726.572176400]: reading paramerter of camera /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/right.yaml
MULTIPLE_THREAD is 1
[ WARN] [1555916726.572294355]: waiting for image and imu...
throw img0
throw img1
init first imu pose
averge acc -1602603144.533333 534233088.000000 534354329.600000
init R0 
 0.426462  0.639509  0.639655
        0  0.707187 -0.707027
-0.904506   0.30152  0.301588
feature tracking not enough, please slowly move your device! 
feature tracking not enough, please slowly move your device! 
feature tracking not enough, please slowly move your device! 
feature tracking not enough, please slowly move your device! 
feature tracking not enough, please slowly move your device! 
[ WARN] [1555916728.459035454]: gyroscope bias initial calibration -0.0933654   0.265827 -0.0616124
[ WARN] [1555916728.463995949]: numerical unstable in preintegration
[ WARN] [1555916728.464149835]: numerical unstable in preintegration
[ WARN] [1555916728.470874921]: numerical unstable in preintegration
...
update the latest version Realsense SDK (2.21.0) and ROS package (2.2)
merge the rs_camera.launch in VINS-Fusion and Realsense, put it in the directory of realsense2_cameara/launch

looks like this:

<launch>
  <arg name="serial_no"           default=""/>
  <arg name="json_file_path"      default=""/>
  <arg name="camera"              default="camera"/>
  <arg name="tf_prefix"           default="$(arg camera)"/>

  <arg name="fisheye_width"       default="640"/>
  <arg name="fisheye_height"      default="480"/>
  <arg name="enable_fisheye"      default="true"/>

  <arg name="depth_width"         default="640"/>
  <arg name="depth_height"        default="480"/>
  <arg name="enable_depth"        default="true"/>

  <arg name="infra_width"         default="640"/>
  <arg name="infra_height"        default="480"/>
  <arg name="enable_infra1"       default="true"/>
  <arg name="enable_infra2"       default="true"/>

  <arg name="color_width"         default="640"/>
  <arg name="color_height"        default="480"/>
  <arg name="enable_color"        default="true"/>

  <arg name="fisheye_fps"         default="30"/>
  <arg name="depth_fps"           default="30"/>
  <arg name="infra_fps"           default="30"/>
  <arg name="color_fps"           default="30"/>
  <arg name="gyro_fps"            default="200"/>
  <arg name="accel_fps"           default="250"/> 
  <arg name="enable_gyro"         default="true"/>
  <arg name="enable_accel"        default="true"/>
<!--
  <arg name="enable_imu"          default="true"/>
-->
  
  <arg name="enable_pointcloud"         default="false"/>
  <arg name="pointcloud_texture_stream" default="RS2_STREAM_COLOR"/>
  <arg name="pointcloud_texture_index"  default="0"/>

  <arg name="enable_sync"           default="true"/>
  <arg name="align_depth"           default="true"/>

  <arg name="filters"               default=""/>
  <arg name="clip_distance"         default="-2"/>
  <arg name="linear_accel_cov"      default="0.01"/>
  <arg name="initial_reset"         default="false"/>
  <arg name="unite_imu_method"      default="linear_interpolation"/>
  <arg name="topic_odom_in"         default="odom_in"/>
  <arg name="calib_odom_file"       default=""/>
  <arg name="publish_odom_tf"       default="true"/>
  <arg name="hold_back_imu_for_frames"      default="true"/>
  
  <group ns="$(arg camera)">
    <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
      <arg name="tf_prefix"                value="$(arg tf_prefix)"/>
      <arg name="serial_no"                value="$(arg serial_no)"/>
      <arg name="json_file_path"           value="$(arg json_file_path)"/>

      <arg name="enable_pointcloud"        value="$(arg enable_pointcloud)"/>
      <arg name="pointcloud_texture_stream" value="$(arg pointcloud_texture_stream)"/>
      <arg name="pointcloud_texture_index"  value="$(arg pointcloud_texture_index)"/>
      <arg name="enable_sync"              value="$(arg enable_sync)"/>
      <arg name="align_depth"              value="$(arg align_depth)"/>

      <arg name="fisheye_width"            value="$(arg fisheye_width)"/>
      <arg name="fisheye_height"           value="$(arg fisheye_height)"/>
      <arg name="enable_fisheye"           value="$(arg enable_fisheye)"/>

      <arg name="depth_width"              value="$(arg depth_width)"/>
      <arg name="depth_height"             value="$(arg depth_height)"/>
      <arg name="enable_depth"             value="$(arg enable_depth)"/>

      <arg name="color_width"              value="$(arg color_width)"/>
      <arg name="color_height"             value="$(arg color_height)"/>
      <arg name="enable_color"             value="$(arg enable_color)"/>

      <arg name="infra_width"              value="$(arg infra_width)"/>
      <arg name="infra_height"             value="$(arg infra_height)"/>
      <arg name="enable_infra1"            value="$(arg enable_infra1)"/>
      <arg name="enable_infra2"            value="$(arg enable_infra2)"/>

      <arg name="fisheye_fps"              value="$(arg fisheye_fps)"/>
      <arg name="depth_fps"                value="$(arg depth_fps)"/>
      <arg name="infra_fps"                value="$(arg infra_fps)"/>
      <arg name="color_fps"                value="$(arg color_fps)"/>
      <arg name="gyro_fps"                 value="$(arg gyro_fps)"/>
      <arg name="accel_fps"                value="$(arg accel_fps)"/>
      <arg name="enable_gyro"              value="$(arg enable_gyro)"/>
      <arg name="enable_accel"             value="$(arg enable_accel)"/>
<!--
      <arg name="enable_imu"               value="$(arg enable_imu)"/>
 -->
 
      <arg name="filters"                  value="$(arg filters)"/>
      <arg name="clip_distance"            value="$(arg clip_distance)"/>
      <arg name="linear_accel_cov"         value="$(arg linear_accel_cov)"/>
      <arg name="initial_reset"            value="$(arg initial_reset)"/>
      <arg name="unite_imu_method"         value="$(arg unite_imu_method)"/>
      <arg name="topic_odom_in"            value="$(arg topic_odom_in)"/>
      <arg name="calib_odom_file"          value="$(arg calib_odom_file)"/>
      <arg name="publish_odom_tf"          value="$(arg publish_odom_tf)"/>

     </include>
  </group>
</launch>
then roslaunch realsense2_camera rs_imu_camera.launch

error

unused args [enable_imu] for include of [/home/william/catkin_ws/src/realsense-2.2/realsense2_camera/launch/includes/nodelet.launch.xml]
The traceback for the exception was written to the log file
Delete everything related to enable_imu.

Seems like the same problem. Change the USB and Type-C hardware interface to another one because sometimes someone might don't work.

Error again.

$ rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
[ INFO] [1556005316.069145892]: init begins
config_file: /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
USE_IMU: 1
IMU_TOPIC: /camera/imu
result path /home/william/Documents/output/vinsFusion//vio.csv
[ WARN] [1556005316.073310738]:  Optimize extrinsic param around initial guess!
camera number 2
[ INFO] [1556005316.073429273]: Unsynchronized sensors, online estimate time offset, initial td: 0
[ INFO] [1556005316.073443294]: ROW: 480 COL: 640 
 exitrinsic cam 0
-0.00575863 -0.00404633    0.999975
  -0.999983 -0.00102246 -0.00576281
 0.00104575   -0.999991 -0.00404037
 0.0203293 0.00793252 0.00285598
 exitrinsic cam 1
-0.00100218 0.000363135    0.999999
  -0.999992 -0.00383034 -0.00100078
 0.00382998   -0.999993 0.000366971
 0.0152858 -0.0524358 0.00869313
set g     0     0 9.805
[ INFO] [1556005316.073557518]: reading paramerter of camera /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/left.yaml
[ INFO] [1556005316.073686245]: reading paramerter of camera /home/william/catkin_ws/src/VINS-Fusion/config/realsense_d435i/right.yaml
MULTIPLE_THREAD is 1
[ WARN] [1556005316.073774275]: waiting for image and imu...
wait for imu ... 
wait for imu ... 
...
grep -rn * -ie wait

src/estimator/estimator.cpp:141:        printf("wait for imu\n");
src/estimator/estimator.cpp:172:                    printf("wait for imu ... \n");
src/KITTIGPSTest.cpp:181:           // cv::waitKey(2);
src/KITTIOdomTest.cpp:117:          //cv::waitKey(2);
src/rosNodeTest.cpp:223:    ROS_WARN("waiting for image and imu...");
src/featureTracker/feature_tracker.cpp:395:    // cv::waitKey(0);
src/featureTracker/feature_tracker.cpp:508:    // cv::waitKey(2);
gedit src/estimator/estimator.cpp

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}
in this function, t1 <= accBuf.back().first. Search the Internet, nothing similar like this.

3rd shot
Refer to blog 如何用Realsense D435i运行VINS-Mono等VIO算法 获取IMU同步数据, this guy had run VINS-MONO with D435i.

Modify realsense_color_config.yaml, then it looks like:

%YAML:1.0

#common parameters
imu_topic: "/camera/imu"
image_topic: "/camera/color/image_raw"
output_path: "/home/william/Documents/output/vinsMono/"

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
   fx: 6.0970550296798035e+02
   fy: 6.0909579671294716e+02
   cx: 3.1916667152289227e+02
   cy: 2.3558360480225772e+02

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

#feature traker paprameters
max_cnt: 150            # max feature number in feature tracking
min_dist: 25            # min distance between two features 
freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
equalize: 0             # if image is too dark or light, trun on equalize to find enough features
fisheye: 0              # if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points

#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 0.2          # accelerometer measurement noise standard deviation. #0.2
gyr_n: 0.05         # gyroscope measurement noise standard deviation.     #0.05
acc_w: 0.002         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 4.0e-5       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.80       # gravity magnitude

#loop closure parameters
loop_closure: 1                    # start loop closure
fast_relocalization: 1             # useful in real-time and large project
load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
pose_graph_save_path: "/home/tony-ws1/output/pose_graph/" # save and load path

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.000                           # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

#rolling shutter parameters
rolling_shutter: 1                      # 0: global shutter camera, 1: rolling shutter camera
rolling_shutter_tr: 0.033               # unit: s. rolling shutter read out time per frame (from data sheet). 

#visualization parameters
save_image: 1                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
visualize_imu_forward: 0        # output imu forward propogation to achieve low latency and high frequence results
visualize_camera_size: 0.4      # size of camera marker in RVIZ
Error [ WARN] [1556065325.371664369]: imu message in disorder!

According to issues getting ros warn imu message in disorder #204 and Doubts about camera-imu temporal calibration #270 in GitHub, definitely it's the problem of unsynchroned image and imu datas.

Reduce fps of imu stream < 100 hz



## ISSUEs

```bash
[ WARN] [1579154696.285136816]: imu message in disorder!

[ INFO] [1579155285.233189641]: IMU excitation not enouth!
[ INFO] [1579155285.233293152]: Not enough features or parallax; Move device around
```