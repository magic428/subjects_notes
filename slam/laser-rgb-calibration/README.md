# 雷达和 RGB 相机的联合标定   


## ros package for lidar camera calibration
  
https://github.com/ankitdhall/lidar_camera_calibration

The package is used to calibrate a Velodyne LiDAR with a camera (works for both monocular and stereo). Specficially, Point Gray Blackfly and ZED camera have been successfully calibrated against Velodyne VLP-16 using lidar_camera_calibration.







## 数据集下载地址  

[] 雷达-相机数据集下载地址 1: https://wiki.nps.edu/pages/viewpage.action?pageId=925958215  
[] 雷达-相机数据集下载地址 2: https://data.tier4.jp/index/  
[] https://waymo.com/open/download/




## 参考资料  

[1] 使用Autoware实践激光雷达与摄像机组合标定: https://blog.csdn.net/AdamShan/article/details/81670732#commentBox  

[2] 激光雷达和相机的联合标定（Camera-LiDAR Calibration）之Autoware: https://blog.csdn.net/learning_tortosie/article/details/82347694   

[3] 激光雷达和相机的联合标定（Camera-LiDAR Calibration）之apollo: https://blog.csdn.net/learning_tortosie/article/details/82351553  

你好，要从自己的设备条件出发，选择合适的工具。 apollo需要激光雷达，广角相机，里程计和惯导，而且需要最准确的初始外参值。 but_calibration_camera_velodyne使用32线激光雷达效果较好，16线效果不好，需要修改源码。 相比前两者，Autoware对硬件要求较低，不要求相机是广角等，同时也支持16线激光雷达。 如果非要说哪个好，个人比较倾向于Autoware。

[4] Apollo 2.0 传感器标定方法使用指南: https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide_cn.md  

[5] SLAM之相机标定: https://blog.csdn.net/learning_tortosie/article/details/79901255  