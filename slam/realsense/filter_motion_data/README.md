# Realsense D435i IMU 数据的最终端输出

使用 Intel Realsense D435i 获取 IMU 数据, 其中使用 "Complementary Filter" 对陀螺仪的旋转角度进行补偿滤波.  





Small test program of integrating Intel Realsense D435i with PCL and using the IMU to rotate the cloud.

This is a small sample I made for the D435i. I found no other sample where the camera was integrated with PCL (to do point cloud maths) as well as using the IMU to rotate the cloud based on camera rotation. I have done a very simple filter for ACCEL + GYRO using the ACCEL to drift-compensate the GYRO. Note that I do not care about YAW in this sample (as the ACCEL cannot drift compensate that and I did not need it for my project).

The code uses the PCL visualizer to view the cloud as well as I added a second coordinate system for the camera rotation in order to visualize when you turn/rotate the camera. The idea with the sample is to rotate the cloud based on camera rotation in order to keep the "floor" (if indoor) aligned with the point cloud coordinate system...but see it as a test of the IMU-data.

If you want more background material on how I calculate the accelerometer angles please see this text:
https://www.instructables.com/id/Accelerometer-Gyro-Tutorial/

I am using a "complementary filter" instead of (for example) Kalman in this example. 
Try googling this and you will get lots of comparisons/explanations on this. 
One sample link to read could be this: http://www.pieter-jan.com/node/11


```cpp
rs2::pipeline pipe;

rs2::config cfg;
cfg.enable_stream(RS2_STREAM_GYRO);
cfg.enable_stream(RS2_STREAM_ACCEL);
cfg.enable_stream(RS2_STREAM_POSE);

pipe.start(cfg);

while (app) // Application still alive?
{
    rs2::frameset frameset = pipe.wait_for_frames();

    // Find and retrieve IMU and/or tracking data
    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
        rs2_vector accel_sample = accel_frame.get_motion_data();
        //std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
        //...
    }

    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
        rs2_vector gyro_sample = gyro_frame.get_motion_data();
        //std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        //...
    }

    if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
    {
        rs2_pose pose_sample = pose_frame.get_pose_data();
        //std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;
        //...
    }
}
```


## 参考资料  

[1] complementary filter: http://www.pieter-jan.com/node/11  
