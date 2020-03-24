




















## Issues  

1) Realsense D435i IMU and Camera using different timestamps on NVIDIA Jetson TX2 #3837
 Closed	hridaybavle opened this issue on Apr 25 · 1 comment
 Closed
Realsense D435i IMU and Camera using different timestamps on NVIDIA Jetson TX2
#3837
hridaybavle opened this issue on Apr 25 · 1 comment
Comments
@hridaybavle
 
hridaybavle commented on Apr 25
Required Info	
Camera Model	D435i
Firmware Version	05.11.06
Operating System & Version	Ubuntu 16
Kernel Version (Linux Only)	4.4.38-tegra
Platform	NVIDIA Jetson TX2
Language	{C++ }
Segment	{Robot }
Issue Description
I am trying to working on intel-realsense D435i on the nvidia jetson TX2, I have followed the jetson hacks tutorials on installing the librealsense with the patches. https://github.com/jetsonhacks/buildLibrealsense2TX. My librealsense version is 2.21.0

I am able to run the D435i, but I am having problems with the timestamps, when I run the realsense viewer the IMU time says it uses the hardware clock whereas the rgb and depth image clock are not using the hardware clocks. I attach the screen shots below.

Screenshot from 2019-04-24 21-39-29

Screenshot from 2019-04-24 21-42-50

I also tested with the D435 with the jetson tx2 and it seems to be using properly the hardware clock.

I need the camera and the imu using the same hardware clock in order to perform VIO on the jetson. The D435i is working perfectly on a normal x64 ubuntu computer. Any helps would be appreciated.

@hridaybavle
 
Author
hridaybavle commented on Apr 25
Hi all,

I was able to solve this issue. For anyone to whom it might be useful, I explain I was able to achieve it.

I was using the https://github.com/jetsonhacks/buildLibrealsense2TXand the version 0.81 of the repo. When I ran the buildPatchedKernel.sh script, it was executing the patches from librealsene version 2.10.4 which did not include the latest changes for the D435i cameras and imu timestamps.

Changing the version of librealsense in the buildPatchedKernel.sh script from 2.10.4 to 2.21.0 and running the script again fixed the issue.


2) 同步问题  

- **enable_sync**: gathers closest frames of different sensors, infra red, color and depth, to be sent with the same timetag. This happens automatically when such filters as pointcloud are enabled.


## 参考资料  

[1] 如何用 Realsense D435i 运行 VINS-Mono 等 VIO 算法 获取 IMU 同步数据: https://blog.csdn.net/qq_41839222/article/details/86552367   

