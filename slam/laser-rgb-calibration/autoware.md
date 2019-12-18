# 使用 Autoware 软件进行标定  

> 官方文档: https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/home.  

## 源码编译安装 - How to build  

> 参考: https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Source-Build  

```bash
$ mkdir -p autoware.ai/src
$ cd autoware.ai

# Download the workspace configuration for Autoware.AI.
For the 1.12.0 release:
$ wget -O autoware.ai.repos "https://gitlab.com/autowarefoundation/autoware.ai/autoware/raw/1.12.0/autoware.ai.repos?inline=false"
# For newer releases, replace 1.12.0 with the version you want to install.
# For the master version (bleeding edge):
$ wget -O autoware.ai.repos "https://gitlab.com/autowarefoundation/autoware.ai/autoware/raw/master/autoware.ai.repos?inline=false"

# Download Autoware.AI into the workspace.
$ vcs import src < autoware.ai.repos

# Install dependencies using rosdep.
$ rosdep update
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO


# Compile the workspace
# With CUDA support
$ AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14
# Without CUDA Support
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**编译过程中可能会遇到的问题:**  

1) Add OpenCV libraries to the linker (autowarefoundation#2090)  

显式指定 OpenCV 3.2.0 版本.  

Signed-off-by: Joshua Whitley <jwhitley@autonomoustuff.com>
@amc-nu
@JWhitleyAStuff
amc-nu authored and JWhitleyAStuff committed on Mar 6
1 parent c2c505f commit 392d16a11d776f14d3cd83c8fd941507017a01bc
Showing  with 2 additions and 0 deletions.


 1  ros/src/sensing/fusion/packages/calibration_publisher/CMakeLists.txt 
@@ -43,6 +43,7 @@ add_dependencies(calibration_publisher

```cpp
set(OpenCV_DIR "/usr/local/opencv3.2/share/OpenCV") 
find_package(OpenCV 3.2.0  REQUIRED)
target_link_libraries(calibration_publisher
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

install(TARGETS calibration_publisher
```
 1  ros/src/sensing/fusion/packages/calibration_publisher/package.xml 
@@ -19,5 +19,6 @@
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>tf</depend>
  <depend>libopencv-dev</depend>

</package> 


lidar_camera_calibration/autoware.ai/src/autoware/utilities/data_preprocessor/CMakeLists.txt

lidar_camera_calibration/autoware.ai/src/autoware/utilities/data_preprocessor/package.xml


2) 'canlib' is not installed. 'can_listener' is not built.


3) 将  OpenCV_LIBRARIES 替换为 OpenCV_LIBS


AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 

/home/magic/work/gitwork/lidar_camera_calibration/autoware.ai/src/autoware/core_perception/pcl_omp_registration/include/pcl_omp_registration/registration.h

添加:  


#include <boost/function.hpp>  





autoware_camera_lidar_calibrator

) boost 库冲突  

这是因为 ROS 安装的时候会安装一个 boost 库版本, 我自己又使用源码编译的方式安装了一个版本. 因此造成了版本冲突.   

```bash
CMake Warning at /opt/ros/melodic/share/catkin/cmake/test/gtest.cmake:171 (add_executable):
  Cannot generate a safe runtime search path for target
  test-costmap_generator because files in some directories may conflict with
  libraries in implicit directories:

  runtime library [libboost_iostreams.so.1.65.1] in /usr/lib/x86_64-linux-gnu may be hidden by files in:
    /usr/local/lib

```

2

If you're dealing with find_library
find_library(LIBRARY_NAME PATHS "/usr/lib/x86_64-linux-gnu" NO_DEFAULT_PATH) where

PATHS stands for the exact path to the libs
NO_DEFAULT_PATH means, that cmake will not search anywhere else
check the values of lib and include paths with message(status, ${LIBRARY_NAME})

If you're dealing with find_package:
It's a bit more complicated than the previous example, but it's essentially the same.

For each package you have to run find_package for:

Create file with name Find<Packagename>.cmake, e. g. if you're looking for cppunit, you'll have to create FindCPPUNIT.cmake.

In that file, you'll have to run find_path on include files and find_library on lib files, like in "If you're dealing with find_library".

find_path(CPPUNIT_INCLUDE_DIR PATHS "/usr/include/x86_64-linux-gnu" NO_DEFAULT_PATH)       
find_library(CPPUNIT_LIBRARY PATHS "/usr/lib/x86_64-linux-gnu" NO_DEFAULT_PATH)
And then you have to add the path to the file to CMAKE_MODULE_PATH.



CUDA 版本会报以下错误:  

```bash
/usr/include/c++/7/bits/uses_allocator.h(138): error: "__is_uses_allocator_constructible_v" is not a function or static data member

/usr/include/c++/7/bits/uses_allocator.h(138): error: "constexpr" is not valid here

/usr/include/c++/7/bits/uses_allocator.h(152): error: "__is_nothrow_uses_allocator_constructible_v" is not a function or static data member

/usr/include/c++/7/bits/uses_allocator.h(151): error: "constexpr" is not valid here
```

最终 CUDA 版本编译失败, 只好放弃...  



## Run  

> 参考: https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/ROSBAG-Demo

1) runtime_manager demo  

```bash
# 创建运行需要的环境 
source install/setup.bash
# 运行 runtime_manager 
roslaunch runtime_manager runtime_manager.launch
```

2) autoware_camera_lidar_calibrator  

```bash
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/s1nh/20190215_1853_autoware_camera_calibration.yaml image_src:=/mynteye/left/image_raw
```

后边的两个参数是根据你要标定的相机和雷达来设置的.  

3) Gazebo-Simulation in Autoware  

参考: https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Gazebo-Simulation-Start  

**由此引发的问题:**   

cmake 中关于 find_library() 函数的使用细节请参考: https://cmake.org/cmake/help/v2.8.12/cmake.html#command%3afind_library  

## 参考资料  


多目相机、Velodyne标定那些破事: ttp://s1nh.org/post/calib-velodyne-camera/  

Autoware Packages & Nodes - Sensing: https://autoware.readthedocs.io/en/feature-documentation_rtd/DevelopersGuide/PackagesNodes.html#sensing  

Autoware github: https://github.com/autowarefoundation/autoware/tree/develop  

Autoware 显示用户界面细节: https://blog.csdn.net/jianxuezixuan/article/details/86015224  

Autoware 编译与安装教程（附带实例）入门自动驾驶
: http://t.manaai.cn/topic/109/autoware-%E7%BC%96%E8%AF%91%E4%B8%8E%E5%AE%89%E8%A3%85%E6%95%99%E7%A8%8B-%E9%99%84%E5%B8%A6%E5%AE%9E%E4%BE%8B-%E5%85%A5%E9%97%A8%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6  

激光雷达和相机的联合标定（Camera-LiDAR Calibration）之Autoware: https://blog.csdn.net/learning_tortosie/article/details/82347694  

Camera-Lidar Calibration with Autoware: https://zhuanlan.zhihu.com/p/65226371   

视觉激光雷达信息融合与联合标定: https://zhuanlan.zhihu.com/p/55825255  

【机器视觉】张氏法相机标定: https://zhuanlan.zhihu.com/p/24651968  

多目相机、Velodyne标定那些破事: ttp://s1nh.org/post/calib-velodyne-camera/   

Autoware-Manuals: https://github.com/CPFL/Autoware-Manuals/tree/master/en  

Autoware_UsersManual_v1.1: https://github.com/CPFL/Autoware-Manuals/blob/master/en/Autoware_UsersManual_v1.1.md#demo-data  

在Ubuntu 18.04 LTS安装ROS Melodic版机器人操作系统（2019年10月更新MoveIt! 1.0 ROS 2.0 Dashing）: https://blog.csdn.net/ZhangRelay/article/details/80241758  

lidar_camera_calibration项目——激光雷达和相机联合标定: https://tumihua.cn/2019/02/20/698adb0e68a9eaa19a52d52c69f34bea.html  

