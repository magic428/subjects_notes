# Ubuntu 1804 运行 ROS-Academy-for-Beginners 工程  

> 源码: git clone -b melodic https://github.com/DroidAITech/ROS-Academy-for-Beginners.git  
> 讲义: https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/  
> 视频教程: https://www.icourse163.org/course/ISCAS-1002580008 

## 源码下载  

ROS-Academy-for-Beginners 的默认仓库是为 ubuntu1604 ros-kinect 开发的, 因此对于 ubuntu 1804, 需要切换到 melodic 分支:  

```bash
git clone -b melodic https://github.com/DroidAITech/ROS-Academy-for-Beginners.git  
```

rosrun image_view image_view image:=/camera/rgb/image_raw

rosrun robot_sim_demo robot_keyboard_teleop.py

rostopic pub /topic_name ...
rostopic list
rostopic info /topic_name
rosmsg list 
rosmsg info /msg_name


rosservice 删除灯光 

Action:  

geometry_msgs/PoseStamped target_pos
geometry_msgs/PoseStamped base_pos

rviz

可视化相机  
机器人模型  
雷达数据  
点云数据  

rqt  

rqt_graph: 显示通信架构  
rqt_plot: 绘制曲线   
rqt_console: 显示日志  

rosbag  

记录和回放数据流  

rosbag record <topic-names>
rosbag record -a
rosbag play <bag-files>

## Issues:  

1) ubuntu 1804 ROS-melodic 平台下无法安装 ros-melodic-hector-mapping  

```bash
executing command [sudo -H apt-get install -y ros-melodic-hector-mapping]
Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package ros-melodic-hector-mapping
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y ros-melodic-hector-mapping] failed
```

这个 # https://github.com/DroidAITech/ROS-Academy-for-Beginners/issues/13
这个需要改两处:  

```bash
# 1. 下载 melodic 未发布的两个包到 catkin_ws/src 下:
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git

# 2. 修改 ROS-Academy-for-Beginners/slam_sim_demo 下的 package.xml，
# 增加：
<build_depend>gmapping</build_depend>
<build_depend>hector_mapping</build_depend>

<run_depend>gmapping</run_depend>
<run_depend>hector_mapping</run_depend>

# 再次安装依赖:
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```

则成功安装:  

#All required rosdeps installed successfully

