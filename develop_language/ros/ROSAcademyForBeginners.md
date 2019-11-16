# Ubuntu 1804 运行 ROS-Academy-for-Beginners 工程  

> 源码: git clone -b melodic https://github.com/DroidAITech/ROS-Academy-for-Beginners.git  
> 讲义: https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/  
> 视频教程: https://www.icourse163.org/course/ISCAS-1002580008 


## ROS 简介  

### 1. Catkin_make 

Catkin 沿用了包管理的传统像 find_package() 基础结构, pkg-config. 扩展了 CMake, 例如:  

- 软件包编译后无需安装就可使用;   
- 自动生成 find_package() 代码,  pkg-config 文件;  
- 解决了多个软件包构建顺序.   

一个 Catkin 的软件包（package）必须要包括两个文件：   

- package.xml: 包括了 package 的描述信息 name, description, version, maintainer(s), licenseopt. authors, url's, dependencies, plugins, etc...  
- CMakeLists.txt: 构建 package 所需的 CMake 文件调用 Catkin 的函数/宏, 解析 package.xml 找到其他依赖的 catkin 软件包, 并将本软件包添加到环境变量中.   

catkin_make 命令也有一些可选参数, 例如：

```cmake
usage: catkin_make [-h] [-C DIRECTORY] [--source SOURCE] [--build BUILD]
                   [--use-ninja] [--use-nmake] [--force-cmake] [--no-color]
                   [--pkg PKG [PKG ...]]
                   [--only-pkg-with-deps ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...]]
                   [--cmake-args [CMAKE_ARGS [CMAKE_ARGS ...]]]
                   [--make-args [MAKE_ARGS [MAKE_ARGS ...]]]
                   [--override-build-tool-check]

optional arguments:
  -h, --help            show this help message and exit
  -C DIRECTORY, --directory DIRECTORY
                        The base path of the workspace (default '.')
  --source SOURCE       The path to the source space (default
                        'workspace_base/src')
  --build BUILD         The path to the build space (default
                        'workspace_base/build')
  --use-ninja           Use 'ninja' instead of 'make'
  --use-nmake           Use 'nmake' instead of 'make'
  --force-cmake         Invoke 'cmake' even if it has been executed before
  --no-color            Disables colored output (only for catkin_make and
                        CMake)
  --pkg PKG [PKG ...]   Invoke 'make' on specific packages only
  --only-pkg-with-deps ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...]
                        Whitelist only the specified packages and their
                        dependencies by setting the CATKIN_WHITELIST_PACKAGES
                        variable. This variable is stored in CMakeCache.txt
                        and will persist between CMake calls unless explicitly
                        cleared; e.g. catkin_make
                        -DCATKIN_WHITELIST_PACKAGES="".
  --cmake-args [CMAKE_ARGS [CMAKE_ARGS ...]]
                        Arbitrary arguments which are passed to CMake. It must
                        be passed after other arguments since it collects all
                        following options.
  --make-args [MAKE_ARGS [MAKE_ARGS ...]]
                        Arbitrary arguments which are passes to make. It must
                        be passed after other arguments since it collects all
                        following options. This is only necessary in
                        combination with --cmake-args since else all unknown
                        arguments are passed to make anyway.
  --override-build-tool-check
                        use to override failure due to using different build
                        tools on the same workspace.


-h, --help 帮助信息
-C DIRECTORY, --directory DIRECTORY 工作空间的路径 (默认为 '.')
--source SOURCE src 的路径 (默认为'workspace_base/src')
--build BUILD build 的路径 (默认为'workspace_base/build')
--use-ninja 用 ninja 取代 make
--use-nmake 用 nmake 取代 make
--force-cmake 强制 cmake, 即使已经 cmake 过
--no-color 禁止彩色输出(只对catkin_make和CMake生效)
--pkg PKG [PKG ...] 只对某个PKG进行make
--only-pkg-with-deps ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...] 将指定的package列入白名单CATKIN_WHITELIST_PACKAGES, 之编译白名单里的package。该环境变量存在于CMakeCache.txt。
--cmake-args [CMAKE_ARGS [CMAKE_ARGS ...]]传给CMake的参数
--make-args [MAKE_ARGS [MAKE_ARGS ...]]传给Make的参数
--override-build-tool-check
```

配置文件 *.yaml 一般放在 config/ 或 param/ 子文件夹下.   

### 2. package  

一个 package 可以编译出来多个目标文件（ROS 可执行程序、动态静态库、头文件等等）。  

一个package下常见的文件、路径有：

```bash
├── CMakeLists.txt # package 的编译规则(必须)
├── package.xml    # package 的描述信息(必须), 
                   # manifest.xml 是属于 rosbuild 编译系统的包清单
├── src/           # 源代码文件
├── include/       # C++头文件
├── scripts/       # 可执行脚本
├── msg/           # 自定义消息
├── srv/           # 自定义服务
├── models/        # 3D模型文件
├── urdf/          # urdf文件
├── launch/        # launch文件 
```

**package 的创建**  

创建一个 package 需要在 catkin_ws/src 下, 用到 catkin_create_pkg 命令, 用法是：  

```bash
catkin_create_pkg package <depends>
```

其中 package 是包名, depends 是依赖的包名, 可以依赖多个软件包。  

例如, 新建一个 package 叫做 test_pkg ,依赖 roscpp、rospy、std_msgs (常用依赖)。

```bash
$ catkin_create_pkg test_pkg roscpp rospy std_msgs
```

**package 相关命令**  

|        rospack 命令        |           功能描述          |
|:-------------------------:|:--------------------------:|
|rospack help               | rospack 命令的用法          |
|rospack list               | 列出本机所有的 packages     |
|rospack depends [package]  | 显示某个 package 的依赖包   | 
|rospack find [packages]    | 定位某个 package          |
|rospack profile            | 刷新所有的 package 路径记录 |
|roscd [pkg-name]           | 直接切换到包所在的目录下     |
|rosls [pkg-name]           | 直接列出包中的文件          |
|rosed [pkg-name] [file]    | 编辑指定包中的某个文件       |

rosdep 是用于管理ROS package依赖项的命令行工具，用法如下：    

|        rosdep 命令        |           功能描述          |
|:-------------------------:|:--------------------------:|
|rosdep install [pkg-name]  | 安装 pacakge 的依赖          |
|rosdep check [pkg-name]    | 检查 package 的依赖是否满足   |
|rosdep db                  | 生成和显示依赖数据库          |
|rosdep init                | 初始化 /etc/ros/rosdep 中的源|
|rosdep keys                | 检查 package 的依赖是否满足   |
|rosdep update              | 更新本地的 rosdep 数据库      |

一个较常使用的命令是:  

```bash
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y 
```

用于安装工作空间中 src 路径下所有 package 的依赖项（由 pacakge.xml 文件指定).  



**package 相关命令**  

CMakeLists.txt 的基本语法都还是按照 CMake，而 Catkin 在其中加入了少量的宏，总体的结
构如下：  

```bash
cmake_minimum_required() #CMake的版本号
project() #项目名称
find_package() #找到编译需要的其他CMake/Catkin package
catkin_python_setup() #catkin新加宏，打开catkin的Python Module的支持
add_message_files() #catkin新加宏，添加自定义Message/Service/Action文件
add_service_files()
add_action_files()
generate_message() #catkin新加宏，生成不同语言版本的msg/srv/action接口
catkin_package() #catkin新加宏，生成当前package的cmake配置，供依赖本包的其他软件包调用
add_library() #生成库
add_executable() #生成可执行二进制文件
add_dependencies() #定义目标文件依赖于其他目标文件，确保其他目标已被构建
target_link_libraries() #链接
catkin_add_gtest() #catkin新加宏，生成测试
install() #安装至至本机
```

## 文件系统使用工具

```bashwps
rospack find [packages]
catkin_create_pkg <pkg-name> [deps]
```


## 源码下载  
出包中的文件
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

param_demo 

ros::NodeHandle  
ros::param  
ros::param::get()  
ros::param::set()  
ros::param::has()  
ros::param::del ()  
nh.getParam()  
nh.setParam()  
nh.hasParam()  
nh.param()  

使用 launch 文件来加载参数.  

TransForm - tf  

坐标变换(位置 + 姿态)  
坐标转换的标准, topic, 工具和接口.  
frame - link   

frame 是坐标系  
每一个 frame 都对应机器人上的一个 link. 任意两个 frame 之间必须是联通的.  

geometry_msgs/TransformStamped

```msg
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w

```

tf2_msgs/TFMessage  

其实就是 geometry_msgs/TransformStamped 可变长数组.  

```msg
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
```

tf 工具  

```bash
# 根据当前 tf tree 创建一个 pdf 文件 (订阅 5s)  
rosrun tf view_frames 

# 查看实时的 tf tree
rosrun rqt_tf_tree rqt_tf_tree 

# 查看两个 frame 之间的变换关系
rosrun tf tf_echo [reference_frame] [target_frame]
```

URDF 文件  Ros

Unified Robot Description Format, 描述机器人的结构.  

link 和 joint, joint 就是连接关系.   


## ROS 中常用的 SLAM 包  

Gmapping  
Katro  
Hector  
Cattographer  
AMCL  


Navigation  

Global Planing Dijkstra, A*
Local Planing: DWA

```msg
[nav_msgs/OccupancyGrid]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id     # /map frame
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution  # m/pixel, 0.05, 0.025
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
int8[] data  # 达到这里的时间 cost, width*height
```

ROS 中既有一个 map frame, 也有一个 map topic.   



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

# 2. 修改 ROS-Academy-for-Beginners/slam_sim_demo 下的 package.xml, 
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

