# ROS rbx1

需要说明的是，`rbx1和rbx2`都是基于`ROS-indigo版本`开发的，因此`不支持高版本的ROS系统`。

使用ros系统，需要用到许多数据包，有些时候你需要使用的ROS数据包并没有Debian包的形式，这时你需要从数据源安装它。代码开发人员通常使用的有三种主流的版本控制系统：SVN，Git和Mercurial。下面介绍一下如何使用git来下载你需要的代码资源。 
首先创建并编译好你需要使用的工作空间，然后执行下面的操作： 

```bash
$ cd ~/catkin_ws/src //此处为你自己创建的工作空间 
$ git clone https://github.com/pirobot/rbx1.git //此处为你需要代码的地址 
$ cd rbx1 //根据你下载生成的文件夹来确定 
$ git checkout indigo-devel //根据你安装的ros版本来确定，我使用的是indigo版本 

$ cd ~/catkin_ws 
$ catkin_make 
$ source ~/catkin_ws/devel/setup.bash 
```

另外rbx1会用到以下这些功能包，因此在使用rbx1之前需要安装：

```bash
sudo apt-get install ros-indigo-turtlebot-bringup ros-indigo-turtlebot-create-desktop ros-indigo-openni-*  ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-usb-cam  ros-indigo-laser-* ros-indigo-hokuyo-node  ros-indigo-audio-common gstreamer0.10-pocketsphinx  ros-indigo-pocketsphinx ros-indigo-slam-gmapping  ros-indigo-joystick-drivers Python-rosinstall  ros-indigo-orocos-kdl ros-indigo-python-orocos-kdl  python-setuptools ros-indigo-dynamixel-motor-*  libopencv-dev python-OpenCV ros-indigo-vision-opencv  ros-indigo-depthimage-to-laserscan ros-indigo-arbotix-*  ros-indigo-turtlebot-teleop ros-indigo-move-base  ros-indigo-map-server ros-indigo-fake-localization  ros-indigo-amcl Git subversion mercurial 
```

