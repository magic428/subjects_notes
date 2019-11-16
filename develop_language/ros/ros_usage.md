# ROS 用法

Package:软件包
Manifest：package.xml：软件包的相关信息。

## rostopic - rosmsg

```bash
rostopic list 
rostopic info /topic_name
rostopic echo /topic_name
rostopic pub /topic_name ...

## rosmsg
rosmsg list 
rosmsg show /msg_name
```

## rosservice - rossrv

```bash
rosservice list 
rostoprosserviceic info /service_name
rosservice call /service_name

## rossrv
rossrv list 
rossrv show /srv_name
```

## Parameter Server 参数服务器  

存储各种参数的字典. 静态的参数传递. 可以用命令行, launch 文件和 node(API) 来读写.    

```bash
rosparam list 
rosparam get param_key
rosparam set param_key param_val
rosparam delete_param_key

# 保存到文件, 从文件加载  
rosparam dump filename
rosparam load file_name
```

文件格式为 YAML. launch.xml 文件中和参数服务器相关的标签: `<parma>` 和 `<rosparam>`.  



## 3 如何创建一个ROS包
	功能模块。可完成一个简单的功能。
roscreate-pkg catkin.

The package must contain a catkin compliant package.xml file.
That package.xml file provides meta information about the package.
The package must contain a CMakeLists.txt which uses catkin. If it is a catkin metapackage it must have the relevant boilerplate CMakeLists.txt file.
There can be no more than one package in each folder.
This means no nested packages nor multiple packages sharing the same directory.
rospack depends和rospack depends1的区别
rospack depends会列出所有的包的依赖关系。

package.xml和CMakeList.txt文件中的主要条目和内容。

## roscore 

- master: 节点管理器
- rosout: log 输出
- parameter server: 参数服务器


node  

一个节点就是一个功能进程; 即 pkg 里的可执行文件运行的实例.   

rosrun [pkg-name] [node-name]

rosnode list 
rosnode info [node_name]
rosnode kill [node_name]



roslaunch  

 