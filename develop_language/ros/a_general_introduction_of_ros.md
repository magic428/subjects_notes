# ROS 操作系统浅析


## 一、ROS基本概念介绍

### turtlesim小例子

启动turtlesim 在三个不同的终端中，分别执行以下三个指令：

```bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key
```
分三个终端的目的是让三个指令同时进行。

## 功能包/软件包（Packages）

在ROS中，所有软件都被组织为软件包的形式，称为ROS软件包或功能包，有时也简称为包。ROS软件包是一组用于实现特定功能的相关文件的集合，包括可执行文件和其他支持文件。比如说，我们前面使用的两个可执行文件`turtlesim_node`和`turtle_teleop_key`都属于`turtlesim`软件包。

毫不夸张的说，所有的ROS软件都是一个软件包或其他软件包的一部分。

### 1. 软件包的4个重要命令

```bash
$ rospack list    //查看软件包列表和定位软件包
$ rospack find package-name   //查找一个软件包的目录
$ rosls package-name   //要查看软件包目录下的文件
$ roscd package-name   //切换到软件包目录
```

每个程序包由一个清单文件(package.xml文件)定义。该文件定义关于包的一些细节，包括其名称、版本、维护者和依赖关系。包含package.xml文件的目录被称为软件包目录（其实，这也是ROS软件包的定义：任何ROS能找到且包含package.xml文件的目录就是软件包目录。）。这个目录存储所在软件包的大部分文件。

> 1. 对于使用apt-get安装的功能包，其所在根目录为`/opt/ros/indigo`。可执行文件存储在这个根目录下的`lib子目录`里。同样，在自动生成的头文件存储在`include子目录`下。
> 2. `catkin`支持源码外部编译ROS的功能包。
> 3. ROS命令支持这种`tab命令补全`。

## 节点管理器(The Master)

ROS的一个基本目标是使机器人专家设计的很多称为`节点（node）`的几乎相对独立的小程序能够同时运行。为此，这些节点必须能够彼此通信。ROS中实现通信的关键部分就是ROS节点管理器。要启动节点管理器，使用如下命令：

```bash
$ roscore
```
> 大多数ROS节点在启动时连接到节点管理器上，如果运行中连接中断，则不会尝试重新连接。因此，如果`roscore`被终止，当前运行的其他节点将无法建立新的连接，即使稍后重启`roscore`也无济于事。

**注意**：
这里的`roscore命令`用来`显式启动`ROS的节点管理器。在第六章中，我们将学习一个称为`roslaunch`的工具，其目的是一次性启动多个节点。这是一个自适应工具，如果启动多节点时没有节点管理器运行，它会自动启动节点管理器；如果已经有一个节点管理器在运行，则会使用已有的。


## 节点
一旦启动`roscore`后，便可以运行ROS程序了。ROS程序的运行实例被称为`节点（node）`。

### 1. 启动节点 —— `rosrun`

启动节点（也称运行ROS程序）的基本命令是`rosrun`：

```bash
$ rosrun package-name executable-name
```
> rosrun没有什么“神奇”的：它只不过是一个`shell脚本`，能够理解ROS的文件组织结构，知道到哪里能找到与给定包名称对应的可执行文件。一旦它找到你要的程序，`rosrun`就会正常启动节点。例如，你可以像执行任何其他程序一样直接执行`turtlesim_node`：

```bash
/opt/ros/indigo/lib/turtlesim/turtlesim_node
```
> 这里还要强调一点，通过节点管理器注册成为ROS节点发生在程序的内部，而不是通过rosrun命令。

### 节点的几个重要命令

```bash
$ rosnode list                           //查看节点列表
$ rosnode info node-name    //要获得特定节点的信息
$ rosnode kill node-name      //终止节点
```
`/rosout节点`是一个特殊的节点，通过`roscore`自动启动。其作用有点类似于控制台程序中使用的`标准输出（即std：：cout）`。`/rosout`前面的反斜杠`“/”`表明该节点名称属于`全局命名空间`。ROS有一个丰富的命名节点和其他对象的体系（将在第5章中详细讨论），该体系使用命名空间组织各种资源。

> 事实上，可以使用`rosrun命令`显式设置节点的名称，语法如下：
`rosrun package-name executable-name __name:=node-name`
这种方法将使用node-name参数给出的名称覆盖节点的默认名。因为ROS中要求每个节点有不同的名称，因此该方法很重要（我们会在第2.8节用_ _name构建一个稍大的演示系统。）。一般来讲，即使你经常用`_ _name`指定节点名称，你仍可能要使用一个启动文件（见第6章），而不是单独启动每个节点。


##话题和消息(topic和message)







## 第二章 ROS中的基本对象

- 1 小海龟示例
  启动turtlesim 在三个不同的终端中，分别执行以下三个指令： 

```bash
$ roscore 
$ rosrun turtlesim turtlesim_node 
$ rosrun turtlesim turtle_teleop_key 
```
分三个终端的目的是让三个指令同时进行。在第三个终端中使用键盘的上下按键进行teleoperation（遥操作）。


- 2 `ros`功能包
 在ROS中，所有软件都被组织为软件包的形式，称为ROS软件包或功能包，有时也简称为包。

    (1) 要找到一个软件包的目录，使用`rospack find`命令： 
```bash
$ rospack find package-name
```

    (2)` roscd和rosls`的用法 
```bash
$ roscd turtlesim /images/
$ eog box−turtle . png
```
命令`eog是“Eye of Gnome”`图像查看器。

- 3 执行`ROS`程序

    (1) `ROS节点管理器`
`ROS`中实现通信的关键部分就是ROS节点管理器。要启动节点管理器，使用如下命令： 

```bash
$ roscore
```
`roscore`命令用来显式启动ROS的节点管理器。

**注意**：节点管理器应该在使用`ROS`的全部时间内持续运行。一个合理的工作流程是在一个终端启动`roscore`，然后打开其他终端运行其他程序。
    
    (2) 启动`ROS节点`(node)
 ros的运行实例被称为节点(node)。启动节点（也称运行ROS程序）的基本命令是`rosrun`：

```bash
$ rosrun package-name executable-name
```
这里还要强调一点，通过节点管理器注册成为ROS节点发生在程序的内部，而不是通过`rosrun`命令。

    (3) 查看节点列表
ROS提供了一些方法来获取任意时间运行节点的信息。要获得运行节点列表，使用如下命令：

```bash
$ rosnode list
```

8 比较rosnode list的输出rosrun命令中可执行文件的名称，你会发现节点名并不一定与对应可执行文件名称相同。 
rosrun
rosnode list
roscore 
rosrun turtlesim turtlesim_node 
rosrun turtlesim turtle_teleop_key
/rosout
/turtlesim
/teleop_turtle

9 显示设置节点名称
可以使用rosrun命令显式设置节点的名称，语法如下： 
rosrun package-name executable-name __name:=node-name 
这种方法将使用node-name参数给出的名称覆盖节点的默认名。因为ROS中要求每个节点有不同的名称，因此该方法很重要（我们会在第2.8节用_ _name构建一个稍大的演示系统。）

9 查看节点信息
要获得特定节点的信息，使用如下命令： 
rosnode info node-name
10 终止节点 
要终止节点，使用如下命令： 
rosnode kill node-name
注意：还可以用Ctrl-C命令终止节点。但使用这种方法时可能不会在节点管理器中注销该节点，因此会导致已终止的节点仍然在rosnode列表中。这虽然没有什么坏处，但可能会让用户对当前系统的行为感到困扰。此时可以使用下面的命令将节点从列表中删除： 
rosnode cleanup


11 消息传递机制
ROS节点之间进行通信所利用的最重要的机制就是消息传递。在ROS中，消息有组织地存放在话题里。消息传递的理念是：当一个节点想要分享信息时，它就会发布(publish)消息到对应的一个或者多个话题；当一个节点想要接收信息时，它就会订阅(subscribe)它所需要的一个或者多个话题。ROS节点管理器负责确保发布节点和订阅节点能找到对方；而且消息是直接地从发布节点传递到订阅节点，中间并不经过节点管理器转交。

12 查看ROS通信话题和节点
在查看一个新的ROS系统时，使用rqt_graph工具，尤其是按照上述进行设置，能帮助你发现自己的程序可以用哪些话题来和现有节点进行通信。


13话题列表 
为了获取当前活跃的话题，使用如下命令： 
rostopic list

14打印消息内容
rostopic echo topic-name 
15测量发布频率
有两个命令可以用来测量消息发布的频率以及这些消息所占用的带宽： 
rostopic hz topic-name 
rostopic bw topic-name 
这些命令订阅指定的话题，并且输出一些统计量，其中第一条命令输出每秒发布的消息数量，第二条命令输出每秒发布消息所占的字节量。
注意：即使你一点都不关心这个特定的频率，但是这些命令对调试很有帮助，因为它们提供了一种简单的方法来验证这些消息确实有规律地在向这些特定的话题发布。

16 查看话题
利用rostopic info命令，你可以获取更多关于话题的信息： 
rostopic info topic-name

理解消息的类型很重要，因为它决定了消息的内容。也就是说，一个话题的消息类型能告诉你该话题中每个消息携带了哪些信息，以及这些信息是如何组织的。

17 查看消息类型 
要想查看某种消息类型的详情，使用类似下面的命令： 
rosmsg show message-type-nam

18 用命令行发布消息 
大多数时候，发布消息的工作是由特定的程序完成的*。但是，你会发现有时候手动发布消息是很实用的。要实现该功能，利用rostopic命令行工具： 
rostopic pub topic-name –r rate-in-hz message-type message-content 
这条命令重复地按照指定的频率给指定的话题发布指定的消息。
linux@ubuntu:~$ rostopic pub /turtle1/cmd_vel -r 1 geometry_msgs/Twist '[2,0,0]' '[0,0,0]'
这些数值按照rosmsg show命令显示的变量顺序赋给了消息中的域变量。在此例中，前面三个数字表示期望的位移线速度，后面三个数字表示期望的角速度。用单引号（’…’）和中括号（[…]）组织这些数值赋给它们对应的两个顶层复合域变量。
注意：上述语法有一个明显的缺点，那就是你必须记住消息类型里所有的域以及这些域出现的顺序。另一种替代方式是以YAML字典的形式给出一个参数，该参数将所有域进行赋值（注：YAML是"YAML Ain't a Markup Language"（YAML不是一种置标语言）的递归缩写）。
下面这条命令（实际上包含了换行符）和上述命令是等价的，只不过它显式地指明了结构域中变量名和对应值的映射关系： 
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist “linear: 
x:2.0 
y:0.0 
z:0.0 
Angular: 
x:0.0 
y:0.0
z:0.0
” 
在bash和YAML之间交互还有很多小技巧，在线文档有一整页来介绍在命令行中如何使用YAML。利用Tab键补齐输入是保证语法正确的最简单方法。在输入消息类型之后按下Tab键，将会插入一个完整的YAML字典，其中包含给定消息类型中所有的结构域变量。Tab键产生的消息将会使用默认值（zero、false、empty string等等），但是你可以根据需要修改消息内容。


19 命令帮助文档
rospack [name] –h 

20 消息类型的命名 
和ROS里其他的程序一样，每条消息类型都属于一个特定的包。消息类型名总会包含一个斜杠，斜杠前面的名字是包含它的包： 
package-name/type-name
这种分解消息类型名的方法有如下几个目的： 
最直接地，把包的名字包含在消息类型名里能避免命名冲突。
正如我们将在第3章看到的那样，当我们编写ROS程序的时候，如果也用到了其他包的消息类型，那么我们需要声明对它们的依赖关系。 
最后一点，包名和其含有的消息类型放在一起将有助于猜测它的含义。 

21 ROS主节点就是roscore

22 一个更加实用性的例子

话题通信的多对多机制
在本例中，远程操作节点C发布的每条消息都会传送给A和B两个仿真节点。同样的，D节点发布的消息也会传送给A和B。当这些消息到达仿真节点时，海龟将会相应地移动，而不管这条消息是哪个节点发布的。此处要强调的是，基于话题和消息的通信机制是多对多的，即多个发布者和多个订阅者可以共享同一个话题。

节点之间的松耦合关系
出现上述现象的根本原因在于，我们的turtlesim节点之间——更一般的，对于绝大多数设计精巧的ROS节点——是松耦合的。每个节点都不需要显式知道其他节点的存在与否；它们的唯一交互方式是间接地发生在基于话题和消息的通信层。这种节点之间的独立性，以及其支持的任务可分解特性（即复杂任务分解成可重用的小模块），是ROS最关键的设计特性之一。

 “生产”消息的程序（例如turtle_teleop_key）只管发布该消息，而不用关心该消息是如何被“消费”的。 
 “消费”消息的程序（例如turtlesim_node）只管订阅该话题或者它所需要消息的所有话题，而不用关心这些消息数据是如何“生产”的。 
此外，ROS为更加直接的一对一通信提供了一种称为服务（services）的机制。第二种通信机制更为少见，但是也有其应用价值。

第三章   编写ROS程序
1 hello ROS程序
2 pubvel程序，给turtle随机发送速度指令。







