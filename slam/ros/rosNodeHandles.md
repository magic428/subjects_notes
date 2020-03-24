# NodeHandles - TODO 

- Automatic Startup and Shutdown  
- Namespaces  

ros::NodeHandle 类通常有两种用途.   

- 在 roscpp 程序内提供一种内部节点的 RAII 风格的启动和关闭;  
- 为 namespace 的解析提供了一层额外层, 使得子 namespace 的书写更为简洁.  

## 1. Automatic Startup and Shutdown

ros::NodeHandle 通过管理一个内部引用计数来启动和关闭一个节点.  

```cpp
ros::NodeHandle nh;
```

当创建这个 ros::NodeHandle 对象时, 如果内部节点没有启动, 那么 ros::NodeHandle 会在创建对象时直接将其启动. 当所有的 ros::NodeHandle 对象都被销毁了, 该节点就会被自动关闭.  

## 2. Namespaces

NodeHandles 提供了一个可以指定默认 namespace 的构造函数:  

```cpp
ros::NodeHandle nh("my_namespace");
```

在这种定义下, nh 将不再使用 `<node_namespace>` 命名空间作为相对命名空间, 而是使用 `<node_namespace>/my_namespace`.  

同时, 可以为一个 namespace 继续指定其父命名空间:  

```cpp
ros::NodeHandle nh1("ns1");
ros::NodeHandle nh2(nh1, "ns2");
```

这样 nh2 就从属于 `<node_namespace>/ns1/ns2` 命名空间.  

### 2.1 Global Names  

如果你还是想为某个节点指定一个全局的命名空间:  

```cpp
ros::NodeHandle nh("/my_global_namespace");
```

但是不推荐这样做, 因为这会导致命名空间无法被丛属到某个指定的功能包(e.g. by roslaunch). 除此之外, 使用全局命名空间也是很有用的.  

### 2.2 Private Names  

Using private names is a little bit tricker than calling a NodeHandle method with a private name ("~name") directly. Instead, 创建一个属于私有命名空间内的 ros::NodeHandle:  

```cpp
ros::NodeHandle nh("~my_private_namespace");
ros::Subscriber sub = nh.subscribe("my_private_topic", ...);
```

上面的例子将会订阅 `<node_name>/my_private_namespace/my_private_topic` 主题.  
