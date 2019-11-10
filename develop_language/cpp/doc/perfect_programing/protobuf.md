# protobuf 编译使用   

在网络传输中，经常有大端小端的区别，并且与我们编写的程序交互的，不一定也是`C/C++`，例如安卓，所以我们需要一个协议，能够进行数据的传输。大部分人最常用的，可能是`json`了，今天我们介绍另外一个工具，`protobuf`，这是一个`google`的开源项目。支持`c++,java,python`等多种语言。   

## 1. 下载与安装
```
# 第一种安装方式
[github代码地址](https://github.com/google/protobuf)
进入代码目录：
./configure --prefix=/usr/local/protobuf
sudo make
sudo make install

# 第二种安装方式  
sudo apt-get install protobuf-compiler
```
## 2. 使用  
使用`protobuf`需要我们定义一个`.proto`文件，用来定义`protobuf`消息。消息由至少一个字段组合而成，类似于`C`语言中的结构。`限定修饰符 | 数据类型 | 字段名称 | = | 字段编码值 | [字段默认值]`，每个字段都有一定的格式。   
2.1 限定修饰符    
required 表示这个字段必填   
optional 可选   
repeated 可以看作是在传递一个数组的值,并且可以为空  
2.2 数据类型   
![](../snapshots/type.png)   
2.3 字段名称   
`protobuf`建议以`下划线`命名而非`驼峰式`。例如`app_name`优于`appName`。   
2.4 字段编码值   
使用`protobuf`转化的`byte`中并没有带字段名称，而是一个字段编码，字段编码是一个`正32位`整数，通常来说，越小效率越高。   

当然，`protobuf`还支持`Import/enum/package`等关键字。   
## 3. 示例  
3.1 新建一个`touch_message.proto`的文件，定义`TouchMesssage`理解为一个`心跳`的`data`。   
```proto
message TouchMessage{
	required string uid = 1;
	required string data_version = 2;
	required string time_stamp = 3;
	required string app_version = 4 [default = 'ver0.99'];
}
```
proto 语言与 C 语言差别不是很大，结构体 struct 字段换成 message， 变量之前需要追加optional 和 repeated 标记字段。分别表示的是单变量，还是容器数组变量。值得一提的是， proto提供 required 字段，但是 Google 程序员都懒得用，经常会出现奇怪 bug，所以一律用 optional 替代 required 。

repeated 标记之后，本质是数组，但实际实现可能是类似于 ST L容器，它提供了不少类似容器的操作。  
[default] 可以提供默认值，对于基本数据类型，不设默认值将会同 C 语言一样产生类似默认值。但我们不推荐使用 proto 自身提供的默认值. 通常变量会生成 has_xxx() 方法, 用来检测该变量是否被设置。人工指定的默认值，has_xxx() 会返回 true，而 proto 提供的自动默认值，则是 false。   

另外，对于 repeated int32 or int64，使用 [packed=true] 似乎可以优化速度，对于 float 其实是无效的。

Caffe 里有些 repeat float 也打上了 [packed=true]，其实没什么意义。

最后，所有数据结构变量，都需要一个唯一的 id，id 从 1 开始。这与 proto 内部编码系统有关，1~20 编码长度小，访问速度快。随着 id 值增加，后续变量访问速度会递减。   

3.2 然后执行命令   
```
protoc touch_message.proto --cpp_out=./
```
我们就能看到生成的对应的`.h`跟`.cpp`文件了。打开`.cpp`文件，我们发现其实是生成一个类，类中有着对应的方法。`SerializeWithCachedSizes`是序列化，`MergePartialFromCodedStream`是反序列化。     


##


### DebugString() 函数   

开发过程中需要经常查看数据，可以调用对象的 DebugString() 函数即可返回可读性好的数据。   
```cpp
NetParameter filtered_param;
FilterNet(in_param, &filtered_param);
...
LOG_IF(INFO, Caffe::root_solver())
      << "Initializing net from parameters: " << std::endl
      << filtered_param.DebugString();
```