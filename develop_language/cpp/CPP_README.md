# C++ 笔记

## 笔记目录  
### VS 使用教程   

[1. vs2013 使用教程](./doc/vs2013_usage.md)   
[1. vs2013 使用教程](/dev_tools/cpp/doc/vs2013_usage.md)   

### C++ primer 5th 笔记 
- [ map 使用实例](./doc/cpp_primer_5nd/map_instance.md)    
- [ vector 操作](./doc/cpp_primer_5nd/vector_operation.md)      
- [ shared_ptr 使用实例](./cpp_primer_5nd/doc/shared_ptr.md)    
- [ C++ Primer 5nd 知识点总结](./doc/cpp_primer_5nd/cpp_primer_5nd.md)       
- [ 模板的使用 ](./doc/cpp_primer_5nd/template.md)       
- [ 命名空间的使用 ](./doc/cpp_primer_5nd/namespace.md)       
- [ 类成员函数作为线程函数使用 ](./doc/thread_class_method.md)       
### boost 库使用
- [boost 常用 API](./doc/boost/boost_api_in_ssd.md)   
- [boost time 库](./doc/boost/time.md)   
- [boost thread 编程指南](./doc/boost/thread.md)   
### C++11 标准   
- [std::ref() 和 std::cref() 函数](./doc/c++11/ref_func.md)
### glog 和 gflags
- [glog 和 gflags 的使用](./doc/gflags.md)
### ProtoBuf 使用 
- [ProtoBuf 的使用](./doc/protobuf.md)

## CPP 使用中的盲点  

- [ C/C++ 中 #pragma once 的使用](./doc/blind_points/pragma_once.md)
- [ 函数默认参数类型为引用? ](./doc/blind_points/default_ref_param_init.md)


### 在山寨中成长      
- [x] convert_annoset.cpp 中的 gflag/glog 语法模仿;   
- [ ] caffe 增广方式的改变;   


## 工程化 cpp 项目       
[让自己的 cpp 工程更加"工程化" ](./doc/projective_cpp.md)
[高质量 C++/C 编程指南 - C++函数的高级特性](./doc/perfect_programing/advance_cpp_features.md)

## 老杜培训
[2. short int 使用注意事项](./doc/shortInt.md)   
1. 结构体中的字节对齐；
progma(push);
progma();
progma(pop);
2. .ini配置文件的读写
3. 字符集编码
`_MBCS, _UNICODE`
4. 多线程
信号量
临界区
[秒杀多线程 more windows]()


## 学习 CPP 的一些开源项目:   

> 知乎专栏: https://www.zhihu.com/question/28341521.  

How to Make a Computer Operating System   

> https://github.com/SamyPesse/How-to-Make-a-Computer-Operating-System   
 
This repository is a remake of my old course. It was written several years ago as one of my first projects when I was in High School, I'm still refactoring some parts. The original course was in French and I'm not an English native. I'm going to continue and improve this course in my free-time.  

Book: An online version is available at http://samypesse.gitbooks.io/how-to-create-an-operating-system/ (PDF, Mobi and ePub). It was generated using GitBook.  

5. BitCoinbitcoin/bitcoin · GitHubBitCoin这两年很火，抛开比特币本身，看BitCoin实现，也是C++的项目，构建在P2P网络之上的一套虚拟的支付系统。我们可以利用的是背后的P2P Protocol，密码学，支付，去中心化这些。现在有很多衍生的开源项目，就是利用BitCoin 的这套P2P的框架在做，bitmessage，https://bitmessage.org/wiki/Main_Page 完全去中心化，能防止老大哥的监听，在现在云计算，大数据风行的时代，privacy问题会越来越多，可以借鉴P2P的思路，构建这些去中心化的服务。

https://github.com/Bitmessage/PyBitmessage