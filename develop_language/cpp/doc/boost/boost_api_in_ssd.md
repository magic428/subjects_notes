# ssd 中用到的 boost api  

```cpp
#include "boost/algorithm/string.hpp"
```

## 字符串操作   

1. boost::split()  

```cpp
boost::split(strings, FLAGS_gpu, boost::is_any_of(","));
``` 

字符串切分.对 FLAGS_gpu 字符串以 "," 进行切分, 最后切分得到的字符串存放在 strings 变量中.   

## 类型转换

1. boost::thread_specific_ptr<xxx>   

```cpp
boost::thread_specific_ptr<Caffe> thread_instance_;   
```

2. boost::lexical_cast()  

模板类函数. 

```cpp  
boost::lexical_cast<int>(strings[i]));
```
一个轻量级的类型转换.将 strings[i] 中的字符转换为 int 类型.   

```cpp
boost::lexical_cast<string>(solver_param.device_id()); 
boost::lexical_cast<string>(0)
```

将数字转换为字符串.   


## 互斥锁和条件变量    

`boost::mutex::scoped_lock` 是区域锁.   
```cpp
mutable boost::mutex mutex_;
boost::condition_variable condition_;

// Productor
boost::mutex::scoped_lock lock(mutex_);  // 离开作用域会自动 unlock
/**
 * Do Something...
*/
lock.unlock();
condition_.notify_one();

// Consumer
boost::mutex::scoped_lock lock(mutex_);  // 离开作用域会自动 unlock
while(!/*condition*/){

    condition_.wait(lock);
}

```

## 文件操作   
```cpp
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"  

std::string fullpath = boost::filesystem::initial_path<boost::filesystem::path>().string();

std::cout << fullpath << std::endl;  

```
