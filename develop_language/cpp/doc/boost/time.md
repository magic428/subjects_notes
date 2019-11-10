# boost 的 time 组件   

## 1. 算法运行时间统计   

> caffe/include/caffe/util/benchmark.hpp   

caffe 提供了每个 layer 前向运算和反向传播所需的时间统计类, 即 Timer 类.   

```cpp
#include <boost/date_time/posix_time/posix_time.hpp>

boost::posix_time::ptime start_cpu_;
boost::posix_time::ptime stop_cpu_;

// 获取当前时间,单位为微秒; 可以将 microsec_clock 替换成 second_clock(以秒为单位);   
start_cpu_ = boost::posix_time::microsec_clock::local_time();
stop_cpu_ = boost::posix_time::microsec_clock::local_time();

// 返回毫秒数
float elapsed_milliseconds_ = (stop_cpu_ - start_cpu_).total_milliseconds();

// 返回微秒数
float elapsed_microseconds_ = (stop_cpu_ - start_cpu_).total_microseconds();
```

boost_time 模块在编译的时候, cmake 中不需要指定 time 编译库.   

## 2. date_time 日期   

### 2.1 输出 `YYYYMMDD-HH:MM:SS`   

最为常用的一种方式.   

```cpp
#include <boost/date_time/posix_time/posix_time.hpp>    
#define BOOST_DATE_TIME_SOURCE    
    
std::string strTime = boost::posix_time::to_iso_string(\    
boost::posix_time::second_clock::local_time());    
    
// 这时候strTime里存放时间的格式是YYYYMMDDTHHMMSS，日期和时间用大写字母T隔开了    
    
int pos = strTime.find('T');    
strTime.replace(pos,1,std::string("-"));    
strTime.replace(pos + 3,0,std::string(":"));    
strTime.replace(pos + 6,0,std::string(":"));    
    
std::cout << strTime.c_str() << std::endl;   
```

### 2.2 输出格式为: `YYYYMMDD`.    

比较少用.   

```cpp
#include <boost/date_time/gregorian/gregorian.hpp>    
#define BOOST_DATE_TIME_SOURCE    
    
std::string strTime = boost::gregorian::to_iso_string(\    
boost::gregorian::day_clock::local_day());    
    
std::cout << strTime.c_str() << std::endl;  


## boost 时间和系统自带时间比较   


```cpp
time_t start = std::clock();

double seconds = (double)(std::clock() - start) / CLOCKS_PER_SEC;
double milseconds = seconds*1000;
```