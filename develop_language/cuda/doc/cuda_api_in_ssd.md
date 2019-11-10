# caffe/ssd 中使用到的 cuda api

关键头文件: 
```
#include "caffe/util/device_alternate.hpp"
```
## CUDA API   
1. CUDA_CHECK(condition)    
检查不同 cuda 函数调用后的返回结果.   
```cpp
#define CUDA_CHECK(condition) \
  /* Code block avoids redefinition of cudaError_t error */ \
  do { \
    cudaError_t error = condition; \
    CHECK_EQ(error, cudaSuccess) << " " << cudaGetErrorString(error); \
  } while (0)
```
2. `cudaError_t cudaGetDeviceProperties(struct cudaDeviceProp *prop, int device)`
获取 gpus 的设备属性. 获取到的属性值保存在 prop 中, device 要获取属性的设备 ID.   
3. `cudaError_t cudaGetDeviceCount(int *count)`    
获取所有 gpus 的个数.   


