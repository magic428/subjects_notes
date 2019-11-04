# 指令集相关的intrinsics介绍

上面介绍的是pragma对intrinsic函数的使用，其中介绍了cos，还有很多类似的“内置函数版本”。有时候将上面的这些称之为”intrinsics函数“，除此之外，intrinsics更广泛的使用是指令集的封装，能直接映射到高级指令集，从而使得程序员可以以函数调用的方式来实现汇编能达到的功能，编译器会生成为对应的SSE等指令集汇编。

1. 如何使用这类函数

在windows上，包含#include <**mmintrin.h>头文件即可（不同的指令集扩展的函数可能前缀不一样），也可以直接包含#include <intrin.h>（这里面会根据使用环境判断使用ADM的一些兼容扩展）。

2. 关于数据类型

这些和指令集相关的函数，一般都有自己的数据类型，不能使用一般的数据类型传递进行计算，一般来说，MMX指令是__m64（http://msdn.microsoft.com/zh-cn/library/08x3t697(v=VS.90).aspx）类型的数据，SSE是__m128类型的数据等等。

3. 函数名：

这类函数名一般以__m开头。函数名称和指令名称有一定的关系。

4. 加法实例：

下面使用SSE指令集进行加法运算，一条指令对四个浮点数进行运算：

```cpp
#include <stdio.h>  
#include <intrin.h>  
  
int main(int argc, char* argv[])  
{  
    __m128  a;  
    __m128  b;  
      
    a = _mm_set_ps(1,2,3,4);        // Assign value to a  
    b = _mm_set_ps(1,2,3,4);        // Assign value to a  
  
    __m128 c = _mm_add_ps(a, b);    // c = a + b  
    float d =  c[0];
  
    printf("size: %lu,  %lu\n", sizeof(c[0]), sizeof(float));  
    printf("0: %lf\n", c.m128_f32[0]);  
    printf("1: %lf\n", c.m128_f32[1]);  
    printf("2: %lf\n", c.m128_f32[2]);  
    printf("3: %lf\n", c.m128_f32[3]);  
  
    return 0;  
}  
```
从代码看，好像很复杂，但是生成的汇编的效率会比较高。一条指令就完成了四个浮点数的加法，其运行结果如下：


（5）总结：

1. Intrinsics函数：能提高性能，会增大生成代码的大小，是编译器的”内置函数“。

2. Intrinsics对指令的封装函数：直接映射到汇编指令，能简化汇编代码的编写，另外，隐藏了寄存器分配和调度等。由于涉及到的数据类型、函数等内容较多，这里只是一个简单的介绍。