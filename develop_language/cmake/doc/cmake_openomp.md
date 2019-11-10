# CMake 调用 OpenMP

```cmake
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11" )
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
```


另外一种命令行编译(From Stack Overflow):  

The OpenMP pragmas are only enabled when compiled with "-fopenmp". Otherwise they are completely ignored by the compiler.   

Environment Variable OMP_NUM_THREADS sets the number of threads.  

```bash
g++ -std=c++11 -c test.cpp -o test.o -fopenmp -lpthread

export OMP_NUM_THREADS=4
./test
```

## OpenMP 基本知识   

OpenMP的并行计算程序总是以 '#pragma omp' 作为开始的。在 'pragma omp' 后面是一条 parallel 指令，用来表明之后是一个结构化代码块，最基本的 parallel 指令可以用如下的形式表示:   

```
#pragma omp parallel
```

如果使用上面这条简单的指令去运行并行计算的话，程序的线程数将由运行时系统决定（这里使用的算法十分复杂），典型的情况下，系统将在每一个核上运行一个线程。如果需要执行使用多少个线程来执行我们的并行程序，就得为 parallel 指令增加 num_threads 子句，这样的话，就允许程序员指定执行后代码块的线程数。  

```
#pragma omp parallel num_threads(thread_count)
```

在程序中，为了能够使用 OpenMP 函数，还需要在程序包含 omp.h 头文件(好像实际中并不需要)。   

```
#include <omp.h>
```

OpenMP 只要添加一条很简单的 parallel for 指令，就能够并行化大量的 for 循环所组成的串行程序。但使用它有几天限制条件：   

(1). OpenMP只能并行化for循环，它不会并行while和do-while循环，而且只能并行循环次数在for循环外面就确定了的for循环。  
(2). 循环变量只能是整型和指针类型（不能是浮点型）  
(3). 循环语句只能是单入口单出口的。循环内部不能改变index，而且里面不能有 goto、break、return。但是可以使用 continue，因为它并不会减少循环次数。另外 exit 语句也是可以用的，因为它的能力太大，它一来，程序就结束了。  

关于数据的同步问题. 两个要点：

(1). OpenMP 编译器不检查被 parallel for 指令并行化的循环所包含的迭代间的依赖关系，而是由程序员来识别这些依赖关系。  
(2). 一个或更多个迭代结果依赖于其它迭代的循环，一般不能被 OpenMP 正确的并行化。  

从求斐波那契（fibonacci）数的这个例子中，我们发现计算 fibo[5] 需要先得到 fibo[3], fibo[4] 这两个数据, 这种依赖关系称之为'数据依赖'。 而 fibo[5] 的值在一个迭代中计算，它的结果是在之后的迭代中使用，这样的依赖关系称之为'循环依赖'。   

因此需要小心循环依赖关系，而不用担心数据依赖。   

缺省情况下，任何在循环前声明的变量，在线程间都是共享的。 为了消除这种循环依赖关系之外，还需要保证每个线程有它自己的变量副本，OpenMP 的语句为我们考虑了，通过添加 private 子句到 parallel 指令中来实现这一目标。  

```
#pragma omp parallel for num_threads(thread_count) reduction(+:sum) private(factor)
```


## 关于 windows vs 中的 OpenMP   

openMP 一没有额外 include 头文件，二没有额外 link 库文件，只是在 for 循环前加了一句#pragma omp parallel for。而且这段代码在单核机器上，或者编译器没有将 openMP 设为 Yes 的机器上编译也不会报错，将自动忽略 #pragma 这行代码，然后按照传统单核串行的方式编译运行！    

我们唯一要多做的一步，是从 C:\Program Files\Microsoft Visual Studio 9.0\VC\redist\x86\Microsoft.VC90.OPENMP 和C:\Program Files\Microsoft Visual Studio 9.0\VC\redist\Debug_NonRedist\x86\Microsoft.VC90.DebugOpenMP 目录下分别拷贝 vcomp90d.dll 和 vcomp90.dll 文件到工程文件当前目录下。   

## 参考资料  

1. [openMP 的一点使用经验(reduction, section, critical)](http://www.cnblogs.com/yangyangcv/archive/2012/03/23/2413335.html)    


