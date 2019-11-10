# C++ 函数的高级特性

## 重载  

如果 C++ 程序要调用已经被编译后的 C 函数,该怎么办?   

假设某个 C 函数的声明如下:  
```cpp
void foo(int x, int y);
```
该函数被 C 编译器编译后在库中的名字为 `_foo`, 而 C++ 编译器则会产生像 `_foo_int_int` 之类的名字用来支持函数重载和类型安全连接。由于编译后的名字不同, C++ 程序不能直接调用 C 函数。C++ 提供了一个 C 连接交换指定符号 `extern “C”` 来解决这个问题。例如:  
```cpp
extern “C”
{
    void foo(int x, int y);
    ... // 其它函数
}
```
或者写成:   
```cpp
extern “C”
{
    #include “myheader.h”
    ... // 其它 C 头文件
}
```
这就告诉 C++ 编译译器, 函数 foo 是个 C 连接, 应该到库中找名字 `_foo` 而不是找 `_foo_int_int`。C++ 编译器开发商已经对 C 标准库的头文件作了 `extern “C”` 处理,所以我们可以用 #include 直接引用这些头文件。   


**注意:** 并不是两个函数的名字相同就能构成重载。全局函数和类的成员函数同名不算重载, 因为函数的作用域不同。为了与同名成员函数 区别, 全局函数被调用时应加 `::` 标志. 

##  当心隐式类型转换导致重载函数产生二义性
下例中, 第一个 output 函数的参数是 int 类型, 第二个 output 函数的参数是 float 类型。由于数字本身没有类型, 将数字当作参数时将自动进行类型转换(称为隐式类型转换)。语句 output(0.5) 将产生编译错误, 因为编译器不知道该将 0.5 转换成int 还是 float 类型的参数。隐式类型转换在很多地方可以简化程序的书写, 但是也可能留下隐患。  

```cpp
# include <iostream.h>
void output( int x);
// 函数声明
void output( float x); // 函数声明
void output( int x)
{
    cout << " output int " << x << endl ;
}
void output( float x)
{
    cout << " output float " << x << endl ;
}
void main(void)
{
    int x = 1;
    float y = 1.0;
    
    output(x); // output int 1
    output(y); // output float 1
    output(1); // output int 1
    
    // output(0.5);     // error! ambiguous call, 因为自动类型转换
    output(int(0.5));   // output int 0
    output(float(0.5)); // output float 0.5
}
```