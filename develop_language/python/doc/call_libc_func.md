# Python 利用 ctypes 实现 C 库函数调用

ctypes 是强大的，使用它我们就能够调用动态链接库中函数，同时创建各种复杂的 C 数据类型和底层操作函数。使得 python 也具备了底层内存操作的能力，再配合 python 本身强大的表达能力，这才知道为什么 python 是黑客必学的编程语言。   
## 1. ctypes 使用   
ctypes 提供了三种方法调用动态链接库:`cdll()`, `windll()`, 和 `oledll()`。    
它们的不同之处就在于，函数的调用方法和返回值。   
`cdll()` 加载的库，其导出的函数必须使用标准的 `cdecl` 调用约定。   
`windll()` 方法加载的库，其导出的函数必须使用 `stdcall` 调用约定(Win32API 的原生约定)。   
`oledll()` 方法和 `windll()`类似，不过如果函数返回一个 `HRESULT` 错误代码，可以使用 `COM` 函数得到具体的错误信息。  
1.1 调用约定   
调用约定专指函数的调用方法。其中包括，函数参数的传递方法，顺序（压入栈或 者传给寄存器），以及函数返回时，栈的平衡处理。   
下面这两种约定是我们最常用到的： `cdecl` 和 `stdcall`。   
1.1.1 cdecl 调用约定    
函数的参数`从右往左`依次压入栈内，函数的调用者， 在函数执行完成后，负责函数的平衡。这种约定常用于 x86 架构的 C 语言里。   
```cpp
int  python_rocks(reason_one,reason_two,reason_three);
In x86 Assembly

push reason_three 
push reason_two 
push reason_one 
call python_rocks 
add  esp,12
```
从上面的汇编代码中，可以清晰的看出参数的传递顺序，最后一行，栈指针增加了 12 个字节(三个参数传递个函数，每个被压入栈的指针都占 4 个字节，共 12 个)， 使得函数调用之后的栈指针恢复到调用前的位置。
1.1.2 stdcall调用约定   
参数传递的顺序也是`从右到左`，不过栈的平衡处理由函数 `my_socks` 自己完成，而不是调用者。  
```cpp
int my_socks(color_onecolor_two,color_three);
In x86 Assembly

push color_three 
push color_two 
push color_one 
call my_socks
```
最后一点，这两种调用方式的返回值都存储在 EAX 中。   
1.1.3 使用方法   
Windows 下：   
```python
from ctypes import *
msvcrt = cdll.msvcrt
msg = "Hello world!\n"
msvcrt.printf("Testing: %s", msg)
```
Linux 下：   
```python
from ctypes import *
libc = CDLL("libc.so.6")
msg = "Hello, world!\n"
libc.printf("Testing: %s", msg)
```
使用 Python 创建一个 C 数据类型很简单，可以很容易的使用由 C 或者 C++ 些的组件。 下面这张图很好的表示了映射关系。    

![](../../snapshots/python_c_types.jpg)   

1.1.4 定义结构和联合   
在 C 中,这样定义:   
```cpp
union 
{
    long  barley_long;
    int    barley_int;
    char    barley_char[8];
}barley_amount;
```
在 python 中,这样定义:   
```python
class barley_amount(Union):
    _fields_ = [
    ("barley_long", c_long),
    ("barley_int", c_int),
    ("barley_char", c_char * 8),
    ]
```
**eg**:   
```python
from ctypes import *
class barley_amount(Union):
    _fields_ = [
    ("barley_long", c_long),
    ("barley_int", c_int),
    ("barley_char", c_char * 8),
    ]
value = raw_input("Enter the amount of barley to put into the beer vat:")
my_barley = barley_amount(int(value))
print "Barley amount as a long: %ld" % my_barley.barley_long
print "Barley amount as an int: %d" % my_barley.barley_int
print "Barley amount as a char: %s" % my_barley.barley_char
```
输出结果：   
```
Enter the amount of barley to put into the beer vat:66
Barley amount as a long: 66
Barley amount as an int: 66
Barley amount as a char: B
```
给联合赋一个值就能得到三种不同的表现方式。最后一个 barley_char 输出的结果是 B， 因为 66 刚好是 B 的 ASCII 码。   
barley_char 成员同时也是个数组，一个八个字符大小的数组。在 ctypes 中申请一个数组， 只要简单的将变量类型乘以想要申请的数量就可以了。   
