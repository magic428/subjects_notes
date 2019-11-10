# C++ 中 cout 如何输出指定精度  

在编写程序中会经常遇到需要输出指定的精度, 可以使用 C++ 中的 setprecision() 函数来实现.  

- 包含 setprecision() 函数头文件: `#include <iomanip>` 
- 使用 setprecision() 控制输出流显示浮点数的有效数字位数, 共同使用 fixed 可以控制小数点右面的位数。  

例子:  

```cpp
// setprecision example
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

int main () 
{
  double f =3.14159;
  std::cout << std::setprecision(5) << f << '\n';
  std::cout << std::setprecision(9) << f << '\n';
  std::cout << std::fixed;
  std::cout << std::setprecision(5) << f << '\n';
  std::cout << std::setprecision(9) << f << '\n';
  return 0;
}
```

Output:   

```
3.1416
3.14159
3.14159
3.141590000
```
