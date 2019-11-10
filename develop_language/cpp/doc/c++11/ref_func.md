# std::ref() 和 std::cref()   

> C++本身有引用（&）, 为什么 C++11 又引入了 std::ref() 和 std::cref()?   

主要是考虑函数式编程（如std::bind(), 创建 tread 传递参数）在使用时是对参数直接拷贝，而不是引用。而函数式编程的函数原型并不是由我们决定的.    

如下例子：    

```cpp
#include <functional>
#include <iostream>

void func(int& n1, int& n2, const int& n3)
{
    std::cout << "In func: " << n1 << ' ' << n2 << ' ' << n3 << '\n';
    ++n1; // increments the copy of n1 stored in the function object
    ++n2; // increments the main()'s n2
    // ++n3; // compile error, 常量的引用    
}

int main()
{
    int n1 = 1, n2 = 2, n3 = 3;
    std::function<void()> bound_f = std::bind(func, n1, std::ref(n2), std::cref(n3));
    n1 = 10;
    n2 = 11;
    n3 = 12;

    std::cout << "Before func: " << n1 << ' ' << n2 << ' ' << n3 << '\n';
    bound_f();
    std::cout << "After func: " << n1 << ' ' << n2 << ' ' << n3 << '\n';
}
```

函数输出:   
```
Before function: 10 11 12
In function: 1 11 12
After function: 10 12 12
```
上述代码在执行 `std::bind` 后，在函数 func() 中 n1 的值仍然是 1， n2 和 n3 改成了修改的值。说明 std::bind() 使用的是参数的拷贝而不是引用。 具体为什么 std::bind 不使用引用，可能确实有一些需求，使得 C++11 的设计者认为默认应该采用拷贝，如果使用者有需求，加上std::ref 即可。    

