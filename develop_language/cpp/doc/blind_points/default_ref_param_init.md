# 函数默认参数类型为引用?  

## 1. 如何赋值?  

直接使用构造函数返回的值不能做左值, 因此不能被用来初始化非常量引用变量.  

~~~cpp

class HasPtr{

public:
    // error: 不能被用来初始化非常量引用变量
    // HasPtr(std::string &s = string("hello")) : str(s) {}
    HasPtr(const std::string &s = string("hello")) : str(s) {}
    string func()
    {
        return string("hello");
    }

private:
    std::string str;
}
~~~


## 2. 不要去使用这种默认引用参数  

可以使用引用作为参数, 但最好不要使用带默认值的引用参数.   

~~~cpp
class HasPtr{

public:
    HasPtr(std::string &s) : str(s) {}

private:
    std::string str;
}
~~~