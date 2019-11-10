# 利用 Boost.Python 实现 Python C/C++ 混合编程   

## 1. 导出全局普通函数  

cpp 源码如下:  

```cpp
#include <string>
#include <boost/python.hpp>

using namespace std;
using namespace boost::python;

char const * greet()
{
    return "hello, world";
}

BOOST_PYTHON_MODULE(hello_ext)
{
    def("greet", greet);
}
```
这里贴出我使用的 CMakeFiles.txt:   

```cmake

cmake_minimum_required(VERSION 2.8)
project( hello_ext )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

SET( CMAKE_BUILD_TYPE Release )
#SET( CMAKE_BUILD_TYPE Debug )
set(LINKER_LIBS "")  

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${PYTHON_LIBRARIES})

find_package(Boost COMPONENTS python-py34)  #find_package(Boost 1.45.0 COMPONENTS python)
include_directories (${Boost_INCLUDE_DIRS})
list(APPEND LINKER_LIBS ${Boost_LIBRARIES})

list(APPEND LINKER_LIBS boost_python)

# Add the source in project root directory
aux_source_directory(src/python DIRSRCS)

# user defined
include_directories(/usr/include/python3.4m)
include_directories(${Boost_INCLUDE_DIRS})

# Target
message("source files: ${DIRSRCS}")
message("LINKER_LIBS files: ${LINKER_LIBS}")

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./exercise/)
# add_executable( hello_ext ${DIRSRCS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../src/python)
add_library(hello_ext SHARED ${DIRSRCS})
target_link_libraries (hello_ext
    ${LINKER_LIBS}
)
set_target_properties(hello_ext PROPERTIES PREFIX "" OUTPUT_NAME "hello_ext")
```

****: `BOOST_PYTHON_MODULE(hello_ext)` 和 `set_target_properties(hello_ext PROPERTIES PREFIX "" OUTPUT_NAME "hello_ext")` 中的 `hello_ext` 必须一致.   

python 调用:  

```python
import hello_ext
print hello_ext.greet()
```

## 导出类   

导出默认构造的函数的类.  

```cpp
#include<string>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct World
{
    void set(string msg) { this->msg = msg; }
    string greet() { return msg; }

    string msg;
};

BOOST_PYTHON_MODULE(hello) // 导出的 module 名字
{
    class_<World>("World")
        .def("greet", &World::greet)
        .def("set", &World::set);
}
```

python 调用方式:  

```python
import hello 
planet = hello.World() # 调用默认构造函数，产生类对象
planet.set("howdy")   # 调用对象的方法
print planet.greet() # 调用对象的方法
```

## 构造函数的导出  

```cpp
#include<string>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct World
{
    World(string msg):msg(msg){} //增加构造函数
    World(double a, double b):a(a),b(b) {} //另外一个构造函数
    void set(string msg) { this->msg = msg; }
    string greet() { return msg; }
    double sum_s() { return a + b; }
    string msg;
    double a;
    double b;
};

BOOST_PYTHON_MODULE(hello) //导出的module 名字
{
    class_<World>("World",init<string>()) 
        .def(init<double,double>()) // expose another construct
        .def("greet", &World::greet)
        .def("set", &World::set)
        .def("sum_s", &World::sum_s);
}
```

python 调用：   

```python
import hello
planet = hello.World(5,6)
planet2 = hello.World("hollo world")

print planet.sum_s()
print planet2.greet()
```
**注意**: 这里用户定义了自己的构造函数, 那么编译器就不会自动生成默认构造函数, 除非用户自己显式定义. 因此类的导出方式也就不能使用: `class_<World>("World")    \.def(init<string>())`, 而只能使用: `class_<World>("World",init<string>())`.   

如果不想导出任何构造函数，则使用 `no_init`:   

```cpp
class_<Abstract>("Abstract", no_init); 
```

## 类的数据成员  

```cpp
#include<string>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct Var
{
    Var(string name):name(name),value(){}
    string const name;

    float value;
};

BOOST_PYTHON_MODULE(hello_var)
{
    class_<Var>("Var", init<string>())
        .def_readonly("name", &Var::name) // 只读
        .def_readwrite("value", &Var::value); // 读写
}
```

python 调用:  

```python
import hello_var

var = hello_var.Var("hello_var")
var.value = 3.14
# var.name = 'hello' # error, variable name is readonly

print var.name
```

C++ 类对象导出为 Python 的类对象，注意 var.name 不能赋值。

## 类的属性

```cpp
// 类的属性
#include<string>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct Num
{
    Num(){}
    float get() const { return val; }
    void set(float val) { this->val = val; }
    float val;

};

BOOST_PYTHON_MODULE(hello_num)
{
    class_<Num>("Num")
        .add_property("rovalue", &Num::get) // 对外：只读
        .add_property("value", &Num::get, &Num::set);// 对外读写 .value值会改变.rovalue值，存储着同样的数据。
}
```

python 调用:  

```python
import hello_num
num = hello_num.Num()
num.value = 10
print (num.rovalue) #  result: 10
```

## 继承

```cpp
// 类的继承
#include<string>
#include<iostream>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct Base {
    virtual ~Base() {};
    virtual string getName() { return "Base"; }

    string str;
};

struct Derived : Base {

    string getName() { return "Derived"; }

};


void b(Base *base) { cout << base->getName() << endl; };

void d(Derived *derived) { cout << derived->getName() << endl; };

Base * factory() { return new Derived; }

/*
    下面的额外的代码如果去掉会报错。
    解决地址：http://stackoverflow.com/questions/38261530/unresolved-external-symbols-since-visual-studio-2015-update-3-boost-python-link/38291152#38291152
*/
namespace boost
{
    template <>
    Base const volatile * get_pointer<class Base const volatile >(
        class Base const volatile *c)
    {
        return c;
    }
}


BOOST_PYTHON_MODULE(hello_derived)
{
    class_<Base>("Base")
        .def("getName", &Base::getName)
        .def_readwrite("str", &Base::str);


    class_<Derived, bases<Base> >("Derived")
        .def("getName", &Derived::getName)
        .def_readwrite("str", &Derived::str);


    def("b", b);
    def("d", d);

    def("factory", factory,
        return_value_policy<manage_new_object>());//

}
```

python 调用:   

```python
import hello_derived

derive = hello_derived.factory()
hello_derived.d(derive)
```

## 导出类的虚函数   

```cpp
/**
 * 类的虚函数，实现的功能是：可以编写 Python 类，来继承 C++类   
*/
#include<boost/python.hpp>

#include<boost/python/wrapper.hpp>
#include<string>
#include<iostream>

using namespace boost::python;
using namespace std;

struct Base
{
    virtual ~Base() {}
    virtual int f() { return 0; };
};


struct BaseWrap : Base, wrapper<Base>
{
    int f()
    {
        if (override f = this->get_override("f"))
            return f(); // 如果函数进行重载了，则返回重载的
        return Base::f(); // 否则返回基类
    }
    int default_f() { return this->Base::f(); }
};

BOOST_PYTHON_MODULE(hello_virtual)
{
    class_<BaseWrap, boost::noncopyable>("Base")
        .def("f", &Base::f, &BaseWrap::default_f);
}
```

python 调用：  
```python
import hello_virtual 

base = hello_virtual.Base()
# 定义派生类，继承C++类
class Derived(hello_virtual.Base):
    def f(self):
        return 42

derived = Derived()

print (base.f())
print (derived.f())
```

## 导出类的运算符 / 特殊函数  

```cpp

// 类的运算符 / 特殊函数
#include<string>
#include<iostream>

// #include<boost/python.hpp> 如果仅包含该头文件，会出错

#include <boost/python/operators.hpp>
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/operators.hpp>

using namespace std;
using namespace boost::python;

class FilePos
{
public:
    FilePos() :len(0) {}
    operator double()const { return len; };//重载类型转换符
    int len;
};

// operator 方法

FilePos operator+(FilePos pos, int a)
{
    pos.len = pos.len + a;

    return pos; //返回的是副本

}

FilePos operator+(int a, FilePos pos)
{
    pos.len = pos.len + a;

    return pos; //返回的是副本

}


int operator-(FilePos pos1, FilePos pos2)
{

    return (pos1.len - pos2.len);

}

FilePos operator-(FilePos pos, int a)
{
    pos.len = pos.len - a;
    return pos;
}

FilePos &operator+=(FilePos & pos, int a)
{
    pos.len = pos.len + a;
    return pos;
}

FilePos &operator-=(FilePos & pos, int a)
{
    pos.len = pos.len - a;
    return pos;
}

bool operator<(FilePos  pos1, FilePos pos2)
{
    if (pos1.len < pos2.len)
        return true;
    return false;
}


// 特殊的方法

FilePos pow(FilePos pos1, FilePos pos2)
{
    FilePos res;
    res.len = std::pow(pos1.len, pos2.len);
    return res;

}
FilePos abs(FilePos pos)
{
    FilePos res;
    res.len = std::abs(pos.len);

    return res;
}

ostream& operator<<(ostream& out, FilePos pos)
{
    out << pos.len;
    return out;
}

BOOST_PYTHON_MODULE(hello_operator)
{
    class_<FilePos>("FilePos")
        .def_readwrite("len",&FilePos::len)
        .def(self + int())
        .def(int() + self)
        .def(self - self)
        .def(self - int())
        .def(self += int())
        .def(self -= other<int>())
        .def(self < self)
        .def(float_(self))//特殊方法 ,     __float__
        .def(pow(self, other<FilePos>()))  // __pow__
        .def(abs(self))         //  __abs__
        .def(str(self));                //  __str__ for ostream


}
```

注意上面的：`.def(pow(self, other<FilePos>()))` 模板后面要加上括号。也要注意头文件的包含，否则会引发错误。   

python 调用:  

```python
import hello_operator

filepos1 = hello_operator.FilePos()
filepos1.len = 10

filepos2 = hello_operator.FilePos()
filepos2.len = 20;

print filepos1 - filepos2
```

## 函数的调用策略  

```cpp
// 函数的调用策略 

#include<string>
#include<iostream>

#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct X
{
    string str;
};
struct Z
{
    int value;
};

struct Y
{
    X x;
    Z *z;
    int z_value() { return z->value; }
};

X & f(Y &y, Z*z)
{
    y.z = z;
    return y.x;  // 因为x是y的数据成员，x的声明周期与y进行了绑定。
}


BOOST_PYTHON_MODULE(hello_call_policy)
{

    class_<Y>("Y")
        .def_readwrite("x", &Y::x)
        .def_readwrite("z", &Y::z)
        .def("z_value", &Y::z_value);
    class_<X>("X")
        .def_readwrite("str", &X::str);
    class_<Z>("Z")
        .def_readwrite("value", &Z::value);

    // return_internal_reference<1 表示返回的值与第一个参数有关系：即第一个参数是返回对象的拥有者（y和x都是引用的形式)。
    // with_custodian_and_ward<1, 2> 表示第二个参数的生命周期依赖于第一个参数的生命周期。
    // 因为我们的目的是：Python接口应尽可能的反映C++接口
    def("f", f, return_internal_reference<1, with_custodian_and_ward<1, 2> >());
}
```

## 导出重载的成员函数  

```cpp
// overloading

#include<string>
#include<iostream>

#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct X
{

    bool f(int a)
    {
        return true;
    }

    bool f(int a, double b)
    {
        return true;
    }

    bool f(int a, double b, char c)
    {
        return true;
    }
        
    int f(int a, int b, int c)
    {
        return a + b + c;
    }
};

bool (X::*fx1)(int) = &X::f;
bool(X::*fx2)(int, double) = &X::f;
bool(X::*fx3)(int, double, char) = &X::f;
int(X::*fx4)(int, int, int) = &X::f;

BOOST_PYTHON_MODULE(hello_overloaded)
{
    class_<X>("X")
        .def("f", fx1)
        .def("f", fx2)
        .def("f", fx3)
        .def("f", fx4);

}
```

python 调用:   

```python
import hello_overloaded

x = hello_overloaded.X() # create a new object

print (x.f(1))  # default int type
print (x.f(2,double(3)))
print (x.f(4,double(5),chr(6)))  # chr(6) convert * to char 
print (x.f(7,8,9))
```

## 导出普通函数的默认参数  

然而通过上面的方式对重载函数进行封装时，就丢失了默认参数的信息。因此我们可以通过一般形式的封装，如下：     

```cpp
int f(int,double = 3.14, char const * = "hello");
int f1(int x){ return f(x); }
int f2(int x,double y){ return f(x,y); }

// int module init
def("f",f);  // 所有参数
def("f",f2); // 两个参数
def("f",f1); // 一个参数
```
但是通过上面的形式封装很麻烦。我们可以通过 `宏` 的形式，为我们批量完成上面的功能。   

C++:

```cpp
// BOOST_PYTHON_FUNCTION_OVERLOADS

#include<string>
#include<iostream>

#include<boost/python.hpp>


using namespace std;
using namespace boost::python;


void foo(int a, char b = 1, unsigned c = 2, double d = 3)
{
    return;
}

BOOST_PYTHON_FUNCTION_OVERLOADS(foo_overloads, foo, 1, 4); // 参数个数的最小为 1，最大为 4

BOOST_PYTHON_MODULE(hello_overloaded)
{
    def("foo", foo, foo_overloads()); // 实现导出带有默认参数的函数
}
```

python 调用:

```python
import hello_overloaded

hello_overloaded.foo(1)
hello_overloaded.foo(1,chr(2))
hello_overloaded.foo(1,chr(2),3)  # 3对应的C++为unsigned int
hello_overloaded.foo(1,chr(2),3,double(4))
```

## 导出带默认参数的普通成员函数

```cpp
// 使用 BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS 宏，完成成员函数默认参数的接口   

#include<string>
#include<iostream>

#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct george
{
    void wack_em(int a, int b = 0, char c = 'x')
    {
        return;
    }

};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(george_overloads, wack_em, 1, 3); // 参数个数的最小为1，最大为3

BOOST_PYTHON_MODULE(hello_member_overloaded)
{

    class_<george>("george")
        .def("wack_em", &george::wack_em, george_overloads());

}
```

python 调用:   

```python
import hello_member_overloaded

c = hello_member_overloaded.george()

c.wack_em(1)
c.wack_em(1,2)
c.wack_em(1,2,chr(3))
```

## 导出带默认参数的构造函数  

利用 init 和 optional 实现构造函数的重载。 使用方法如下：  

```cpp
// init  optional

#include<string>
#include<iostream>
#include<boost/python.hpp>

using namespace std;
using namespace boost::python;

struct X
{
    X(int a, char b = 'D', string c = "constructor", double b = 0.0) {}
};

BOOST_PYTHON_MODULE(hello_construct_overloaded)
{
    class_<X>("X")
        .def(init<int, optional<char, string, double> >()); // init 和 optional
}
```

## 对象接口

Python 是动态类型的语言，C++ 是静态类型的。Python 变量可能是：integer, float, list, dict, tuple, str, long, 等等，还有其他类型。从 Boost.Python 和 C++ 的观点来看，Python 中的变量是类 object 的实例， 在本节，我们看一下如何处理 Python 对象。  

### 1. 基本接口  

```cpp
// init  optional
#include<string>
#include<iostream>
#include<boost/python.hpp>
#include <numpy/arrayobject.h>
using namespace std;
using namespace boost::python;

namespace bp = boost::python;


void f(object x)
{
    int y = extract<int>(x); // retrieve an int from x

}

int g(object x)
{
    extract<int> get_int(x);
    if (get_int.check())
        return get_int();
    else
        return 0;
}


int test(object &x)
{
    dict d = extract<dict>(x.attr("__dict__"));
    d["whatever"] = 4;
    return 0;
}

int test2(dict & d)
{
    d["helloworld"] = 3;
    return 0;
}
class A {

public:
    list lst;
    void listOperation(list &lst) {};
};

// 传入 np.array 数组对象，让 C++ 进行处理
int add_arr_1(object & data_obj, object rows_obj, object cols_obj)
{
    PyArrayObject* data_arr = reinterpret_cast<PyArrayObject*>(data_obj.ptr());
    float * data = static_cast<float *>(PyArray_DATA(data_arr));
    
    // using data
    int rows = extract<int>(rows_obj);
    int cols = extract<int>(cols_obj);
    for (int i = 0; i < rows*cols; i++)
    {
        data[i] += 1;
    }
    return 0;

}
BOOST_PYTHON_MODULE(hello_object)
{
    def("test", test);
    def("test2", test2);
    def("add_arr_1", add_arr_1);
}
```

python 调用：   

```python
import hello_object

dic1 = {"whatever":1}

hello_object.test2(dic1)

arr = np.array([1,2,3],dtype = float32)

print (arr.dtype)
print (arr)

hello_object.add_arr_1(arr,1,3)
print (arr)
```

## 参考

[1]: [Boost.Python 官方文档](https://www.boost.org/doc/libs/1_65_1/libs/python/doc/html/tutorial/index.html)    