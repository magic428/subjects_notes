#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <iostream>
#include <string>
#include <numpy/arrayobject.h>
#include <boost/python.hpp>

using namespace std;
using namespace boost::python;
namespace bp = boost::python;


char const * greet()
{
    return "hello world";
}

// struct World
// {
//     void set(string msg) { this->msg = msg; }

//     string greet() { return "hello " + msg; }
//     string msg;
// };

struct World
{
    World(){} // 构造函数
    World(string msg):msg(msg){} //增加构造函数
    World(double a, double b):a(a),b(b) {} //另外一个构造函数
    void set(string msg) { this->msg = msg; }
    string greet() { return "hello " + msg; }
    double sum_s() { return a + b; }
    string msg;
    double a;
    double b;
};

struct Var
{
    Var(string name): name(name){}
    string const name;

    float value;
};

struct Num
{
    Num(){}
    float get() const { return val; }
    void set(float val) { this->val = val; }
    float val;
};

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

Base * factory() { return new Derived(); }

/**
 *    在 VS2015 update3 中下面的额外的代码如果去掉会报错, Ignore it if you are using linux.  
 *    解决地址：*
 *     stackoverflow.com/questions/38261530/unresolved-external-symbols-since-visual-studio-2015-update-3-boost-python-link/38291152#382* 91152
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

struct VBase
{
    virtual ~VBase() {}
    virtual int f() { return 0; };
};


struct BaseWrap : VBase, wrapper<VBase>
{
    int f()
    {
        if (override f = this->get_override("f"))
            return f(); // 如果函数进行重载了，则返回重载的
        return VBase::f(); // 否则返回基类
    }
    int default_f() { return this->VBase::f(); }
};


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

// overload global functions with default paramters
void foo(int a, char b = 1, unsigned c = 2, double d = 3)
{
    return;
}


// overload member functions with default paramters
struct george
{
    void wack_em(int a, int b = 0, char c = 'x')
    {
        return;
    }

};

// PyObject 
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
    // dict d = extract<dict>(x);   //  dict d = extract<dict>(x.attr("__dict__"));
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

BOOST_PYTHON_FUNCTION_OVERLOADS(foo_overloads, foo, 1, 4); // 参数个数的最小为 1，最大为 4
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(george_overloads, wack_em, 1, 3); // 参数个数的最小为1，最大为3

BOOST_PYTHON_MODULE(hello_ext) // 导出的 module 名字
{
    bp::def("greet", greet);

    // default constructor
    // bp::class_<World>("World")
    //     .def("greet", &World::greet)
    //     .def("set", &World::set);
   
   // Export class
    bp::class_<World>("World")
        .def(init<string>()) 
        .def(init<double,double>()) // expose another construct
        .def("greet", &World::greet)
        .def("set", &World::set)
        .def("sum_s", &World::sum_s);

    // Export member of class
    bp::class_<Var>("Var", init<string>())
        .def_readonly("name", &Var::name) // 只读
        .def_readwrite("value", &Var::value); // 读写

    // Export method of class
    bp::class_<Num>("Num")
        .add_property("rovalue", &Num::get) // 对外：只读
        .add_property("value", &Num::get, &Num::set);// 对外读写 .value 值会改变 .rovalue值，存储着同样的数据。

    // Export Derived and base class
    bp::class_<Base>("Base")
        .def("getName", &Base::getName)
        .def_readwrite("str", &Base::str);

    bp::class_<Derived, bases<Base> >("Derived")
        .def("getName", &Derived::getName)
        .def_readwrite("str", &Derived::str);

    bp::def("b", b);
    bp::def("d", d);
    bp::def("factory", factory, return_value_policy<manage_new_object>());  //    
    
    bp::class_<BaseWrap, boost::noncopyable>("VBase")
        .def("f", &VBase::f, &BaseWrap::default_f);

    // overload
    bp::class_<X>("X")
        .def("f", fx1)
        .def("f", fx2)
        .def("f", fx3)
        .def("f", fx4);

    // overload with default paramters
    // 实现导出带有默认参数的函数
    bp::def("foo", foo, foo_overloads()); 

    bp::class_<george>("george")
        .def("wack_em", &george::wack_em, george_overloads());

    bp::def("test", test);
    bp::def("test2", test2);
    bp::def("add_arr_1", add_arr_1);
}
