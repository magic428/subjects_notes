# Python 调用普通 C 函数 
> Python与C/C++相互调用
Python 模块和 C/C++ 的动态库间相互调用在实际的应用中会有所涉及，在此作一总结。   

## 1. python 调用 C 动态链接库    
Python 调用 C 的动态库比较简单，不经过任何封装打包成 so，再使用 python 的 ctypes 调用即可。   
1.1 C 语言文件：`pycall.c`    
```
/***gcc -o libpycall.so -shared -fPIC pycall.c*/  
#include <stdio.h>  
#include <stdlib.h>  
int foo(int a, int b)  
{  
  printf("you input %d and %d\n", a, b);  
  return a+b;  
}  
```
1.2 编译生成动态库    
使用下面的命令编译生成动态库 libpycall.so：  
```
gcc -o libpycall.so -shared -fPIC pycall.c
```
使用 g++ 编译生成 C 动态库的代码中的函数或者方法时，需要使用 extern "C" 来进行编译。   
1.3 调用动态库的文件
```python
# pycall.py   

import ctypes  
ll = ctypes.cdll.LoadLibrary   
lib = ll("./call_c/libpycall.so")    
ret = lib.foo(1, 3)  
print "return val: " + str(ret)  
```
1.4 运行结果：
```
you input 1 and 3
return val: 4
```
## 2. Python 调用 C++(类) 动态链接库    
需要 extern "C" 来辅助，也就是说还是只能调用 C 函数，不能直接调用方法，但是能解析 C++ 方法。如果不使用 extern "C"，构建后的动态链接库没有这些函数的符号表。   
2.1 C++ 类文件   
```cpp
/* pycall_cls.cpp */

#include <iostream>  
using namespace std;  
  
class TestLib  
{  
    public:  
        void display();  
        void display(int a);  
};  
void TestLib::display() {  
    cout << "First display" << endl;  
}  
  
void TestLib::display(int a) {  
    cout << "Second display: " << a << endl;  
}  
extern "C" {  
    TestLib obj;  

    void display() 
    {  
        obj.display();   
    }  

    void display_int() 
    {  
        obj.display(2);   
    }  
}  
```
2.2 编译生成动态库   
使用下面的命令编译生成动态库 pycall_cls.so：   
```bash
g++ -o libpycall_cls.so -shared -fPIC pycall_cls.cpp 
```
2.3 调用动态库的文件    
```python
# pycall_cls.py

import ctypes  
so = ctypes.cdll.LoadLibrary   
lib = so("./call_c/libpycall_cls.so")   
print 'display()'  
lib.display()  
print 'display(2)'  
lib.display_int()  
```
2.4 运行结果    
```
display()
First display
display(2)
Second display:2
```

## 3. Python 调用 C/C++ 可执行程序   
3.1 C/C++程序   
```cpp
/* main.cpp */
#include <iostream>  
using namespace std;  
int test()  
{  
    int a = 10, b = 5;  
    return a+b;  
}  
int main()  
{  
    cout << "---begin---" << endl;  
    int num = test();  
    cout << "num=" << num << endl;  
    cout << "---end---" << endl;  
}
```
3.2 编译成二进制可执行文件   
```bash
g++ -o testmain main.cpp
```
3.3 调用程序
```python
# main.py   

import commands  
import os  
main = "./testmain"  
if os.path.exists(main):  
    rc, out = commands.getstatusoutput(main)  
    print 'rc = %d, \nout = %s' % (rc, out)  
  
print '*'*10  
f = os.popen(main)    
data = f.readlines()    
f.close()    
print data  
  
print '*'*10  
os.system(main)  
```
3.4 运行结果   
```
rc = 0, 
out = ---begin---
num=15
---end---
**********
['---begin---\n', 'num=15\n', '---end---\n']
**********
---begin---
num=15
---end---
```
## 4. 扩展 Python (C++为Python编写扩展模块)   
所有能被整合或导入到其它 python 脚本的代码，都可以被称为扩展。可以用 Python 来写扩展，也可以用 C和C++ 之类的编译型的语言来写扩展。Python在设计之初就考虑到要让模块的导入机制足够抽象。抽象到让使用模块的代码无法了解到模块的具体实现细节。    
Python 的可扩展性具有的优点：方便为语言增加新功能、具有可定制性、代码可以实现复用等。   
为 Python 创建扩展需要三个主要的步骤：创建应用程序代码->利用样板来包装代码->编译与测试。   
4.1 创建应用程序代码   
```cpp
#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
  
int fac(int n)  
{  
    if (n < 2) return(1); /* 0! == 1! == 1 */  
    return (n)*fac(n-1); /* n! == n*(n-1)! */  
}  
  
char *reverse(char *s)  
{  
    register char t,                    /* tmp */  
            *p = s,                     /* fwd */  
            *q = (s + (strlen(s) - 1)); /* bwd */  
  
    while (p < q)               /* if p < q */  
    {  
        t = *p;         /* swap & move ptrs */  
        *p++ = *q;  
        *q-- = t;  
    }  
    return(s);  
}  
  
int main()  
{  
    char s[BUFSIZ];  
    printf("4! == %d\n", fac(4));  
    printf("8! == %d\n", fac(8));  
    printf("12! == %d\n", fac(12));  
    strcpy(s, "abcdef");  
    printf("reversing 'abcdef', we get '%s'\n", \  
        reverse(s));  
    strcpy(s, "madam");  
    printf("reversing 'madam', we get '%s'\n", \  
        reverse(s));  
    return 0;  
}
```
上述代码中有两个函数，一个是递归求阶乘的函数 fac()；另一个reverse()函数实现了一个简单的字符串反转算法，其主要目的是修改传入的字符串，使其内容完全反转，但不需要申请内存后反着复制的方法。    
4.2 用样板来包装代码   
接口的代码被称为“样板”代码，它是应用程序代码与 Python 解释器之间进行交互所必不可少的一部分。   
样板主要分为 4 步：   
a、包含`Python`的头文件；   
b、为每个模块的每一个函数增加一个型如`PyObject* Module_func()`的包装函数；   
c、为每个模块增加一个型如`PyMethodDef ModuleMethods[]的数组`；    
d、增加模块初始化函数 `void initModule()`。   
```cpp
#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
  
int fac(int n)  
{  
    if (n < 2) return(1);  
    return (n)*fac(n-1);  
}  
  
char *reverse(char *s)  
{  
    register char t,  
            *p = s,  
            *q = (s + (strlen(s) - 1));  
  
    while (s && (p < q))  
    {  
        t = *p;  
        *p++ = *q;  
        *q-- = t;  
    }  
    return(s);  
}  
  
int test()  
{  
    char s[BUFSIZ];  
    printf("4! == %d\n", fac(4));  
    printf("8! == %d\n", fac(8));  
    printf("12! == %d\n", fac(12));  
    strcpy(s, "abcdef");  
    printf("reversing 'abcdef', we get '%s'\n", \  
        reverse(s));  
    strcpy(s, "madam");  
    printf("reversing 'madam', we get '%s'\n", \  
        reverse(s));  
    return 0;  
}  
/*-----------------------我是分割线---------------------------*/
#include "Python.h"  
  
static PyObject *  
Extest_fac(PyObject *self, PyObject *args)  
{  
    int num;  
    if (!PyArg_ParseTuple(args, "i", &num))  
        return NULL;  
    return (PyObject*)Py_BuildValue("i", fac(num));  
}  
  
static PyObject *  
Extest_doppel(PyObject *self, PyObject *args)  
{  
    char *orig_str;  
    char *dupe_str;  
    PyObject* retval;  
  
    if (!PyArg_ParseTuple(args, "s", &orig_str))  
        return NULL;  
    retval = (PyObject*)Py_BuildValue("ss", orig_str,  
        dupe_str=reverse(strdup(orig_str)));  
    free(dupe_str);             #防止内存泄漏  
    return retval;  
}  
  
static PyObject *  
Extest_test(PyObject *self, PyObject *args)  
{  
    test();  
    return (PyObject*)Py_BuildValue("");  
}  
  
static PyMethodDef  
ExtestMethods[] =  
{  
    { "fac", Extest_fac, METH_VARARGS },  
    { "doppel", Extest_doppel, METH_VARARGS },  
    { "test", Extest_test, METH_VARARGS },  
    { NULL, NULL },  
};  
  
void initExtest()  
{  
    Py_InitModule("Extest", ExtestMethods);  
} 
```
`Python.h` 头文件在大多数类 Unix 系统中会在`/usr/local/include/python2.x`或`/usr/include/python2.x`目录中，系统一般都会知道文件安装的路径。   
增加包装函数，所在模块名为 `Extest`，那么创建一个包装函数叫 `Extest_fac()`，在 Python 脚本中使用时先 `import Extest` ，然后调用 `Extest.fac()`，当 `Extest.fac()`被调用时，包装函数 `Extest_fac()` 会被调用，包装函数接受一个 Python 的整数参数，把它转为 C 语言的整数，然后调用 C 语言类型的 fac() 函数， 得到一个整型的返回值，最后把这个返回值转为 Python 的整型数做为整个函数调用的结果返回。   
其他两个包装函数 Extest_doppel() 和 Extest_test() 类似。  
从 Python 参数到 C 语言参数的转换用 PyArg_Parse* 系列函数.   
`int PyArg_ParseTuple()`：把 Python 传过来的参数转为 C；   
`int PyArg_ParseTupleAndKeywords()`与`PyArg_ParseTuple()`作用相同，但是同时解析关键字参数；   
它们的用法跟 C 的 sscanf 函数很像，都接受一个字符串流，并根据一个指定的格式字符串进行解析，把结果放入到相应的指针所指的变量中去，它们的返回值为 1 表示解析成功，返回值为0表示失败。   
从 C 到 Python 的转换函数是 `PyObject* Py_BuildValue()`：把 C 的数据转为 Python 的一个对象或一组对象，然后返回之；Py_BuildValue 的用法跟 sprintf 很像，把所有的参数按格式字符串所指定的格式转换成一个 Python 的对象。     
C 与 Python 之间数据转换的转换代码：    
为每个模块增加一个型如`PyMethodDef ModuleMethods[]`的数组，以便于 Python 解释器能够导入并调用它们，每一个数组都包含了函数在 Python 中的名字，相应的包装函数的名字以及一个 `METH_VARARGS` 常量，`METH_VARARGS` 表示参数以 `tuple` 形式传入。   
若需要使用 `PyArg_ParseTupleAndKeywords()` 函数来分析命名参数的话，还需要让这个标志常量与 `METH_KEYWORDS` 常量进行逻辑与运算常量 。数组最后用两个 NULL 来表示函数信息列表到此结束。   
所有工作的最后一部分就是模块的初始化函数，调用 `Py_InitModule()` 函数，并把模块名和 `ModuleMethods[]` 数组的名字传递进去，以便于解释器能正确的调用模块中的函数。   
4.3 编译   
为了让新 Python 的扩展能被创建，需要把它们与 Python 库放在一起编译，distutils 包被用来编译、安装和分发这些模块、扩展和包。   
创建一个 setup.py 文件，编译最主要的工作由 setup() 函数来完成：   
```python
#!/usr/bin/env python  
  
from distutils.core import setup, Extension  
  
MOD = 'Extest'  
setup(name=MOD, ext_modules=[Extension(MOD, sources=['python_ext.c'])])  
```
`Extension()` 第一个参数是(完整的)   扩展的名字，如果模块是包的一部分的话，还要加上用'.'分隔的完整的包的名字。上述的扩展是独立的，所以名字只要写"Extest"就行；   
`sources` 参数是所有源代码的文件列表，只有一个文件 python_ext.c。   
`setup()` 需要两个参数：一个名字参数表示要编译哪个内容；另一个列表参数列出要编译的对象，上述要编译的是一个扩展，故把 ext_modules 参数的值设为扩展模块的列表。   
运行 `python setup.py build` 命令就可以开始编译我们的扩展了，提示部分信息：   
```bash
creating build/lib.linux-x86_64-2.6  
gcc -pthread -shared build/temp.linux-x86_64-2.6/Extest2.o -L/usr/lib64 -lpython2.6 -o build/lib.linux-x86_64-2.6/Extest.so  
```
4.4 导入和测试   
你的扩展会被创建在运行 setup.py 脚本所在目录下的 `build/lib.*`目录中，可以切换到那个目录中来测试模块，或者也可以用命令把它安装到 Python 中：   
```bash
python setup.py install
```
会提示相应信息。   
测试模块：  
```
import Extest
Extest.test()
Extest.fac(5)
Extest.doppel("abcdefg")
```
4.5 引用计数和线程安全    
Python 对象引用计数的宏：Py_INCREF(obj)增加对象 obj 的引用计数，Py_DECREF(obj)减少对象 obj 的引用计数。Py_INCREF() 和 Py_DECREF() 两个函数也有一个先检查对象是否为空的版本，分别为 Py_XINCREF() 和 Py_XDECREF()。   编译扩展的程序员必须要注意，代码有可能会被运行在一个多线程的 Python 环境中。这些线程使用了两个 C 宏 Py_BEGIN_ALLOW_THREADS 和 Py_END_ALLOW_THREADS ，通过将代码和线程隔离，保证了运行和非运行时的安全性，由这些宏包裹的代码将会允许其他线程的运行。   
## 5. C/C++调用Python   
C++ 可以调用 Python 脚本，那么就可以写一些 Python 的脚本接口供 C++ 调用了，至少可以把 Python 当成文本形式的动态链接库， 需要的时候还可以改一改，只要不改变接口。缺点是 C++ 的程序一旦编译好了，再改就没那么方便了。   
5.1 Python脚本   
```python
# pytest.py

#test function  
def add(a,b):  
    print "in python function add"  
    print "a = " + str(a)  
    print "b = " + str(b)  
    print "ret = " + str(a+b)  
    return  
  
def foo(a):  
    print "in python function foo"  
    print "a = " + str(a)  
    print "ret = " + str(a * a)  
    return   
  
class guestlist:  
    def __init__(self):  
        print "aaaa"  
    def p():  
      print "bbbbb"  
    def __getitem__(self, id):  
      return "ccccc"  

def update():  
    guest = guestlist()  
    print guest['aa']  
```
5.2 C++ 代码中调用 Python 脚本   
```cpp
/**g++ -o call_py call_py.cpp -I/usr/include/python2.7 -L/usr/lib/python2.7/config-x86_64-linux-gnu -lpython2.7**/  

#include <Python.h>  
int main(int argc, char** argv)  
{  
    // 初始化Python  
    //在使用Python系统前，必须使用Py_Initialize对其  
    //进行初始化。它会载入Python的内建模块并添加系统路  
    //径到模块搜索路径中。这个函数没有返回值，检查系统  
    //是否初始化成功需要使用Py_IsInitialized。  
    Py_Initialize();  
  
    // 检查初始化是否成功  
    if ( !Py_IsInitialized() ) {  
        return -1;  
    }  
    // 添加当前路径  
    //把输入的字符串作为Python代码直接运行，返回0  
    //表示成功，-1表示有错。大多时候错误都是因为字符串  
    //中有语法错误。  
    PyRun_SimpleString("import sys");  
    PyRun_SimpleString("print '---import sys---'");   
    PyRun_SimpleString("sys.path.append('./')");  
    PyObject *pName,*pModule,*pDict,*pFunc,*pArgs;  
  
    // 载入名为pytest的脚本  
    pName = PyString_FromString("pytest");  
    pModule = PyImport_Import(pName);  
    if ( !pModule ) {  
        printf("can't find pytest.py");  
        getchar();  
        return -1;  
    }  
    pDict = PyModule_GetDict(pModule);  
    if ( !pDict ) {  
        return -1;  
    }  
  
    // 找出函数名为add的函数  
    printf("----------------------\n");  
    pFunc = PyDict_GetItemString(pDict, "add");  
    if ( !pFunc || !PyCallable_Check(pFunc) ) {  
        printf("can't find function [add]");  
        getchar();  
        return -1;  
     }  
  
    // 参数进栈  
    PyObject *pArgs;  
    pArgs = PyTuple_New(2);  
  
    //  PyObject* Py_BuildValue(char *format, ...)  
    //  把C++的变量转换成一个Python对象。当需要从  
    //  C++传递变量到Python时，就会使用这个函数。此函数  
    //  有点类似C的printf，但格式不同。常用的格式有  
    //  s 表示字符串，  
    //  i 表示整型变量，  
    //  f 表示浮点数，  
    //  O 表示一个Python对象。  
  
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("l",3));  
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("l",4));  
  
    // 调用Python函数  
    PyObject_CallObject(pFunc, pArgs);  
  
    //下面这段是查找函数foo 并执行foo  
    printf("----------------------\n");  
    pFunc = PyDict_GetItemString(pDict, "foo");  
    if ( !pFunc || !PyCallable_Check(pFunc) ) {  
        printf("can't find function [foo]");  
        getchar();  
        return -1;  
     }  
  
    pArgs = PyTuple_New(1);  
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("l",2));   
  
    PyObject_CallObject(pFunc, pArgs);  
       
    printf("----------------------\n");  
    pFunc = PyDict_GetItemString(pDict, "update");  
    if ( !pFunc || !PyCallable_Check(pFunc) ) {  
        printf("can't find function [update]");  
        getchar();  
        return -1;  
     }  
    pArgs = PyTuple_New(0);  
    PyTuple_SetItem(pArgs, 0, Py_BuildValue(""));  
    PyObject_CallObject(pFunc, pArgs);       
  
    Py_DECREF(pName);  
    Py_DECREF(pArgs);  
    Py_DECREF(pModule);  
  
    // 关闭Python  
    Py_Finalize();  
    return 0;  
}  
``` 
5.3 C++编译成二进制可执行文件   
```bash
g++ -o call_py call_py.cpp -I/usr/include/python2.7 -L/usr/lib/python2.7/config-x86_64-linux-gnu -lpython2.7
```
编译选项需要手动指定 Python 的 include 路径和链接接路径（Python 版本号根据具体情况而定）。   
5.4 运行结果   
```
---import sys---
hello py_cpp
----------------------
```
## 6. 总结
6.1 Python和C/C++的相互调用仅是测试代码，具体的项目开发还得参考Python的API文档。   
6.2 两者交互，C++可为Python编写扩展模块，Python也可为C++提供脚本接口，更加方便于实际应用。   
