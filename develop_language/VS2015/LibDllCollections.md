# Windows 开发中的动态库和静态库问题汇总  

## 1. 静态库和动态库的区别  

动态链接库 (DLL) 是作为共享函数库的可执行文件. 动态链接提供了一种方法, 使进程可以调用不属于其可执行代码的函数. 函数的可执行代码位于一个 DLL 中, 该 DLL 包含一个或多个已被编译、链接并与使用它们的进程分开存储的函数. DLL 还有助于共享数据和资源. 多个应用程序可同时访问内存中单个 DLL 副本的内容.   

动态链接允许可执行模块 (.dll 文件或 .exe 文件) 仅包含在运行时定位 DLL 函数的可执行代码所需的信息. 
在静态链接中, 链接器从静态链接库获取所有被引用的函数, 并将库同代码一起放到可执行文件中. 

**不足之处**:  

(1) 使用静态链接生成的可执行文件体积较大, 包含相同的公共代码, 造成浪费;   
(2) 使用动态链接库的应用程序不是自完备的, 它依赖的DLL模块也要存在, 如果使用载入时动态链接, 程序启动时发现 DLL 不存在, 系统将终止程序并给出错误信息. 另外动态库的运行速度比静态链接慢.  

## 2. 静态库和动态库中的 lib 有什么区别

- 静态库中的 lib: 该 lib 包含函数代码本身 (即包括函数的索引, 也包括实现), 在编译时直接将代码加入程序当中;   
- 动态库中的 lib: 为动态连接库 dll 的导入库 (Import Libary), 该 lib 包含了函数所在的 dll 文件和文件中函数位置的信息 (索引), 而函数实现部分的代码位于 dll 中, 在运行时才被加载到进程空间中;  

总之, lib 是编译时用到的, dll 是运行时用到的. 如果要完成源代码的编译, 只需要 lib; 如果要使动态链接的程序运行起来, 只需要 dll.   

## 3. _declspec(dllexport)/(dllimport) 之间的区别

__declspec 是 Microsoft VC 中专用的关键字, 它配合着一些属性可以对标准 C/C++ 进行扩充. __declspec 关键字应该出现在声明的前面. 它们是将 DLL 内部的类与函数以及变量导出与导入时使用的.   

**如果动态库中没有使用 __declspec(dllexport), 则最终只会生成 dll, 没有 lib, 说明没有导出任何类与函数以及变量.**  

二者的主要区别在于:   

类, 函数和变量可以作为 (dllexport) 关键字声明的内容, (dllexport) 的作用是将其后声明的 DLL 中的内容 (类, 函数和变量) 暴露出来供其他应用程序使用.   

dllimport 关键字是在外部程序需要使用 DLL 内相关内容时使用的关键字. (dllimport) 作用是把 DLL 中的相关代码插入到应用程序中.   

_declspec(dllexport) 与 _declspec(dllimport) 是相互呼应, 只有在 DLL 内部用 dllexport 作了声明, 才能在外部函数中用 dllimport 导入相关代码. 实际上, 在应用程序访问 DLL 时, 实际上就是应用程序中的导入函数与 DLL 文件中的导出函数进行链接.   

dll 的调用不一定需要 .lib 引入库. dll 调用分为两种方式, 一是隐式链接, 二是显式链接.

- 隐式链接: 隐式链接需要使用 .lib , 这就需要在编译 .dll 的时候使用 dllexport 生成这个 .lib 文件, .lib 文件提供了有关 DLL 的名称和 DLL 函数的链接地址, 在应用程序编译时不需要显式地将 DLL 加载到内存;  
- 显式链接: 就是使用 LoadLibrary() API 函数来实现动态加载, 因此, 不需要 lib 这样的引入库.  

如果你要发布一个 dll 形式的 SDK, 就很有必要将头文件, dll 文件连同引入库 lib 一起发布, 以方便使用者调用.  

## 4. dll 中导出类的 static 成员变量问题   

在 DLL 编写时, 使用 __declspec(dllexport) 的作用, 就是为了省掉在 DEF 文件中手工定义导出哪些函数的一个方法. 当然, 如果你的 DLL 里全是 C++ 的类的话, 你无法在 DEF 里指定导出的类, 只能用 __declspec(dllexport) 导出类.  

但是, 对于__declspec(dllimport), MSDN 文档里面的说明让人感觉有点奇怪, 先来看看 MSDN 里面是怎么说的:   

> " 不使用 __declspec(dllimport) 也能正确编译代码, 但使用 __declspec(dllimport) 使编译器可以生成更好的代码. 编译器之所以能够生成更好的代码, 是因为它可以确定函数是否存在于 DLL 中, 这使得编译器可以生成跳过间接寻址级别的代码, 而这些代码通常会出现在跨 DLL 边界的函数调用中. 但是, 必须使用 __declspec(dllimport) 才能导入 DLL 中使用的变量".   
`

初看起来, 这段话前面的意思是, 不用它也可以正常使用 DLL 的导出库, 但最后一句话又说, 必须使用 __declspec(dllimport) 才能导入 DLL 中使用的变量这个是什么意思?  
 
那我就来试验一下, 假定, 你在 DLL里只导出一个简单的类, 注意, 我假定你已经在项目属性中定义了 `SIMPLEDLL_EXPORT` 宏开关.  

```cpp
// ----------------------------------------------------
// SimpleDLLClass.h
#ifdef SIMPLEDLL_EXPORT
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT
#endif

class DLL_EXPORT SimpleDLLClass
{
public:
　　SimpleDLLClass();
　　virtual ~SimpleDLLClass();
　　virtual getValue() { return m_nValue;};
private:
　　int m_nValue;
};

// ----------------------------------------------------
// SimpleDLLClass.cpp
#include "SimpleDLLClass.h"

SimpleDLLClass::SimpleDLLClass()
{
    m_nValue = 0;
}

SimpleDLLClass::~SimpleDLLClass()
{
}
```

然后你在自己的工程 APP 中使用这个 DLL 类, `#include SimpleDLLClass.h` 时, 你的 APP 的项目不用定义 `SIMPLEDLL_EXPORT` 宏开关. 因此 `DLL_EXPORT` 宏就为空. 

此时在 APP 中可以正常使用该动态库, 不会遇到问题. 这正好对应 MSDN 上说的 __declspec(dllimport) 定义与否都可以正常使用. 但我们也没有遇到变量不能正常使用呀.  

那好, 我们改一下 SimpleDLLClass, 把它的 m_nValue 改成 static, 然后在 cpp 文件中加一行:  
 

```cpp
int SimpleDLLClass::m_nValue = 0;
```

如果你不知道为什么要加这一行, 那就回去看看 C++ 的基础. 改完之后, 再去 LINK 一下你的 APP, 看结果如何, 结果是 LINK 告诉你找不到这个 m_nValue. 明明已经定义了, 为什么又没有了??  肯定是因为我把 m_nValue 定义为 static 的原因. 但如果我一定要使用 Singleton 的 Design Pattern 的话, 那这个类肯定是要有一个静态成员, 那该怎么办? 如果你有 Platform SDK, 用里面的 depend 程序看一下, DLL 中又的确是有这个 m_nValue 导出的呀.  

再回去看看我引用 MSDN 的那段话的最后一句.  那我们再改一下 SimpleDLLClass.h, 把那段改成下面的样子:  

```cpp
#ifdef SIMPLEDLL_EXPORT
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __declspec(dllimport)
#endif
```

再 LINK, 一切正常. 原来 __declspec(dllimport) 是为了更好的处理类中的静态成员变量的, 如果没有静态成员变量, 那么有没有这个__declspec(dllimport) 都无所谓.   

----------------------------

## 5. 用 VS SDK 中的 dumpbin.exe 查看 DLL 导出的符号   

开始 | 所有程序 | Microsoft Visual Studio 2015 | Visual Studio Tools  | "Visual Studio 命令提示(2015)"  

打开后输入:  

dumpbin -exports D:\C_engineering\DLL1\Debug\DLL1.dll   

这里要输入需要查看的 DLL 的绝对地址, 然后就能看到返回值了:  

```
File Type: DLL

  Section contains the following exports for dll_lib.dll

    00000000 characteristics
    5DA95A9C time date stamp Fri Oct 18 14:24:28 2019
        0.00 version
           1 ordinal base
           8 number of functions
           8 number of names

    ordinal hint RVA      name

          1    0 00001320 ??1Detector@@QEAA@XZ
          2    1 000016B0 ?createNew@Detector@@SAPEAV1@AEAV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@HHHH@Z
          4    3 000018A0 ?get_net_color_depth@Detector@@QEBAHXZ
          5    4 000018B0 ?get_net_height@Detector@@QEBAHXZ
          6    5 000018C0 ?get_net_width@Detector@@QEBAHXZ
          7    6 00005000 ?maxSize@Detector@@2HA
          8    7 000018D0 ?sum@@YAHHH@Z

  Summary

        1000 .data
        1000 .pdata
        2000 .rdata
        1000 .reloc
        1000 .rsrc
        2000 .text
```

其中包含了 DLL 中使用 _declspec(dllexport) 声明的所有函数, 类和变量.  

## 6. DLL 导出函数的两种方式  

DLL 可以使用两种方法将公共符号导入到应用程序中或从 DLL 导出函数:   

- 生成 DLL 时使用模块定义 (.DEF) 文件;  
- 在主应用程序的函数定义中使用 __declspec(dllimport) 或 __declspec(dllexport) 关键字.  

**使用 .DEF 文件**: 模块定义 (.DEF) 文件是包含一个或多个描述各种 DLL 属性的 Module 语句的文本文件. 如果不使用 __declspec(dllimport) 或 __declspec(dllexport) 导出 DLL 函数, 则 DLL 需要 .DEF 文件.  

**使用 __declspec**: 32 位版的 Visual C++ 用 __declspec(dllimport) 和 __declspec(dllexport) 取代以前在 16 位版的 Visual C++ 中使用的 __export 关键字.   
