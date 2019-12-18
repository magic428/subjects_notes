# CMake 如何查找库路径  

> 原文地址: https://blog.csdn.net/bytxl/article/details/50637277  

**这篇文章的描述非常的官方化, 可以进行二次加工后深入理解.**  

如果你的代码使用了外部库 (external libraries) , 并且你事先不知道这些库的头文件和库文件在当前平台的位置. 那么如何自动地将库的路径和头文件路径添加到编译命令中呢?   

CMake 通过 find_package 命令可以完成这个功能.  

本文简要介绍如何在 CMake工程中使用外部库, 然后介绍如何写自己的 find module.   

Contents 

- 1. 使用外部库 
- 2. 使用CMake没有提供 find module 的外部库 
- 3. package finding 的工作机制 
- 4. 利用 pkg-config 
- 5. 编写 find modules 
- 6. 执行和缓存 
- 7. 常见 bugs

1.使用外部库  

CMake本身提供了很多常用库的module来帮助查找该库. 使用命令 cmake –help-module-list 
可以查看提供了哪些module, 或者到module的存放路径下查看, 在ubuntu Linux上的位置通常在／usr/share/cmake/Modules/下. 

2.使用CMake本身没有提供module的外部库
假设你要使用LibXML++库, 但是CMake没有提供该库的module. 同时你发现网上有其他人提供了FindLibXML++.cmake.  
此时可以下载该文件并把它丢到CMake module路径下. 

有组件的包 (components) 
有些库不是只有一个整体组成, 可能包含多个依赖库和组件. 一个明显的例子是Qt 库, 包含组建QtOpenGL和QtXML. 为了使用这些组建, 使用如下命令

find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
1
在包为可选项时可以省略关键词REQUIRED. 这样就可以使用变量<PACKAGE>_<COMPONENTS>_FOUND, 例如Qt_QtXml_FOUND, 来检查对应组建是否被找到. 

3.包查找的机制
find_package()命令会查找moudle目录下的Find <name>.cmake文件. 

首先, CMake查找${CMAKE_MODULE_PATH}里的所有文件夹. 

然后,  CMake查找自己的module目录<CMAKE_ROOT>/share/cmake-x.y/Modules/.

如果找不到上述文件, CMake会查找<Name>Config.cmake或者<lower-case-name&gt-config.cmake文件, 这些应该是由库来安装的. 

前者叫做module mode后者叫做config module. 不论是哪个mode被使用, 如果一个package 被找到了, 都会生成一系列的变量.  
<Name>_FOUND 
<Name>_INCLUDE_DIRS 或 <Name>_INCLUDES 
<Name>_LIBRARIES 或<Name>_LIBS 
<Name>_DEFINITIONS

所有这些变量都在Find <name>.cmake文件里替换. 



find_package与CMake如何查找链接库详解

时间  2016-02-05
栏目 应用数学
原文   http://blog.csdn.net/bytxl/article/details/50637277

如果编译软件使用了外部库, 事先并不知道它的头文件和链接库的位置. 得在编译命令中加上包含它们的查找路径. CMake使用 find_package 命令来解决这个问题. 本文讨论了如何在CMake项目中使用外部库, 以及如何给没有查找模块的库写一个. 

## 1 FIND_PACKAGE

```cpp
FIND_PACKAGE( <name> [version] [EXACT] [QUIET] [NO_MODULE] [ [ REQUIRED | COMPONENTS ] [ componets... ] ] )
```

用来调用预定义在 CMAKE_MODULE_PATH 下的 `Find<name>.cmake` 模块. 也可以自己定义 `Find<name>` 模块, 将其放入工程的某个目录中, 通过 SET(CMAKE_MODULE_PATH dir) 设置查找路径, 供工程 FIND_PACKAGE 使用.  

这条命令执行后, CMake 会到变量 CMAKE_MODULE_PATH 指示的目录中查找文件 Findname.cmake 并执行.  

- **version 参数**  

    需要一个版本号, 它是正在查找的包应该兼容的版本号 (格式是major[.minor[.patch[.tweak]]]) .  

- **EXACT选项**  

    要求版本号必须精确匹配. 如果在 find-module 内部对该命令的递归调用没有给定 [version] 参数, 那么 [version] 和 EXACT 选项会自动地从外部调用前向继承. 对版本的支持目前只存在于包和包之间 (详见下文) .   

- **QUIET 参数**  

    会禁掉包没有被发现时的警告信息. 对应于 `Find<name>.cmake` 模块中的 NAME_FIND_QUIETLY.  

- **REQUIRED 参数**  

    其含义是指是否是工程必须的, 表示如果报没有找到的话, cmake 的过程会终止, 并输出警告信息. 对应于 `Find<name>.cmake` 模块中的 NAME_FIND_REQUIRED 变量.  

- **COMPONENTS 参数**  

    在 REQUIRED 选项之后, 或者如果没有指定 REQUIRED 选项但是指定了 COMPONENTS 选项, 在它们的后面可以列出一些与包相关 (依赖) 的部件清单 (components list)   

    示例: `FIND_PACKAGE( libdb_cxx REQUIRED)`  

### 1.1 包查找是如何工作的  

find_package() 命令会在模块路径中寻找 `Find<name>.cmake`, 这是查找库的一个典型方式. 首先 CMake 查看 ${CMAKE_MODULE_PATH} 中的所有目录, 然后再查看它自己的模块目录 `<CMAKE_ROOT>/share/cmake-x.y/Modules/`.   

如果没找到这样的文件, 会寻找 `<Name>Config.cmake` 或者 `<lower-case-name>-config.cmake`, 它们是假定库会安装的文件 (但是目前还没有多少库会安装它们). 不做检查, 直接包含安装的库的固定值. 

前面的称为**模块模式**, 后面的称为**配置模式**.   

**配置模式**的文件的编写见 [这里的文档](https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file). 可能还会用到 [importing and exporting targets](https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/Exporting-and-Importing-Targets) 这篇文档.  

**模块系统好像还没有文档, 所以本文主要讨论这方面的内容**.   

不管使用哪一种模式, 只要找到包, 就会定义下面这些变量:   

```js
<NAME>_FOUND
<NAME>_INCLUDE_DIRS or <NAME>_INCLUDES
<NAME>_LIBRARIES or <NAME>_LIBRARIES or <NAME>_LIBS
<NAME>_DEFINITIONS
```

这些都在 `Find<name>.cmake` 文件中.  

现在, 在你的代码 (要使用库 `<name>` 的代码) 的顶层目录中的 CMakeLists.txt 文件中, 我们检查变量 `<NAME>_FOUND` 来确定包是否被找到. 大部分包的这些变量中的包名是全大写的, 如 LIBFOO_FOUND , 有些包则使用包的实际大小写, 如 LibFoo_FOUND . 如果找到这个包, 我们用 `<NAME>_INCLUDE_DIRS` 调用 include_directories() 命令, 用 `<NAME>_LIBRARIES` 调用 target_link_libraries() 命令.  

这些约定的文档在 CMake 模块目录中的 readme.txt 文件中.  

REQUIRED 和其他可选的 find_package 的参数被 find_package 传给模块, 模块由此确定操作.  

用户代码总体上应该使用上述的简单调用格式查询需要的包.   

本文档的剩余部分则详述了 find_package 的完整命令格式以及具体的查询过程. 期望通过该命令查找并提供包的项目维护人员, 我们鼓励你能继续读下去.  


该命令在搜索包时有两种模式: “模块”模式和“配置”模式. 当该命令是通过上述的精简格式调用的时候, 用的就是模块模式. 在该模式下, CMake 搜索所有名为 `Find<package>.cmake` 的文件, 这些文件的路径由安装CMake 时指定的 CMAKE_MODULE_PATH 变量指定. 如果查找到了该文件, 它会被 CMake 读取并被处理. 该模式对查找包, 检查版本以及生成任何别的必须信息负责. 许多查找模块 (find-module) 仅仅提供了有限的, 甚至根本就没有对版本化的支持； 具体信息查看该模块的文档. 如果没有找到任何模块, 该命令会进入配置模式继续执行. 

完整的配置模式下的命令格式是:  

```cpp
find_package( <package> [version] [EXACT] [QUIET]
              [[REQUIRED|COMPONENTS] [components...]] [NO_MODULE]
              [NO_POLICY_SCOPE]
              [NAMES name1 [name2 ...]]
              [CONFIGS config1 [config2 ...]]
              [HINTS path1 [path2 ... ]]
              [PATHS path1 [path2 ... ]]
              [PATH_SUFFIXES suffix1 [suffix2 ...]]
              [NO_DEFAULT_PATH]
              [NO_CMAKE_ENVIRONMENT_PATH]
              [NO_CMAKE_PATH]
              [NO_SYSTEM_ENVIRONMENT_PATH]
              [NO_CMAKE_PACKAGE_REGISTRY]
              [NO_CMAKE_BUILDS_PATH]
              [NO_CMAKE_SYSTEM_PATH]
              [CMAKE_FIND_ROOT_PATH_BOTH |
              ONLY_CMAKE_FIND_ROOT_PATH |
              NO_CMAKE_FIND_ROOT_PATH])
```
　　
NO_MODULE 可以用来明确地跳过模块模式. 它也隐含指定了不使用在精简格式中使用的那些选项.  

配置模式试图查找一个由待查找的包提供的配置文件的位置. 包含该文件的路径会被存储在一个名为`<package>_DIR` 的 cache 条目里. 默认情况下, 该命令搜索名为 `<package>` 的包. 如果指定了 NAMES 选项, 那么其后的 names 参数会取代 `<package>` 的角色. 该命令会为每个在 names 中的 name 搜索名为 `<name>Config.cmake` 或者 `<name全小写>-config.cmake` 的文件. 

通过使用 CONFIGS 选项可以改变可能的配置文件的名字. 以下描述搜索的过程. 如果找到了配置文件, 它将会被CMake 读取并处理. 由于该文件是由包自身提供的, 它已经知道包中内容的位置. 配置文件的完整地址存储在cmake 的变量 `<package>_CONFIG` 中.  

所有 CMake 要处理的配置文件将会搜索该包的安装信息, 并且将该安装匹配的适当版本号 (appropriate version) 存储在 cmake 变量 `<package>_CONSIDERED_CONFIGS` 中, 与之相关的版本号 (associated version) 将被存储在 `<package>_CONSIDERED_VERSIONS` 中.  

如果没有找到包配置文件, CMake 将会生成一个错误描述文件, 用来描述该问题——除非指定了 QUIET 选项. 如果指定了 REQUIRED 选项, 并且没有找到该包, 将会报致命错误, 然后配置步骤终止执行. 如果设置了`<package>_DIR` 变量被设置了, 但是它没有包含配置文件信息, 那么 CMake 将会直接无视它, 然后重新开始查找.  

如果给定了 [version] 参数, 那么配置模式仅仅会查找那些在命令中请求的版本 (格式是 `major[.minor[.patch[.tweak]]]`) 与包请求的版本互相兼容的那些版本的包. 如果指定了 EXACT 选项, 一个包只有在它请求的版本与 [version] 提供的版本精确匹配时才能被找到. CMake 不会对版本数的含义做任何的转换. 包版本号由包自带的版本文件来检查. 对于一个备选的包配置文件 `<config-file>.cmake`, 对应的版本文件的位置紧挨着它, 并且名字或者是 `<config-file>-version.cmake` 或者是 `<config-file>Version.cmake`. 如果没有这个版本文件, 那么配置文件就会认为不兼容任何请求的版本. 当找到一个版本文件之后, 它会被加载然后用来检查 (find_package) 请求的版本号. 版本文件在一个下述变量被定义的嵌套域中被加载:  

```js
PACKAGE_FIND_NAME          = <package> 名字. 
PACKAGE_FIND_VERSION       = 请求的完整版本字符串
PACKAGE_FIND_VERSION_MAJOR = 如果被请求了, 那么它是major版本号, 否则是0. 
PACKAGE_FIND_VERSION_MINOR = 如果被请求了, 那么它是minor版本号, 否则是0. 
PACKAGE_FIND_VERSION_PATCH = 如果被请求了, 那么它是patch版本号, 否则是0. 
PACKAGE_FIND_VERSION_TWEAK = 如果被请求了, 那么它是tweak版本号, 否则是0. 
PACKAGE_FIND_VERSION_COUNT = 版本号包含几部分, 0 到 4. 
```

版本文件会检查自身是否满足请求的版本号, 然后设置了下面这些变量:  

```js
PACKAGE_VERSION            = 提供的完整的版本字符串. 
PACKAGE_VERSION_EXACT      = 如果版本号精确匹配, 返回true. 
PACKAGE_VERSION_COMPATIBLE = 如果版本号相兼容, 返回true. 
PACKAGE_VERSION_UNSUITABLE = 如果不适合任何版本, 返回true. 
```

下面这些变量将会被 find_package 命令检查, 用以确定配置文件是否提供了可接受的版本. 在 find_package 命令返回后, 这些变量就不可用了. 如果版本可接受, 下述的变量会被设置:  

```js
<package>_VERSION       = 提供的完整的版本字符串. 
<package>_VERSION_MAJOR = 如果被请求了, 那么它是major版本号, 否则是0. 
<package>_VERSION_MINOR = 如果被请求了, 那么它是minor版本号, 否则是0. 
<package>_VERSION_PATCH = 如果被请求了, 那么它是patch版本号, 否则是0. 
<package>_VERSION_TWEAK = 如果被请求了, 那么它是tweak版本号, 否则是0. 
<package>_VERSION_COUNT = 版本号包含几部分, 0到4. 
```

然后, 对应的包配置文件才会被加载. 当多个包配置文件都可用时, 并且这些包的版本文件都与请求的版本兼容, 选择哪个包将会是不确定的. 不应该假设 cmake 会选择最高版本或者是最低版本.  (以上的若干段是对find_package 中版本匹配步骤的描述, 并不需要用户干预——译注. ) 

配置模式提供了一种高级接口和搜索步骤的接口. 这些被提供的接口的大部分是为了完整性的要求, 以及在模块模式下, 包被 find-module 加载时供内部使用. 大多数用户仅仅应该调用:  

```js
find_package(<package> [major[.minor]] [EXACT] [REQUIRED|QUIET])
```

来查找包. 鼓励那些需要提供 CMake 包配置文件的包维护人员应该命名这些文件并安装它们, 这样下述的整个过程将会找到它们而不需要使用附加的选项. 

CMake 为包构造了一组可能的安装前缀. 在每个前缀下, 若干个目录会被搜索, 用来查找配置文件. 下述的表格展示了待搜索的路径. 每个条目都是专门为 Windows(W), UNIX(U) 或者 Apple(A) 约定的安装树指定的.  

```js
<prefix>/                                               (W)
<prefix>/(cmake|CMake)/                                 (W)
<prefix>/<name>*/                                       (W)
<prefix>/<name>*/(cmake|CMake)/                         (W)
<prefix>/(share|lib)/cmake/<name>*/                     (U)
<prefix>/(share|lib)/<name>*/                           (U)
<prefix>/(share|lib)/<name>*/(cmake|CMake)/             (U)
```
　　
在支持 OS X 平台和 Application Bundles 的系统上, 包含配置文件的框架或者 bundles 会在下述的路径中被搜索:  

```js
<prefix>/<name>.framework/Resources/                    (A)
<prefix>/<name>.framework/Resources/CMake/              (A)
<prefix>/<name>.framework/Versions/*/Resources/         (A)
<prefix>/<name>.framework/Versions/*/Resources/CMake/   (A)
<prefix>/<name>.app/Contents/Resources/                 (A)
<prefix>/<name>.app/Contents/Resources/CMake/           (A)
```

在所有上述情况下, `<name>` 是区分大小写的, 并且对应于在 `<package>` 或者由 NAMES 给定的任何一个名字.  

这些路径集用来与那些在各自的安装树上提供了配置文件的工程协作. 上述路径中被标记为(W)的是专门为 Windows 上的安装设置的, 其中的 `<prefix>部分` 可能是一个应用程序的顶层安装路径. 那些被标记为(U)的是专门为 UNIX 平台上的安装设置的, 其中的 `<prefix>` 被多个包共用. 这仅仅是个约定, 因此, 所有(W)和(U)路径在所有平台上都仍然会被搜索. 那些被标记为(A)的路径是专门为 Apple 平台上的安装设置的. CMake 变量 CMAKE_FIND_FRAMEWORK 和 CMAKE_FIND_APPBUNDLE 确定了偏好的顺序, 如下所示:  

安装前缀是通过以下步骤被构建出来的. 如果指定了 NO_DEFAULT_PATH 选项, 所有 NO_* 选项都会被激活.  

1、搜索在 cmake 特有的 cache 变量中指定的搜索路径. 这些变量是为了在命令行中用 `-DVAR=value` 选项指定而设计的. 通过指定 NO_CMAKE_PATH 选项可以跳过该搜索路径. 搜索路径还包括:  

```js
CMAKE_PREFIX_PATH
CMAKE_FRAMEWORK_PATH
CMAKE_APPBUNDLE_PATH
```

2、搜索 cmake 特有的环境变量. 这些变量是为了在用户的 shell 配置中进行配置而设计的. 通过指定NO_CMAKE_ENVIRONMENT_PATH 选项可以跳过该路径. 搜索的路径包括:  

```js
<package>_DIR
CMAKE_PREFIX_PATH
CMAKE_FRAMEWORK_PATH
CMAKE_APPBUNDLE_PATH
```
　　
3、搜索 HINTS 选项指定的路径. 这些路径应该是由操作系统内省时计算产生的, 比如由其它已经找到的项的位置而提供的线索. 硬编码的参考路径应该在 PATHS 选项中指定.  

4、搜索标准的系统环境变量. 如果指定了 NO_SYSTEM_ENVIRONMENT_PATH 选项, 这些路径会被跳过. 以 "/bin" 或 "/sbin" 结尾的路径条目会被自动转换为它们的父路径. 搜索的路径包括:   

```js
PATH
```

5、搜索在 CMake GUI 中最新配置过的工程的构建树. 可以通过设置 NO_CMAKE_BUILDS_PATH 选项来跳过这些路径. 这是为了在用户正在依次构建多个相互依赖的工程时而准备的.  

6、搜索存储在 CMake 用户包注册表中的路径. 通过设置 NO_CMAKE_PACKAGE_REGISTRY 选项可以跳过这些路径. 当 CMake 调用 `export(PACKAGE<name>)` 配置一个工程时, 这些路径会被存储在注册表中. 参见export(PACKAGE) 命令的文档阅读更多细节.  

7、搜索在当前系统的平台文件中定义的 cmake 变量. 可以用 NO_CMAKE_SYSTEM_PATH 选项跳过这些路径.  

```js
CMAKE_SYSTEM_PREFIX_PATH
CMAKE_SYSTEM_FRAMEWORK_PATH
CMAKE_SYSTEM_APPBUNDLE_PATH
```
　
8、搜索由 PATHS 选项指定的路径. 这些路径一般是硬编码的参考路径.  

在 Darwin 或者支持 OS X 框架的系统上, cmake 变量 CMAKE_FIND_FRAMEWORK 可以用来设置为空, 或者下述值之一:   

    "FIRST"  - 在标准库或头文件之前查找框架. 在Darwin系统上这是默认选项.  
    "LAST"   - 在标准库或头文件之后查找框架.  
    "ONLY"   - 仅仅查找框架.  
    "NEVER" - 从不查找框架.  

在 Darwin 或者支持 OS X Application Bundles 的系统, cmake 变量 CMAKE_FIND_APPBUNDLE 可以被设置为空或者下面这些值中的一个: 

    "FIRST"  - 在标准库或头文件之前查找application bundles. 在Darwin系统上这是默认选项.  
    "LAST"   - 在标准库或头文件之后查找application bundles.  
    "ONLY"   - 仅仅查找application bundles.  
    "NEVER" - 从不查找application bundles.  

CMake 变量 CMAKE_FIND_ROOT_PATH 指定了一个或者多个优先于其他搜索路径的搜索路径. 该变量能够有效地重新定位在给定位置下进行搜索的根路径. 该变量默认为空. 当使用交叉编译时, 该变量十分有用: 用该变量指向目标环境的根目录, 然后 CMake 将会在那里查找. 默认情况下, 在 CMAKE_FIND_ROOT_PATH 中列出的路径会首先被搜索, 然后是“非根”路径. 该默认规则可以通过设置 CMAKE_FIND_ROOT_PATH_MODE_LIBRARY 做出调整. 在每次调用该命令之前, 都可以通过设置这个变量来手动覆盖默认行为. 如果使用了NO_CMAKE_FIND_ROOT_PATH 变量, 那么只有重定位的路径会被搜索.  

默认的搜索顺序的设计逻辑是按照使用时从最具体到最不具体. 通过多次调用 find_library 命令以及 NO_* 选项, 可以覆盖工程的这个默认顺序:  

```js
find_library(<VAR> NAMES name PATHS paths... NO_DEFAULT_PATH)  
find_library(<VAR> NAMES name)  
```
　
只要这些调用中的一个成功返回, 结果变量就会被设置并且被存储到 cache 中；这样随后的调用都不会再行搜索. 如果那找到的库是一个框架, VAR 将会被设置为指向框架 “<完整路径>/A.framework” 的完整路径. 当一个指向框架的完整路径被用作一个库文件, CMake 将使用 -framework A, 以及 -F<完整路径> 这两个选项将框架连接到目标上.  


参见 cmake_policy() 命令的文档中关于 NO_POLICY_SCOPE 选项讨论. 



## 2 使用 cmake 自带查找模块的外部库  

为了能支持各种常见的库和包, CMake 自带了很多模块. 可以通过命令 cmake --help-module-list  (输入cmake --help, 然后双击 Tab 会有命令提示) 得到你的 CMake 支持的模块的列表:  

```js
cmake version 2.8.12.2
AddFileDependencies
BundleUtilities
CMakeAddFortranSubdirectory
...
CPackRPM
CPackWIX
CTest
CTestScriptMode
CTestUseLaunchers
CheckCCompilerFlag
...
Dart
...
FindALSA
...
FindBLAS
FindBZip2
...
```

或者直接查看模块路径. 比如 Ubuntu linux上, 模块的路径是 /usr/share/cmake/Modules/, 该目录下保存的 .cmake 文件就是 CMake 自带的模块.  

让我们以 bzip2 库为例. CMake中有个 FindBZip2.cmake 模块. 只要使用 find_package(BZip2) 调用这个模块, cmake 会自动给一些变量赋值, 然后就可以在 CMake 脚本中使用它们了. 变量的列表可以查看 cmake 模块文件, 或者使用命令 cmake --help-module FindBZip2:  

```js
cmake version 2.8.12.2
  FindBZip2
       Try to find BZip2

       Once done this will define

         BZIP2_FOUND - system has BZip2
         BZIP2_INCLUDE_DIR - the BZip2 include directory
         BZIP2_LIBRARIES - Link these to use BZip2
         BZIP2_NEED_PREFIX - this is set if the functions are prefixed with BZ2_
         BZIP2_VERSION_STRING - the version of BZip2 found (since CMake 2.8.8)

       Defined in: /usr/share/cmake-2.8/Modules/FindBZip2.cmake
```

比如一个使用 bzip2 的简单程序, 编译器需要知道 bzlib.h 的位置, 链接器需要找到 bzip2 库 (动态链接的话, Unix 上是 libbz2.so 类似的文件, Windows 上是 libbz2.dll ).  

```js
cmake_minimum_required(VERSION 2.8)
project(helloworld)
add_executable(helloworld hello.c)
find_package (BZip2)
if (BZIP2_FOUND)
  include_directories(${BZIP_INCLUDE_DIRS})
  target_link_libraries (helloworld ${BZIP2_LIBRARIES})
endif (BZIP2_FOUND)
```

可以用 cmake 和 make VERBOSE=1 来验证传给编译器和链接器的 flag 是否正确. 也可以用 ldd 或者 dependency walker 之类的工具在编译后验证 helloworld 链接的文件.  

## 3 使用 cmake 没有自带查找模块的外部库  

假设你想要使用 LibXML++ 库. 在写本文时, CMake 还没有一个 libXML++ 的查找模块. 但是可以在网上搜索到一个 (FindLibXML++.cmake) . 在 CMakeLists.txt 中写:  

```js
find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})
set(LIBS ${LIBS} ${LibXML++_LIBRARIES})
```

如果包是可选的, 可以忽略 REQUIRED 关键字, 通过 LibXML++_FOUND 布尔变量来判断是否找到. 检测完所有的库后, 对于链接目标有:  

```js
target_link_libraries(exampleProgram ${LIBS})
```

为了能正常的工作, 需要把 FindLibXML++.cmake 文件放到 CMake 的模块路径 (/usr/share/cmake/Modules/) . 因为 CMake 还不包含它, 需要在项目中指定. 在自己的项目根目录下创建一个 cmake/Modules/ 文件夹, 并且在主 CMakeLists.txt 中包含下面的代码:  

```js
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
```

把刚才的需要用到的 CMake 模块放到这个文件夹下.  

一般来说就是这样. 有些库可能还需要些其他的什么, 所以要再看一下 FindSomething.cmake 文件的文档.  

### 3.1 包含组件的依赖包  

有些库不是一个整体, 还包含一些依赖的库或者组件. 一个典型的例子是 Qt 库, 它其中包含 QtOpenGL 和 QtXml 组件. 使用下面的 find_package 命令来使用这些组件:  

```js
find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
```

如果包是可选的, 这里同样可以忽略 REQUIRED 关键字. 这时可以使用 `<PACKAGE>_<COMPONENT>_FOUND` 变量 (如Qt_QtXml_FOUND) 来检查组件是否被找到. 下面的 find_package 命令是等价的:  

```js
find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
find_package(Qt REQUIRED COMPONENTS QtOpenGL QtXml)
find_package(Qt REQUIRED QtOpenGL QtXml)
```

如果包中的组件有些是必需的, 有些不是, 可以调用 find_package 两次: 

```js
find_package(Qt COMPONENTS QtXml REQUIRED)
find_package(Qt COMPONENTS QtOpenGL)
```

或者也可以不加 REQUIRED 关键字用 find_package 同时查找全部组件, 然后再显式地检查必需的组件:  

```js
find_package(Qt COMPONENTS QtOpenGL QtXml)
if ( NOT Qt_FOUND OR NOT QtXml_FOUND )
  message(FATAL_ERROR "Package Qt and component QtXml required, but not found!")
endif( NOT Qt_FOUND OR NOT QtXml_FOUND )
```

## 4 捎带介绍下 pkg-config

pkg-config 是个用来帮助构建的工具, 它基于记录库文件和头文件位置的 .pc 文件. 主要用在类 Unix 系统上. 可以在[pkg-config](http://www.freedesktop.org/wiki/Software/pkg-config/) 的网站 找到更多的信息.  

CMake 可以利用 pkg-config, 可以在 CMake 的模块目录下的 FindPkgConfig.cmake 文件中找到相关的文档. 这在当你处理一个没有 cmake 脚本的库的时候, 或者遇到 CMake 的查找脚本失效的情况, 非常有帮助. 

但是, 直接使用 pkg-config 的结果需要非常小心. 一个主要原因是对于 ccmake 手动定义的库路径, 可能覆盖到或者发生冲突. 此外, 也有可能 pkg-config 提供了错误的信息 (错误的编辑器等) . 对于这些情况, 让 CMake 不依赖 pkg-config 做检测, 而只用 pkg-config 作为查找路径的提示.  

## 5 编写查找模块

首先, 注意传给 find_package 的名字或者前缀, 是用于全部变量的部分文件名和前缀. 这很重要, 名字必须完全匹配. 不幸的是很多情况下, 即使是 CMake 自带的模块, 也有不匹配的名字, 导致各种问题.  

模块的基本操作应该大体按下面的顺序:  

使用 find_package 检测库依赖的其他的库  

需要转发 QUIETLY 和 REQUIRED 参数 (比如, 如果当前的包是 REQUIRED 的, 它的依赖也应该是) 
可选地使用pkg-config来检测 include/library 的路径 (如果pkg-config可用的话) 
分别使用 find_path 和 find_library 寻找头文件和库文件
pkg-config提供的路径只用来作为查找位置的提示
CMake也有很多其他查找路径是写死的
结果应该保存在 <name>_INCLUDE_DIR 和 <name>_LIBRARY 变量中 (注意不是复数形式) 
设置 <name>_INCLUDE_DIRS 到 <name>_INCLUDE_DIR <dependency1>_INCLUDE_DIRS ...
设置 <name>_LIBRARIES 到 <name>_LIBRARY <dependency1>_LIBRARIES ...
依赖使用复数形式, 包自身使用 find_path 和 find_library 定义的单数形式
调用 find_package_handle_standard_args() 宏来设置 <name>_FOUND 变量, 并打印一条成功或者失败的消息

```js
# - Try to find LibXml2
# Once done this will define
#  LIBXML2_FOUND - System has LibXml2
#  LIBXML2_INCLUDE_DIRS - The LibXml2 include directories
#  LIBXML2_LIBRARIES - The libraries needed to use LibXml2
#  LIBXML2_DEFINITIONS - Compiler switches required for using LibXml2

find_package(PkgConfig)
pkg_check_modules(PC_LIBXML QUIET libxml-2.0)
set(LIBXML2_DEFINITIONS ${PC_LIBXML_CFLAGS_OTHER})

find_path(LIBXML2_INCLUDE_DIR libxml/xpath.h
          HINTS ${PC_LIBXML_INCLUDEDIR} ${PC_LIBXML_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

find_library(LIBXML2_LIBRARY NAMES xml2 libxml2
             HINTS ${PC_LIBXML_LIBDIR} ${PC_LIBXML_LIBRARY_DIRS} )

set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARY} )
set(LIBXML2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibXml2  DEFAULT_MSG
                                  LIBXML2_LIBRARY LIBXML2_INCLUDE_DIR)

mark_as_advanced(LIBXML2_INCLUDE_DIR LIBXML2_LIBRARY )
```

(这一段看不懂, 应该是排版错误, 是5.2的. 

第一行包含了 LibFindMacros. 因为当前 CMake 中并没有, 所以要想生效, 就必须把 LibFindMacros.cmake 文件放到模块路径下. ) 

5.1 查找文件  

然后是实际的检测. 给 find_path 和 find_library 提供一个变量名作为第一个参数. 如果你需要多个 include 路径, 用不同的变量名多次调用 find_path ,  find_library 类似. 

NAMES 指定目标的一个或多个名字, 只要匹配上一个, 就会选中它. 在 find_path 中应该使用主头文件或者C/C++代码导入的文件. 也有可能会包含目录, 比如 alsa/asound.h, 它会使用 asound.h 所在文件夹的父目录作为结果. 

PATHS 用来给CMake提供额外的查找路径, 他不应该用于定义pkg-config以外的东西 (CMake有自己的内置默认值, 如果需要可以通过各种配置变量添加更多) . 如果你不使用它, 忽略这部分内容. 

PATH_SUFFIXES 对于某些系统上的库很有用, 这类库把它们的文件放在类似 /usr/include/ExampleLibrary-1.23/ExampleLibrary/main.h 这样的路径. 这种情况你可以使用 NAMES ExampleLibrary/main.h PATH_SUFFIXESExampleLibrary-1.23 . 可以指定多个后缀, CMake会在所有包含的目录和主目录逐一尝试, 也包括没有后缀的情况. 

库名不包括UNIX系统上使用的前缀, 也不包括任何文件扩展名或编译器标准之类的, CMake会不依赖平台地检测它们. 如果库文件名中有库的版本号, 那么它仍然需要. 

5.2 使用LibFindMacros
有一个 LibFindMacros.cmake 文件, 用来便于写查找模块. 它包含对于每个库都相同的各种 libfind 宏. 使用它的脚本看起来像这样: 

# - Try to find ImageMagick++
# Once done, this will define
#
#  Magick++_FOUND - system has Magick++
#  Magick++_INCLUDE_DIRS - the Magick++ include directories
#  Magick++_LIBRARIES - link these to use Magick++

include(LibFindMacros)

# Dependencies
libfind_package(Magick++ Magick)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Magick++_PKGCONF ImageMagick++)

# Include dir
find_path(Magick++_INCLUDE_DIR
  NAMES Magick++.h
  PATHS ${Magick++_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Magick++_LIBRARY
  NAMES Magick++
  PATHS ${Magick++_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Magick++_PROCESS_INCLUDES Magick++_INCLUDE_DIR Magick_INCLUDE_DIRS)
set(Magick++_PROCESS_LIBS Magick++_LIBRARY Magick_LIBRARIES)
libfind_process(Magick++)
第一行包含了LibFindMacros. 因为当前CMake中并没有, 所以要想生效, 就必须把 LibFindMacros.cmake 文件放到模块路径下. 

libfind_pkg_check_modules 是CMake自己的pkg-config模块的一个用来简化的封装. 你不用再检查CMake的版本, 加载合适的模块, 检查是否被加载, 等等. 参数和传给 pkg_check_modules 的一样: 先是待返回变量的前缀, 然后是包名 (pkg-config的) . 这样就定义了 <prefix>_INCLUDE_DIRS 和其他的这种变量. 

5.2.1 依赖 (可选) 

libfind_package 和 find_package 类似, 区别是它转发 QUIETLY 和 REQUIRED 参数. 第一个参数是当前的包名. 即, 这里Magick++依赖于Magick. 其他参数比如版本可以添加在Magick后面, 它们被转发给CMake的内部 find_package命令. 对你的库依赖的每个库加上其中一行, 并且提供查找模块. 

5.2.2 最后处理

最后的处理, 幸运的是非常程序化, 可以通过 libfind_process 宏和示例中的最后三行来完成. 你需要把<name>_PROCESS_INCLUDES 设置为 <name>_INCLUDE_DIRS 包含的全部变量, 把 <name>_PROCESS_LIBS 设置为<name>_LIBRARIES 包含的全部变量. 然后调用 libfind_process(<name>) 完成剩下的事情. 

只有提供的全部变量都有有效值时, 库被认为 FOUND . 

6 性能和缓存

CMake的变量系统要比初看起来的要复杂得多. 有些变量做了缓存. 做了缓存的变量有内部的 (不能用ccmake编辑) 和外部的 (可以被ccmake修改) . 另外, 外部变量只能在ccmake的高级模式可见. 

默认情况下, 所有变量都是不缓存的. 

为了避免每次执行时都重复检测全部的库, 更为了允许用户在ccmake中设置include目录和库, 需要支持缓存. 幸运的是, 这已经被 find_path 和 find_library 支持, 它们可以缓存它们的变量. 如果变量已经设置为有效值 (比如不是 -NOTFOUND 或者未定义) , 这些函数将什么也不做, 保持旧值. 类似地,  pkg_check_modules 支持结果的内部缓存, 因此不需要每次都再调用pkg-config. 

另一方面, 查找模块的输出值 ( <name>_FOUND, <name>_INCLUDE_DIRS 和 <name>_LIBRARIES ) 不应该被缓存, 否则修改其他缓存的变量就不能改变输出, 这显然是不期望的. 


7 查找模块的常见问题

文件名和变量名中的大小写问题或者名字不匹配问题
模块不检查 <name>_FIND_REQUIRED 或 <name>_FIND_QUIETLY , 因此 find_package 的参数 QUIET 和 REQUIRED 没有效果
没有设置 <name>_INCLUDE_DIRS 和 <name>_LIBRARIES , 只有单数形式的可用


8 链接

Tronic's CMake modules

 http://vtk.org/Wiki/CMake:How_To_Find_Libraries

 http://www.yeolar.com/note/2014/12/16/cmake-how-to-find-libraries/

http://www.yeolar.com/note/2014/12/16/cmake-how-to-find-libraries/
http://www.cnblogs.com/coderfenghc/archive/2012/07/15/2592758.html