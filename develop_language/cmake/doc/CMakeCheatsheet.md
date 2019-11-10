# CMake CheatSheet   

## 基本语法规则  


(1) 使用 `${}` 来引用变量, 但是在 IF 控制语句中是直接使用变量名;  

**但是有一些例外**, 比如在 IF 控制语句中, 变量是直接使用变量名引用, 而不需要 ${}. 如果使用了 ${} 去引用变量, 其实 IF 会去判断名为${}所代表的值的变量, 那当然是不存在的了.   

(2) 指令( 参数1 参数2... ), 参数使用括弧括起, 参数之间使用空格或分号分开.    

(3) 指令是大小写无关的, 参数和变量是大小写相关的; 但推荐你全部使用大写指令.    

(4) 在编写CMakeLists.txt时注意形成统一的风格.


## 系统内置变量    

通过外部编译进行工程构建，HELLO_SOURCE_DIR仍然指代工程路径，即
/backup/cmake/t1
而HELLO_BINARY_DIR则指代编译路径，即/backup/cmake/t1/build

cmake 系统内置变量 

在此之前先介绍两个术语: in source 编译和 out-of-source 编译.  

- in source 编译: 直接在顶层 CMakeLists.txt 所在的路径编译,编译出的文件可能导致整个项目看起来很乱;  
- out-of-source 编译: 创建一个独立的编译目录, 比如 build/, 然后在 build/ 目录中编译, 这样可以保持整个项目的整洁性.  

### (1) PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR    

先给出官方的说明: 

- `PROJECT_SOURCE_DIR` 表示工程的根目录, 即顶层 CMakeLists.txt 所在的目录; 
- `PROJECT_BINARY_DIR` 是指 build 工程时所在的目录;

使用 **in source** 编译时 (即 `cmake ./`), PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR 是相同是的, 即都是当前源码的目录。  

使用 **out-of-source** 编译时, 比如在 build 目录下执行 `cmake ../`, 这时的 PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR 就不同了, 分别是:   

```
PROJECT_BINARY_DIR = /path/to/project-source/build
PROJECT_SOURCE_DIR = /path/to/project-source/
```

另外还有几个变量和这两个变量的含义非常相近, 这里一并列出:  


- `CMAKE_BINARY_DIR`: build 时的顶层目录;   
- `PROJECT_BINARY_DIR`: 工程的编译(build)目录;  
- `<projectname>_BINARY_DIR`: 工程的编译(build)目录.  

`CMAKE_BINARY_DIR, PROJECT_BINARY_DIR 和 <projectname>_BINARY_DIR`, 这三个变量指代的内容是一致的, 都是工程编译时所在的目录, out-of-source 时对应的是 build/, in source 时对应的是顶层 CMakeLists.txt 所在的目录.  

`CMAKE_SOURCE_DIR, PROJECT_SOURCE_DIR 和 <projectname>_SOURCE_DIR`, 这三个变量指代的内容是一致的, 不论采用何种编译方式, 都是工程顶层目录. 即, 顶层 CMakeLists.txt 所在的目录。  

### (2) CMAKE_CURRENT_SOURCE_DIR   

> 官方解释是: 当前正在处理的 CMakeLists.txt 所在的目录.    

这个变量和上面提到的 `PROJECT_SOURCE_DIR` 变量的区别是:   

- 一旦开始编译, `PROJECT_SOURCE_DIR` 变量在全局范围内就是不变的;  
- 而 `CMAKE_CURRENT_SOURCE_DIR` 变量就对应于包含 CMakeLists.txt 的所有目录(顶层目录和子目录);  

### (3) CMAKE_CURRENT_LIST_FILE  

输出当前处理的 CMakeLists.txt 文件的绝对路径; 这个变量和上面提到的 `CMAKE_CURRENT_SOURCE_DIR` 变量的区别是:  

- `CMAKE_CURRENT_LIST_FILE` 输出的是当前处理的 CMakeLists.txt 文件的绝对路径 ( 如 ~/cmake-test/3rdparty/CMakeLists.txt );  
- `CMAKE_CURRENT_SOURCE_DIR` 输出的是当前正在处理的 CMakeLists.txt 所在的目录 (如 ~/cmake-test/3rdparty).  

### (4) CMAKE_CURRENT_LIST_LINE   

输出当前 CMakeLists.txt 文件中这个变量所在的行数, 一般用于调试.  

### (5) 设置编译生成的结果文件的保存路径 

```
CMAKE_LIBRARY_OUTPUT_DIRECTORY  
CMAKE_ARCHIVE_OUTPUT_DIRECTORY  
CMAKE_RUNTIME_OUTPUT_DIRECTORY   
```

这里需要明确几个概念:   

1) Archive Output  

- add_library() 命令使用 STATIC 选项创建的静态库文件 (如 .lib 或 .a);  
- DLL 平台上, add_library() 命令使用 SHARED 选项创建的可导入的库文件 (.lib);  
- DLL 平台上, add_executable() 命令使用 ENABLE_EXPORTS 属性创建的可导入的库文件 (.lib);  

ARCHIVE_OUTPUT_DIRECTORY 用于设置 Archive 文件的输出路径.  
ARCHIVE_OUTPUT_NAME 用于设置 Archive 文件名.  

2) Runtime Output  

- add_executable() 命令创建的可执行文件 (.exe);  
- DLL 平台上, add_library() 命令使用 SHARED 选项创建的可执行文件 (.dll);  

RUNTIME_OUTPUT_DIRECTORY 用于设置 Runtime 的输出路径.  
RUNTIME_OUTPUT_NAME 用于设置 Runtime 文件名.  

3) Library Output  

- add_library() 命令使用 MODULE 选项创建的模块库文件 (如 .dll 或 .so);  
- 在 non-DLL 平台上, add_library() 命令使用 SHARED 选项创建的共享库文件 (.dylib 或 .so);  

LIBRARY_OUTPUT_DIRECTORY 用于设置 Library 的输出路径.  
LIBRARY_OUTPUT_NAME 用于设置 Library 文件名.  

回到主题,    

`CMAKE_LIBRARY_OUTPUT_DIRECTORY 和 CMAKE_ARCHIVE_OUTPUT_DIRECTORY`, 这两个变量用于指定编译生成的库文件的输出路径设置.  
`CMAKE_RUNTIME_OUTPUT_DIRECTORY` 变量用于指定编译生成的可执行文件的输出路径设置.  

以下是常用的解决方案：   

```
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Bin)
```

最后提一下两个旧的变量: `EXECUTABLE_OUTPUT_PATH 和 LIBRARY_OUTPUT_PATH`.  

EXECUTABLE_OUTPUT_PATH 曾用于设置可执行文件输出路径的变量; 它和 CMAKE_RUNTIME_OUTPUT_DIRECTORY 变量的功能相同, 都是用于设置二进制可执行文件的存放位置. 在较新的版本中, CMAKE_RUNTIME_OUTPUT_DIRECTORY 代替了 EXECUTABLE_OUTPUT_PATH 变量.   

LIBRARY_OUTPUT_DIRECTORY 和 LIBRARY_OUTPUT_PATH 也是类似的关系, LIBRARY_OUTPUT_PATH 是旧的用于设置库文件输出路径的变量.   

### (6) PROJECT_NAME  

返回通过 PROJECT() 指令定义的项目名称。  

### (7) CMAKE_BUILD_TYPE  

设置使用 Debug 或 Release 模式。 Debug 允许断点调试，而 Release 更快. 例如:  

SET( CMAKE_BUILD_TYPE Release )

### (8) CMAKE_CXX_FLAGS  

设置 C++ 编译选项, 也可以通过指令 ADD_DEFINITIONS() 添加。 例如根据不同的平台来设置不同的编译选项:   

```cmake
IF(MSVC)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8 /wd4828")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14")
    # /arch:SSE and /arch:SSE2 are only available when you compile for the x86 platform.
	ADD_DEFINITIONS(/arch:SSE /arch:SSE2) 
ELSEIF(CMAKE_COMPILER_IS_GNUCC) 
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-switch-enum -Wno-switch -Wno-error=non-virtual-dtor")
    ADD_DEFINITIONS( -D__ANDROID__)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
```

### (9) CMAKE_MODULE_PATH   

定义自己的 cmake 模块所在的路径, INCLUDE() 或 FIND_PACKAGE() 函数会从这些路径中搜索并加载模块; 如果有多个目录, 用分号隔开.  

如果你的工程比较复杂, 有可能会自己编写一些 cmake 模块, 这些 cmake 模块是随你的工程发布的, 为了让 cmake 在处理 CMakeLists.txt 时找到这些模块, 你需要设置 CMAKE_MODULE_PATH 以指定模块所在的路径。比如:  

```cmake
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)  
```

这时候你就可以通过 INCLUDE() 指令来调用自己的模块了。  

### (10) CMAKE_INCLUDE_PATH 和 CMAKE_LIBRARY_PATH  

CMAKE_INCLUDE_PATH 用于设置 find_file() 和 find_path() 指令的搜索路径.  

CMAKE_LIBRARY_PATH 用于设置 find_library() 指令的搜索路径.  

### 设置 Debug 库的后缀   

```
SET(CMAKE_DEBUG_POSTFIX "d")
```

## CMake 常用指令   

讨论的指令包括: PROJECT, ADD_EXECUTABLE, INSTALL, ADD_SUBDIRECTORY, SUBDIRS, INCLUDE_DIRECTORIES, LINK_DIRECTORIES, TARGET_LINK_LIBRARIES, SET 等。 

### (1) PROJECT 指令   

```
PROJECT(projectname [CXX] [C] [Java])
```
这个指令隐式的定义了两个 cmake 变量:  

- <projectname>_BINARY_DIR ;  
- <projectname>_SOURCE_DIR ;  

同时, cmake 系统也预定义了 PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR 变量, 它们的值分别跟 HELLO_BINARY_DIR 与 HELLO_SOURCE_DIR 一致.    

为了统一起见, 建议以后直接使用 PROJECT_BINARY_DIR, PROJECT_SOURCE_DIR, 即使修改了工程名称, 也不会影响这两个变量. 如果使用了 <projectname>_SOURCE_DIR, 修改工程名称后, 需要同时修改这些变量.   

### (2) SET 指令  

```
SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])
```

用来显式的定义变量.    

比如, 在定义源文件列表时使用 SET(SRC_LIST main.c); 如果有多个源文件, 则定义成：
SET(SRC_LIST main.c t1.c t2.c).  


### (3) MESSAGE 指令   

```
MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display" ...)
```

这个指令用于向终端输出用户定义的信息, 包含了三种类型:   

- SEND_ERROR,   产生错误, 生成过程被跳过;   
- SATUS,        输出前缀为 '—' 的信息;   
- FATAL_ERROR,  立即终止所有 cmake 过程.  

一般输出调试的变量信息时使用的是 STATUS 信息输出.  

### (4) 生成目标文件 (可执行文件或库文件)


```
add_executable(<name> [WIN32] [MACOSX_BUNDLE]
               [EXCLUDE_FROM_ALL]
               source1 [source2 ...])
               add_executable(<name> IMPORTED [GLOBAL])

add_executable(<name> IMPORTED [GLOBAL])
add_executable(<name> ALIAS <target>)


ADD_LIBRARY(libname 
            [SHARED | STATIC | MODULE] 
            [EXCLUDE_FROM_ALL]    
            source1 source2 ... sourceN)   
```

ADD_EXECUTABLE 用于定义工程根据哪些源文件编译一个可执行文件.   

ADD_LIBRARY 用于定义根据哪些源文件编译一个库文件, 可以是动态库或静态库.   

ADD_SUBDIRECTORY 指定一个嵌套的子编译目录, 目录下必须包含一个CMakeLists.txt 文件.   

其中关于 ADD_LIBRARY():   

- SHARED, STATIC 或 MODULE 关键字可以用来设置生成的库文件类型.  
- 在使用 dyld 的系统上, MODULE 类型的库被设置为 MH_BUNDLE;  
- 在非 dyld 的系统上, MODULE 类型的库作为动态库使用;  
- 如果该命令没有设置第二个参数, 则默认为动态库;  
- 如果这个变量没有被设置,  默认使用 STATIC 参数;  
- 如果设置了 EXCLUDE_FROM_ALL 选项, 那么只有在构建这个库或者其他依赖于这个库的目标时才会被编译.  

#### (5) 指定头文件搜索路径或库文件搜索路径  

- INCLUDE_DIRECTORIES() 用于添加查找头文件的路径. 基本语法为:  

```
INCLUDE_DIRECTORIES([AFTER|BEFORE] [SYSTEM] dir1 dir2 ...)   
```

将给定的目录添加到 compiler 的头文件搜索路径. 使用 BEFORE or AFTER 选项可以选择在当前目录列表之前或是之后添加, 如果没有使用, 则默认情况是在目录列表之后追加. 也可以通过设置 CMAKE_INCLUDE_DIRECTORIES_BEFORE 为 ON 来改变这种默认行为. 另外使用 SYSTEM 选项可以指定包含的目录是系统目录.    

- LINK_DIRECTORIES() 用于指定编译器的 libraries 文件搜索路径. 基本语法为:  

```
LINK_DIRECTORIES(directory1 directory2 ...)   
```

### (6) 指定目标所需要链接库

TARGET_LINK_LIBRARIES() 将一系列指定的库文件链接到 target 文件中.   

```
TARGET_LINK_LIBRARIES(target library1  <debug | optimized> library2  ...)
```

debug 和 optimized 选项用来指示下一个列出的库文件将会以何种类型 build.    

> LINK_LIBRARIES() 是 TARGET_LINK_LIBRARIES() 之前的 old CMake 命令, 用于为接下来的 targets 文件指定一系列链接文件. 在该命令之前通常已经使用 ADD_EXECUTABLE() 或 ADD_LIBRARY() 命令指定了 targets. 因为这个命令会给所有的子目录传递下去. `LINK_LIBRARIES(library1 <debug | optimized> library2 ...)`

除非你必须为每个 target 都指定相同的链接库, 否则就使用 TARGET_LINK_LIBRARIES().  

### (7) ADD_DEFINITIONS() 

向 C/C++ 编译器添加 -D 宏定义, 支持两种常用的宏定义: 宏开关和常量宏. 多个宏开关参数之间用空格分割。

```py
ADD_DEFINITIONS(-DDEBUG)     # (宏开关)
ADD_DEFINITIONS(-DVERSION=1) # (常量宏)
```

如果要添加其它的编译器开关, 可以通过 CMAKE_C_FLAGS 变量和 CMAKE_CXX_FLAGS 变量设置。   

### (8) ADD_DEPENDENCIES()  

定义 target 依赖的其它 target, 确保在编译本 target 之前, 其它的 target 已经被构建。   

ADD_DEPENDENCIES(target-name depend-target1 depend-target2 ...)

### (9) ADD_SUBDIRECTORY()

ADD_SUBDIRECTORY 指定一个嵌套的子编译目录, 目录下必须包含一个CMakeLists.txt 文件.   

### (10) SET_TARGET_PROPERTIES() 指令

```
SET_TARGET_PROPERTIES(target1 target2 ...
                      PROPERTIES prop1 value1
                      prop2 value2 ...)
```

支持的 PROPERTIES 有(遇到时再做补充):   

- IMPORTED_LOCATION: 指定要导入的目标的绝对路径;  
- PREFIX: 设置库名的前缀, 如果未设置, 默认使用 "lib";  
- OUTPUT_NAME: 指定输出目标的文件名;  

### (11) AUX_SOURCE_DIRECTORY   

```
AUX_SOURCE_DIRECTORY(dir VARIABLE)  
```

查找一个目录下所有的源代码文件并将列表存储在一个变量中, 这个指令临时被用来自动构建源文件列表。  

### (12) INCLUDE 指令 

可用来载入 CMakeLists.txt 文件或预定义的 cmake 模块.   

```cmake
INCLUDE(file1 [OPTIONAL])  
INCLUDE(module [OPTIONAL])  
```

OPTIONAL 参数的作用是文件不存在也不会产生错误。  

如果指定载入的是一个模块, 那么将在 CMAKE_MODULE_PATH 中搜索这个模块并载入。载入的内容将在处理到 INCLUDE 语句时直接执行。 

### (13) OPTION 指令  

提供一个用户可以选择性地使用的选项.   

```cmake
OPTION(<variable> "<help_text>" [value])
```

value 的取值为 ON 或 OFF. 如果没有提供初始值 <value> 或者指定了其他非 BOOL 类型的值, 那么默认为 OFF.   

如果 <variable> 已经被设置为一个普通变量, 那么 OPTION() 指令就不起作用. 因此它和 SET() 指令的区别是:  

- OPTION() 命令不会覆盖现有的变量, 而 SET() 命令会覆盖现有变量; 从这种意义上来讲, OPTION() 指令更为安全.  
- OPTION() 命令定义的变量值的默认类型是 BOOL, 而 SET() 命令可以定义任何变量类型.

可以在命令行使用 "-D<variable>=ON" 定义变量 <variable> 的值.  

### (14) FILE 指令   

文件操作指令, 基本语法为:   

```
Reading
  file(READ <filename> <out-var> [...])
  file(STRINGS <filename> <out-var> [...])
  file(<HASH> <filename> <out-var>)
  file(TIMESTAMP <filename> <out-var> [...])

Writing
  file({WRITE | APPEND} <filename> <content>...)
  file({TOUCH | TOUCH_NOCREATE} [<file>...])
  file(GENERATE OUTPUT <output-file> [...])

Filesystem
  file({GLOB | GLOB_RECURSE} <out-var> [...] [<globbing-expr>...])
  file(RENAME <oldname> <newname>)
  file({REMOVE | REMOVE_RECURSE } [<files>...])
  file(MAKE_DIRECTORY [<dir>...])
  file({COPY | INSTALL} <file>... DESTINATION <dir> [...])
  file(SIZE <filename> <out-var>)
  file(READ_SYMLINK <linkname> <out-var>)
  file(CREATE_LINK <original> <linkname> [...])

Path Conversion
  file(RELATIVE_PATH <out-var> <directory> <file>)
  file({TO_CMAKE_PATH | TO_NATIVE_PATH} <path> <out-var>)

Transfer
  file(DOWNLOAD <url> <file> [...])
  file(UPLOAD <file> <url> [...])

Locking
  file(LOCK <path> [...])
```

关于 FILE() 指令的详细解释可参考[cmake FILE 指令详解](/dev_tools/cmake/doc/FILE_command.md)  .  

#### (15) ADD_TEST() 与 ENABLE_TESTING() 指令。   

ENABLE_TESTING() 指令用来控制 Makefile 是否构建 test 目标, 涉及工程所有目录。语法很简单, 没有任何参数; 一般情况这个指令放在工程的主 CMakeLists.txt 中.  

ADD_TEST 指令的语法是:   

ADD_TEST(testname Exename arg1 arg2 ...)  

testname 是自定义的 test 名称, Exename 可以是构建的目标文件也可以是外部脚本等等。后面连接传递给可执行文件的参数。如果没有在同一个 CMakeLists.txt 中打开 ENABLE_TESTING() 指令, 任何 ADD_TEST 都是无效的。   

比如可以在工程主 CMakeLists.txt 中添加:   

```cmake
ADD_TEST(mytest ${PROJECT_BINARY_DIR}/bin/main)  
ENABLE_TESTING()  
```

生成 Makefile 后, 就可以运行 `make test` 来执行测试了。  

### (16) CMAKE_MINIMUM_REQUIRED  

```
CMAKE_MINIMUM_REQUIRED(VERSION versionNumber [FATAL_ERROR])  
```

设置 cmake 所需的最低版本, 如果 cmake 版本小于设置的版本号, 则出现严重错误, 整个预编译过程中止。  

### (17) EXEC_PROGRAM   

```
EXEC_PROGRAM(Executable [directory in which to run]  
                [ARGS <arguments to executable>]  
                [OUTPUT_VARIABLE <var>]  
                [RETURN_VALUE <var>])  
```

在处理 CMakeLists.txt 的过程中执行指定路径下的外部程序, 并不会在生成的 Makefile 中执行。  

- ARGS 用于为程序指定需要的实参;
- 如果要获取输出和返回值, 可通过 OUTPUT_VARIABLE 和 RETURN_VALUE 分别定义两个变量.   

这个指令可以帮助你在 CMakeLists.txt 处理过程中支持任何命令, 比如根据系统情况去修改代码文件等等。  

举个简单的例子. 比如要在 src 目录执行 ls 命令, 并把结果和返回值存下来。  

可以在 src/CMakeLists.txt 中添加:   

```cmake
EXEC_PROGRAM(ls ARGS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" OUTPUT_VARIABLE LS_OUTPUT RETURN_VALUE LS_RVALUE)  

IF(NOT LS_RVALUE)  
  MESSAGE(STATUS "ls result: " ${LS_OUTPUT})  
ENDIF(NOT LS_RVALUE)  
```

其中的变量分别对应:  

- LS_OUTPUT: 'ls *.cpp' 的输出结果;  
- LS_RVALUE: 'ls *.cpp' 的执行成功与否;  

在 cmake 生成 Makefile 的过程中, 就会执行 ls 命令.  

如果返回 0, 则说明成功执行, 那么就输出 ls *.cpp 的结果。 

### (18) INSTALL() 指令  

INSTALL 系列指令已经在前面的章节有非常详细的说明, 这里不在赘述, 可参考: https://cmake.org/cmake/help/latest/command/install.html?highlight=install   

### (19) FIND_xxx()指令  

FIND_系列指令主要包含以下指令:  

```cmake
FIND_FILE(<VAR> name1 path1 path2 ...)      # VAR 变量代表找到的文件全路径,包含文件名
FIND_LIBRARY(<VAR> name1 path1 path2 ...)   # VAR 变量表示找到的库全路径,包含库文件名
FIND_PATH(<VAR> name1 path1 path2 ...)      # VAR 变量代表包含这个文件的路径。
FIND_PROGRAM(<VAR> name1 path1 path2 ...)   # VAR 变量代表包含这个程序的全路径。
FIND_PACKAGE(<name> [major.minor] [QUIET] [NO_MODULE]
                [[REQUIRED|COMPONENTS] [componets...]])
```

用来调用预定义在 CMAKE_MODULE_PATH 下的 Find<name>.cmake 模块, 你也可以自己定义 Find<name> 模块, 通过 SET(CMAKE_MODULE_PATH dir) 将其放入工程的某个目录中供工程使用, 后面会详细介绍 FIND_PACKAGE 的使用方法和 Find 模块的编写。  

FIND_LIBRARY 示例:  

```cmake
FIND_LIBRARY(libX X11 /usr/lib)  
IF(NOT libX)  
  MESSAGE(FATAL_ERROR "libX not found")  
ENDIF(NOT libX)  
```

详细介绍请参考: [cmake FIND_xxx 指令详解](/dev_tools/cmake/doc/FILE_command.md).  


## 关于语法的疑惑

### (1) SET(SRC_LIST main.c) 和 SET(SRC_LIST “main.c”) 的区别   

在这个例子中是没有区别的，但是假设一个源文件的文件名是 fu nc.c (文件名中间包含了空格)。这时候就必须使用双引号，如果写成了 `SET(SRC_LIST fu nc.c)` ，就会出现错误，提示你找不到 fu 文件和 nc.c 文件。这种情况，就必须写成: `SET(SRC_LIST “fu nc.c”)`. 

## ISSUES  

### (1) make distclean 不起作用   

make distclean 一般用来清理构建过程中产生的中间文件，在 cmake 工程中这个命令是无效的.  

是的，cmake 并不支持 make distclean，关于这一点，官方是有明确解释的:   

因为 CMakeLists.txt 可以执行脚本并通过脚本生成一些临时文件，但是却没有办法来跟踪这些临时文件到底是哪些。因此，没有办法提供一个可靠的 make distclean 方案。  

因此 cmake 强烈推荐的是外部构建 (out-of-source build) 方式.    




## 四. 主要的开关选项  

### BUILD_SHARED_LIBS

这个开关用来控制默认的库编译方式.  

如果不进行设置, 使用 ADD_LIBRARY 并没有指定库类型的情况下, 默认编译生成的库都是静态库; 如果 SET(BUILD_SHARED_LIBS ON) 后, 默认生成的为动态库。 

默认情况下: BUILD_SHARED_LIBS = OFF.  

### CMAKE_INCLUDE_CURRENT_DIR   

> Automatically add the current source and build directories to the include path.

自动添加 CMAKE_CURRENT_BINARY_DIR 和 CMAKE_CURRENT_SOURCE_DIR 到当前处理的 CMakeLists.txt。相当于在每个 CMakeLists.txt 加入:   

```cmake
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
```

如果头文件和实现文件在同一个目录内, 这个开关在 out-of-source 编译过程中就非常有用, 可以将源码目录中自动添加到 INCLUDE_DIRECTORIES() 中.  

默认情况下: CMAKE_INCLUDE_CURRENT_DIR = OFF.

可以使用如下代码开启:  

```cmake
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
```

### CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE  

是否将工程提供的头文件目录始终至于系统头文件目录的前面, 当你定义的头文件确实跟系统发生冲突时可以提供一些帮助。   

如果设置为 ON, 那么 CMAKE_SOURCE_DIR 和 CMAKE_BINARY_DIR 就会被添加到最前边.


## 五. 系统信息  

1. CMAKE_MAJOR_VERSION. CMAKE 主版本号,比如 2.4.6 中的 2  
2. CMAKE_MINOR_VERSION. CMAKE 次版本号,比如 2.4.6 中的 4  
3. CMAKE_PATCH_VERSION. CMAKE 补丁等级,比如 2.4.6 中的 6  
4. CMAKE_SYSTEM,系统名称,比如 Linux-2.6.22   
5. CMAKE_SYSTEM_NAME,不包含版本的系统名,比如 Linux  
6. CMAKE_SYSTEM_VERSION,系统版本,比如 2.6.22  
7. CMAKE_SYSTEM_PROCESSOR,处理器名称, 比如 x86_64.   
8. UNIX, 在所有的类 UNIX 平台为 TRUE,包括 OS X 和 cygwin; 如果满足, 值为 1, 否则为空
9. WIN32, 在所有的 win32 平台为 TRUE,包括 cygwin; 如果满足, 值为 1, 否则为空



## 七. 测试平台相关信息    

```cmake
IF (UNIX)
   MESSAGE("这个是UNIX操作系统")
ENDIF (UNIX)
IF (MSVC)
   MESSAGE("这个需要 VC 的项目文件")
ENDIF ()
```

```cmake
IF (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    SET(option WIN32)
    SET(win32_LIBRARIES comctl32.lib shlwapi.lib shell32.lib odbc32.lib odbccp32.lib  kernel32.lib user32.lib   gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib   odbccp32.lib)
    #SET(defs -DUNICODE -D_UNICODE)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
```

## 八. cmake 中使用 c++11 特性 

使用以下的代码, cmake 可以检测 g++ 编译器是否支持 c++11.  

```
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
```

## 参考资料   

[1]: [cmake 常用变量和常用环境变量查表手册](https://blog.csdn.net/gubenpeiyuan/article/details/8667279)  