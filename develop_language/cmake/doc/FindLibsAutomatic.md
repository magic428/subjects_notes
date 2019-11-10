# CMake 如何查找库路径 

如果你的代码使用了外部库（external libraries），并且你事先不知道这些库的头文件和库文件在当前平台的位置。那么适当的文件夹路径和库的搜索路径就应该被添加到编译命令中去。 
CMake 通过提供find_package命令来帮助你实现这点（）。 
本文简要介绍如何在CMake工程中使用外部库，然后介绍如何写自己的find module。

Contents 

- 1. 使用外部库 
- 2. 使用CMake没有提供find module 的外部库 
- 3. package finding的工作机制 
- 4. 利用pkg-config 
- 5. 编写find modules 
- 6. 执行和缓存 
- 7. 常见bugs

1.使用外部库
CMake本身提供了很多常用库的module来帮助查找该库。使用命令 cmake –help-module-list 
可以查看提供了哪些module，或者到module的存放路径下查看，在ubuntu Linux上的位置通常在／usr/share/cmake/Modules/下。

2.使用CMake本身没有提供module的外部库
假设你要使用LibXML++库，但是CMake没有提供该库的module。同时你发现网上有其他人提供了FindLibXML++.cmake。 
此时可以下载该文件并把它丢到CMake module路径下。

有组件的包（components）
有些库不是只有一个整体组成，可能包含多个依赖库和组件。一个明显的例子是Qt 库，包含组建QtOpenGL和QtXML。为了使用这些组建，使用如下命令

find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
1
在包为可选项时可以省略关键词REQUIRED。这样就可以使用变量<PACKAGE>_<COMPONENTS>_FOUND，例如Qt_QtXml_FOUND，来检查对应组建是否被找到。

3.包查找的机制
find_package()命令会查找moudle目录下的Find <name>.cmake文件。

首先，CMake查找${CMAKE_MODULE_PATH}里的所有文件夹。

然后， CMake查找自己的module目录<CMAKE_ROOT>/share/cmake-x.y/Modules/.

如果找不到上述文件，CMake会查找<Name>Config.cmake或者<lower-case-name&gt-config.cmake文件，这些应该是由库来安装的。

前者叫做module mode后者叫做config module。不论是哪个mode被使用，如果一个package 被找到了，都会生成一系列的变量。 
<Name>_FOUND 
<Name>_INCLUDE_DIRS 或 <Name>_INCLUDES 
<Name>_LIBRARIES 或<Name>_LIBS 
<Name>_DEFINITIONS

所有这些变量都在Find <name>.cmake文件里替换。