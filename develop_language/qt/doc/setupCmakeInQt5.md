# QT 配置 CMake 编译  

build and run kits.

## 1. 在 Qt 中添加 CMake  

Qt Creator 支持 CMake 3.0 及以上版本.  

1) 将 CMake 添加到 build and run kits 中  

选择 Tools | Options | Build & Run. 选择 CMake 标签, 点击 Add:  

![](../snapshots/qtcreator-cmakeexecutable.png)

- Name 域填 CMake.  
- Path 域选择 cmake.exe 所在的安装路径.  

点击 Apply 保存更改.  

选择 Kits 标签, 将刚刚添加好的 CMake 工具设置到 CMake 编译 Kit 中. CMake 编译 Kit 中设置了 C/C++ 编译器, Qt 版本和刚刚设置好的 CMake 工具:  

![](../snapshots/qtcreator-kits.png)  

点击 Apply 保存更改.  

## 2. 新建 CMake-Qt 工程  

选择 File | New File or Project | Non-Qt Project | Plain C Application 或 Plain C++ Application | Choose.  

- 更改创建的工程名.  
- 设置好工程创建的路径, 点击 Next.  
- 在 Define Build system 界面中选择 CMake, 点击 Next.  
- 选择上一步设置好的 CMake kit, 点击 Next.  
- 检查一下工程的配置信息, 然后点击 Finish.  
- 选择 Build | Run CMake, 生成一个 .cbp 文件.  
- 选择 Build | Build Project xxx, 或 Ctrl+B 生成一个 .exe 文件.  

Qt Creator 默认会生成一个 main.cpp 和 CMakeLists.txt 文件. 默认情况下 Qt Creator 使用的是 Default build 配置信息.   

## 打开 CMake-Qt 工程

选择 File | Open File or Project, 然后从某个工程路径中选择 CMakeLists.txt; Build Project or press Ctrl+B.

Qt Creator 使用 make, mingw32-make, nmake, 或 ninja 来编译 CMake 生成的工程. 默认情况下 Qt Creator 使用的是 Default build 配置信息.  

左侧导航栏, 选择 Projects, 打开项目 build 配置信息, 可以设置 Build directory 到指定的 build 目录下.  

选中 Advanced 复选框可以打开更多 CMake 变量.  

可以点击对应的 build 变量修改其值, 修改完之后记得点击 Apply Configuration Changes.  

可以在 Build Steps 中指定 Tool arguments.  

之后, Qt Creator 会自动运行工程中的 CMakeLists.txt 文件.  

## 定制化设置  

(1) 设置 Default Build Directory  

选择 Tools | Options | Build & Run | General | Default Build Directory. 将其设置为 build/. 

默认的的 Build 路径为: `../%{JS: Util.asciify("build-%{CurrentProject:Name}-%{CurrentKit:FileSystemName}-%{CurrentBuild:Name}")}`  

其中 `%{CurrentBuild:Name}` 表示编译类型, 例如 Release / Default / Debug.  

(2) 工程创建完毕后会直接利用当前的 CMakeLists.txt 在 build/ 目录中生成 Makefile. 因此不必显式指定 ../ 为编译目录.  

(3) 去掉: QT_QMAKE_EXECUTABLE 预定义 CMake 变量  

a) 选择 Tools | Options | Build & Run | Kits | Manual | Desktop Qt 5.10.0 MSVC2017 64bit. 

- 在 CMake Configuration 中去掉 QT_QMAKE_EXECUTABLE:STRING=%{Qt:qmakeExecutable};   

b) 选择 Tools | Options | Build & Run | Kits | Manual | cmake. 

**注意这个 cmake 构建套件需要提前设置好**. 

- 设置好 QT 版本和编译器版本;  
- 在 CMakeConfiguration 中去掉 QT_QMAKE_EXECUTABLE:STRING=%{Qt:qmakeExecutable};  

## Qt VTK PCL CMake 工程实例  

```cmake
cmake_minimum_required(VERSION 3.10)

project(pcl_visualizer)

set(CMAKE_BUILD_TYPE Release)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
set (CMAKE_AUTOUIC_SEARCH_PATHS src)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories( include )

# Find the QtWidgets library
set( VTK_DIR "D:/thirdparty20170624/vtk/build")
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

set( PCL_DIR "C:/PCL_181/cmake")
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Qt5 REQUIRED Widgets)  #  Core Gui

# Fix a compilation bug under ubuntu 16.04 (Xenial)
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

FILE(GLOB SRCS_FILES src/*.cpp src/*.c src/*.cc src/*.cxx)
FILE(GLOB HDS_FILES include/*.hpp include/*.h include/*.hh include/*.hxx)
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${SRCS_FILES})
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${HDS_FILES})

add_executable(${PROJECT_NAME} ${SRCS_FILES} ${HDS_FILES})
# Qt5::Core  Qt5::Gui
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets)  
```

## 在 VS2015 环境下使用 Qt  

5.1) 安装 Qt Visual Studio Tools  

需要在 VS2015 的扩展和更新中安装 Qt Visual Studio Tools 扩展工具.   

**注意: 不要安装 QtPackage, 因为不能使用 QVTKWidget 插件.**   

安装步骤如下：  

- 打开 VS2015，选择菜单 "工具", 点击 "扩展和更新"  . 
- 在 "扩展和更新" 窗口中，点击 "联机"，然后在右侧输入框输入 "Qt" ，并按下 "Enter" 键;  
- 选择 Qt Visual Studio Tools 工具下载安装即可. 

**再次提示：不要装 QtPackage 工具**.   

安装后，重新启动 VS2015，在菜单栏上就会看到 "Qt VS Tools" 菜单项, 点击 Launch Qt Designer 就能看到 Qt Designer 界面.  

另外，Qt Visual Studio Tools 工具也可实现 QT 在 VS 下编译运行. 此时需要配置 Qt 5 选项:  

选择：Qt VS Tools | Qt Options，点击 "Add" 按钮，Path 选择 D:\Qt\Qt5.7.1\5.7\msvc2015_64，然后点击 "Ok" 进行保存. 之后就可以像编译普通 C++ 程序一样执行:  

```bash
cmake ../ -G "Visual Studio 14 2015 Win64"
```

然后打开生成的 .sln 工程文件就可以开始编译了.  

**注意：如果没有配置 QT，在新建 Qt 项目时会出现 "Unable to find a Qt build!" 错误**.  

更为详细的安装配置请参考 Qt 在 VS(Visual Studio) 中使用: https://www.cnblogs.com/techiel/p/7942352.html.   

## 在嵌入式 Linux 设备上部署 CMake 工程  

Qt Creator cannot extract files to be installed from a CMake project, and therefore, only executable targets are automatically added to deployment files. You must specify all other files in the QtCreatorDeployment.txt file that you create and place in either the root directory of the CMake project or the build directory of the active build configuration. Currently, Qt Creator first checks the root directory and only if no QtCreatorDeployment.txt exists it checks the active build directory.

Use the following syntax in the file:

```xml
<deployment/prefix>
<relative/source/file1>:<relative/destination/dir1>
...
<relative/source/filen>:<relative/destination/dirn>
```

Where:

<deployment/prefix> is the (absolute) path prefix to where files are copied on the remote machine.  
<relative/source/file> is the file path relative to the CMake project root. No directories or wildcards are allowed in this value.  
<relative/destination/dir> is the destination directory path relative to deployment/prefix.  

To automate the creation of QtCreatorDeployment.txt file:  

Define the following macros in the top level CMakeLists.txt file:  

```cmake
file(WRITE "${CMAKE_SOURCE_DIR}/QtCreatorDeployment.txt" "<deployment/prefix>\n")

macro(add_deployment_file SRC DEST)
    file(RELATIVE_PATH path ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    file(APPEND "${CMAKE_SOURCE_DIR}/QtCreatorDeployment.txt" "${path}/${SRC}:${DEST}\n")
endmacro()

macro(add_deployment_directory SRC DEST)
    file(GLOB_RECURSE files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRC}/*")
    foreach(filename ${files})
        get_filename_component(path ${filename} PATH)
        add_deployment_file("${filename}" "${DEST}/${path}")
    endforeach(filename)
endmacro()
```

Use add_deployment_file(<file/name>) to add files and add_deployment_directory(<folder/name>) to add directories (including subdirectories) to the QtCreatorDeployment.txt file.
Re-run cmake after you add or remove files using the macros.
Adding External Libraries to CMake Projects
Through external libraries, Qt Creator can support code completion and syntax highlighting as if they were part of the current project or the Qt library.

Qt Creator detects the external libraries using the FIND_PACKAGE() macro. Some libraries come with the CMake installation. You can find those in the Modules directory of your CMake installation.

Note: If you provide your own libraries, you also need to provide your own FindFoo.cmake file. For more information, see CMake FAQ.

Syntax completion and highlighting work once your project successfully builds and links against the external library.


## issues  

1) 遇到问题 xxx is not able to compile a simple test

```bash
 xxx is not able to compile a simple test
:-1: error: Generator: execution of make failed. Make command was:  "jom"   "/NOLOGO"   "all" 
```

问题就应该出现在 QT Creator 调用 cmake 时出现了问题. 一番搜索之后，发现 jom.exe 是 QTCreator/bin 下面的一个可执行文件，是 QT Creator 调用该可执行文件出现了问题吗？找不到该文件？  

解决方法: 将 QTCreator 的可执行目录 ( C:\Qt\Qt5.9.4\Tools\QtCreator\bin) 加入 Windows 的 Path 变量后，重新启动 QTCreator, 打开上述项目成功.   

