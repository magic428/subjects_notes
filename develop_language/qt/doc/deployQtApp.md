# Qt5 程序打包发布方法 (使用官方提供的windeployqt工具)  

Qt 官方开发环境使用的动态链接库方式，在发布生成的exe程序时，需要复制一大堆 dll，如果自己去复制dll，很可能丢三落四，导致exe在别的电脑里无法正常运行。因此 Qt 官方开发环境里自带了一个工具：windeployqt.exe。  

以官方 Qt 5.4.0+MinGW 开发环境为例，windeployqt工具在 %QTDIR%\Qt5.4.0\5.4\mingw491_32\bin 目录下，其中 QTDIR 是 Qt 的安装目录，是环境变量。  

在集成开发环境 QtCreator 中可选择 “Qt Widgets Application” 或 “Qt Quick Application” 两种方式生成图形界面应用程序。  

下面分别介绍这两种方式创建应用的发布方式。  

## 1. Qt Widgets Application  

首先用 QtCreator 新建一个 Qt Widgets Application 项目，直接用默认的 QMainWindow 程序就可以了，项目名字假定是 hellomw。然后以 Release 方式编译生成 exe 程序.  

生成的程序运行正常之后，找到项目的生成目录，比如项目源码路径：C:\QtPros\hellomw\ 。它的项目生成目录是 C:\QtPros\build-hellomw-Desktop_Qt_5_4_0_MinGW_32bit-Release\ 。进入这个文件夹，在进入它的子文件夹 release 里面，找到 hellomw.exe，将这个exe 复制到一个新的单独的文件夹里用于发布，比如存到 D:\hellomw\ 文件夹里面。

然后从开始菜单打开 Qt 命令行，输入命令：`cd /d D:\hellomw`, 然后使用 windeployqt 工具命令：  

```bash
windeployqt hellomw.exe
```

然后可以在 D:\hellomw 文件夹里看到 windeployqt 工具自动复制的插件文件夹和 dll文件、qm文件。这时候得到的就完整的 exe 程序发布集合，依赖关系都解决好了。把 D:\hellomw 文件夹 打包就可以发布了，不用自己一个个找 dll 文件了。D:\hellomw 文件夹里的qm文件是多国语言翻译文件，不需要可以删了，其他的都保留。  


## 2. Qt Quick Application  

首先用 QtCreator 新建一个 Qt Quick Application 项目，直接用默认的项目模版，点击下一步生成项目，项目名字假定是 helloqml。然后以 Release 方式编译生成 exe 程序.  

然后找到项目的构建目录，比如项目源码目录 C:\QtPros\helloqml。它的构建目录是：C:\QtPros\build-helloqml-Desktop_Qt_5_4_0_MinGW_32bit-Release\。进入这个目录，再进入 release 子文件夹，找到 helloqml.exe ，复制到一个新的单独的文件夹里面，比如 D:\helloqml\ 文件夹里面。  

然后从开始菜单打开 Qt 命令行，进入D:\helloqml\文件夹：cd /d D:\helloqml
然后使用 windeployqt 工具命令：  

```bash 
windeployqt helloqml.exe –qmldir C:\Qt\Qt5.4.0\5.4\mingw491_32\qml
```

注意不要跟完全一样照抄上条命令！–qmldir 是指出 Qt 库里面的 qml 文件夹位置，上面命令里 C:\Qt\Qt5.4.0 是 Qt 官方开发环境安装的文件夹，C:\Qt\Qt5.4.0\5.4\mingw491_32 是Qt类库的目录 (QTDIR) ，因此使用的 –qmldir 后面写的是 C:\Qt\Qt5.4.0\5.4\mingw491_32\qml ，读者Qt环境安装路径不一样，要根据实际情况修改！  

然后可以看到 D:\helloqml 文件夹里有一大堆文件，就是 QtQuick程序需要的依赖文件。将整个 D:\helloqml 文件夹 打包就可以发布出去，在别的电脑上使用。这个 D:\helloqml 文件夹里的东西很多，看不懂就不要删，老老实实打包发布就行了。  

上面是最简单的程序发布，实际复杂程序可能还带一些图片文件、数据库文件、配置文件之类的，可以按自己需要添加这些文件到发布文件夹里面。  
