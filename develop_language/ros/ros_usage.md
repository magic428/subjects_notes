# ROS 用法
Package:软件包
Manifest：package.xml：软件包的相关信息。

文件系统使用工具
rospack find [packages]
roscd [package] 直接切换到包所在的目录下。必须设置好ROS_PACKAGE_PATH环境变量
rosls [package] 直接列出包中的文件。
TAB键补全


3 如何创建一个ROS包
	功能模块。可完成一个简单的功能。
roscreate-pkg catkin.

The package must contain a catkin compliant package.xml file.
That package.xml file provides meta information about the package.
The package must contain a CMakeLists.txt which uses catkin. If it is a catkin metapackage it must have the relevant boilerplate CMakeLists.txt file.
There can be no more than one package in each folder.
This means no nested packages nor multiple packages sharing the same directory.
rospack depends和rospack depends1的区别
rospack depends会列出所有的包的依赖关系。

package.xml和CMakeList.txt文件中的主要条目和内容。
