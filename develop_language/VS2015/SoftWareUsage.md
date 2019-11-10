# Visual Studio 2015 软件使用手册

## 关于VS2015使用的所有小技巧

1. Peek View
可以在不新建TAB的情况下快速查看、编辑一个函数的代码。
用法：在光标移至某个函数下，按下alt+F12。

然后在Peek窗口里可以继续按alt+F12。然后按ctrl+alt+-，或者ctrl+alt++就可以前后跳转。按ESC关闭Peek窗体。
这下就不需要来回跳转了……（alt+G可以跳转至函数定义的地方，alt+左箭头可以回退）
2.自动补全工具Productivity Power Tools
可以自动补全你的需要的东西，比如你输入了（，他会自动补全），就变成了（），如果你输入{，他会补全}，变为{}，还会补全“；”，还有自动换行，很是方便。在vs上的工具==》扩展和更新，搜索输入Productivity进行安装即可。

3.NET Web Development and Tools Blog.
在不新建TAB的情况下快速查看、编辑一个函数的代码。以前要看一个函数的实现，需要在使用的地方点击F12跳转到该函数，实际上这是很浪费时间的。VS2015Peek View便解决了这个问题。在光标移至某个函数下，按下alt+F12。

4.// TODO
Resharper实在是一款非常强大的插件。我这里先介绍其中一个好处 抛砖引玉。有兴趣大家可以看一些关于Resharper的教程

右边有一栏，会有红色和黄色，黄色就是代表不规范或者是可以改善的代码，点上去后就会有提示，而且会有选择点击后即可自动优化！红色就是指错误，会实时把错误提示给你。这些对于代码规范性来说是一个非常大的帮助。
5.在光标指定位置的上面添加一行，并将光标移至新添加行的行首位置
按快捷键 ,Ctrl + Enter.如果在光标指定位置的下面添加一行，并将光标移至新添加行的行首位置，使用Ctrl + Shift + Enter。
以下是各种简单的快捷操作



560655063

群内有大量的项目开发和新手教学视频千人大群等着你来加入。

Ctrl+E,D 	格式化全部代码
Ctrl+E,F 	格式化选中的代码
CTRL + SHIFT + B 	生成解决方案
CTRL + F7 	生成编译
CTRL + O 	打开文件
CTRL + SHIFT + O 	打开项目
CTRL + SHIFT + C 	显示类视图窗口
F4 	显示属性窗口
SHIFT + F4	显示项目属性窗口
CTRL + SHIFT + E 	显示资源视图

<查看源码相关>
F12 	转到定义
CTRL + F12 	转到声明
CTRL + -	返回定义，与F12组合使用


CTRL + ALT + J 	对象浏览
CTRL + ALT + F1 	帮助目录
CTRL + F1 	动态帮助
F1 	帮助
SHIFT + F1 	当前窗口帮助
CTRL + ALT + F3 	搜索
SHIFT + ALT + ENTER
全屏显示
CTRL + -
向后定位
CTRL + SHIFT + -
向前定位
CTRL + F4
关闭文档窗口
CTRL + PAGE DOWN
光标定位到窗口上方
CTRL + PAGE UP
光标定位到窗口下方
CTRL + F6
CTRL + TAB
下一个文档窗口
CTRL + SHIFT + F6
CTRL + SHIFT + TAB
上一个文档窗口
ALT + F6
下一个面板窗口
CTRL + K, CTRL + L
取消
remark
CTRL + K, CTRL + C
注释选择的代码
CTRL + K, CTRL + U
取消对选择代码的注释
CTRL + M, CTRL + O
折叠代码定义
CTRL + M, CTRL + L
展开代码定义
CTRL + DELETE
删除至词尾
CTRL + BACKSPACE
删除至词头
SHIFT + TAB
取消制表符
CTRL + U
转小写
CTRL + SHIFT + U
转大写
CTRL + SHIFT + END
选择至文档末尾
CTRL + SHIFT + HOME
选择至文档末尾开始
SHIFT + END
选择至行尾
SHIFT + HOME
选择至行开始处
SHIFT + ALT + END
垂直选择到最后尾
SHIFT + ALT + HOME
垂直选择到最前面
CTRL + SHIFT + PAGE UP
选择至本页前面
CTRL + SHIFT + PAGE DOWN
选择至本页后面
CTRL + END
文档定位到最后
CTRL + HOME
文档定位到最前
CTRL + A
全选
CTRL + W
选择当前单词
CTRL + G
转到
…
CTRL + K, CTRL + P
上一个标签
CTRL + K, CTRL + N
下一个标签
ALT + F10
调试
-ApplyCodeChanges
CTRL + ALT+ Break
停止调试
CTRL + SHIFT + F9
取消所有断点
CTRL + F9
允许中断
CTRL + SHIFT + F5
重新开始
F5
运行调试
CTRL + F5
运行不调试
F10
跨过程序执行
F11
单步逐句执行
CTRL + J
列出成员
下一个视图
CTRL + B
格式
粗体
CTRL + SHIFT + T
文字缩进
调试快捷键
F6:
Ctrl+F6:
生成当前项目
F7:
查看代码
Shift+F7:
查看窗体设计器
F5:
启动调试
Ctrl+F5:
开始执行
(
不调试
)
Shift+F5:
Ctrl+Shift+F5:
重启调试
F9:
切换断点
Ctrl+F9:
启用
/
停止断点
Ctrl+Shift+F9:
删除全部断点
F10:
逐过程
Ctrl+F10:
运行到光标处
F11:
逐语句
编辑快捷键
Shift+Alt+Enter:
切换全屏编辑
Ctrl+B,T / Ctrl+K,K:
切换书签开关
Ctrl+B,N / Ctrl+K,N:
移动到下一书签
Ctrl+B,P:
移动到上一书签
Ctrl+B,C:
清除全部标签
Ctrl+I:
渐进式搜索
Ctrl+Shift+I:
反向渐进式搜索
Ctrl+F:
查找
Ctrl+Shift+F:
在文件中查找
F3:
查找下一个
Shift+F3:
查找上一个
Ctrl+H:
替换
Ctrl+Shift+H:
在文件中替换
Alt+F12:
查找符号
列出所有查找结果
Ctrl+Shift+V:
剪贴板循环
Ctrl+
左右箭头键
:
一次可以移动一个单词
上下箭头键
滚动代码屏幕，但不移动光标位置。
Ctrl+Shift+L:
删除当前行
Ctrl+M,M:
隐藏或展开当前嵌套的折叠状态
Ctrl+M,L:
将所有过程设置为相同的隐藏或展开状态
Ctrl+M,P:
停止大纲显示
Ctrl+E,S:
查看空白
Ctrl+E,W:
自动换行
Ctrl+G:
转到指定行
Shift+Alt+
箭头键
选择矩形文本
Alt+
鼠标左按钮
Ctrl+Shift+U:
全部变为大写
Ctrl+U:
全部变为小写
代码快捷键
Ctrl+J / Ctrl+K,L:
Ctrl+Shift+
空格键
/ Ctrl+K,P:
参数信息
Ctrl+K,I:
快速信息
Ctrl+E,C / Ctrl+K,C:
注释选定内容
Ctrl+E,U / Ctrl+K,U:
取消选定注释内容
Ctrl+K,M:
生成方法存根