# Visual Studio 2015 软件使用手册

## VS2015 的快捷键操作  

|         查看源码相关         |           功能           |
|-----------------------------|--------------------------|
| Ctrl + Enter                | 在光标指定位置的上面添加一行 |
| Ctrl + Shift + Enter        | 在光标指定位置的下面添加一行 |
| F12                         | 转到定义                |
| CTRL + F12                  | 转到声明                |
| CTRL + -                    | 返回定义，与F12组合使用 |
| CTRL + SHIFT + -            | 向前定位               |
| SHIFT + ALT + ENTER         | 全屏显示  |
| CTRL + PAGE DOWN            | 光标定位到窗口上方 |
| CTRL + PAGE UP              | 光标定位到窗口下方 |
| CTRL + TAB / F6             | 下一个文档窗口 |
| CTRL + SHIFT + TAB / F6     | 上一个文档窗口    |
| CTRL + K, CTRL + C          | 注释选择的代码    |
| CTRL + K, CTRL + U          | 取消对选择代码的注释  |
| CTRL + M, CTRL + O          | 折叠代码定义  |
| CTRL + M, CTRL + L          | 展开代码定义  |
| CTRL + DELETE               | 删除至词尾    |
| CTRL + BACKSPACE            | 删除至词头    |
| CTRL + U                    | 转小写    |
| CTRL + SHIFT + U            | 转大写    |
| CTRL + SHIFT + END          | 选择至文档末尾    |
| CTRL + SHIFT + HOME         | 选择至文档末尾开始    |
| CTRL + END                  | 文档定位到最后    |
| CTRL + HOME                 | 文档定位到最前    |
| SHIFT + END                 | 选择至行尾    |
| SHIFT + HOME                | 选择至行开始处    |
| SHIFT + ALT + END           | 垂直选择到最后尾    |
| SHIFT + ALT + HOME          | 垂直选择到最前面    |
| CTRL + SHIFT + PAGE UP      | 选择至本页前面  |
| CTRL + SHIFT + PAGE DOWN    | 选择至本页后面  |
| CTRL + W                    | 选择当前单词    |
| CTRL + G                    | 转到    |
| ALT + F10                   | 调试    |
| CTRL + ALT+ Break           | 停止调试     |
| CTRL + SHIFT + F9           | 取消所有断点     |
| CTRL + F9                   | 允许中断     |
| CTRL + SHIFT + F5           | 重新开始     |
| F5                          | 运行调试     |
| CTRL + F5                   | 运行不调试       |
| F10                         | 跨过程序执行     |
| Ctrl+F10                    | 运行到光标处     |
| F11                         | 单步逐句执行     |
| F9                          | 切换断点     |
| Ctrl+F9                     | 启用     |
| Ctrl+F                      | 查找     |
| Ctrl+Shift+F                | 在文件中查找     |
| CTRL + SHIFT + B            | 生成解决方案     |
| CTRL + F7                   | 生成编译     |
| CTRL + SHIFT + O            | 打开项目     |

## VS2015 在 release 模式下进行调试  

步骤如下:   

1. 工程项目上右键 | 属性;  
2. c++ | 常规 | 调试信息格式 | 程序数据库(/Zi)或(/ZI), 注意: 如果是库的话，只能(Zi);  
3. c/c++ | 优化 | 优化 | 配置 | 禁止（/Od）;  
4. 连接器 | 调试 | 生成调试信息 | 是 （/DEBUG）.  

**注意**: 在发布程序的时候将这些徐安全设置为合适的参数即可.  
