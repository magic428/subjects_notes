# VS2015 中将代码自动保存为 UTF8 格式   

在利用 VS2015 在编写代码时, 源代码会自动编码为 `GBK` 字符集, `GBK` 可以识别中文, 但是在英文编译环境下 `GBK` 则会显示成乱码. 这时我们需要把 `GBK` 字符集转换为另一种国际通用字符集, 即 `UTF8` 国际编码字符集. 这样在编译中遇到中文字符时就不会出现乱码现象了.   

下面就需要在 VS2015 中进行设置, 首先需要找到 VS2015 的如下路径:    

```bash
D:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcprojectitems
```

该目录下有两个文件: `hfile.h` 和 `newc++file.cpp`.  

在上面两个文件中添加如下语句:  

```cpp
#pragma once
#pragma exectuion_character_set("utf-8")
```

然后保存退出, 之后在 VS 中新建的 `头文件` 和 `cpp文件` 都会使用 `utf-8` 编码.  

## 如何将已经存在的 GBK 编码文件的编码格式设置为 UTF-8  

在 VS2015 的菜单栏中找到 文件|高级保存选项, 在高级选项中选择 UTF8 编码格式.   