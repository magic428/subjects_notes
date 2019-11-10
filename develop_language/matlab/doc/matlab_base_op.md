# `MATLAB`基本操作

1. 小数点显示格式命令   
默认情况下，`MATLAB`显示`四位小数`位数。这称为：短格式。但是，如果要更精确，则需要使用`format`命令。       
```matlab
format short   # 默认显示
format long    # 命令显示十进制后的16位数字。
format bank	   # 命令将数字舍入到小数点后两位
format short e # 命令以指数形式显示四位小数加上指数。
format long e  # 命令允许以指数形式显示十六位小数加上指数。 例如，
format rat	   # 命令给出计算结果最接近的合理表达式。 例如，
```

2. 创建向量    
`行向量`是通过用方括号中的元素集合来创建的，使用空格或逗号分隔元素。    
`列向量`通过用方括号中的元素集合来创建，使用分号(;)来分隔元素。   
3. 创建矩阵
矩阵是数字的二维数组。在`MATLAB`中，通过将每行作为一系列空格或逗号分隔的元素输入矩阵，并以行号分隔一行。 例如，创建一个`3x3`的矩阵：  
```
m = [1 2 3; 4 5 6; 7 8 9]
```
4. 使用脚本文件   
在命令行中也可以创建文件夹，如：创建一个名为`progs`的文件夹。在命令提示符下键入以下命令(>>)：
```matlab
mkdir progs    % create directory progs under default directory
chdir progs    % changing the current directory to progs
edit  prog1.m  % creating an m file named prog1.m
```
   







