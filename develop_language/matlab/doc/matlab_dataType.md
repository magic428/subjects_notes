# `Matlab`数据类型
`MATLAB`不需要任何类型声明或维度语句。当`MATLAB`遇到新的变量名称时，它将创建变量并分配适当的内存空间。   
如果变量已经存在，则`MATLAB`将使用新内容替换原始内容，并在必要时分配新的存储空间。      

## `MATLAB`数据类型   
`MATLAB`提供`15种`基本数据类型。   
每种数据类型存储矩阵或数组形式的数据。矩阵或数组的最小值是0到0，并且是可以到任何大小的矩阵或数组。    
下表显示了`MATLAB`中最常用的数据类型：    

|数据类型|	描述|
|-----|-----|
|int8	|8位有符号整数|
|uint8	|8位无符号整数|
|int16	|16位有符号整数|
|uint16	|16位无符号整数|
|int32	|32位有符号整数|
|uint32	|32位无符号整数|
|int64	|64位有符号整数|
|uint64	|64位无符号整数|
|single	|单精度数值数据|
|double	|双精度数值数据|
|logical|	逻辑值为1或0，分别代表true和false|
|char	|字符数据(字符串作为字符向量存储)|
|单元格阵列	|索引单元阵列，每个都能够存储不同维数和数据类型的数组|
|结构体	|C型结构，每个结构具有能够存储不同维数和数据类型的数组的命名字段|
|函数处理	|指向一个函数的指针|
|用户类	|用户定义的类构造的对象|
|Java类	|从Java类构造的对象|

使用以下代码创建脚本文件:    
```matlab
str = 'Hello World!'
n = 2345
d = double(n)
un = uint32(789.50)
rn = 5678.92347
c = int32(rn)
```

## `MATLAB`数据类型转换    
`MATLAB`提供了各种用于将一种数据类型转换为另一种数据类型的函数。    
下表显示了数据类型转换函数:    

|函数			|描述说明|
|-----|-----|
|char			|转换为字符数组(字符串)|
|int2str		|将整数数据转换为字符串|
|mat2str		|将矩阵转换为字符串|
|num2str		|将数字转换为字符串|
|str2double		|将字符串转换为双精度值|
|str2num		|将字符串转换为数字|
|native2unicode	|将数字字节转换为Unicode字符|
|unicode2native	|将Unicode字符转换为数字字节|
|base2dec		|将基数N字符串转换为十进制数|
|bin2dec		|将二进制数字串转换为十进制数|
|dec2base		|将十进制转换为字符串中的N数字|
|dec2bin		|将十进制转换为字符串中的二进制数|
|dec2hex		|将十进制转换为十六进制数字|
|hex2dec		|将十六进制数字字符串转换为十进制数|
|hex2num		|将十六进制数字字符串转换为双精度数字|
|num2hex		|将单数转换为IEEE十六进制字符串|
|cell2mat		|将单元格数组转换为数组|
|cell2struct	|将单元格数组转换为结构数组|
|cellstr		|从字符数组创建字符串数组|
|mat2cell		|将数组转换为具有潜在不同大小的单元格的单元阵列|
|num2cell		|将数组转换为具有一致大小的单元格的单元阵列|
|struct2cell	|将结构转换为单元格数组|

## 数据类型确定
`MATLAB`提供了用于识别变量数据类型的各种函数。    
下表提供了确定变量数据类型的函数:   

|函数|述说明|
|-----|-----|
|is			|检测状态|
|isa			|确定输入是否是指定类的对象|
|iscell		|确定输入是单元格数组|
|iscellstr	|确定输入是字符串的单元格数组|
|ischar		|确定项目是否是字符数组|
|isfield		|确定输入是否是结构数组字段|
|isfloat		|确定输入是否为浮点数组|
|ishghandle	|确定是否用于处理图形对象句柄|
|isinteger	|确定输入是否为整数数组|
|isjava		|确定输入是否为Java对象|
|islogical	|确定输入是否为逻辑数组|
|isnumeric	|确定输入是否是数字数组|
|isobject	|确定输入是否为MATLAB对象|
|isreal		|检查输入是否为实数数组|
|isscalar	|确定输入是否为标量|
|isstr		|确定输入是否是字符数组|
|isstruct	|确定输入是否是结构数组|
|isvector	|确定输入是否为向量|
|class		|确定对象的类|
|validateattributes	|检查数组的有效性|
|whos		|在工作区中列出变量，其大小和类型|

使用以下代码创建脚本文件:   
```matlab
x = 3
isinteger(x)
isfloat(x)
isvector(x)
isscalar(x)
isnumeric(x)

x = 23.54
isinteger(x)
isfloat(x)
isvector(x)
isscalar(x)
isnumeric(x)

x = [1 2 3]
isinteger(x)
isfloat(x)
isvector(x)
isscalar(x)

x = 'Hello'
isinteger(x)
isfloat(x)
isvector(x)
isscalar(x)
isnumeric(x)
```
运行文件后，产生以下结果:   
```matlab
x = 3
ans = 0
ans = 1
ans = 1
ans = 1
ans = 1
x = 1177/50
ans = 0
ans = 1
ans = 1
ans = 1
ans = 1
x =
          1          2          3
ans = 0
ans = 1
ans = 1
ans = 0
x = Hello
ans = 0
ans = 0
ans = 1
ans = 0
ans = 0
```

