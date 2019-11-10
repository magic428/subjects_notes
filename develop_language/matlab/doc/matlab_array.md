# Matlab数组
`MATLAB`中所有数据类型的所有变量都是多维数组。向量是一维数组，矩阵是二维数组。

## 1. `MATLAB`中的特殊数组
这些函数，单个参数创建一个方阵，两个参数创建矩形数组。   

|函数|说明|
|-----|-----|
|`zeros()`  |创建一个全零的数组。      |
|`ones()`   |创建一个所有元素为1的数组。   |
|`eye()`    |创建一个单位矩阵。   |
|`rand()`   |在`(0,1)`上创建均匀分布的随机数的数组。     |
|`magic()`  |创建一个魔术方阵，需要一个参数，参数必须是大于或等于`3`的标量。|

**Note**
魔方是一个平方，它产生相同的和，它的元素被逐行，逐列或者对角线地添加时。   
## 2.多维数组          
具有二维以上的数组在`MATLAB`中被称为多维数组。
### 2.1 二维数组扩展    
通常要生成一个多维数组，首先创建一个二维数组然后再扩展它。   
```matlab
a = [7 9 5; 6 1 9; 4 3 2]
a(:, :, 2)= [ 1 2 3; 4 5 6; 7 8 9]

a =
     7     9     5
     6     1     9
     4     3     2
a(:,:,1) =
     7     9     5
     6     1     9
     4     3     2
a(:,:,2) =
     1     2     3
     4     5     6
     7     8     9
```
### 2.2 使用特殊数组函数创建
还可以使用`ones()`，`zeros()`或`rand()`函数来创建多维数组。    
```matlab
b = rand(4,3,2)

b(:,:,1) =
    0.1419    0.9595    0.9340
    0.4218    0.6557    0.6787
    0.9157    0.0357    0.7577
    0.7922    0.8491    0.7431
b(:,:,2) =
    0.3922    0.0318    0.8235
    0.6555    0.2769    0.6948
    0.1712    0.0462    0.3171
    0.7060    0.0971    0.9502
```
### 2.3 使用`cat()`函数来构建多维数组
`cat()`函数沿着指定的维度连接数组列表。cat()函数的语法是：    
```matlab
B = cat(dim, A1, A2...)
```
**其中**：
  `B`是创建的新阵列;     
  `A1，A2，...`是要连接的数组;    
  `dim`是指定连接维度;    
```matlab
a = [9 8 7; 6 5 4; 3 2 1];
b = [1 2 3; 4 5 6; 7 8 9];
c = cat(3, a, b, [ 2 3 1; 4 7 8; 3 9 0])

c(:,:,1) =
     9     8     7
     6     5     4
     3     2     1
c(:,:,2) =
     1     2     3
     4     5     6
     7     8     9
c(:,:,3) =
     2     3     1
     4     7     8
     3     9     0
```
## 3. 数组函数
`MATLAB`提供以下函数来对数组内容进行排序，旋转，排列，重新成形或移位。    

|函数 描述|
|-----|-----|
|length	|向量的大小或数组的长度|
|ndims	|数组的维数|
|numel	|数组的元素数量|
|size	  |数组的每个维度的长度|
|iscolumn	|确定输入是否为列向量|
|isempty	|确定数组是否为空|
|ismatrix	|确定输入是否为矩阵|
|isrow	  |确定输入是否为行向量|
|isscalar	|确定输入是否为标量|
|isvector	|确定输入是否为向量|
|blkdiag	|从输入参数构造块对角矩阵|
|circshift|	循环移位|
|ctranspose	|复共轭转置|
|diag	    |矩阵对角矩阵和对角线|
|flipdim	|沿着指定的尺寸翻转数组|
|fliplr	  |从左到右翻转矩阵|
|flipud	  |向下翻转矩阵|
|ipermute	|反转N-D阵列的置换维度|
|permute	|重新排列N-D数组的维度|
|repmat	  |复制和平铺数组|
|reshape	|重塑数组|
|rot90	  | 旋转矩阵90度|
|shiftdim	|移动维度|
|issorted	|确定设置元素是否按排序顺序|
|sort	    |按升序或降序排列数组元素|
|sortrows	|按升序排列行|
|squeeze	|删除单例维度|
|transpose|	转置|
|vectorize|	向量化表达式|

长度，尺寸和元素数量：
```matlab
x = [7.1, 3.4, 7.2, 28/4, 3.6, 17, 9.4, 8.9];
length(x)  % length of x vector
y = rand(3, 4, 5, 2);
ndims(y)    % no of dimensions in array y
s = ['Zara', 'Nuha', 'Shamim', 'Riz', 'Shadab'];
numel(s)   % no of elements in s
```
MATLAB
运行文件时，显示以下结果 -

ans =  8
ans =  4
ans =  23
Shell
数组元素的循环移位

创建脚本文件并在其中键入以下代码 -

a = [1 2 3; 4 5 6; 7 8 9]  % the original array a
b = circshift(a,1)         %  circular shift first dimension values down by 1.
c = circshift(a,[1 -1])    % circular shift first dimension values % down by 1 
                           % and second dimension values to the left % by 1.
MATLAB
运行文件文件时，显示以下结果 -

a =
     1     2     3
     4     5     6
     7     8     9

b =
     7     8     9
     1     2     3
     4     5     6

c =
     8     9     7
     2     3     1
     5     6     4
Shell
排序数组

创建脚本文件并在其中键入以下代码 -

v = [ 23 45 12 9 5 0 19 17]  % horizontal vector
sort(v)                      % sorting v
m = [2 6 4; 5 3 9; 2 0 1]    % two dimensional array
sort(m, 1)                   % sorting m along the row
sort(m, 2)                   % sorting m along the column
MATLAB
运行文件文件时，显示以下结果 -

v =
    23    45    12     9     5     0    19    17
ans =
     0     5     9    12    17    19    23    45
m =
     2     6     4
     5     3     9
     2     0     1
ans =
     2     0     1
     2     3     4
     5     6     9
ans =
     2     4     6
     3     5     9
     0     1     2
Shell
单元阵列

单元格阵列是索引单元的数组，其中每个单元格可以存储不同维度和数据类型的数组。

单元格函数用于创建单元格数组。单元格函数的语法是 -

C = cell(dim)
C = cell(dim1,...,dimN)
D = cell(obj)
MATLAB
其中，

C是单元阵列;
dim是一个整数或整数向量，它指定单元格数组C的维数;
dim1，...，dimN是指定C大小的标量整数;
obj是以下之一：
Java数组或对象
类型为System.String或System.Object的.NET数组
示例

创建脚本文件并在其中键入以下代码 -

c = cell(2, 5);
c = {'Red', 'Blue', 'Green', 'Yellow', 'White'; 1 2 3 4 5}
MATLAB
运行文件时，得到以下结果 -

c = 
{
  [1,1] = Red
  [2,1] =  1
  [1,2] = Blue
  [2,2] =  2
  [1,3] = Green
  [2,3] =  3
  [1,4] = Yellow
  [2,4] =  4
  [1,5] = White
  [2,5] =  5
}
Shell
访问单元格数组数据

有两种方法来引用单元格数组的元素 -

将第一个括号()中的索引包围，以引用单元格集
将大括号{}中的索引括起来，以引用单个单元格内的数据
当将索引包围在第一个括号中时，它指的是这组单元格。

括号中的单元格数组索引是指单元格集。

例如：

c = {'Red', 'Blue', 'Green', 'Yellow', 'White'; 1 2 3 4 5};
c(1:2,1:2)
MATLAB
运行文件时，得到以下结果 -

ans = 
{
  [1,1] = Red
  [2,1] =  1
  [1,2] = Blue
  [2,2] =  2
}
Shell
还可以通过用花括号索引来访问单元格的内容。

例如 -

c = {'Red', 'Blue', 'Green', 'Yellow', 'White'; 1 2 3 4 5};
c{1, 2:4}
MATLAB
运行文件时，得到以下结果 -

ans = Blue
ans = Green
ans = Yellow