# numpy 的使用   

numpy.array implements a one or more dimensional array. Its type is numpy.ndarray, and we will refer to this as an ndarray for short.

```python 
import numpy as np
```

## 1.创建 ndarray 对象  

(1) 使用已有的数组创建 ndarray 对象.   

```py 
l = [1, 2, 3]     # you can also use tuples: (4,5,6)
x = np.array(l)
```

(2) 使用多层方括号创建多维向量.   

```py 
x = np.array([[1, 2, 3],
              [4, 5, 6]])
```

(3) 使用数值范围创建数组.  

3.1) numpy.arange  

根据 start 与 stop 指定的范围以及 step 设定的步长，生成一个 ndarray。  
其中, start 默认为1, step 默认为 1.  

```py
numpy.arange([start,] stop[, step,], dtype=None)
```

3.2) numpy.linspace  

numpy.linspace 函数用于创建一个一维数组，数组是一个等差数列构成的.  

```py
numpy.linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None)
```
3.3) numpy.logspace  

numpy.logspace 函数用于创建一个于等比数列。格式如下：

```py
numpy.logspace(start, stop, num=50, endpoint=True, base=10.0, dtype=None)
```
start  序列的起始值为：base ** start  
stop   序列的终止值为：base ** stop。如果 endpoint 为 True，该值包含于数列中  
base   参数是取对数的时候 log 的下标。  

使用实例:  

```py
# 使用 reshape 中转   
npa_arange = np.arange(24)  
# 现在调整其大小
b = npa_arange.reshape(2,4,3)  # b 现在拥有三个维度

npa_lins = np.linspace(1, 10, 10)
npa_logs = np.logspace(1.0, 2.0, num = 10)  
```

(4) numpy.empty, numpy.zeros, numpy.ones 方法

numpy.empty 方法用来创建一个指定形状（shape）、数据类型（dtype）且未初始化的数组.  

```py
numpy.empty(shape, dtype = float, order = 'C')
numpy.zeros(shape, dtype = float, order = 'C')
numpy.ones(shape, dtype = float, order = 'C')
# eg.
x = np.empty([3,2], dtype = int) 
x = np.zeros([3,2], dtype = int) 
x = np.ones([3,2], dtype = int) 
```


## 2.元素的数据类型.   

通过重写 dtype 参数实现, 支持的数据类型有:  

- bool_ 布尔型数据类型（True 或者 False）;  
- complex64, complex128;
- np.int8, np.int16, np.int32, np.int64;   
- np.uint8,np.uint16,np.uint32,np.uint64;  
- np.float16, np.float32, np.float64, np.float128;  

```py 
x = np.array([1, 2, 3], dtype=float)
```

## 3.numpy 对象的属性  

ndarray.ndim        秩，即轴的数量或维度的数量
ndarray.shape       数组的维度，对于矩阵，n 行 m 列
ndarray.size        数组元素的总个数，相当于 .shape 中 n*m 的值
ndarray.dtype       ndarray 对象的元素类型
ndarray.itemsize    ndarray 对象中每个元素的大小，以字节为单位
ndarray.flags       ndarray 对象的内存信息
ndarray.real        ndarray 元素的实部
ndarray.imag        ndarray 元素的虚部
ndarray.data        包含实际数组元素的缓冲区 

## 4.访问元素 

(1) 使用下标访问向量元素.   

```py 
x = np.array([[1, 2, 3],
              [4, 5, 6]])
print(x[1,2])
```

(2) "切片" 访问整行或整列.  

通过内置的 slice 函数，并设置 start, stop 及 step 参数进行，从原数组中切割出一个新数组。  

```py
a = np.arange(10)
s = slice(2,7,2)   # 从索引 2 开始到索引 7 停止，间隔为2
```

也可以通过冒号分隔切片参数 start:stop:step 来进行切片操作：  

实例
```py
b = a[2:7:2]   # 从索引 2 开始到索引 7 停止，间隔为 2
```

**冒号 (:) 的解释**  

- 对于某一维度内部的冒号. 如果只放置一个参数，如 [2]，将返回与该索引相对应的单个元素。如果为 [2:]，表示从该索引开始以后的所有项都将被提取。如果使用了两个参数，如 [2:7]，那么则提取两个索引(不包括停止索引)之间的项。  
- 对于维度之间的冒号. 表示某行或者某列的所有元素. 因此 x[:, 0] 返回第一列的所有数据.    
- 特别地, 在多维数组内使用冒号, 应注意逗号的书写.  

```py 
x = np.array([[1,2,3], [3,4,5], [4,5,6]])

x_col0 = x[:, 0]    # 获取第一列
x_row1 = x[1, :]     # 获取第二行

print(x)
print('从数组索引 x[1:] 处开始切割')
print(x[1:])    # 等价于 print(x[1:,:])
```

**省略号 …**  

切片还可以包括省略号 …，来使选择元组的长度与数组的维度相同。 如果在行位置使用省略号，它将返回包含行中元素的 ndarray。  

```py 
x_col0 = x[..., 0]     # 获取第一列, 等价于 x[:, 0]
x_row1 = x[1, ...]     # 获取第二行, 等价于 x[1, :] 
```

(3) 负数索引 

对于 Python 的 list, 可以使用负数的 indexes 倒着访问 array 中的元素.   

-1 访问的是最后一个元素, -2 访问的是倒数第二个元素, 以此类推.   

```py
x_row2_last2 = x[1,-2:]   # 获取第二行的最后两个元素
```

(4) 高级索引  

包括整数数组索引、布尔索引及花式索引。  

(4.1) 整数数组索引   

以下实例获取数组中(0,0)，(1,1)和(2,0)位置处的元素。

```py
x = np.array([[1,  2],  [3,  4],  [5,  6]]) 
y = x[[0,1,2],  [0,1,0]]  
print (y)
```
输出结果为：

[1  4  5]

可以看到输出的数组为一维数组.  

以下实例获取了 4x3 数组中的四个角的元素。 行索引是 [0,0] 和 [3,3]，而列索引是 [0,2] 和 [0,2]。

```py 
x = np.array([[  0,  1,  2],[  3,  4,  5],[  6,  7,  8],[  9,  10,  11]])  
rows = np.array([[0,0],[3,3]]) 
cols = np.array([[0,2],[0,2]]) 
y = x[rows,cols]  
print  ('这个数组的四个角元素是：')
print (y)
```
输出结果为：

[[ 0  1  2]
 [ 3  4  5]
 [ 6  7  8]
 [ 9 10 11]]
这个数组的四个角元素是：
[[ 0  2]
 [ 9 11]]

返回的结果是包含每个角元素的 ndarray 对象, 可以看到输出的数组为二维数组。  

可以借助切片 : 或 … 与索引数组组合。如下面例子：  

```py
a = np.array([[1,2,3], [4,5,6],[7,8,9]])
b = a[1:3, 1:3]
c = a[1:3,[1,2]]  # 行, 列同时切片  
d = a[...,1:]     # 第2列到最后一列  
print(b)
print(c)
print(d)
```

输出结果为：

[[5 6]
 [8 9]]
[[5 6]
 [8 9]]
[[2 3]
 [5 6]
 [8 9]]

(4.2) 布尔索引  

我们可以通过一个布尔数组来索引目标数组, 通过布尔运算（如：比较运算符）来得到符合指定条件的布尔数组。  

```py
# 1. 获取大于 5 的元素  
x = np.array([[  0,  1,  2],[  3,  4,  5],[  6,  7,  8],[  9,  10,  11]])  
print (x[x >  5])

# 2. 使用 ~（取补运算符）来过滤 NaN。
a = np.array([np.nan, 1, 2, np.nan, 3, 4, 5])  
print (a[~np.isnan(a)])

# 3. 从数组中过滤掉非复数元素。
y = np.array([1,  2+6j,  5,  3.5+5j])  
print (y[np.iscomplex(y)])
```

(4.3) 花式索引  

花式索引指的是利用整数数组进行索引。花式索引根据索引数组的值作为目标数组的某个轴的下标来取值。对于使用一维整型数组作为索引，如果目标是一维数组，那么索引的结果就是对应位置的元素；如果目标是二维数组，那么就是对应下标的行。  

花式索引跟切片不一样，它总是将数据复制到新数组中。   

```py
# 1、传入顺序索引数组   
x=np.arange(32).reshape((8,4))
print (x[[4,2,1,7]])

# 2、传入倒序索引数组
print (x[[-4,-2,-1,-7]])

# 3、传入多个索引数组（要使用 np.ix_） 
print (x[np.ix_([1,5,7,2], [0,3,1,2])])
```
np.ix_() 内的参数进行叉乘之后,确定最终的 index. 例如:  

`a[np.ix_([1,3],[2,5])]` 返回的是 `[[a[1,2] a[1,5]], [a[3,2] a[3,5]]]`.  






7. 矩阵运算.  
|operator|运算|
|-----|------|
|+                 	|矩阵加法  |
|*                 	|矩阵对应元素相乘|
|np.dot         	|矩阵乘法(线性代数) |
|.T                 |矩阵转置   |
|numpy.linalg.inv 	|矩阵的逆|
```python
x = np.array([[1., 2.],
              [3., 4.]])
print('abddition:\n', x + x)
print('\nelement-wise multiplication\n', x * x)
print('\nmultiplication\n', np.dot(x, x))
print('\ndot is also a member of np.array\n', x.dot(x))
```
**Note**:  
Python 3.5 引入了 @ 操作符来表示矩阵乘法.   
8. Helper functions, 矩阵生成辅助函数.    
zeros() to create a matrix of all zeros.    
ones() to get all ones.    
eye() to get the identity matrix.    
这三个函数均可以传入一个 tuple 参数来指定生成矩阵的维度.   
```python
print('zeros\n', np.zeros(7))
print('\nzeros(3x2)\n', np.zeros((3, 2)))
print('\neye\n', np.eye(3))
```
9. 等间距矩阵.   
(1) np.arange(start, stop, step), 类似于 Python 的 range() 函数, 不同的是它返回的是 NumPy array 类型.   
(2) np.linspace(start, stop, num), 线性等分向量和 np.arange() 稍微不同, num 是 array 的长度.   
```python
print('\narange\n', np.arange(0, 2, 0.1))
print('\nlinspace\n', np.linspace(0, 2, 20))
```
10. 数据图形化.  
Matplotlib 包含了图形显示库 pyplot. 标准作坊式 import it as plt. 一旦导入之后,就可以使用 plt.plot 来调用成员函数绘图.可以通过多次调用 plt.plot 函数绘制一系列的曲线,不同曲线以不同颜色区分.   
```python
import matplotlib.pyplot as plt
a = np.array([6, 3, 5, 2, 4, 1])
plt.plot([1, 4, 2, 5, 3, 6])
plt.plot(a)
plt.show()
```
**Note:**   
plt.plot 假设 x 轴以 1 的长度递增, 因此用户可以向 plt.plot() 函数传递 x, y 来指定自己的 x 轴数据.    
```python
plt.plot(np.arange(0,1, 0.1), [1,4,3,2,6,4,7,3,4,5]);```
