# ndarray 对象    

> 所有语句块都要添加的头文件    

```python
import numpy as np
from __future__ import print_function
#print(help(print))
debug = True
debug = False
def printd(*args):
    if debug:
        print(args)
```

## 创建 array
```python
a = np.array((1,2,3,4), dtype = np.int16)
b = np.array([5,6,7,8], dtype = np.int32)
c = np.array([[1,2,3,4], [5,6,7,8], [9, 10, 11, 12]])

printd(a.shape, b.shape, c.shape)
c.shape = (4,3)     # 按照元素在内存中的存储顺序重新分配行和列
c.shape = (1, -1)   # 第二个维度自动推算
d = c.reshape(2,-1) # 使用reshape()方法可以创建指定形状的新数组,而原数组的形状保持不变.需要注意的是 c 和 d 会共享存储空间.
printd('c:\n',c,'\nd:\n',d)
```

## 元素类型


```python
[key for key,val in np.typeDict.items() if val is np.float64]
#print(set(np.typeDict.values()))
printd(a.dtype.type, b.dtype.type, c.dtype.type)

n = np.int16(200)    # 创建一个16位的符号整数对象  
printd(n*n)

v1 = 3.14
v2 = np.float64(v1)  # 创建一个16位的符号整数对象  
%timeit v1*v1
%timeit v2*v2

# 使用 astype() 方法可以对数组的元素类型进行转换
t1 = np.array([1,2,3,4], dtype = np.float)
t2 = np.array([1,2,3,4], dtype = np.complex)

t3 = t1.astype(np.int32)
t4 = t2.astype(np.complex64)
printd(t1.dtype.type, t2.dtype.type, t3.dtype.type, t4.dtype.type)
```

    /usr/local/lib/python2.7/dist-packages/ipykernel_launcher.py:6: RuntimeWarning: overflow encountered in short_scalars
      


    The slowest run took 39.69 times longer than the fastest. This could mean that an intermediate result is being cached.
    10000000 loops, best of 3: 54.1 ns per loop
    The slowest run took 61.52 times longer than the fastest. This could mean that an intermediate result is being cached.
    10000000 loops, best of 3: 81.4 ns per loop


2.1.3 自动生成数组   
(1) arange(), linspace(), logspace()   
logspace()中默认的 base 为 10;    
(2) zeros() ones() empty() full()   
其中 empty() 只分配数组所使用的内存, 不对数组元素进行初始化操作, 因此其运行速度速度是最快的.   
zeros()将数组初始化为 0;   
ones()将数组初始化为 1;   
full() 将数组指定为特定值;   
(3) zeros_like() ones_like() empty_like() full_like()   
这些函数创建与参数数组形状和类型相同的数组, 因此, zeros_like(a) 和 zeros(a.shape, a.dtype)的效果相同.   
(4) fromebuffer() fromstring() fromfile()等函数可以从字节序列或文件创建数组.
fromstring() 会复制字符串的字节序列的一份副本;   
frombuffer() 创建的数组和字符串共享存储空间;   
由于字符串是只读的(不可变)对象,因此不能对数组元素进行修改.   
(5) fromfunction()   
可以先定义一个使用下标计算数值的函数,然后用fromfunction()调用此函数创建数组.   


```python
# arange(), linspace(), logspace() 
a = np.arange(0,1,0.1)
b = np.linspace(0,1,10,endpoint = False)
c = np.logspace(0,2,10, base = 2)      # 默认的 base 为 10, 这里重写为 2, 从 10^0 到 10^2 
printd(a, '\n', b, '\n', c)

# zeros() ones() empty()
# 其中 empty() 只分配数组所使用的内存, 不对数组元素进行初始化操作, 因此其运行速度速度是最快的.
a = np.empty((2,3), np.int)
b = np.zeros((4,5), np.int)
c = np.ones((3,4), np.int)  
d = np.full(4, np.pi)        # np.pi 就是 π
printd(a,'\n', b, '\n', c, '\n', d)

# zeros_like() ones_like() empty_like() full_like()
# 等函数创建与参数数组形状和类型相同的数组, 因此, zeros_like(a) 和 zeros(a.shape, a.dtype)的效果相同

# fromebuffer() fromstring() fromfile()等函数可以从字节序列或文件创建数组.
s = "abcdefgh"
s1 = np.fromstring(s, dtype = np.int8)   # 创建一个 8 位的整数数组, 每个字符表示一个元素    
s2 = np.fromstring(s, dtype = np.int16)  # 创建一个 16 位的整数数组, 两个字符表示一个元素   
printd(s1, s2)

# fromfunction()
def func(i, j):
    return (i+1)*(j+1)
a = np.fromfunction(func, (9,9))
printd(a)
```

2.1.4 存取元素   
(1) 切片存取;   
(2) 整数列表存取, 使用列表中的每个元素作为下标;   
(3) 整数数组存取, 当数组为一维数组时,效果和整数列表相同. 当下标数组为多维数组时. 得到的也是和下标数组形状相同的数组;   
(4) 布尔数组存取, 注意区别于布尔列表.它获得的数组不和原始数组共享数据内存;   
在numpy 1.10 之后的版本中, 布尔列表会被当做布尔数组.   
布尔数组一般不是手工产生,而是使用布尔运算函数ufunc()函数产生.


```python
a = np.arange(10)

# 切片存取
print(a[:-1])    # 选取除最后一个元素之外的所有元素   
print(a[::2])    # 正向每隔一个元素选择   
print(a[::-2])   # 逆向每隔一个元素选择
print(a[4:1:-2]) # 从第五个元素开始切片选择  
a[2:4] = 10, 12   # 左闭右开
# 整数列表存取
idx1 = [3,3,1,8]
idx2 = [3,-3,1,8]
print(a[idx1], a[idx2])
# 布尔数组存取
x = np.random.randint(0, 10, 6)  # 生成数组长度为 6 , 元素值为 0 到 9 的随机数组.   
print(x, x[x > 5])
```

    [0 1 2 3 4 5 6 7 8]
    [0 2 4 6 8]
    [9 7 5 3 1]
    [4 2]
    [12 12  1  8] [12  7  1  8]
    [8 8 5 0 9 4] [8 8 9]



```python
# python 的广播原理  
a = np.arange(0, 60, 10).reshape(-1,1)
b = np.arange(0, 6)
c = a + b
printd(a, "\n", b, "\n", c)
printd(c[1,2], c[(1,2)])
print(c[0, 3:5])     # 第一行, 第四个到第五个元素
print(c[4:, 4:])     # 从第五行和第五列的元素开始到右下角
print(c[2::2, ::2])  # 从第三行开始每隔一行选取,从第一列开始,每隔一列选取.

```

    [3 4]
    [[44 45]
     [54 55]]
    [[20 22 24]
     [40 42 44]]
    [[20 21]
     [40 41]]


切片对象   
(1) 单独生成切片对象要使用slice()来创建, 三个参数分别为开始值, 结束值, 间隔步长.   
当这些值需要省略时,可以使用 None.   
例如 (slice(None, None, None), 2) 和 (:, 2) 相同.   
(2) 使用 s_ 对象来帮助我们创建数组下标, s_ 是 IndexExpression 类的一个对象.   
np.s_[::2, 2:]   





```python
np.s_[::2, 2:] 
```




    (slice(None, None, 2), slice(2, None, None))



结构数组   
```python
person_type = np.dtype({
    'names': ['name', 'age', 'weight'],
    'formats': ['|S30', '<i4', '<f4']
    
    }, align = True)
```
这里使用类型字符串定义字段类型:   
- 'S30': 长度为30个字节的字符串类型;    
- 'i':  32 位的整数类型;   
- 'f':  32 位的单精度浮点类型, 相当于np.float32;   
还可以使用字节顺序字段:    
- '|': 忽视字节顺序;   
- '>': 低位字节在前, 即小端模式(little end);   
- '<': 高位字节在前, 即大端模式(big end); 
(2) a.tostring() 和 a.tofile() 可以将数组 a 以二进制的方式转换成字符串或写入文件.    
(3) %%file 为 IPython 的魔法指令, 它将该单元格中的文本保存成文件 read_srtuct_array.c .


```python
# 使用类型字符串定义字段类型   
person_type = np.dtype({
    'names': ['name', 'age', 'weight'],
    'formats': ['|S30', '<i4', '<f2']
    }, align = True)

a = np.array([("zhang", 23, 75.5), ("Wang", 24, 65.2)], dtype=person_type)
print(a.dtype, a)

# 访问字段值
b = a[0]
print(b['name'], b['age'], b['weight'])
b = a[1]
print(b['name'], b['age'], b['weight'])

# 保存到二进制文件
a.tofile("test.bin")
```

    {'names':['name','age','weight'], 'formats':['S30','<i4','<f2'], 'offsets':[0,32,36], 'itemsize':40, 'aligned':True} [('zhang', 23,  75.5   ) ('Wang', 24,  65.1875)]
    zhang 23 75.5
    Wang 24 65.188



```python
%%file read_struct_array.c
#include <stdio.h>
struct person{
    char name[30];
    int age;
    float weight;
};

struct person p[3];

void main()
{
    FILE *fp = NULL;
    int i = 0;
    fp = fopen("test.bin", "rb");
    fread(p, sizeof(struct person), 2, fp);
    fclose(fp);
    
    for(i = 0; i < 2; i++){
        printf("%s, %d, %.4f\n", p[i].name, p[i].age, p[i].weight);
    }
}
```

    Overwriting read_struct_array.c


## ufunc 函数   
ufunc 是 universal function 函数的缩写, 它是一种能对数组的每个元素进行运算的函数.
1. np.sin()  函数
np.sin() 是一个 ufunc 函数, 可以通过 out 参数来指定计算结果的保存变量.   
对于数组, np.sin() 函数比 math.sin() 函数快 10 倍多.   
对于单个数值, np.sin() 的计算速度只有 math.sin() 的 1/6.   
math.sin() 返回的是标准的 float 类型; 而 np.sin() 的返回值是 np.float64.   
通过使用数组的 item() 方法可以直接以标准的 python 数值类型返回数组中的单个元素.    
2. np.<op>.reduce(array, axis = 0, dtype = None)   
沿着参数 axis 指定的轴对数组进行操作,相当于将 <op> 运算符插入到沿着 axis 轴的所有元素之间.  
3. accumulate()   
和 reduce() 类似, 只是它返回的数组和输入数组的形状相同,保存所有的中间计算结果.  
4. outer()   
是通过广播的方式计算出来的.   
    


```python
# sin()
import math
x = np.linspace(0, 2*np.pi, 10)
%time y = np.sin(x)
%time y = [math.sin(t) for t in x]
print(x)
y is x
print(x.item(1), type(x.item(1)), type(x[1]))

## reduce()
x = np.linspace(0, 9, 10)
y = np.array([[1,2,3], [4,5,6]], dtype = np.int)
r1 = np.add.reduce(x)
r2 = np.add.reduce(y, axis = 1)  #(1+2+3), (4+5+6)
print(r1, "\n", r2)
## accumulate
r1 = np.add.accumulate(x)
r2 = np.add.accumulate(y, axis = 1)  #(1+2+3), (4+5+6)
print(r1, "\n", r2)
```

    CPU times: user 0 ns, sys: 0 ns, total: 0 ns
    Wall time: 21 µs
    CPU times: user 0 ns, sys: 0 ns, total: 0 ns
    Wall time: 36 µs
    [ 0.          0.6981317   1.3962634   2.0943951   2.7925268   3.4906585
      4.1887902   4.88692191  5.58505361  6.28318531]
    0.698131700798 <type 'float'> <type 'numpy.float64'>
    45.0 
     [ 6 15]
    [  0.   1.   3.   6.  10.  15.  21.  28.  36.  45.] 
     [[ 1  3  6]
     [ 4  9 15]]


## 四则运算  
ufunc 函数    
np.add()  # np.add(a, b, a) => a += b    
np.substract()
np.multiply()   
np.divide()    
np.true_divide()   
np.floor_divide()   
np.negtive()
np.power()   
np.remainder() = np.mod()  取模.   


```python
x = np.linspace(0, 9, 10)
print(x, np.power(x,x))
print(np.remainder(x,x))
```

    [ 0.  1.  2.  3.  4.  5.  6.  7.  8.  9.] [  1.00000000e+00   1.00000000e+00   4.00000000e+00   2.70000000e+01
       2.56000000e+02   3.12500000e+03   4.66560000e+04   8.23543000e+05
       1.67772160e+07   3.87420489e+08]
    [ nan   0.   0.   0.   0.   0.   0.   0.   0.   0.]


    /usr/local/lib/python2.7/dist-packages/ipykernel_launcher.py:3: RuntimeWarning: invalid value encountered in remainder
      This is separate from the ipykernel package so we can avoid doing imports until


The elements of `a` are read using this index order. 'C' means
to index the elements in row-major, C-style order,
with the last axis index changing fastest, back to the first
axis index changing slowest.  'F' means to index the elements
in column-major, Fortran-style order, with the
first index changing fastest, and the last index changing
slowest. Note that the 'C' and 'F' options take no account of
the memory layout of the underlying array, and only refer to
the order of axis indexing.  'A' means to read the elements in
Fortran-like index order if `a` is Fortran *contiguous* in
memory, C-like order otherwise.  'K' means to read the
elements in the order they occur in memory, except for
reversing the data when strides are negative.  By default, 'C'
index order is used.