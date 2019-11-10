# jupyter-notebook 中的神奇功能      

> jupyter-notebook 中的工具.   

## 逐行模式 和 逐单元模式    

逐行模式(% 开头)是执行一行的命令, 而逐单元模式(%% 开头)是执行一个 cell.  

`%` 开头为行命令, `%%` 开头为 cell 命令.

## % magic 指令   
### 1. 测试代码运行时间  time   
`%timeit` 用于测试单行语句的执行时间, 会重复计算多次取平均值.  
`%%timeit` 用于测试整个单元中代码的执行时间, 会重复计算多次取平均值.  
`%time` 则只输出本次执行消耗的时间.

```python
%%time
a = []
for i in range(10):
    a.append(i)
```
最终会被转换为 ipython 的以下代码:   
```cpp
get_ipython().run_cell_magic('time', '', 'a = []\nfor i in range(10):\n    a.append(i)')
```

### 2. 捕获程序输出信息   
%%capture 将单元格的输出保存为一个对象.    
```python
%%capture time_results
import random
for n in [1000, 5000, 10000, 50000, 100000,500000]:
    print "n = {0}".format(n)
    alist = range(n)
    %time random.shuffle(alist)
```
最终会被转换为 ipython 的以下代码:   
```cpp
get_ipython().run_cell_magic('capture', 'time_results', 'import random\nfor n in [1000, 5000, 10000, 50000, 100000,500000]:\n    print "n = {0}".format(n)\n    alist = range(n)\n    %time random.shuffle(alist)')
```
这样就将上面 cell 的输出保存到 time_results 对象中. 可以使用 print 来查看对象中的内容:      
```python
print time_results.stdout   

n = 1000
CPU times: user 4 ms, sys: 0 ns, total: 4 ms
Wall time: 1.48 ms
n = 5000
CPU times: user 12 ms, sys: 0 ns, total: 12 ms
Wall time: 7.02 ms
n = 10000
CPU times: user 16 ms, sys: 0 ns, total: 16 ms
Wall time: 13 ms
n = 50000
CPU times: user 52 ms, sys: 4 ms, total: 56 ms
Wall time: 51.1 ms
n = 100000
CPU times: user 44 ms, sys: 0 ns, total: 44 ms
Wall time: 46.2 ms
n = 500000
CPU times: user 216 ms, sys: 0 ns, total: 216 ms
Wall time: 217 ms
```

### 3. 性能分析
%%prun 命令调用 profile 模块对单元中的代码进行性能分析.   

### 4. 代码调试
%debug 命令用于调试代码.    
一种是在执行代码之前设置断点进行调试;    
另一种是在代码抛出异常之后,执行 %debug 命令查看调用堆栈.   

### 5. ipython 相关的目录操作   
```python
import os
ipython = get_ipython()
print ipython.ipython_dir
print ipython.config.ProfileDir.location

/home/klm/.ipython
<traitlets.config.loader.LazyConfigValue object at 0x7f14080bacd0>
```


## 交互式仪表盘   

```python
from ipywidgets import widgets

outputText = widgets.Text()
outputText
```
```python
inputText = widgets.Text()

def makeUpperCase(sender):
    outputText.value = inputText.value.upper()
    

inputText.on_submit(makeUpperCase)
inputText
```