# 愉快地迁移到 Python 3

英文：Alex Rogozhnikov: http://python.jobbole.com/89031/  

Python 已经成为机器学习和一些需处理大量数据的科学领域的主流语言。它支持了许多深度学习框架和其他已确立下来的数据处理和可视化的工具集。  

为了让大家能够顺利过渡，我收集了一系列 Python 3 特性，希望对大家有用。   

## 1. 使用 pathlib 更好的对路径进行处理

pathlib 是 Python 3 中的默认模块，能帮你避免过多的使用 os.path.join：   

```python
from pathlib import Path
 
dataset = 'wiki_images'
datasets_root = Path('/path/to/datasets/')
 
train_path = datasets_root / dataset / 'train'
test_path = datasets_root / dataset / 'test'
 
for image_path in train_path.iterdir():
    with image_path.open() as f: # note, open is a method of Path object
        # do something with an image
```

在之前的版本中总是不可避免的使用字符串连接（简洁但明显可读性很差），如今使用 pathlib 后，代码会更安全、简洁、易读。   

同时 pathlib.Path 提供了一系列方法和特性，这样一来 python 的初学者就不需搜索了：  

```python
p.exists()
p.is_dir()
p.parts
p.with_name('sibling.png') # only change the name, but keep the folder
p.with_suffix('.jpg') # only change the extension, but keep the folder and the name
p.chmod(mode)
p.rmdir()
```

pathlib 会节省你大量的时间.  

## 2. 类型提示现在是 Python 的一部分啦

Python 已不再是一个小型的脚本语言了， 如今的数据处理流程包含许多步骤，每步涉及不同的构架（而且有时会涉及不同的逻辑）  

引入类型提示功能有助于处理日渐复杂的程序，因此机器就可以帮助实现代码验证。 而以前是不同的模块需使用自定义的方式在文档字符串（doctrings）中指定类型（提示：pycharm 能够将旧的 doctrings 转换为新的 Type hinting）。  

下图是一个简单的例子，这段代码对不同类型的数据均有效（这正是我们喜欢 Python 数据栈的原因）。  

```python
def repeat_each_entry(data):
    """ Each entry in the data is doubled
    <blah blah nobody reads the documentation till the end>
    """
    index = numpy.repeat(numpy.arange(len(data)), 2)
    return data[index]
```

这段代码样例适用于 numpy.array（含多维数组）、astropy.Table 及 astropy.Column、bcolz、cupy、mxnet.ndarray 等。   

这段代码可用于 pandas.Series，但方式不对：  

```python
repeat_each_entry(pandas.Series(data=[0, 1, 2], index=[3, 4, 5])) # returns Series with Nones inside
```

这还仅是两行代码。想象一下一个复杂系统的行为将是多么的难以预测，仅因一个函数就有可能行为失常。在大型系统中，明确各类方法的期望类型是非常有帮助的，这样会在函数未得到期望的参数类型时给出警告。   

```python
def repeat_each_entry(data: Union[numpy.ndarray, bcolz.carray]):
```

如果你有重要的代码库， MyPy 这样的提示工具很可能成为持续集成途径的一部分。由 Daniel Pyrathon 发起的名为“让类型提示生效”的在线教程可为您提供一个很好的介绍。  

旁注：不幸的是，类型提示功能还未强大到能为 ndarrays 或 tensors 提供细粒度分型，但是或许我们很快就可拥有，这也将是 DS 的特色功能。   

1) 类型提示 - 运行中的类型检查

在默认情况下，函数注释不会影响你代码的运行，但也仅能提示你代码的目的。  

然而，你可以使用像 enforce 这样的工具在运行中强制类型检查，这有助你调试（当类型提示不起作用时会出现很多这样的情况）。  

```python
@enforce.runtime_validation
def foo(text: str) -> None:
    print(text)
 
foo('Hi') # ok
foo(5)    # fails
 
 
@enforce.runtime_validation
def any2(x: List[bool]) -> bool:
    return any(x)
 
any ([False, False, True, False]) # True
any2([False, False, True, False]) # True
 
any (['False']) # True
any2(['False']) # fails
 
any ([False, None, "", 0]) # False
any2([False, None, "", 0]) # fails
```

## 函数注释的其他惯例

如前所述，函数注释不会影响代码的执行，但是它可提供一些供你随意使用的元信息（译者注：关于信息的信息）。   

例如，计量单位是科学领域的一个常见问题， astropy 包能够提供一种简单装饰器来控制输入量的单位并将输出量转换成所需单位。  

```python
# Python 3
from astropy import units as u

@u.quantity_input()
def frequency(speed: u.meter / u.s, wavelength: u.m) -> u.terahertz:
    return speed / wavelength
    
frequency(speed=300_000 * u.km / u.s, wavelength=555 * u.nm)
# output: 540.5405405405404 THz, frequency of green visible light
```

如果你正使用 Python 处理表格式科学数据（数据量很大），你应该试一试 astropy。

你也可以定义你的专用装饰器以同样的方法对输入量和输出量进行控制或转换。

## 使用 @ 进行矩阵乘积

我们来执行一个简单的机器学习模型，带 L2 正则化的线性回归（也称脊回归）：

```python
# l2-regularized linear regression: || AX - b ||^2 + alpha * ||x||^2 -> min
 
# Python 2
X = np.linalg.inv(np.dot(A.T, A) + alpha * np.eye(A.shape[1])).dot(A.T.dot(b))
# Python 3
X = np.linalg.inv(A.T @ A + alpha * np.eye(A.shape[1])) @ (A.T @ b)
```

使用 `@` 的代码更可读也更容易在各深度学习架构间转译： 一个单层感知器可以在　numpy、cupy、pytorch、tensorflow（和其他操作张量的框架）下运行相同的代码　X @ W + b[None, :] 实现。  

## 使用 `**` 作通配符

递归文件夹的通配符在 python 2 中实现起来并不简单，实际上我们要自定义 glob2 模块来克服这个问题。而从 Python 3.6 以后将支持遍历标志：  

```python
import glob
 
# Python 2
found_images =
    glob.glob('/path/*.jpg')
  + glob.glob('/path/*/*.jpg')
  + glob.glob('/path/*/*/*.jpg')
  + glob.glob('/path/*/*/*/*.jpg')
  + glob.glob('/path/*/*/*/*/*.jpg')
 
# Python 3
found_images = glob.glob('/path/**/*.jpg', recursive=True)
```

在 python 3 中有更好的选择，那就是使用 pathlib（-1 导入！）：  

```python
# Python 3
found_images = pathlib.Path('/path/').glob('**/*.jpg')
```

## print 现在是函数

没错，现在写代码需要这些烦人的圆括号，但是这有许多好处：   

简化使用文件描述符的语法：

```python
print >> sys.stderr, "critical error"      # Python 2
print("critical error", file=sys.stderr)  # Python 3
```

无需 str.join 输出制表符：   

```python
# Python 3
print(*array, sep='\t')
print(batch, epoch, loss, accuracy, time, sep='\t')
```

改写或重定义 print 的输出  

```python
# Python 3
_print = print # store the original print function
def print(*args, **kargs):
    pass  # do something useful, e.g. store output to some file
```

在 jupyter 中，可以将每一个输出记录到一个独立的文档（以跟踪断线之后发生了什么），这样一来我们就可以重写 print 函数了。  


下面你可以看到名为 contextmanager 的装饰器暂时重写 print 函数的方式：  

```python
@contextlib.contextmanager
def replace_print():
    import builtins
    _print = print # saving old print function
    # or use some other function here
    builtins.print = lambda *args, **kwargs: _print('new printing', *args, **kwargs)
    yield
    builtins.print = _print
 
with replace_print():
    <code here will invoke other print function>
```

这种方法并不推荐，因为此时有可能出现些小问题。   

- print 函数可参与列表理解和其他语言构建。  

```python
# Python 3
result = process(x) if is_valid(x) else print('invalid item: ', x)
```

## 数值中的下划线（千位分隔符）

PEP-515 在数值中引入下划线。在 Python 3 中，下划线可用于整数、浮点数、复数的位数进行分组，增强可视性。

```python
# grouping decimal numbers by thousands
one_million = 1_000_000
 
# grouping hexadecimal addresses by words
addr = 0xCAFE_F00D
 
# grouping bits into nibbles in a binary literal
flags = 0b_0011_1111_0100_1110
 
# same, for string conversions
flags = int('0b_1111_0000', 2)
```

使用 f-strings 简便可靠的进行格式化

默认的格式化系统具有一定的灵活性，但这却不是数据实验所需要的。这样改动后的代码要么太冗长，要不太零碎。

典型的数据科学的代码会反复的输出一些固定格式的日志信息。常见代码格式如下：

```python
# Python 2
print('{batch:3} {epoch:3} / {total_epochs:3}  accuracy: {acc_mean:0.4f}±{acc_std:0.4f} time: {avg_time:3.2f}'.format(
    batch=batch, epoch=epoch, total_epochs=total_epochs,
    acc_mean=numpy.mean(accuracies), acc_std=numpy.std(accuracies),
    avg_time=time / len(data_batch)
))
 
# Python 2 (too error-prone during fast modifications, please avoid):
print('{:3} {:3} / {:3}  accuracy: {:0.4f}±{:0.4f} time: {:3.2f}'.format(
    batch, epoch, total_epochs, numpy.mean(accuracies), numpy.std(accuracies),
    time / len(data_batch)
))

```

样本输出：

    120  12 / 300  accuracy: 0.8180±0.4649 time: 56.60

f-strings 全称为格式化字符串，引入到了 Python 3.6：

```python
# Python 3.6+
print(f'{batch:3} {epoch:3} / {total_epochs:3}  accuracy: {numpy.mean(accuracies):0.4f}±{numpy.std(accuracies):0.4f} time: {time / len(data_batch):3.2f}')
```

同时，写查询或者进行代码分段时也非常便利：

```python
query = f"INSERT INTO STATION VALUES (13, {city!r}, {state!r}, {latitude}, {longitude})"
```

重点：别忘了转义字符以防 SQL 注入攻击。

‘真实除法’与‘整数除法’的明确区别

这对于数据科学而言是非常便利的改变（但我相信对于系统编程而言却不是）

```python
data = pandas.read_csv('timing.csv')
velocity = data['distance'] / data['time']
```

在 Python 2 中结果正确与否取决于‘时间’和‘距离’（例如，以秒和米做测量单位）是否以整型来存储。而在 Python 3 中这两种除法的结构都正确，因为商是以浮点型存储的。

另一个案例是整数除法现在已经作为一个显式操作：

```python
n_gifts = money // gift_price  # correct for int and float arguments
```

注意：这个特性既适用于内置类型又适用于由数据包（比如：numpy 或者 pandas）提供的自定义类型。

严格排序

```python
# All these comparisons are illegal in Python 3
3 < '3'
2 < None
(3, 4) < (3, None)
(4, 5) < [4, 5]
 
# False in both Python 2 and Python 3
(4, 5) == [4, 5]
```

防止不同类型实例的偶然分类

```python
sorted([2, '1', 3])  # invalid for Python 3, in Python 2 returns [2, 3, '1']
```

有助于指示处理原始数据时发生的问题
旁注：适当的检查 None（两种版本的 Python 均需要）

```python

if a is not None:
  pass
  
if a: # WRONG check for None
  pass
```

自然语言处理（NLP）中的统一编码标准（Unicode）

```python
s = '您好'
print(len(s))
print(s[:2])
```

输出：

    Python 2: 6n��
    Python 3: 2n您好.

x = u'со'
x += 'co' # ok
x += 'со' # fail

Python 2 失效而 Python 3 如期输出（因为我在字符串中使用了俄文字母）

在 Python 3 中 strs 是 unicode 字符串，这更方便处理非英语文本的 NPL。

还有其他好玩的例子，比如：

'a' < type < u'a'  # Python 2: True
'a' < u'a'         # Python 2: False

from collections import Counter
Counter('Möbelstück')

Python 2 是：Counter({‘xc3’: 2, ‘b’: 1, ‘e’: 1, ‘c’: 1, ‘k’: 1, ‘M’: 1, ‘l’: 1, ‘s’: 1, ‘t’: 1, ‘xb6’: 1, ‘xbc’: 1})
Python 3 是：Counter({‘M’: 1, ‘ö’: 1, ‘b’: 1, ‘e’: 1, ‘l’: 1, ‘s’: 1, ‘t’: 1, ‘ü’: 1, ‘c’: 1, ‘k’: 1})
虽然在 Python 2 中这些可以正确处理，但在 Python 3 下会更加友好。

## 字典和 **kwargs 的保存顺序

在 CPython 3.6+ 中，默认情况下字典的行为类似于 OrderedDict（这在 Python 3.6+ 版本中已被保证）。这样可在理解字典（及其他操作，例如：json 序列化或反序列化）时保持了顺序。

```python
import json
x = {str(i):i for i in range(5)}
json.loads(json.dumps(x))
# Python 2
{u'1': 1, u'0': 0, u'3': 3, u'2': 2, u'4': 4}
# Python 3
{'0': 0, '1': 1, '2': 2, '3': 3, '4': 4}
```
这同样适用于 **kwargs（在 Python 3.6+ 中），即按照 **kwargs 在参数中出现的顺序来保存。在涉及到数据流这个顺序是至关重要的，而以前我们不得不用一种麻烦的方法来实现。


```python
from torch import nn
 
# Python 2
model = nn.Sequential(OrderedDict([
          ('conv1', nn.Conv2d(1,20,5)),
          ('relu1', nn.ReLU()),
          ('conv2', nn.Conv2d(20,64,5)),
          ('relu2', nn.ReLU())
        ]))
 
# Python 3.6+, how it *can* be done, not supported right now in pytorch
model = nn.Sequential(
    conv1=nn.Conv2d(1,20,5),
    relu1=nn.ReLU(),
    conv2=nn.Conv2d(20,64,5),
    relu2=nn.ReLU())
)      
```

你注意到了吗？命名的惟一性也是自动检查的。

## 迭代拆封

```python
# handy when amount of additional stored info may vary between experiments, but the same code can be used in all cases
model_paramteres, optimizer_parameters, *other_params = load(checkpoint_name)
 
# picking two last values from a sequence
*prev, next_to_last, last = values_history
 
# This also works with any iterables, so if you have a function that yields e.g. qualities,
# below is a simple way to take only last two values from a list
*prev, next_to_last, last = iter_train(args)
```

使用默认的 pickle 工具更好的压缩数组

```python
# Python 2
import cPickle as pickle
import numpy
print len(pickle.dumps(numpy.random.normal(size=[1000, 1000])))
# result: 23691675
 
# Python 3
import pickle
import numpy
len(pickle.dumps(numpy.random.normal(size=[1000, 1000])))
# result: 8000162
```

节省三倍空间，并且更快速。实际上 protocol=2 参数可以实现相同的压缩（但是速度不行），但是使用者基本上都会忽视这个选项（或者根本没有意识到）。

更安全的解析

```python

labels = <initial_value>
predictions = [model.predict(data) for data, labels in dataset]
 
# labels are overwritten in Python 2
# labels are not affected by comprehension in Python 3
```

## 类构造函数中超级简单的 super 函数

在 Python 2 中，super(…) 是代码里常见的错误源。

# Python 2
class MySubClass(MySuperClass):
    def __init__(self, name, **options):
        super(MySubClass, self).__init__(name='subclass', **options)
        
# Python 3
class MySubClass(MySuperClass):
    def __init__(self, name, **options):
        super().__init__(name='subclass', **options)

更多关于 super 函数及其方法的解析顺序参见 stackoverflow.

更好的 IDE：支持变量注释

使用类似 Java、C# 这类编程语言最享受的就是 IDE 会给出非常棒的建议，因为在执行程序前每种标识符都是已知的。

在 Python 中这是很难实现的，但是变量注释可以帮你

以一种清晰的格式写出你的期望值
从 IDE 中得到很好的建议


这是一个 PyCharm 中使用变量注释的例子。即使你使用的函数是未注释的（比如：由于向后兼容），这也仍然生效。

## 多重拆封

如下是现在如何合并两个字典：

x = dict(a=1, b=2)
y = dict(b=3, d=4)
# Python 3.5+
z = {**x, **y}
# z = {'a': 1, 'b': 3, 'd': 4}, note that value for `b` is taken from the latter dict.

可参见 StackOverflow 中的帖子与 Python 2 比较。

同样的方法也适用于列表（list），元组（tuple）和集合（set）（a，b，c 是任意可迭代对象）：

[*a, *b, *c] # list, concatenating
(*a, *b, *c) # tuple, concatenating
{*a, *b, *c} # set, union

对于使用的 *args 和 **kwargs 的函数也同样支持：

Python 3.5+
do_something(**{**default_settings, **custom_settings})
 
# Also possible, this code also checks there is no intersection between keys of dictionaries
do_something(**first_args, **second_args)

## 永不过时的 API：使用仅带关键字的参数

我们考虑下这段代码

model = sklearn.svm.SVC(2, 'poly', 2, 4, 0.5)

很明显，这段代码的作者还没有掌握 Python 的代码风格（作者极可能是刚从 C++ 或者 Rust 跳过来的）。很不幸，这个问题不仅仅是风格的问题，因为在 SVC 函数中改变参数顺序（增或删）会导致代码崩溃。特别是函数 sklearn 会经常对大量的算法参数进行重拍序或重命名以保持和 API 的一致性。每次的重构都可能导致破坏代码。

在 Python 3 中，库的编写者可使用 * 来明确的命名参数：

class SVC(BaseSVC):
    def __init__(self, *, C=1.0, kernel='rbf', degree=3, gamma='auto', coef0=0.0, ... )

现在使用者必须明确输入参数名，比如：sklearn.svm.SVC(C=2, kernel=’poly’, degree=2, gamma=4, coef0=0.5)
这种机制将 API 的可靠性和灵活性进行了极好的融合

次重点：math 模块中的常量

# Python 3
math.inf # 'largest' number
math.nan # not a number
 
max_quality = -math.inf  # no more magic initial values!
 
for model in trained_models:
    max_quality = max(max_quality, compute_quality(model, data))

次重点：单整型

Python 2 提供两种基本的整型：int 型（64 位有符整型）和用于长时计算的 long 型（C++ 后非常让人困惑）。

Python 3 有单精度的 int 型，它整合了长时计算的要求。

下面是怎样检查整型值：

isinstance(x, numbers.Integral) # Python 2, the canonical way
isinstance(x, (long, int))      # Python 2
isinstance(x, int)              # Python 3, easier to remember

其他

Enums 理论是有用处的，但：
在 python 的数据栈中，字符串输入已被广泛采用
Enums 似乎并不与 numpy 交互，也不属于 pandas 范畴
协程听起来也很有希望做数据流程（参考 David Beazley 的幻灯片），但是我还没有看到他们被采用。
Python 3 有稳定的 ABI
Python 3 支持 unicode（因此 ω = Δφ / Δt 是可以的），但是最好还是使用虽旧但好用的 ASCII 码。
一些库，比如：jupyterhub（云服务版 jupyter）、django 和新版 ipython，只支持 Python 3，因此一些听起来对你无用的特性却对那些你也许只想用一次的库非常有用。
数据科学特有的代码迁移难题（以及如何解决它们）

## 放弃支持嵌套参数

```python
map(lambda x, (y, z): x, z, dict.items())
```

然而，它依然能很好的对不同的理解起效。

```python
{x:z for x, (y, z) in d.items()}
```

通常来说，在 Python 2 和 Python 3 之间，理解也更好于‘翻译’。

## map()、.keys()、.values()、.items()、.filter() 函数返回类型是迭代器而不再是列表  

在 Python3 中使用下面的语句，会报错 TypeError:   

    float() argument must be a string or a number, not 'map'  

```python
np.array(map(float, line.strip().split(' ')), dtype=np.float32)
```

原因是 python3 的 map()、.keys()、.values()、.items() 函数返回类型为 iterators， 不再是 list， 所以可将上述语句修改为

```python
np.array(list(map(float, line.strip().split(' '))), dtype=np.float32))
```

map()、.keys()、.values()、.items() 返回的是迭代器而不再是列表。迭代器的主要问题是：

- 没有琐碎的分片  
- 不能迭代两次  

几乎全部的问题都可以通过将结果转化为列表来解决。  

## 使用 python 教授机器学习和数据科学的主要问题

教授者应该首先花时间讲解什么是迭代器，它不能像字符串一样被分片、级联、倍乘、迭代两次（以及如何处理）。

我认为大部分教授者会很高兴规避这些细节，但是现在这几乎是不可能的。

## 结论

Python 2 与 Python 3 共存了将近 10 年，但是我们应当转移到 Python 3 了。

迁移到仅有 Python 3 的代码库后，研究所写的代码和产品开发的代码都会变得更简短、更可读、更安全。

现在大部分库同时支持这两个 Python 版本。我都有点等不及了，工具包放弃支持 Python 2 享受新语言特性的美好时刻快来吧。

迁移后代码绝对会更顺畅，参见“我们再也不要向后兼容啦！”

## 参考

Key differences between Python 2.7 and Python 3.x
Python FAQ: How do I port to Python 3?
10 awesome features of Python that you can’t use because you refuse to upgrade to Python 3
Trust me, python 3.3 is better than 2.7 (video)
Python 3 for scientists



## draft
示例解读 Python 2 和 Python 3 之间的主要差异 
 

本教程主要介绍内容：

表达式
Print 选项
Unequal 操作
Range
自动迁移
性能问题

主要的内部事务更改

1、表达式 raw_input vs input  

在 Python 2 中为获得计算表达式，你会键入：   

```python
X = raw_input ("enter some values)
```

但在 Python 3 中，你会键入：   

```python
X = input ("enter some values")
```

因此，无论我们输入什么，值都会分配给 2 和 3 中的变量 x 。当在 Python 2 中输入 2*6 时，结果将是 12，这是评估值。  

但是，当在 Python 3 中运行相同的程序时，结果是字符串值。 在这种情况下，它看起来像字符串格式的 2*6。  

那么，我们如何获得评估表达式呢？ 现在，我们必须使用一个名为 eval 的表达式或函数。 当您在输入之前编写 eval 时，它会将表达式转换为计算值。  

```python
x= eval(input("enter some values")) = 12  
```


2、Print 选项   

在 Python 2 中，print 是一个不需要括号的语句。 在 Python 3 中，print 是一个函数，值需要用括号括起来。  

3、Unequal 操作   

当我们在 Python 2 中使用 Unequal 运算符时，我们需要使用大于 > 或小于 < 符号。 但是，在 Python 3 中，有一个通用运算符。 感叹号 ! 和等号 = 用于表示值是否相等。  


4、Range   

Range 用于生成数字列表，通常用于迭代 for 循环。  

在 Python 2 中，Range 是列表的类型。当我写 X 之后，得到一个对象列表，这里是：0 1 2 3 4 5 6 7 8 9。  

现在让我们转到 Python 3，当我们写 X 等于 Range 5，这个值就被赋给变量 X；Python 3 中，Range 是一个Range 对象本身。需要显式地调用 list() 函数将该对象转换为 list 对象.   

5、自动迁移   

那么，我们如何自动执行脚本以将代码从 Python 2 移动到 3？  

Python 提供了自己的工具，名为 2to3.py，它运行了一堆脚本来将你的 Python 2 代码转换为 3。虽然它并不完美，但它总体上做得非常出色。转换任何代码后，您可以手动修复任何问题。   


7、一些主要的内部事务变更

print 功能括号选填。
使用 u 作为前缀字符串以生成 unicode 字符串。
整数除法总是返回整数 -5/2=2。
Raw_input() 读取字符串。
input() 评估读取的数据。
generator .next()。

Python 3：

print 功能括号必填。
默认情况下为字符串 unicode。
整数除法可能导致浮动 -5/2=2.5。
Raw_input() 不可用。
输入始终读取字符串。
Next (generator)。
Py2 to py3 实用程序。
Dictionary .keys() 和 .values() 返回的为视图不是列表。
在非自然比较中不能再使用比较运算符。
例如，None < None 将引发 TypeError 而不是返回 false。
不推荐使用百分比（％）字符串格式化运算符，使用 .format() 函数或连接。  
