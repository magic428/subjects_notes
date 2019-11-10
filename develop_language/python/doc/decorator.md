# 装饰器   

装饰器本质上是一个Python函数，它可以让其他函数在不需要做任何代码变动的前提下增加额外功能，装饰器的返回值也是一个函数对象。它经常用于有切面需求的场景，比如：插入日志、性能测试、事务处理、缓存、权限校验等场景。装饰器是解决这类问题的绝佳设计，有了装饰器，我们就可以抽离出大量与函数功能本身无关的雷同代码并继续重用。概括的讲，装饰器的作用就是为已经存在的对象添加额外的功能。   
## 一个简单例子    
```python
def foo():
    print('i am foo')
```
现在有一个新的需求，希望可以记录下函数的执行日志，于是在代码中添加日志代码：   
```python
def foo():
    print('i am foo')
    logging.info("foo is running")
```
bar()、bar2() 也有类似的需求， 怎么做？ 再写一个logging在bar函数里？ 这样就造成大量雷同的代码，为了减少重复写代码，我们可以这样做，重新定义一个函数：专门处理日志, 日志处理完之后再执行真正的业务代码.    
```python
def use_logging(func):
    logging.warn("%s is running" % func.__name__)
    func()

def bar():
    print('i am bar')

# 调用方式  
use_logging(bar)
```
逻辑上不难理解， 但是这样的话，我们每次都要将一个函数作为参数传递给use_logging函数。 而且这种方式已经破坏了原有的代码逻辑结构，之前执行业务逻辑时，执行运行bar()，但是现在不得不改成use_logging(bar)。那么有没有更好的方式的呢？ 当然有，答案就是装饰器。   
## 简单装饰器   
```python
def use_logging(func):

    def wrapper(*args, **kwargs):
        logging.warn("%s is running" % func.__name__)
        return func(*args, **kwargs)
    return wrapper

def bar():
    print('i am bar')

# 调用方式   
bar = use_logging(bar)
bar()
```
函数 `use_logging`就是装饰器，它把执行真正业务方法的`func`包裹在函数里面，看起来像`bar`被`use_logging`装饰了。在这个例子中，函数进入和退出时 ，被称为一个横切面(Aspect)，这种编程方式被称为面向切面的编程(Aspect-Oriented Programming)。   
`@`符号是装饰器的语法糖，在定义函数的时候使用，避免再一次赋值操作.   
```python
def use_logging(func):

    def wrapper(*args, **kwargs):
        logging.warn("%s is running" % func.__name__)
        return func(*args)
    return wrapper

@use_logging
def foo():
    print("i am foo")

@use_logging
def bar():
    print("i am bar")

# 调用方式  
bar()
```
如上所示，这样我们就可以省去 `bar = use_logging(bar)` 这一句了，直接调用 `bar()` 即可得到想要的结果。如果我们有其他的类似函数，我们可以继续调用装饰器来修饰函数，而不用重复修改函数或者增加新的封装。这样，我们就提高了程序的可重复利用性，并增加了程序的可读性。   
装饰器在 Python 使用如此方便都要归因于 Python 的函数能像普通的对象一样能作为参数传递给其他函数，可以被赋值给其他变量，可以作为返回值，可以被定义在另外一个函数内。   

## 带参数的装饰器   
装饰器还有更大的灵活性，例如带参数的装饰器：在上面的装饰器调用中，比如 `@use_logging`， 该装饰器唯一的参数就是执行业务的函数。装饰器的语法允许我们在调用时提供其它参数，比如`@decorator(a)`。这样，就为装饰器的编写和使用提供了更大的灵活性。   
```python
def use_logging(level):
    def decorator(func):
        def wrapper(*args, **kwargs):
            if level == "warn":
                logging.warn("%s is running" % func.__name__)
            return func(*args)
        return wrapper

    return decorator

@use_logging(level="warn")
def foo(name='foo'):
    print("i am %s" % name)

# 调用方法
foo()
```
上面的`use_logging`是允许带参数的装饰器。它实际上是对原有装饰器的一个函数封装，并返回一个装饰器。我们可以将它理解为一个含有参数的闭包。当我 们使用`@use_logging(level="warn")`调用的时候，Python 能够发现这一层的封装，并把参数传递到装饰器的环境中。   

## 类装饰器   
再来看看类装饰器，相比函数装饰器，类装饰器具有灵活度大、高内聚、封装性等优点。使用类装饰器还可以依靠类内部的 `__call__` 方法，当使用 `@类名`的形式将类装饰器附加到函数上时，就会调用此方法。     
```python

class Foo(object):
    def __init__(self, func):
    self._func = func

def __call__(self):
    print ('class decorator runing')
    self._func()
    print ('class decorator ending')

@Foo
def bar():
    print ('bar')

# 调用方法
bar()
```

## functools.wraps   
使用装饰器极大地复用了代码，但是他有一个缺点就是原函数的元信息不见了，比如函数的 `docstring、__name__`、参数列表，先看例子：   
1. 装饰器   
```python
def logged(func):
    def with_logging(*args, **kwargs):
        print func.__name__ + " was called"
        return func(*args, **kwargs)
    return with_logging
2. 函数   
```python
@logged
def f(x):
   """does some math"""
   return x + x * x
```
该函数完成等价于：   
```python
def f(x):
    """does some math"""
    return x + x * x
f = logged(f)
```
不难发现， `函数f()`被`with_logging`取代了， 当然它的 `docstring，__name__` 就是变成了 `with_logging` 函数的信息了。   
```python
print f.__name__    # prints 'with_logging'
print f.__doc__     # prints None
```
这个问题就比较严重的， 好在我们有`functools.wraps`，`wraps`本身也是一个装饰器， 它能把原函数的元信息拷贝到装饰器函数中，这使得装饰器函数也有和原函数一样的元信息了。   
```python
from functools import wraps
def logged(func):
    @wraps(func)
    def with_logging(*args, **kwargs):
        print func.__name__ + " was called"
        return func(*args, **kwargs)
    return with_logging

@logged
def f(x):
    """does some math"""
    return x + x * x

print f.__name__  # prints 'f'
print f.__doc__   # prints 'does some math'
```

## 内置装饰器   
@staticmathod、@classmethod、@property    
装饰器的顺序:    
```python
@a
@b
@c
def f ():
等效于
```
f = a(b(c(f)))   
