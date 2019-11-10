# Python 源码剖析

## Py_Object中重要的三个操作族函数。

```cpp
PyNumberMethods *tp_as_number;
PySequenceMethods *tp_as_sequence;
PyMappingMethods *tp_as_mapping;
```

`PySequenceMethods`和`PyMappingMethods`定义了作为序列对象和关联对象应该支持的行为。这两种对象的典型例子是`list`和`dict`。

## 引用计数
现代的开发语言中一般都选择由语言本身负责内存的管理和维护，即采用了垃圾回收机制。比如Java和C#。
 这样做的好处是提高了开发效率，降低了bug发生的几率。
 在引用计数ob_refcnt减为0之后，就会触发对象的销毁事件。python的每个对象都提供了不同的事件处理函数，而事件的注册动作正是在

## python_int对象
python_int对象是不可变对象，这意味着对象池中的 每一个PyIntObject对象都能被任意的共享。

## 创建通用整数对象池
需要注意的是：Python对fill_free_list()的调用不只发生在对PyInt_FromLong()的首次调用中，在Python运行期间，只要所有block的空闲内存都被用光了，就会导致free_list变为NULL，从而在下一次PyInt_FromLong()的调用中激发对fill_free_list()的调用。
当free_list指向最后一个元素时，它的下一次使用就会变成无效的(指向NULL)。

## free_list和block_list的设计哲学
python是通过block_list来维护整个整数对象的通用对象池的。值得注意的是，block_list始终指向最新创建的PyIntBlock对象。
对于free_list来说，block_list是不可见的。正向创建的一个新的block时(此时变量池空间不够用)，free_list前面是有一个断点的，也就是说它不知道它前面的元素。因为链表是单向的，因此这样是合理的。在int_dealloc()函数中会将新的block的第一个节点和旧block中的最后一个节点联系起来。从而free_list的断点被联系起来，同时加长了free_list。

## intern机制的必要性
 PyStringObject对象的intern机制的目的是：对于intern修饰的字符串，比如"Ruby"，在整个python的运行期间，系统