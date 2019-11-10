# eventFilter的使用方法     

> [原文链接](http://www.cnblogs.com/smoozer/archive/2009/02/09/1386704.html)

一直对 Event Filter 似懂非懂, 通过看 C++ GUI Programming with Qt4, Second Edition, 争取搞明白. 顺便自己把英文翻译成中文, 算是自己做的笔记了.    
## 1. Installing Event Filters    
QT 事件模块一个真正强大的特性是可以设置一个 QObject 的实例去监测另外一个 QObject 实例的事件,在被监测的实例 see 之前.   
假设我们有一个 CustomerInfoDialog 这样的 widget, 它由几个 QLineEdit 组成.我们想用 Space 键来转变 focus 到下一个 QLineEdit.这个非标准的行为可能对一个内部的程序很合适, 需要培训它的用户来使用它. 一个直接的方法是子类 QLineEdit,然后重新实现keyPressEvent()来调用 focusNextChild(), 像这样:    
```cpp
void MyLineEdit::keyPressEvent(QKeyEvent *event) 
{ 
	if (event->key() == Qt::Key_Space) { 
		focusNextChild(); 
	} else { 
		QLineEdit::keyPressEvent(event); 
	} 
}
```
这个方法有一个主要的弊端: 如果我们在这个 form 中用到几个不同类型的 widget(比如 QComboBox 和 QSpinBox), 我们必须也子类化它们来表现出相同的行为.    一个更好的方案是让 CustomerInfoDialog 来监控它的子 widget 的按键事件.在监测的代码里执行需要的行为. 这可以用event Filter来达到.    
设置一个 event filter 有两个步骤:    
- 在目标对象上调用 installEventFilter(),将监测对象注册到目标对象上.    
- 在监测对象的 eventFilter() 方法里处理目标对象的事件.
在 CustomerInfoDialog 的构造函数里注册监测对象是一个好地方:   
```cpp
CustomerInfoDialog::CustomerInfoDialog(QWidget *parent) :QDialog(parent) 
{ 
	... 
	firstNameEdit->installEventFilter(this); 
	lastNameEdit->installEventFilter(this); 
	cityEdit->installEventFilter(this); 
	phoneNumberEdit->installEventFilter(this); 
}
```
一旦 event Filter 注册了, 发送到 firstNameEdit, lastNameEdit, cityEdit 和 phoneNumberEdit 的事件在被发送到原来的目的地之前, 会先发到 CustomerInfoDialog 的 eventFilter() 函数(因此 CustomerInfoDialog 的 eventFilter()函数名一定是固定的).    
这是接收这些事件的 eventFilter() 函数:    
```cpp
bool CustomerInfoDialog::eventFilter(QObject *target, QEvent *event)
{
    if (target == firstnameEdit || target == lastNameEdit
        || target == cityEdit || target == phoneNumberEdit)
    {
        if(event->type() == QEvent::KeyPress)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
            if (keyEvent->key() == Qt::Key_Space)
            {
                focusNextChild();
                return true;
            }
        }
    }
    return QDialog::eventFilter(target, event);
}

```
首先,我们检查是否目标 widget 是一个 QLineEdit. 如果是个 key Press 事件, 把它转换为 QKeyEvent, 并检查哪个键值被按下.   
如果是 space, 我们调用 focusNextChild() 把 focus 传到 focus 链上的下一个 widget 上, 然后 Qt 会发送这个 event 到它原来的目的地,导致一个假的空格被插入到 QLineEdit.   
如果目标 widget 不是 QLineEdit, 或者这个 event 不是一个 space 按键, 我们把控制权传回到基类的 eventFilter 去. 目标 wdiget 可以是基类-QDialog 正在监测的某个 widget.

## 2. Qt 提供了 5 个级别来处理和过滤事件.    
1. 我们可以重新实现特定的 event handler. 
重新实现像 mousePressEvent(), keyPressEvent() 和 paintEvent() 这样的 event Handler 是目前处理 event 最普通的方式. 
2. 我们可以重新实现 QObject::event(). 
通过重新实现 event(),我们可以在事件到达特定的 event handler 之前对它们作出处理. 这个方法主要是用来覆写 Tab 键的缺省实现. 也可以用来处理不同发生的事件类型,对它们,就没有特定的 event handler. 当重新实现 event() 的时候,我们必须调用基类的 event() 来处理我们不显式处理的情况.   
3. 我们可以安装一个 event filter 到一个单独的 QObject. 
一旦一个对象用 installEventFilter() 注册了, 发到目标对象的所有事件都会先发到监测对象的 eventFilter(). **如果同一 object 安装了多个 event filter, filter 会依次被激活, 从最近安装的回到第一个**.    
4. 我们可以在 QApplication 对象上安装 event filter.   
一旦一个 event filter 被注册到 qApp(唯一的 QApplication 对象), 程序里发到每个对象的每个事件在发到其他 event filter 之前,都要首先发到 eventFilter(). 这个方法对 debugging 非常有用. 也可以用来处理发到 disable 的 widget 上的事件, QApplication 通常会丢弃它们.    
5. 我们可以子类 QApplication 并重新实现 notify().    
Qt 调用 QApplication::notify() 来发出事件. 在任何 event filter 得到之前, 重新实现这个函数是得到所有事件的唯一方法. event filter 通常更有用, 因为可以有任意数目且同时存在的 event filter, 而 notify() 函数只有一个.   

## 3. 总结
许多事件类型,包括鼠标和按键事件,可以被传播. 如果一个事件没有在传到目标对象的过程中被处理, 或者被目标对象本身处理, 整个事件处理过程会重复, 不过这次目标对象的 parent 作为新的目标对象.    从 parent 到 parent ,这样继续下去,知道事件被处理了,或者到达了顶层的对象.     

