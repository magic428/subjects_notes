## C++ Boost Thread 编程指南

## 0 前言

标准C++线程即将到来。CUJ预言它将衍生自Boost线程库，现在就由Bill带领我们探索一下Boost线程库。
 
就在几年前，用多线程执行程序还是一件非比寻常的事。然而今天互联网应用服务程序普遍使用多线程来提高与多客户链接时的效率；为了达到最大的吞吐量，事务服务器在单独的线程上运行服务程序；GUI应用程序将那些费时，复杂的处理以线程的形式单独运行，以此来保证用户界面能够及时响应用户的操作。这样使用多线程的例子还有很多。
 
但是C++标准并没有涉及到多线程，这让程序员们开始怀疑是否可能写出多线程的C++程序。尽管不可能写出符合标准的多线程程序，但是程序员们还是会使用支持多线程的操作系统提供的多线程库来写出多线程C++程序。但是这样做至少有两个问题：这些库大部分都是用C语言完成的，如果在C++程序中要使用这些库就必须十分小心；还有，每一个操作系统都有自己的一套支持多线程的类库。因此，这样写出来得代码是没有标准可循的，也不是到处都适用的（non-portable)。Boost线程库就是为了解决所有这些问题而设计的。
Boost是由C++标准委员会类库工作组成员发起，致力于为C++开发新的类库的组织。现在它已经有近2000名成员。许多库都可以在Boost源码的发布版本中找到。为了使这些类库是线程安全的（thread-safe)，Boost线程库被创建了。
许多C++专家都投身于Boost线程库的开发中。所有接口的设计都是从0开始的，并不是C线程API的简单封装。许多C++特性（比如构造函数和析构函数，函数对象（function object)和模板）都被使用在其中以使接口更加灵活。现在的版本可以在POSIX,Win32和Macintosh Carbon平台下工作。

## 1 创建线程

就像 std::fstream 类就代表一个文件一样，boost::thread 类就代表一个可执行的线程。缺省构造函数创建一个代表当前执行线程的实例。一个重载的构造函数以一个不需任何参数的函数对象作为参数，并且没有返回值。这个构造函数创建一个新的可执行线程，它调用了那个函数对象。   
 
起先，大家认为传统 C 创建线程的方法似乎比这样的设计更有用，因为C创建线程的时候会传入一个 void* 指针，通过这种方法就可以传入数据。然而，由于 Boost 线程库是使用函数对象来代替函数指针，那么函数对象本身就可以携带线程所需的数据。这种方法更具灵活性，也是类型安全（type-safe)的。当和 Boost.Bind 这样的功能库一起使用时，这样的方法就可以让你传递任意数量的数据给新建的线程。  

目前，由 Boost 线程库创建的线程对象功能还不是很强大。事实上它只能做两项操作。线程对象可以方便使用 == 和 != 进行比较来确定它们是否是代表同一个线程；你还可以调用 boost::thread::join 来等待线程执行完毕。   

其他一些线程库可以让你对线程做一些其他操作（比如设置优先级，甚至是取消线程）。然而，由于要在普遍适用（portable）的接口中加入这些操作不是简单的事，目前仍在讨论如何将这些操组加入到 Boost 线程库中。   
 
Listing1 展示了 boost::thread 类的一个最简单的用法。 新建的线程只是简单的在 std::out上打印“hello,world”，main 函数在它执行完毕之后结束。

例1：
```cpp
#include <boost/thread/thread.hpp>
#include <iostream>

void hello()
{
        std::cout << "Hello world, I'm a thread!" << std::endl;
}

int main(int argc, char* argv[])
{
        boost::thread thrd(&hello);
        thrd.join();
        return 0;
}
```
 
## 2 互斥体

任何写过多线程程序的人都知道避免不同线程同时访问共享区域的重要性。如果一个线程要改变共享区域中某个数据，而与此同时另一线程正在读这个数据，那么结果将是未定义的。为了避免这种情况的发生就要使用一些特殊的原始类型和操作。其中最基本的就是互斥体（mutex，mutual exclusion的缩写）。一个互斥体一次只允许一个线程访问共享区。当一个线程想要访问共享区时，首先要做的就是锁住（lock）互斥体。如果其他的线程已经锁住了互斥体，那么就必须先等那个线程将互斥体解锁，这样就保证了同一时刻只有一个线程能访问共享区域。  
 
互斥体的概念有不少变种。Boost 线程库支持两大类互斥体，包括简单互斥体（simple mutex)和递归互斥体（recursive mutex)。如果同一个线程对互斥体上了两次锁，就会发生死锁（deadlock），也就是说所有的等待解锁的线程将一直等下去。有了递归互斥体，单个线程就可以对互斥体多次上锁，当然也必须解锁同样次数来保证其他线程可以对这个互斥体上锁。  
 
在这两大类互斥体中，对于线程如何上锁还有多个变种。一个线程可以有三种方法来对一个互斥体加锁：   
- 一直等到没有其他线程对互斥体加锁。
- 如果有其他互斥体已经对互斥体加锁就立即返回。
- 一直等到没有其他线程互斥体加锁，直到超时。
似乎最佳的互斥体类型是递归互斥体，它可以使用所有三种上锁形式。然而每一个变种都是有代价的。所以 Boost 线程库允许你根据不同的需要使用最有效率的互斥体类型。Boost线程库提供了6中互斥体类型，下面是按照效率进行排序：   

```cpp
boost::mutex,
boost::try_mutex, 
boost::timed_mutex, 
boost::recursive_mutex, 
boost::recursive_try_mutex,   
boost::recursive_timed_mutex 
```

如果互斥体上锁之后没有解锁就会发生死锁。这是一个很普遍的错误，Boost 线程库就是要将其变成不可能（至少时很困难）。直接对互斥体上锁和解锁对于 Boost 线程库的用户来说是不可能的。mutex 类通过 typedef 定义在 RAII 中实现的类型来实现互斥体的上锁和解锁。这也就是大家知道的 Scope Lock 模式。为了构造这些类型，要传入一个互斥体的引用。构造函数对互斥体加锁，析构函数对互斥体解锁。C++保证了析构函数一定会被调用，所以即使是有异常抛出，互斥体也总是会被正确的解锁。   

这种方法保证正确的使用互斥体。然而，有一点必须注意：尽管 Scope Lock 模式可以保证互斥体被解锁，但是它并没有保证在异常抛出之后贡献资源仍是可用的。所以就像执行单线程程序一样，必须保证异常不会导致程序状态异常。另外，这个已经上锁的对象不能传递给另一个线程，因为它们维护的状态并没有禁止这样做。   
 
List2 给出了一个使用 boost::mutex 的最简单的例子。例子中共创建了两个新的线程，每个线程都有 10 次循环，在 std::cout 上打印出线程 id 和当前循环的次数，而 main 函数等待这两个线程执行完才结束。std::cout 就是共享资源，所以每一个线程都使用一个全局互斥体来保证同时只有一个线程能向它写入。   
 
许多读者可能已经注意到 List2 中传递数据给线程还必须得手工写一个函数。尽管这个例子很简单，如果每一次都要写这样的代码实在是让人厌烦的事。别急，有一种简单的解决办法。函数库允许你通过将另一个函数绑定，并传入调用时需要的数据来创建一个新的函数。   

List3 向你展示了如何使用 Boost.Bind 库来简化 List2 中的代码，这样就不必手工写这些函数对象了。  

例2：
```cpp
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>

boost::mutex io_mutex;

struct count
{
    count(int id) : id(id) { }
    
    void operator()()
    {
        for (int i = 0; i < 10; ++i)
        {
            boost::mutex::scoped_lock
            lock(io_mutex);
            std::cout << id << ": "
            << i << std::endl;
        }
    }
    
    int id;
};

int main(int argc, char* argv[])
{
    boost::thread thrd1(count(1));
    boost::thread thrd2(count(2));
    thrd1.join();
    thrd2.join();
    return 0;
}
```
例3： // 这个例子和例2一样，除了使用 Boost.Bind 来简化创建线程携带数据，避免使用函数对象

```cpp
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <iostream>

boost::mutex io_mutex;

void count(int id)
{
    for (int i = 0; i < 10; ++i)
    {
        boost::mutex::scoped_lock  lock(io_mutex);
        std::cout << id << ": " << i << std::endl;
    }
}

int main(int argc, char* argv[])
{
    boost::thread thrd1( boost::bind(&count, 1) );
    boost::thread thrd2( boost::bind(&count, 2) );
    boost::bind(&count, _1)(1); 
    
    thrd1.join();
    thrd2.join();

    return 0;
}
```
## 3 条件变量

有的时候仅仅依靠锁住共享资源来使用它是不够的。有时候共享资源只有某些状态的时候才能够使用。比方说，某个线程如果要从堆栈中读取数据，那么如果栈中没有数据就必须等待数据被压栈。这种情况下的同步使用互斥体是不够的。另一种同步的方式－条件变量，就可以使用在这种情况下。   

条件变量的使用总是和互斥体及共享资源联系在一起的。线程首先锁住互斥体，然后检验共享资源的状态是否处于可使用的状态。如果不是，那么线程就要等待条件变量。要指向这样的操作就必须在等待的时候将互斥体解锁，以便其他线程可以访问共享资源并改变其状态。它还得保证从等到得线程返回时互斥体是被上锁得。当另一个线程改变了共享资源的状态时，它就要通知正在等待条件变量得线程，并将之返回等待的线程。   

List4是一个使用了boost::condition的简单例子。有一个实现了有界缓存区的类和一个固定大小的先进先出的容器。由于使用了互斥体boost::mutex，这个缓存区是线程安全的。put和get使用条件变量来保证线程等待完成操作所必须的状态。有两个线程被创建，一个在buffer中放入100个整数，另一个将它们从buffer中取出。这个有界的缓存一次只能存放10个整数，所以这两个线程必须周期性的等待另一个线程。为了验证这一点，put和get在std::cout中输出诊断语句。最后，当两个线程结束后，main函数也就执行完毕了。

```cpp
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <iostream>

const int BUF_SIZE = 10;
const int ITERS = 100;

boost::mutex io_mutex;

class Buffer
{
public:
    typedef boost::mutex::scoped_lock scoped_lock;
    
    Buffer() : p(0), c(0), full(0) {    }
    
    void put(int m)
    {
        scoped_lock lock(mutex);
        if (full == BUF_SIZE)
        {
            {
                boost::mutex::scoped_lock lock(io_mutex);
                std::cout << "Buffer is full. Waiting..." << std::endl;
            }
            while (full == BUF_SIZE)
                cond.wait(lock);
        }
        
        buf[p] = m;
        p = (p+1) % BUF_SIZE;
        ++full;
        cond.notify_one();
    }
    
    int get()
    {
        scoped_lock lk(mutex);
        if (full == 0)
        {
            {
                boost::mutex::scoped_lock lock(io_mutex);
                std::cout << "Buffer is empty. Waiting..." << std::endl;
            }
            while (full == 0)
                cond.wait(lk);
        }
        int i = buf[c];
        c = (c+1) % BUF_SIZE;
        --full;
        cond.notify_one();
        return i;
    }
    
private:
    boost::mutex mutex;
    boost::condition cond;
    unsigned int p, c, full;
    int buf[BUF_SIZE];
};

Buffer buf;

void writer()
{
    for (int n = 0; n < ITERS; ++n)
    {
        {
            boost::mutex::scoped_lock lock(io_mutex);
            std::cout << "sending: " << n << std::endl;
        }
        buf.put(n);
    }
}

void reader()
{
    for (int x = 0; x < ITERS; ++x)
    {
        int n = buf.get();
        {
            boost::mutex::scoped_lock lock(io_mutex);
            std::cout << "received: " << n << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{   
    boost::thread thrd1(&reader);
    boost::thread thrd2(&writer);
    thrd1.join();
    thrd2.join();
    return 0;
}
```
 

## 4 线程局部存储   

大多数函数都不是可重入的。这也就是说在某一个线程已经调用了一个函数时，如果你再调用同一个函数，那么这样是不安全的。一个不可重入的函数通过连续的调用来保存静态变量或者是返回一个指向静态数据的指针。 举例来说，std::strtok 就是不可重入的，因为它使用静态变量来保存要被分割成符号的字符串。  
 
有两种方法可以让不可重用的函数变成可重用的函数。第一种方法就是改变接口，用指针或引用代替原先使用静态数据的地方。比方说，POSIX 定义了 strok_r，std::strtok 中的一个可重入的变量，它用一个额外的 char** 参数来代替静态数据。这种方法很简单，而且提供了可能的最佳效果。但是这样必须改变公共接口，也就意味着必须改代码。另一种方法不用改变公有接口，而是用本地存储线程（thread local storage）来代替静态数据（有时也被成为特殊线程存储，thread-specific storage）。   
 
Boost 线程库提供了智能指针 boost::thread_specific_ptr 来访问本地存储线程。每一个线程第一次使用这个智能指针的实例时，它的初值是 NULL，所以必须要先检查这个它的值是否为空，并且为它赋值。Boost 线程库保证本地存储线程中保存的数据会在线程结束后被清除。   
 
List5 是一个使用 boost::thread_specific_ptr 的简单例子。其中创建了两个线程来初始化本地存储线程，并有 10 次循环，每一次都会增加智能指针指向的值，并将其输出到 std::cout 上（由于 std::cout 是一个共享资源，所以通过互斥体进行同步）。main 线程等待这两个线程结束后就退出。从这个例子输出可以明白的看出每个线程都处理属于自己的数据实例，尽管它们都是使用同一个boost::thread_specific_ptr。   
 
例5：
```cpp
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/tss.hpp>
#include <iostream>

boost::mutex io_mutex;
boost::thread_specific_ptr<int> ptr;  // 每个线程都会维护自己的局部数据, 线程运行结束后自动释放.  

struct Count
{
    Count(int id) : id(id) { }
    
    void operator()()
    {
        if (ptr.get() == 0)
            ptr.reset(new int(0));
        
        for (int i = 0; i < 10; ++i)
        {
            (*ptr)++;
            boost::mutex::scoped_lock lock(io_mutex);
            std::cout << id << ": "  << *ptr << std::endl;
        }
    }
    
    int id;
};

int main(int argc, char* argv[])
{
    boost::thread thrd1(Count(1));
    boost::thread thrd2(Count(2));

    thrd1.join();
    thrd2.join();
    
    return 0;
}
```

## 5 仅运行一次的例程

还有一个问题没有解决：如何使得初始化工作（比如说构造函数）也是线程安全的。比方说，如果一个引用程序要产生唯一的全局的对象，由于实例化顺序的问题，某个函数会被调用来返回一个静态的对象，它必须保证第一次被调用时就产生这个静态的对象。这里的问题就是如果多个线程同时调用了这个函数，那么这个静态对象的构造函数就会被调用多次，这样错误产生了。   
  
解决这个问题的方法就是所谓的“一次实现”（once routine）。“一次实现”在一个应用程序只能执行一次。如果多个线程想同时执行这个操作，那么真正执行的只有一个，而其他线程必须等这个操作结束。为了保证它只被执行一次，这个 routine 由另一个函数间接的调用，而这个函数传给它一个指针以及一个标志着这个 routine 是否已经被调用的特殊标志。这个标志是以静态的方式初始化的，这也就保证了它在编译期间就被初始化而不是运行时。因此也就没有多个线程同时将它初始化的问题了。Boost 线程库提供了 boost::call_once 来支持“一次实现”，并且定义了一个标志 boost::once_flag 及一个初始化这个标志的宏 BOOST_ONCE_INIT。   
 
List6 是一个使用了 boost::call_once 的例子。其中定义了一个静态的全局整数，初始值为 0；还有一个由 BOOST_ONCE_INIT 初始化的静态 boost::once_flag 实例。main 函数创建了两个线程，它们都想通过传入一个函数调用 boost::call_once 来初始化这个全局的整数，这个函数是将它加1。main 函数等待着两个线程结束，并将最后的结果输出的到 std::cout。由最后的结果可以看出这个操作确实只被执行了一次，因为它的值是 1。  

 
```cpp
#include <boost/thread/thread.hpp>
#include <boost/thread/once.hpp>
#include <iostream>

int i = 0;
boost::once_flag flag = BOOST_ONCE_INIT;  

void init()
{
    ++i;
}

void thread()
{
    boost::call_once(&init, flag);
}

int main(int argc, char* argv[])
{
    boost::thread thrd1(&thread);
    boost::thread thrd2(&thread);
    thrd1.join();
    thrd2.join();
    std::cout << i << std::endl;
    return 0;
}
```
## 6 Boost线程库的未来

Boost 线程库正在计划加入一些新特性。其中包括 boost::read_write_mutex，它可以让多个线程同时从共享区中读取数据，但是一次只可能有一个线程向共享区写入数据； boost::thread_barrier，它使得一组线程处于等待状态，知道所有得线程都都进入了屏障区；boost::thread_pool, 他允许执行一些小的 routine 而不必每一都要创建或是销毁一个线程。   
  
Boost 线程库已经作为标准中的类库技术报告中的附件提交给 C++ 标准委员会，它的出现也为下一版 C++ 标准吹响了第一声号角。委员会成员对Boost线程库的初稿给予了很高的评价,当然他们还会考虑其他的多线程库。他们对在 C++ 标准中加入对多线程的支持非常感兴趣。从这一点上也可以看出，多线程在 C++ 中的前途一片光明。    