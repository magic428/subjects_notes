# thread_specific_ptr

> boost::thread_specific_ptr

thread_specific_ptr 定义了一个跟线程相关的存储接口。实际上，它就是对 TLS(Thread-Locally Storage) 的包装。它可用于封装线程独立的全局变量。其作用和使用方法有点类似于 shared_ptr。

thread_specific_ptr 是一个全局的变量，而在每个线程中都各自 new 一个线程本地的对象交给它进行管理，这一点与 shared_ptr 非常相似。  

线程之间就不会因为访问同一全局对象而引起资源竞争导致性能下降。在线程结束时， 这个资源会被自动释放。  

它可以应用在以下两种场景：

 - 改编一个原本设计用于单线程的库接口，比如 libc 里的 strtok 函数。这种库一般隐含的使用一个全局变量，可以使用 thread_specific_ptr 控制全局变量，使其可用于多线程。  
- 线程中使用了一系列的方法/函数，它们需要一个逻辑上的全局变量来共享数据，但实际上这个变量是线程独立的。  

thread_specific_ptr 代表了某个全局变量的本地存储，各个线程可以各自独立地通过它访问这个全局变量的本地副本，起到了井水不犯河水的效果。  

caffe 中用于它来管理 Caffe 对象的实例, Caffe 设计使用了单例模式.  


```cpp
Caffe& Caffe::Get() {
  if (!thread_instance_.get()) {
    thread_instance_.reset(new Caffe());
  }
  return *(thread_instance_.get());
}
```