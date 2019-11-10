# 命令空间的使用   

命名空间可以是全局的，也可以位于另一个命名空间之中，但是不能位于类和代码块中。   

在所有命名空间之外，还存在一个全局命名空间，它对应于文件级的声明域。因此，在命名空间机制中，原来的全局变量，现在被认为位于全局命名空间中。   

不能在命名空间的定义中`声明`（另一个嵌套的）子命名空间，只能在命名空间的定义中`定义`子命名空间。   

## 使用实例   
1. 头文件定义  
```cpp
// .hpp
namespace my_nsp {
void print_func();
}
void print_func();
```
2. 实现文件定义  
```cpp
// .cpp
namespace my_nsp {
void print_func() {
    std::cout << "namespace my_nsp" << std::endl;
}
} // namespace my_np

void print_func() {
    std::cout << "namespace global" << std::endl;
}
```
3. 主文件 main() 定义   
```cpp
// .cpp
int main(int argc, char **argv)
{
    ::print_func();
    my_nsp::print_func();

    return 0;
}
```

程序输出:  
```
namespace global
namespace my_nsp
```

### caffe 中使用命令空间的方式    

`ssd/include/caffe/common.hpp` 文件.   

```cpp
// We will use the boost shared_ptr instead of the new C++11 one mainly
// because cuda does not work (at least now) well with C++11 features.
using boost::shared_ptr;

// Common functions and classes from std that caffe often uses.
using std::ios;
using std::isnan;
using std::isinf;
using std::string;
using std::stringstream;
using std::ostringstream;
using std::fstream;
using std::iterator;
using std::vector;
using std::map;
using std::set;
using std::pair;
using std::make_pair;

```