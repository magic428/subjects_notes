# vector 操作   
> 顺序容器选用标准: (1) 向容器中添加或从容器中删除元素的代价. (2) 非顺序访问容器中元素的代价.     

**顺序容器的一些特点:**    
(1) vector:       在尾部之外的位置插入或删除元素可能很慢;   
(2) list:         只支持双向顺序访问, 任何位置的插入删除都很快, 随机访问可能很慢;   
(3) forward_list: 只支持单向顺序访问, 任何位置的插入删除都很快, 随机访问可能很慢;   

**如果访问操作和删除(或添加)操作是在不同阶段完成的, 那么可以根据不同顺序容器的特点在一个阶段完成操作后拷贝到另一个顺序容器中**.   

## vector 容器操作    
**推荐使用迭代器操作顺序容器, 而不是下标操作**.   
### 1. vector 容器特点   
vector 是线性容器，它的元素严格的按照线性序列排序，可以看成是动态数组。和数组一样,它的元素存储在一块连续的存储空间中，这也意味着我们不仅可以使用迭代器(iterator)访问元素，还可以使用指针的偏移方式访问。   
和常规数组不一样的是，vector 能够自动存储元素，可以自动增长或缩小存储空间。以下是 vector的 优点:
(1). 可以使用下标访问个别的元素    
(2). 迭代器可以按照不同的方式遍历容器   
(3). 可以在容器的末尾增加或删除元素   
和数组相比，虽然容器在自动处理容量的大小时会消耗更多的内存，但是容器能提供和数组一样的性能，而且能很好的调整存储空间大小。和其他标准的顺序容器相比(deques or lists)，能更有效访问容器内的元素和在末尾添加和删除元素，在其他位置添加和删除元素，vector 则不及其他顺序容器，在迭代器和引用也不比 lists 支持的好。   
### 2. 容器的大小和容器的容量
容器的大小(size) 和 容器的容量(capacity) 是有区别的，大小是当前容器中包含的元素个数。容量是容器在不重新分配空间的情况下可容纳的元素个数，容量一般等于或大于容器的大小。   
`vector::size()`返回容器的大小，`vector::capacity()`返回容量值，容量大于容器大小的部分用于以防容器的元素增加使用，每次重新分配内存都会很影响程序的性能，所以一般分配的容量大于容器的大小。   
若要自己指定分配的容量的大小，则可以使用`vector::reserve(n)`，但函数的参数值 `n` 要大于 `vector::size()` 的值。    
`vector::resize(n)` 操作可以对容器中的元素个数进行控制重新指定, 如果函数的参数值 `n` 大于 `vector::size()`, 则会在当前容器的元素末尾添加 `n - vector::size()` 个元素. 如果函数的参数值 `n` 小于 `vector::size()`, 则会从尾部开始删除元素,最后会剩下 `n` 个元素.  
**需要注意的是, `vector::resize(n)` 操作不会改变c apacity 的大小**.   
### 3. 基本操作   
(1) 头文件 `#include <vector>`.   
(2) 创建 vector 对象: vector<int> vec;   
(3) 尾部插入数字：     vec.push_back(a);   
(4) 使用下标访问元素:  cout << vec[0] << endl; 记住下标是从 `0` 开始的。   
(5) 清空:            vec.clear()　　　|清空之后，vec.size()为０  
(6) 插入元素：        vec.insert(vec.begin()+i, a); 在第 `i` 个元素后面插入`a`;  
(7) 删除元素：        vec.erase();  
(8) 容器大小:         vec.size();  
(9) 使用迭代器访问元素.   
```cpp
vector<int>::iterator it;  
for(it = vec.begin(); it != vec.end(); it++)  
    cout<< *it << endl;
```
```cpp
vec.erase(vec.begin()+2);                        |删除第3个元素
vec.erase(vec.begin()+i, vec.end()+j);       |删除区间[i,j-1];区间从0开始
```

下表是 vector 常用的成员函数：   

|vector方法(常用)|功能|
|:--------------|:-----------|
|c.begin()        | 传回迭代器中的一个数据|
|c.end()          | 指向迭代器中的最后一个数据地址|
|c.clear()        | 移除容器中所有数据|
|c.empty()        | 判断容器是否为空|
|c.erase(pos)     | 删除pos位置的数据，传回下一个数据的位置|
|c.erase(beg,end) | 删除[beg,end)区间的数据，传回下一个数据的位置|
|c.capacity()     | 返回容器中数据个数|
|get_allocator    | 使用构造函数返回一个拷贝|
|c.push_back(elem)| 在尾部加入一个数据|
|c.insert(pos,elem)   | 在pos位置插入一个elem拷贝，传回新数据位置|
|c.insert(pos,n,elem) | 在pos位置插入n个elem数据。无返回值。|
|c.insert(pos,beg,end)| 在pos位置插入在[beg,end)区间的数据。无返回值|
|c.max_size()         | 返回容器中最大数据的数量|
|c.pop_back()         | 删除最后一个数据|
|c.size()             | 返回容器中实际数据的个数|
|c1.swap(c2)       | 将c1和c2元素互换|
|vector c1(c2)     | 用c2拷贝c1|
|vector c(n)       | 创建一个vector，含有n个数据，数据均已缺省构造产生|
|vector c(n, elem) | 创建一个含有n个elem拷贝的vector|
|vector c(beg,end) |  创建一个以[beg;end)区间的vector|

下表是 vector 其余的成员函数：   

|vector方法|功能|
|:--------------|:-----------|
|c.~vector () |  销毁所有数据，释放内存|
|c.rbegin() | 传回一个逆向队列的第一个数据|
|c.rend()   | 传回一个逆向队列的最后一个数据的下一个位置|
|c.resize(num) | 重新指定队列的长度|
|c.reserve()  | 保留适当的容量|
|operator[] | 返回容器中指定位置的一个引用|
|c.at(idx)   | 传回索引idx所指的数据，如果idx越界，抛出out_of_range|
|c.front()  | 传回第一个数据|
|c.back()   | 传回最后一个数据，不检查这个数据是否存在|
|c.assign(beg,end)  | 将[beg; end)区间中的数据赋值给c|
|c.assign(n,elem)| 将n个elem的拷贝赋值给c|
|swap(c1,c2) | 同上操作|

### 4. vector 的使用
(1). vector 的初始化---构造函数和拷贝构造函数   
```cpp
explicit vector ( const Allocator& = Allocator() );
explicit vector ( size_type n, const T& value= T(), const Allocator& = Allocator() );
template <class InputIterator>
vector ( InputIterator first, InputIterator last, const Allocator& = Allocator() );
vector ( const vector<T,Allocator>& x );

explicit: 是防止隐式转换, Allocator 是一种内存分配模式,一般是使用默认的
```cpp
vector<int> A;  // 创建一个空的的容器
vector<int> B(10,100); // 创建一个个元素,每个元素值为100
vector<int> C(B.begin(),B.end()); // 使用迭代器,可以取部分元素创建一个新的容器
vector<int> D(C); // 复制构造函数,创建一个完全一样的容器
```  
(2). 析构函数   
```cpp
~vector()
```
销毁容器对象并回收了`所有`分配的内存.    
(3). 重载了 `=` 符号   
`=` 运算符将其左边容器中的全部元素替换为右边容器中元素的拷贝. 如果两个容器原来大小不同, 则赋值运算后两者的大小都与右边容器的大小相同.     
```cpp
vector<int> B(10,100); // 创建一个个元素,每个元素值为 100

vector<int> E;
E = B;              // 使用 = 符号
B = vector<int>();  // 将 B 置为空容器
```
(4). vector::begin() 返回容器中第一个元素的迭代器   
```
iterator begin ();             // 返回一个可变迭代器
const_iterator begin () const; // 返回一个常量的迭代器，不可变
``` 
(5).vector::end()  返回容器中越界后的第一个位置，也就是最后一个元素的下一个位置    
(6).vector::rbegin() 反序的第一个元素，也就是正序最后一个元素   
(7).vector::rend() 反序的最后一个元素下一个位置，也相当于正序的第一个元素前一个位置   
(8).vector::size() 返回容器中元素个数   
(9).vector::max_size()    
```cpp
size_type max_size () const;
```
返回容器的最大可以存储的元素个数，这是个极限，当容器扩展到这个最大值时就不能再自动增大.   
(10). vector::resize()   
```cpp
void resize ( size_type sz, T c = T() );
```
重新分配容器的元素个数，这个还可以改容器的容量，如果重新分配的元素个数比原来的小，将截断序列(即删除后面多余的元素)，如果大于原来的个数，则将值是 c(默认值为 0) 新元素添加到容器的后部.   
(11). vector::capacity()   
```cpp
size_type capacity () const;
```
返回 vector 不分配新内存空间的前提下可以存储元素的大小，这个一般大于或等于 vector 元素个数，注意与 size() 函数的区别.   
(12). vector::empty()   
```cpp
bool empty () const;
```
当元素个数为 0 时返回 true, 否则为 false, 根据的是元素个数而不是容器的存储空间的大小.   
(13). vector::reserve()   
```cpp
void reserve ( size_type n );
```
重新分配空间的大小，不过这个 n 值要比原来的 capacity() 返回的值大, 不然存储空间保持不变, n 值要比原来的实际存储空间大才能重新分配空间, 但是最大值不可以大于 max_size 的值，否则会抛出异常.   
(14). vector::operator[]--重载了[]符号   
```cpp
reference  operator[] ( size_type n );
const_reference  operator[] ( size_type n ) const;
```
下标访问元素, 返回容器中该位置的元素的引用.    
(15). vector::at()   
```cpp
const_reference at ( size_type n ) const;
reference at ( size_type n );
```
在函数的操作方面和下标访问元素一样，返回容器中该位置的元素的引用. 不同的是当这个函数越界时会抛出一个异常 out_of_range.   
(16). vector::front()   
```cpp
reference front ( );
const_reference front ( ) const;
```
返回第一个元素引用，与 begin() 函数有区别, begin() 函数返回的是第一个元素的迭代器(而不是元素值).   
注意:    
a) 对一个空容器调用 front 和 back, 就像使用一个越界的下标一样, 是一种严重的程序设计错误.   
b) front 和 back 返回的是引用类型. 使用 auto 定义的变量接受返回值时一定要将其定义为引用类型.   
(17). vector::back()    
```cpp
reference back ( );
const_reference back ( ) const;
```
返回最后一个元素引用，注意与 end() 函数的区别.     
(18). vector::clear()   
```cpp
void clear ();
```
将容器里的内容清空, size 值为 0, 但是存储空间没有改变.    
(19). vector::push_back()   
当我们将一个对象插入到容器中时, 实际上放入容器中的是对象值的一个拷贝, 而不是对象本身.   
```cpp
void push_back ( const T& x );
```
在容器的最后一个位置插入元素 x, 如果 size 值大于 capacity 值，则将重新分配空间.   
(20). vector::pop_back()   
```cpp
void pop_back ( );
```
删除最后一个元素.   
(21). vector::insert()   
```cpp
iterator insert ( iterator position, const T& x );
void insert ( iterator position, size_type n, const T& x );
void insert ( iterator position, InputIterator first, InputIterator last );
template <class InputIterator>
```
通过使用 insert() 的返回值, 可以在容器中的一个特定位置反复插入元素.   
注意: 只有一个 insert() 函数的返回值是指向插入的这个新元素的迭代器, 其他的返回值则为 void.   
第一个函数，在迭代器指定的位置`前`插入值为 x 的元素
第二个函数，在迭代器指定的位置`前`插入 n 个值为x的元素
第三个函数，在迭代器指定的位置`前`插入指定的另外一个容器的迭代器范围(first 到 last)内的元素.   
若插入新的元素后总得元素个数大于 capacity, 则重新分配空间.   
(22). vector::erase()
```cpp
iterator erase ( iterator position );
iterator erase ( iterator first, iterator last );
```
删除迭代器指定的元素或迭代器范围内指定的元素, 返回一个指向被删除元素之后的元素迭代器.   
注意:   
a) 删除元素的成员函数并不会检查参数是否有效, 因此在删除元素前, 用户必须确保它(们)是存在的.   
b) 删除 deque 中除首尾之外的任何元素都会使所有迭代器, 引用和指针失效.   
c) vector 或 string 中删除点之后位置的迭代器, 指针和引用都会失效.   
(23). vector::swap()   
交换这两个容器的内容，这涉及到存储空间的重新分配.    
```cpp
void swap ( vector<T,Allocator>& vec );
```
注意: 
a) c1.swap(c2) 通常比从 c2 向 c1 拷贝元素快的多, 因为 swap() 只是交换了两个容器的内部结构, 元素本身并未交换.   
b) 元素不会被移动的事实意味着(除 string 外), 指向容器的迭代器, 引用和指针在 swap() 操作之后不会失效. 只是 swap() 操作之后这些元素已经属于不同的容器了.   
c) 与其他容器不同, 对一个 string 类型调用 swap() 会导致迭代器,引用和指针失效.   
d) 与其他容器不同, 对 array 调用 swap() 操作会真正交换他们的元素. 因此交换所需的时间与 array 中元素的数目成正比.     
e) 统一使用非成员版本的 swap() 是一个好习惯.   
(24). vector::assign()   
仅适用于顺序容器. 允许我们从一个不同但相容的类型赋值, 或者从容器的一个子序列赋值.    
assign() 操作用指定的元素的拷贝替换左边容器中的所有元素.   
```cpp
template <class InputIterator> void assign ( InputIterator first, InputIterator last );
void assign ( size_type n, const T& u );
```
将丢弃原来的元素, 然后重新分配存放新元素的空间.   
第一个函数是使用迭代器. 因为旧元素将被替换, 因此传递给 assign() 的迭代器不能指向调用 assign() 的容器.    
第二个函数是使用 n 个元素, 每个元素的值为 u.    

## 5. emplace -- 新标准   
新标准引入了三个新成员 --- emplace_front(), emplace() 和 emplace_back(). 这些操作构造而不是拷贝元素. emplace 成员使用这些参数在容器管理的空间内直接构造元素对象.     
emplace 成员的参数根据元素类型而变化, 参数必须与元素类型的构造函数相匹配.   

## 6. 反向和排序算法
(1) 使用`reverse()`将元素翻转：需要头文件`#include <algorithm>`;
```cpp
reverse(vec.begin(),vec.end())
```
将元素翻转。在 vector 中，如果一个函数中需要两个迭代器，一般后一个都不包含。   
(2) 使用`sort()`排序：需要头文件`#include <algorithm>`;
```cpp
sort(vec.begin(),vec.end())；
```
默认是按升序排列，即从小到大。可以通过重写排序比较函数按照降序比较，如下：   
```cpp
bool Comp(const int &a,const int &b)
{
    return a>b;
}
```
调用时:   
```cpp
sort(vec.begin(),vec.end(),Comp);
// 或者使用 lambda 函数
sort(vec.begin(),vec.end(), [](){});
```
这样就降序排序。   

## 7. vector 容器内存是如何增长的   
与其他容器不同，其内存空间只会增长，不会减小。先来看看"C++ Primer"中怎么说：为了支持快速的随机访问，vector  容器的元素以连续方式存放，每一个元素都紧挨着前一个元素存储。    
设想一下，当 vector 添加一个元素时，为了满足连续存放这个特性，都需要重新分配空间、拷贝元素、撤销旧空间，这样性能难以接受。因此 STL 实现者在对 vector 进行内存分配时，其实际分配的容量要比当前所需的空间多一些。就是说，vector 容器预留了一些额外的存储区，用于存放新添加的元素，这样就不必为每个新元素重新分配整个容器的内存空间。   
关于 vector 的内存空间，有两个函数需要注意：    
(1) `size()` 成员指当前拥有的元素个数；    
(2) `capacity()` 成员指当前(容器必须分配新存储空间之前)可以存储的元素个数，一般是 2 的幂次方。   
`reserve()`成员可以用来控制容器的预留空间。   
vector另外一个特性在于它的内存空间会自增长，每当 vector 容器不得不分配新的存储空间时，会以加倍当前容量的分配策略实现重新分配。例如，当前 capacity 为 50，当添加第 51 个元素时，预留空间不够用了，vector 容器会重新分配大小为 100 的内存空间，作为新连续存储的位置。   

## 8. vector 内存释放
由于 vector 的内存占用空间只增不减，比如你首先分配了 10,000 个字节，然后 erase 掉后面 9,999个，留下一个有效元素，但是内存占用仍为 10,000 个。所有内存空间是在 vector 析构时候才能被系统回收。empty() 用来检测容器是否为空的，clear() 可以清空所有元素。但是即使调用了 clear()函数, vector所占用的内存空间依然如故，无法保证内存的回收。   
如果需要空间动态缩小，可以考虑使用`deque`。 如果非 `vector` 不可，可以用 `swap()` 来帮助你释放内存。swap()是交换函数，使 vector 离开其自身的作用域，从而强制释放 vector 所占的内存空间，总而言之，释放 vector 内存最简单的方法是 vector<int>.swap(nums). 但是如果 nums 是一个类的成员，不能把 vector<int>.swap(nums) 写进类的析构函数中，否则会导致 double free or corruption (fasttop)错误，原因可能是重复释放内存。   

## 9. 利用 vector 释放指针
如果 vector 中存放的是指针，那么当 vector 销毁时，这些指针指向的对象不会被销毁，那么内存就不会被释放，从而引起大面积的内存泄露。这时应该在销毁 `vector`之前将这些指针指向的空间释放(delete)掉。

## 10. 其他顺序容器的操作    
1. array   
(1) 与内置数组不同, 标准库 array 类型允许赋值. 
```cpp
array<int, 10> a1 = {0, 1,2,3,4,5,6,7,8,9};
array<int, 8> a2 = {0};
array<int, 10> a3 = {0};
cout << "a2.size()=" << a2.size()<< endl;

a1 = a3;         // 正确: 替换 a1 中的元素
// a1 = a2;         // error: a1 和 a2 中的元素个数不同
a2 = {0, 1, 2};  // 覆盖 a2 中的前 3 个元素
// a2 = {0, 1,2,3,4,5,6,7,8,9};  // error: 初始化列表中元素个数大于 a2 中的元素个数
cout << "a2.size()=" << a2.size()<< endl;
```
a) 如果是对象赋值, 赋值号两边的运算对象必须有相同的类型和大小.    
b) 如果是初始化列表赋值, 初始化列表的大小要小于等于左侧对象的大小.  
2. assign() 两个重载函数的使用   
```cpp
vector<const char *> old_style;
list<string> names;
// names = old_style;  // error: 容器类型不匹配
names.assign(old_style.cbegin(), old_style.cend());

list<string> slist1(1);
    // 等价于 slist1.clear(); --> slist1.insert(slist1.begin(), 10, "Hiya");
    slist1.assign(10, "Hiya");
```
3. 迭代器失效操作.   
a) 调用 insert() 或 erase() 操作;   
b) 如果在用迭代器遍历容器时调用了 insert(), 那么应该递增迭代器两次之后才会指向下一个元素.   
c) 如果在用迭代器遍历容器时调用了 erase(), 那么不必递增迭代器就可指向下一个元素.   
d) 不要保存 end() 返回的迭代器.      
