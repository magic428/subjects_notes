# map 使用实例   

> 《 C++ Primer 5nd 》 P374    

关联容器支持高效的关键字查找和访问. 两个主要的关联容器的类型是 **map** 和 **set**.    

标准库提供了 8 个关联容器. 区分的主要依据是:   
- 或者是一个 set, 或者是一个 map;    
- 或者要求不重复的关键字, 或者要求重复的关键字(带 multi);    
- 按顺序存储或无序存储(带 unordered_);    

map 和 multimap 定义在头文件 map 中;   
set 和 multiset 定义在头文件 set 中;   
unordered_map 定义在头文件 unordered_map 中;   
unordered_set 定义在头文件 unordered_set 中;   

这里主要介绍 map, set 的操作是类似的.       

## map 的基本操作   

map 类型通常被称为 "关联数组". 关联数组与正常数组类似, 不同之处在于其下标不是整数. 我们是通过一个关键字而不是位置来查找数组中的值.    

```cpp
#include <map>
/***
 *  输出流文件结束标志 Ctrl + D
 */
using namespace std;

int main(int argc, char *argv[])
{
    map<string, int> word_count;
    string word;

    while(cin >> word){
        ++word_count[word];
    }

    for(const auto & w : word_count){  // 使用基于范围的 for 语句遍历 map
        cout << w.first << " occurs " << w.second <<
        (w.second > 1 ? " times" : "time") << endl;
    }
    return 0;
}
```

### pair 类型    

当从 map 中提取一个元素时, 会得到一个 pair 类型的对象, 一个 pair 保存两个数据成员.   

**注意: map 的元素是 pair**;    

```cpp
pair<string, string> anon;  // anon = pair<string, string>()
pair<string, vector<int>> line; 

pair<string, string> author = {"Joyce", "James"}; 
pair<string, string> eta = make_pair("Joyce", "James"); 
pair<string, string> book = map<string, string>::value_type("Joyce", "James"); 
```
与其他类型不同, pair 的数据成员是 public 的. 分别为 first 和 second.   

### 关联容器中关键字和值的类型    

|类型别名|描述|
|:----|:----|
|key_type|关键字类型|
|mapped_type|只有 map 类型定义了, 为 map 中的关联值类型|
|value_type|对于 set, 与 key_type 相同; 对于 map, 为 pair<const key_type, mapped_type> |

使用作用域运算符来提取一个类型的成员 - 例如: map<string, string>::key_type key;    

pair 的关键字部分是 const 的, 即我们不能改变一个元素的关键字.   

### 关联容器的迭代器   

当解引用一个关联容器迭代器时, 我们会得到一个类型为容器的 value_type 的值的引用.     

对 map 而言, value_type 是一个 pair 类型, 其 first 成员保存 const 类型的关键字, second 成员保存值.  

对 set 而言, 虽然定义了 iterator 和 const_iterator 类型, 但是两种类型都是允许只读访问 set 中的元素.     

可以使用迭代器遍历关联容器.   

### 插入   

因为 map 和 set 中不包含重复的关键字, 因此插入已存在的元素对容器没有任何影响.   

对一个 map 进行 insert() 操作时, 必须记住元素类型是 pair. 通常对于想要插入的数据并没有一个现成的 pair 对象.    



```cpp
// c.insert(v), c.emplace(args)   
map<string, size_t> word_count;
string word = "hello Map";

word_count.insert({word, 1});
word_count.insert(pair<string, size_t>(word, 1));
word_count.insert(make_pair(word, 1);
// 返回值类型
pair<map<string, size_t>::iterator , bool> ret = word_count.insert(map<string, size_t>::value_type(word, 1));

// 很少用.  c.insert(pos, v), c.emplace(pos, args)  
word_count.insert(word_count.begin(), make_pair(word, 1);   

// 很少用.  c.insert(begin, end)   
word_count.insert(word_count.begin(), word_count.end());   

while(cin >> word){
    auto ret = word_count.insert(make_pair(word, 1));
    if(!ret->second)
        ++ret->first->second;
}


map<int, int> label_hist;
label_hist.insert(std::make_pair(0, 0));
label_hist.insert(std::make_pair(label, 0));
int label = 2;
label_hist[label]++;

label_hist[0]++;
```
尝试着解读 `++ret->first->second;` 这条语句的含义.  

insert() 和 emplace() 的返回一个 pair, 告诉我们插入操作是否成功. pair 的 fisrt 成员是一个迭代器, 指向具有给定关键字的元素; second 成员是一个 bool 类型的值, 只是元素是成功插入还是已经存在于容器中.    

如果关键字已经在容器中, 则 insert 什么都不做, 且返回值中的 bool 类型值为 false. 如果关键字不存在, 则元素被插入容器中, 且 bool 值为 true.    

### 删除元素   

关联容器定义了 3 个版本的 erase() 操作. 但是这 3 个版本的返回值类型并不完全相同.   

|类型别名|描述|
|:----|:----|
|c.erase(k)|根据 Key 来进行删除， 返回删除的元素数量(在 map 里结果非 0 即 1)|
|c.erase(pos)| 删除迭代器指向位置的键值对元素， 返回一个指向 pos 之后元素的迭代器|
|c.erase(begin, end)| 删除一定范围内的元素，并返回迭代器 end|
|c.clear()| 清空 map，清空后的 size 为 0|

```cpp
auto iter = iset.begin();
iter++;
iter+=1;   // error: 没有这个运算符操作
```

### map 的下标操作    

map 和 unordered_map 提供了下标运算符和一个对应的 at 函数.  

set  类型不支持下标, 因为 set 中没有与关键字相关联的"值".    

multimap 和 unordered_multimap 也没有下标操作, 因为其中可能有多个值与一个关键字相关联.   

**注意: 与其他下标运算不同的是, 如果关键字并不在 map 中, 下标操作会为它创建一个元素并插入到 map 中, 关联值将进行值初始化**;    

由于下标运算符可能插入一个新元素, 我们只可以对非 const 的 map 使用下标操作.    

at(k) 操作和下标操作的效果相同, 区别是如果 k 不在 map 中, 会抛出一个 out_of_range 异常.    

当对一个 map 进行下标操作时, 会获得一个 mapped_type 对象; 但当解引用一个 map 迭代器时, 会得到一个 value_type 对象.    

### 查找元素    

如果我们只关心一个特定的元素是否在给定容器中, 而不想改变它, 那么使用 find 操作是最佳选择.    

|操作|描述|
|:---|:---|
|c.find(k)|返回一个迭代器, 指向第一个 k 所在的元素.  如果没有这个关键字, 返回尾后迭代器|
|c.count(k)|返回容器中关键字等于 k 的元素个数|
|c.lower_bound(k)|返回一个迭代器, 指向第一个关键字不小于 k 的元素(k 这个元素)|
|c.upper_bound(k)|返回一个迭代器, 指向第一个关键字大于 k 的元素(k 之后的元素)|
|c.equal_range(k)|返回一个迭代器 pair, 表示关键字等于 k 的元素的范围. 若 k 不存在, pair 的两个成员均等于 c.end()|

以上操作对 multimap(set) 和 map(set) 均适用.   

如果一个 multimap 或 multiset 中有多个元素具有给定的关键字, 则这些元素在容器中是相邻存储的.  

## 关联容器的定义   

关联容器不支持顺序容器的位置相关的操作, 如 push_front 和 push_back. 原因是关联容器中元素是根据关键字存储的(有序容器会进行排序), 这些操作对关联容器没有意义.    

一个 map 或 set 中的关键字必须是唯一的. 即对于一个给定的关键字, 只能有一个元素的关键字等于它. 容器 multimap 和 multiset 没有此限制, 他们都允许多个元素具有相同的关键字.    

```cpp
map<string, string> authors = { {"Joyce", "James"},
                                {"Austen", "Jane"},
                                {"Dickens", "Charles"}

vector<int> ivec;
for (vector<int>::size_type i = 0; i <= 10; ++i){
    ivec.push_back(i);
    ivec.push_back(i);
}

set<int> iset(ivec.begin(), ivec.end());
multiset<int> imset(ivec.begin(), ivec.end());
cout << "ivec: " << ivec.size() << endl;
cout << "iset: " << iset.size() << endl;
cout << "imset: " << imset.size() << endl;
```
输出为:    
```
ivec: 22
iset: 11
imset: 22
```

## 关键字类型的要求    

关联容器对关键字类型有一些限制: 关键字类型必须定义元素比较的方法. 默认情况下, 标准库使用关键字类型的 "<"　运算符来比较两个关键字．　　　

可以向一个算法提供我们自定义的比较操作．与此类似, 也可以提供自己定义的操作来代替关键字上的 "<" 运算符(必须满足严格弱序).   　　

为了指定使用自定义的比较操作, 必须在定义关联容器类型时提供此操作的类型(在尖括号中紧跟着元素类型给出), 同时在定义容器类型对象时提供想要使用的操作的指针.     

```cpp
bool compareIsbn(const Sales_data &lhs, const Sales_data & rhs)
{
    return lhs.isbn() < rhs.isbn();
}
...

typedef bool (*isbn_type) (const Sales_data &lhs, const Sales_data & rhs);
multiset<Sales_data, decltype(compareIsbn) *> bookStore_1(compareIsbn); 
multiset<Sales_data, isbn_type> bookStore_2(compareIsbn); 
```
这里我们使用自定义的比较函数 compareIsbn 来定义关联容器的 key 类型.   

## map 排序    

关联容器可用于只读取元素的算法. 由于关联容器中的圆度不能通过他们的关键字进行快速查找, 因此也一般不直接对关联容器使用泛型算法.    

如果一定要使用算法的话, 要么是将它当成一个源序列, 要么当做一个目的位置.    

`pair` 的使用.    

```cpp
vector<std::pair<int, int> > mapping;
const Dtype* perm = bottom[1]->cpu_data();
for (int i = 0; i < bottom[1]->count(); ++i) {
    mapping.push_back(pair<int, int>(static_cast<int>(perm[i]), i));
}
std::sort(mapping.begin(), mapping.end(), pair_sort_first());
```
