# string 操作   

## 构造函数    

|string 构造函数|功能|
|:--------------|:-----------|
|string s(cp)| s 是 cp 字符指针指向的数组中的字符串|
|string s(cp, n)| s 是 cp 字符指针指向的数组中前 n 个字符的拷贝|
|string s(s1, pos1)| s 是 s1 从下标 pos1 开始的所有字符拷贝, 默认从下标 0 开始拷贝|
|string s(s2, pos2, len)| s 是 s2 从下标 pos2 开始的 len 个字符拷贝|

**原则**:   

能使用 string 的构造函数就不要使用复制运算符.  

## string 属性方法  


|string 属性方法|描述 |
|:--------------|:-----------|
|size()  | 返回值类型为 string::size_type, 禁止和 int 类型混用 |
|empty() | 返回值类型为 bool |

## substr 操作   

substr 返回一个 string, 它是原始 string 的一部分或全部的拷贝.   

可以传递给 substr 一个 `可选的` 开始位置和计数值.    

```cpp
substr(pos, n)
```
返回包含 s 中从 pos 开始的 n 个字符的拷贝. pos 的默认值为 0, n 的默认值为 s.size() - pos.   

## 改变 string 的其他方法   

|修改 string 的操作|功能|
|:--------------|:-----------|
|s.insert(pos, args)| 在 pos 之前插入 args 指定的字符, pos 可以是下标或迭代器|
|s.erase(pos, len)| 删除从 pos 开始的 len 个字符, 返回一个指向 a 的引用|
|s.assign(args)| 将 s 中的字符替换为 args 中指定的字符, 返回一个指向 a 的引用 |
|s.append(args)| 将 args 中指定的字符追加到 s, 返回一个指向 a 的引用|
|s.replace(pos, len, args)| 先从 pos 位置开始删除 n 个元素, 然后将 str 插入到 pos 位置之前, 返回一个指向 a 的引用|
|s.replace(b, e, args)| 删除 b, e 范围内的元素, 然后将 args 指定的字符插入到 b 位置之前, 返回一个指向 a 的引用|
| + |  "+" 运算符两侧至少要有一个 string 对象 |


args 可以是以下 形式之一, append 和 assign 可以使用所有形式.   
str 不能和 s 相同, 迭代器 b 和 e 不能指向 s.    
str                 字符串 str   
str, pos, len       str 中从 pos 开始的(最多) len 个字符   
cp                  cp 指向空字符结尾的字符数组   
cp, len             从 cp 中指向的数组的前(最多) len 个字符   
n, c                n 个字符 c   
b, e                迭代器 b 和 e 指定的范围内的字符   
初始化列表            花括号包围的, 以逗号分隔的字符列表   

1. string 类型支持顺序容器的赋值运算符, assign, insert 和 erase 操作. 除此之外, 它还定义了额外的 insert 和 erase 版本.   

除了接受迭代器的 insert 和 erase 版本外, string 还提供了接受下标的版本. 下标指出了开始删除的位置, 或是 insert 到给定下标之前的位置.   
```cpp
a.insert(a.size(), 5, '!');
a.insert(a.size(), "@@");

const char * cp_ = "*^_^*";
a.insert(a.size()-7, cp_);

a.erase(a.size()-7, 7);

``` 
2. append 和 replace 函数   

```cpp
a.append(" Chinese Edition.");
a.replace(pos, n, str)
```
replace 操作是先从 pos 位置开始删除 n 个元素, 然后将 str 插入到 pos 位置. 返回一个指向 a 的引用.     

## string 的搜索操作   

如果搜索失败, 返回一个名为 string::npos 的 static 成员(实际为 static const string::size_type npos = -1).   

|string 的搜索操作|功能|
|:--------------|:-----------|
|s.find(args)| 查找 s 中 args 第一次出现的位置 |
|s.rfind(args)| 查找 s 中 args 最后一次出现的位置 |
|s.find_first_of(args)| 在 s 中查找 args 中任何一个字符第一次出现的位置|
|s.find_last_of(args)| 在 s 中查找 args 中任何一个字符最后一次出现的位置|
|s.find_first_not_of(args)| 在 s 中查找第一个不在 args 中的字符|
|s.find_last_not_of(args)| 在 s 中查找最后一个不在 args 中的字符|

其中 args 必须是以下形式之一:   
c, pos   
s2, pos   
cp, pos   
cp, pos, n   
pos 指定在哪里开始搜索, 前三个 pos 的默认值为 0, 最后一个的 pos 和 n 无默认值.     

## 数值转换.    

|string 和数值之间的转换操作|功能|
|:--------------|:-----------|
|to_string(val)| 一组重载函数, 返回数值 val 的 string表示 |
|stoi(s, p, b)| 返回 s 表示的整数值, s 的起始子串内容必须表示整数|
|stol(s, p, b)| 返回 s 表示的整数值, s 的起始子串内容必须表示整数|
|stoul(s, p, b)| 返回 s 表示的整数值, s 的起始子串内容必须表示整数|
|stoll(s, p, b)| 返回 s 表示的整数值, s 的起始子串内容必须表示整数|
|stoull(s, p, b)| 返回 s 表示的整数值, s 的起始子串内容必须表示整数|
|stof(s, p)| 返回 s 表示的浮点数, s 的起始子串内容必须表示浮点数|
|stod(s, p)| 返回 s 表示的浮点数, s 的起始子串内容必须表示浮点数|
|stold(s, p)| 返回 s 表示的浮点数, s 的起始子串内容必须表示浮点数|

其中 p 用来保存 s 中第一个非数值字符的下标.   

## 处理 string 对象中的字符.    

|cctype 函数|功能|
|:--------------|:-----------|
| isalnum(c) |  |
| isalpha(c) | |
|  | |
|  | |
|  | |
|  | |
|  | |
|  | |
|  | |

## 处理每个字符: 使用基于范围的 for 循环

```cpp
for (declaration : expression)
    statement;
```

如果需要改变字符的内容, for循环中的变量应该使用"引用"类型.  

## 大小写转换   

```cpp
string word = "Hello String!";

// 全部转换为小写
transform(word.begin(), word.end(), word.begin(), (int (*)(int))tolower);

// 全部转换为大写
transform(word.begin(), word.end(), word.begin(), (int (*)(int))toupper);
```

## 如果将 string 按照空白字符分割  

使用 stringstream 对象简化类型转换. 注意，<sstream>使用 string 对象来代替字符数组。这样可以避免缓冲区溢出的危险。   

而且，传入参数和目标对象的类型被自动推导出来，即使使用了不正确的格式化符也没有危险。

一个实例.  

```cpp
#include<iostream>
#include<string>
#include<sstream>
#include<vector>
using namespace std;

int main(){
    // 用于存放分割后的字符串 
    vector<string> res;
    // 待分割的字符串，含有很多空格 
    string word="   Hello, I want   to learn C++!   ";
    // 暂存从 word 中读取的字符串 
    string result;
    // 将字符串读到 input 中 
    stringstream input(word);
    // 依次输出到result中，并存入res中 
    while(input>>result)
        res.push_back(result);
    // 输出res 
    for(int i=0;i<res.size();i++){
        cout<<res[i]<<endl;
    }
    return 0;
}
```

总结  

1. <sstream> 还能实现自动类型转换。 
2. 在C++ Primer书中曾经提到过:  

```cpp
int main(){
    string s;
    cin>>s;
    cout<<s<<endl;
    return 0;
}
```

cin >> s 从标准输入终端读取内容并保存到 s 中, >> 运算符会进行以下操作:   

(1) 读取并丢弃开头的空格字符 (如空格,换行符和 tabs).  
(2) 然后读取字符直到遇到下一个非空白字符.  

在执行读取操作时, string 对象会自动忽略开头的空白 (即空格符、换行符和制表符等)并从第一个真正的字符开始读起，直到遇到下一处空白为止。  

例如：输入的字符串为 "  Hello, I want to learn C++! "，则传入 cin 的只有 "Hell0,".   

这一点非常重要，必须牢记在心。  

**使用 getline 读取一整行**   

getline() 函数的参数是一个输入流和 string 对象. 函数从给定的输入流中读入内容, 直到遇到换行符为止(注意换行符也被读进来了), 然后把所读到的内容存到那个 string 对象中 (注意不保存换行符).   

getline() 只要一遇到换行符就结束读取并返回结果.   

**注意**: getline() 读取时会把输入流中的换行符读取进来, 但是在保存内容到 string 对象的时候并不会保存换行符.  

