#include "vector_operation.h"


/**
 * (1) resize(n), reserve(n) - size(), capacity()
 *     当参数 n 不超过 vector 原来 size() 时:
 *      - resize() 只改变 vector 中 size, 即当前元素的个数, vector 的 capacity 并不改变;  
 *      - reserve() 只改变 vector 中 capacity, 即可保存的元素个数, 
 *              vector 中当前元素的个数(即 size)并不改变;  
 *     当参数 n 超过 vector 原来 size() 时:
 *      - resize() 会同时改变 vector 中 size 和 capacity; 
 * 
 * (2) insert()     
 *      - insert(p, t), 在指定的 p 位置插入值 t, (新标准下)返回指向新添加元素的迭代器;
 * 
 *      - insert(p, n, t), 在指定的 p 位置插入 n 个值 t;
 *      - insert(p, b, e), 在指定的 p 位置插入 [b, e] 指定的区间元素;
 *      - insert(p, il), 在指定的 p 位置插入由 il 指定的初始化列表值;
 *        以上三个 insert(), (在新标准下)返回指向新添加元素的迭代器;
 * 
 * 
*/ 

/**
 * (1) resize(n), reserve(n) - size(), capacity()
 *     当参数 n 不超过 vector 原来 size() 时:
 *      - resize() 只改变 vector 中 size, 即当前元素的个数, vector 的 capacity 并不改变;  
 *      - reserve() 只改变 vector 中 capacity, 即可保存的元素个数, 
 *              vector 中当前元素的个数(即 size)并不改变;  
 *     当参数 n 超过 vector 原来 size() 时:
 *      - resize() 会同时改变 vector 中 size 和 capacity; 
 */
void resize_reserve(std::vector<int> &vec)
{
    // resize(), reserve() - size(), capacity()
    cout << "vec's size = " << vec.size() << endl;
    vec.reserve(30); // 为 capacity 分配空间
    cout << "vec.reserve(30):    " << endl;
    cout << "size = " << vec.size() << endl;
    cout << "capacity = " << vec.capacity() << endl << endl;
    vec.resize(10);
    cout << "ivec.resize(10):   " << endl;
    cout << "size = " << vec.size() << endl;
    cout << "capacity = " << vec.capacity() << endl << endl;
    vec.resize(40);
    cout << "ivec.resize(40):   " << endl;
    cout << "size = " << vec.size() << endl;
    cout << "capacity = " << vec.capacity() << endl << endl;
}

/**
 *  (2) insert()     
 *      - insert(p, t), 在指定的 p 位置插入值 t, (新标准下)返回指向新添加元素的迭代器;
 * 
 *      - insert(p, n, t), 在指定的 p 位置插入 n 个值 t;
 *      - insert(p, b, e), 在指定的 p 位置插入 [b, e] 指定的区间元素;
 *      - insert(p, il), 在指定的 p 位置插入由 il 指定的初始化列表值;
 *        以上三个 insert(), (在新标准下)返回指向新添加元素的迭代器;
*/ 
void insert_test(std::vector<int> &vec)
{
    std::vector<int> vec_il = {1, 2, 3, 4, 5, 6};

    auto i_v = vec.insert(vec.begin()+2, 4);
    vec.insert(i_v, 22);
    vec.insert(vec.begin(), 4, 35);
    vec.insert(vec.begin(), vec_il.begin(), vec_il.end());
    vec.insert(vec.begin(), {33, 44, 55, 66});
    cout << "vec.insert:   " << endl;
    cout << "size = " << vec.size() << endl;
    cout << "capacity = " << vec.capacity() << endl;
    for(auto it = vec.begin(); it != vec.end(); it++){

        if( (it - vec.begin()) % 6 == 0 )
            cout << endl;
        cout << setiosflags(ios::right) << setw(5) << *it << " "; 
    }
    cout << endl;
}

// 容器类型不匹配, 但是元素类型可以相容时; 使用 assign() 操作.
void assign_test()
{
    vector<const char *> old_style = {"hello", "vector"};
    list<string> names;

    // names = old_style;  // error: 容器类型不匹配
    names.assign(old_style.cbegin(), old_style.cend());

    // assign 等价于:
    // step(1): slist.clear();  
    // step(2): slist.insert(slist.begin(), 10, "Hiya");
    list<string> slist(3);
    slist.assign(10, "Hiya");
  
}

void copy_create_assign()
{
    // vector 赋值,比较
    vector<int> v1(10, 3);
    vector<int> v2;
    v2 = v1;             // 拷贝赋值运算符
    vector<int> v3 = v1; // 拷贝构造函数, 因为 v3 对象之前并没有被创建;
}

void array_test()
{
    array<int, 10> a1 = {0,1,2,3,4,5,6,7,8,9};
    array<int, 8> a2 = {0};
    array<int, 10> a3 = {0};
    cout << "Init: a2.size = " << a2.size()<< endl;

    a1 = a3;         // 正确: 替换 a1 中的元素
    // a1 = a2;         // error: a1 和 a2 中的元素个数不同
    a2 = {0, 1, 2};  // 覆盖 a2 中的前 3 个元素, 并不会改变其他位置的元素;
    cout << "after assign: a2.size = " << a2.size()<< endl;
    // a2 = {0,1,2,3,4,5,6,7,8,9};  // error: 初始化列表中元素个数大于 a2 中的元素个数
}

int main(int argc, char * argv[])
{
    cp5::VectorTest vt;

    std::vector<int> ivec(10, 111);

    resize_reserve(ivec);  // resize(), reserve() - size(), capacity()
    insert_test(ivec);  // insert()
    assign_test();  // assign
    copy_create_assign(); // 拷贝构造函数和拷贝赋值运算符
    array_test();  // array

    return 0;
}

// 按照精度提取浮点数为字符串  
string do_fraction(long double value, int decplaces = 3)
{
    ostringstream out;
    int prec = numeric_limits<long double>::digits10;    // 18
    out.precision(prec);  //覆盖默认精度
    out<<value;
    string str= out.str(); //从流中取出字符串
    size_t n=str.find(".");
    if ((n!=string::npos) //有小数点吗?
        && (str.size()> n+decplaces))
        //后面至少还有decplaces位吗？ 　
    {
        str[n+decplaces]='\0';//覆盖第一个多余的数
    }
    // str.swap(string(str.c_str()));//删除nul之后的多余字符

    return str;
 }