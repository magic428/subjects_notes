/**
 *  shared_ptr 的用法
 *  需要区分的几个概念:  
 *  - 指针的指向 和 指针的引用计数.   
 *  - p.use_count() 指示的是当前与 p 共享对象的智能指针的个数.   
 *  - p.use_count() 和当前指针的引用计数不是一回事.   
 *  
 *  计数递减
 *  - 赋值操作会递减赋值号左侧 shared_ptr 的引用计数.
 *  - shared_ptr 被销毁 (例如一个局部的 shared_ptr 离开其作用域)
 *  
 *  计数递增   
 *  - 拷贝一个 shared_ptr 会递增其计数.
 *  - 赋值操作会递增赋值号右侧 shared_ptr 的引用计数, 
 *   
 *  如果一个 shared_ptr 的引用计数变为 0, 它所指向的对象会被自动销毁.  
 *  
 *  智能指针会自动依赖于类对象的拷贝,复制和销毁操作
 *  - 当我们拷贝,赋值,销毁一个 StrBlob 对象时, 它的 shared_ptr  指针也会拷贝,赋值,销毁;  
*/
#include <iostream> 
#include <initializer_list>  
#include <vector>
#include <string>
#include <memory>

using namespace std;


class StrBlob {
public:
    typedef std::vector<std::string>::size_type size_type ;

    StrBlob();
    StrBlob(std::initializer_list<std::string> il);

    size_type size() const { return data->size(); }
    bool empty() const { return data->empty(); }

    // 添加和删除元素
    void push_back(const std::string &t) { data->push_back(t);}



    std::shared_ptr<std::vector<std::string>> data;
private:
    void check(size_type i, const std::string &msg);
};

StrBlob::StrBlob() : data(make_shared<vector<string>>()) {}
StrBlob::StrBlob(initializer_list<string> il) : data(make_shared<vector<string>>(il)) {}

struct Test{
    int a = 1;
};

void end_test(void *)
{
    cout << "delete..." << endl;
}

int main()
{
    std::cout << "###################################################" << std::endl;
    std::cout << "################ EXAMPLE: shared_ptr ##############" << std::endl;
    std::cout << "###################################################" << std::endl;

    StrBlob b1;   // b1.data 的引用计数 +1 (=1) 
    {
        StrBlob b2 = {"a", "an", "the"};    // b2.data 的引用计数 +1,  (=1);   

        // b1.data 指针的引用计数 -1, (=0), b1.data 原来指向的对象已经没有引用者, 会被自动释放;
        // b2.data 指针的引用计数 +1, (=2);    
        b1 = b2;   

        b2.push_back("about");
    } // b2.data 指针的引用计数 -1, (=1);

    // 自定义 delete 函数
    Test q;
    // shared_ptr<Test> p(&q, end_test);
    shared_ptr<Test> p(&q, [](void *) { cout << "del.." << endl; });
    
    // unique_ptr
    unique_ptr<int> p_u(new int(23));
    unique_ptr<int> q_u;
    //q_u = p_u;

    return 0;
} // b1.data 指针的引用计数 -1, (=0);    
