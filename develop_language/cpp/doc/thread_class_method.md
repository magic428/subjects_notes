# 类成员函数作为线程函数使用

C++ 类成员函数使用时，都会隐式传递一个 this 指针给该函数，this 指针指向当前对象。函数体可以显式地使用 this 指针或直接访问类内成员。   

`回调函数` 是用指针来调用的函数， 最常使用的回调函数就是在创建线程时， 以一个函数指针以及传递给这个函数多个参数来调用线程函数。而一般的类成员函数是不能用作回调函数的， 因为回调函数被使用时，都会传递符合回调函数声明的参数给使用者，而类成员函数隐式包地含一个 this 指针参数，所以把类成员函数当作回调函数编译时因为参数不匹配会出错。   

因为一般都是用`静态函数`作为线程的回调函数实现，但是有时候总是感觉不是很顺畅，更改，就好像破坏了类的封装性，不改，访问实在是麻烦。 

## 方法 1. 联合 - 指针转换  

为了让类的成员函数作为线程的回调函数， 其中使用的一个比较特殊的结构就是:    

```cpp
// 例如对于 Student 类
union
{
    void (*ThreadProc)(void* pvParam);
    void (Student::*MemberProc)(void* pvParam);
} Proc;
```

联合，用于转换类成员方法指针到普通函数指针。   

下面看实例：

```cpp
// 下面例子中用 printInfo 作为线程回调函数
#include"stdafx.h"
#include<string>
#include<Windows.h>
#include<iostream>

using namespacestd;

class Student
{
public:
    Student()
    {
        m_handle = NULL;
        name = "Member fun is ThreadFun.";
        age = 13;
    }
    void printInfo(LPVOID pvParam);  //作为线程回调函数
    void startUp();

private:
    HANDLE m_handle;
    int age;
    string name;
};

union   //用于转换类成员方法指针到普通函数指针(地址相同)
{
    void ( *ThreadProc)(LPVOID pvParam);
    void (Student::*MemberProc)(LPVOID pvParam);
} Proc;

void  Student::printInfo(LPVOID pvParam)
{
    Student * pS = (Student * )pvParam;
    
    //线程每隔一时间执行，如果要唤醒就加一个事件，改变信号状态   
    while( WaitForSingleObject(pS->m_handle,30000) != WAIT_OBJECT_0 )  
    {
        cout<< "age" <<"" <<pS->age<< endl;
        cout<< "name" <<" " <<pS->name<< endl;
        Sleep(2000);
    }
}

void Student::startUp()
{
    Proc.MemberProc = &Student::printInfo;  //指向成员函数

    // ThreadProc 和 MemberProc 在共联体类中，所以也间接指向成员函数
    m_handle = CreateThread(NULL,0,LPTHREAD_START_ROUTINE(Proc.ThreadProc),this,0,0);
}


int main(int argc, char* argv[])
{
    Student s1;
    s1.startUp();

    system("pause");
    _CrtDumpMemoryLeaks();
    return 0;
}
```

## 方法2. 通过友元函数实现   

```cpp
#include"stdafx.h"
#include<string>
#include<Windows.h>
#include<iostream>

using namespace std;

class Student
{
public:
    Student()
    {
         m_handle = NULL;
         name = "Member fun  is ThreadFun.";
         age = 13;
    }
    friend UINT WINAPI printInfo(LPVOID pvParam);
    void startUp();

private:
    HANDLE m_handle;
    int age;
    string name;
};

UINT WINAPI printInfo (LPVOID pvParam)
{
    Student * pS = (Student * ) pvParam;
    while(true ){
        cout <<"age"<<pS-> age<<endl ;
        cout <<"name"<<pS->name <<endl;
        Sleep (2000);
    }
    
    return 0 ;
}

void Student::startUp()
{
    m_handle = CreateThread(NULL,0,LPTHREAD_START_ROUTINE(printInfo),this,0,0);
}

int main(int argc, char* argv[])
{
    Student s1;
    s1.startUp();

    system("pause");
    _CrtDumpMemoryLeaks();
    return 0;
}

以上两种方法可以将一个类的成员函数作为线程函数去执行， 从而让对象维护线程且要做到线程同步，要创建一个自动重置类型的时间对象。如果是人工重置事件对象，当一个线程等待到一个人工重置的事件对象后这个事件对象任然处于有信号状态，所以其他线程也可以得到该事件对象。   
