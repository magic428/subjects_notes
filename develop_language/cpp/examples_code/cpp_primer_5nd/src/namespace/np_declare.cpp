/* file1.cpp */  
/**
 * 命名空间   
 *  1. 命名空间可以是全局的，也可以位于另一个命名空间之中，但是不能位于类和代码块中。
 *  2. 在所有命名空间之外，还存在一个全局命名空间，它对应于文件级的声明域。因此，在命名空间机制中，
 *  原来的全局变量，现在被认为位于全局命名空间中。
 */

/**
 *  先考虑一下，先不妨假设没有 namespace np 的限制，即相关变量和函数都直接定义和声明
 * 	在全局空间里面，以全局(global)的视角来看这个程序：
 *	 1. 如 print_values() 不在 npp_declare.cpp 中声明时， npp_declare.cpp 能编译通过吗？
 *	 2. 如果 npp_declare.cpp 中整数b的声明中把 extern 去掉，链接的时候是不是会报重复定义的错？
 *	 3. npp_declare.cpp 中的 print_values() 函数打印的是哪个 c 的值？
 *
*/
  
#include <iostream>  
#include <string>  
#include <map>

using namespace std;
  
namespace np
{  
    extern int b;  
    static int c = 7;  
    void print_values();  

#define TEST_MARCO(obj) (cout << "test_"#obj << endl)
}  
 
namespace Spaceone
{
	int a_;
}

namespace Spacetwo
{
    int a = 50;
    namespace Spacethree
    {
        struct date
        {
            int year;
            int month;
            int day;
        };
    }
}

// A simple registry for caffe commands.
typedef int (*BrewFunction)();
typedef map<string, BrewFunction> BrewMap;
BrewMap g_brew_map;

#define RegisterBrewFunction(func) \
namespace { \
class __Registerer_##func { \
 public: /* NOLINT */ \
  __Registerer_##func() { \
    g_brew_map[#func] = &func; \
  } \
}; \
__Registerer_##func g_registerer_##func; \
}

static BrewFunction GetBrewFunction(const string& name) {
  if (g_brew_map.count(name)) {
    return g_brew_map[name];
  } else {
    cout << "Available caffe actions:";
    for (BrewMap::iterator it = g_brew_map.begin();
         it != g_brew_map.end(); ++it) {
      cout << "\t" << it->first;
    }
    cout << endl << "Unknown action: " << name << endl;
    return NULL;  // not reachable, just to suppress old compiler warnings.
  }
}

int test_1()
{
	/*...*/
	cout << "test_1" << endl;
	return 1;
}
RegisterBrewFunction(test_1)

int test_2()
{
	/*...*/
	cout << "test_2" << endl;
	return 2;
}
RegisterBrewFunction(test_2)


// using namespace Spacetwo;
// using Spacetwo::a;
int a = 10;

int main_()  
{  
    np::print_values();  

    using namespace Spaceone;
    using  Spacetwo::Spacethree::date;
    int a = 10;

    date d;
    cout << "Spaceone a = " << a << "\n";
    cout << "spacetwo a = " << Spacetwo::a << "\n";
    d.year = 2017;
    d.month = 6;
    d.day = 2;
    cout << d.year << "年" << d.month << "月" << d.day << "日";
    cout << endl;

    TEST_MARCO(d);
    GetBrewFunction("test_2")();
}
