#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <limits>

using namespace std;

int main(int argc, char * argv[])
{
    // 构造函数
    const char *cp = "hello c++ primer 5nd";   // const 变量
    string s1(cp);
    string s2(cp, 9);
    string s3(s1, 0, 9);
    string s4(s1);
    cout << "s1 = " << s1 << endl;
    cout << "s2 = " << s2 << endl;
    cout << "s3 = " << s3 << endl;
    cout << "s4 = " << s4 << endl;

    //substr()
    string a("hello c++ primer 5nd");
    string a2 = a.substr(0, 5);
    string a3 = a.substr(6);
    string a4 = a.substr(6, 3);
    string a5 = a.substr(10);
    cout << "a = " << a << endl;
    cout << "a2 = " << a2 << endl;
    cout << "a3 = " << a3 << endl;
    cout << "a4 = " << a4 << endl;
    cout << "a5 = " << a5 << endl;

    // insert erase
    a.insert(a.size(), 5, '!');
    a.insert(a.size(), "@@");
    cout << "a = " << a << endl;
    
    const char * cp_ = "*^_^*";
    a.insert(a.size()-7, cp_);
    cout << "a = " << a << endl;

    a.erase(a.size()-7, 7);
    cout << "a = " << a << endl;

    /// string 类型的数值转换   
    double d = 0.341;
    auto s = to_string(d);

    d = stod("0.347ss");
    s = "pi = 3.14";
    size_t pos;
    auto f = stod(s.substr(s.find_first_of("+-.0123456789")), &pos);
    int i = stoi("123ss", &pos, 10);
    cout << "d = " << d << endl;
    cout << "s = " << s << endl;
    cout << "f = " << f << endl;
    cout << "p = " << pos << endl;   // 返回表示整数的字符子串后第一个非数值字符的下标.   
    cout << "f = " << i << endl;

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