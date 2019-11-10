#ifndef _MAGIC_SIMPLE_FACTORY_HEADER_
#define _MAGIC_SIMPLE_FACTORY_HEADER_

#include <string>
#include <iostream>

using namespace std;


namespace magic {
/**
 * 简单工厂模式解决的问题:  
 * 1. 将主业务逻辑抽象: 使用多态性(虚函数);  
 * 2. 使用继承实现具体业务逻辑;  
 * 3. 用一个单独的类完成对象的实例化操作, 这个类就是工厂类;  
 * 
 * 可复用: 将具体的逻辑实现用类封装;  
 * 易维护: 重复代码越少;  
 * 易扩展: 只需要修改工厂类即可;  
*/

using std::string;

/**
 * Base Class with a virtual method.  
 * 
 * Note:
 *      注意这里的 const 的使用  
 * 
 * issues: undefined reference to vtable xxx
 *      原因是: 继承关系中, 如果父类中的虚函数没有实现，就会导致这个链接错误 
*/
class Operator {
public:
    Operator() { }

    void set_number_a(double number) { number_a_ = number; }
    void set_number_b(double number) { number_b_ = number; }

    double get_number_a() const { return number_a_; }
    double get_number_b() const { return number_b_; }

    virtual double get_result() const { std::cout << "Base Operator"  << std::endl; }
private:
    double number_a_;
    double number_b_;
};

class Add : public Operator{
public:
    double get_result() const { return get_number_a() + get_number_b(); }
};

class Subtract : public Operator{
public:
    double get_result() const { return get_number_a() - get_number_b(); }
};

class Multiply : public Operator{
public:
    double get_result() const { return get_number_a() * get_number_b(); }
};

class Divide : public Operator{
public:
    double get_result() const { return get_number_a() / (get_number_b() + eps); }
private: 
    double eps = 0.00000001;
};

/**
 * Simple Factory of Operator.
 *      All instance will be managed here. 
*/
class OperatorFactory {
public:
    // Get the instance of a sepcified operator.  
    // 然后利用多态性的运行时确定最终调用的函数.
    static Operator* getOperator(const char oper);
};

}; // end of magic

#endif