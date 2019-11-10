#ifndef _MAGIC_FACTORY_METHOD_HEADER_
#define _MAGIC_FACTORY_METHOD_HEADER_

#include <string>
#include <iostream>

using namespace std;

/**
 * 简单工厂模式中依然存在的问题:
 * 1.如果客户端使用多个相同的工厂, 导致重复代码较多: 
 * 2.如果要添加一个新的功能, 需要更改工厂类: 
 * 
 * 工厂方法解决的问题:  
 * 1. 保证相同的工厂在客户端只需出现一次实例化;
 * 2. 继续抽象出一个工厂接口, 只需要继承添加新的类即可;  
 * 3. 克服了简单工厂违背的 "开放-封闭" 原则的缺点, 保持了对对象创建过程的封装;
 * 4. 它是简单工厂模式的进一步抽象和推广;
 * 
 * 区别:  
 *  如果要添加功能, 在简单工厂中是修改工厂类的; 在工厂方法中是修改客户端.
*/

namespace magic {

using std::string;

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


// Factory Method 

class IFactory {
public:
    virtual Operator* createOperator() = 0;  // Pure Virtual
};

class AddFactory : public IFactory {
public:
    Operator* createOperator()
    {
        return new Add();
    }
};

class SubtractFactory : public IFactory {
public: 
    Operator* createOperator()
    {
        return new Subtract();
    }
};

class MultiplyFactory : public IFactory {
public: 
    Operator* createOperator()
    {
        return new Multiply();
    }
};

class DivideFactory : public IFactory {
public: 
    Operator* createOperator()
    {
        return new Divide();
    }
};

}; // end of magic

#endif