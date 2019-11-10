#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <limits>

using namespace std;

template<typename T>
class MyCalculator {
public:
    explicit MyCalculator();

    T add(const T& t1, const T& t2);




private:
    std::shared_ptr<int> init_;
}; 

template<typename T>
MyCalculator<T>::MyCalculator() 
    : init_(new int()) {
}

template<typename T>
T MyCalculator<T>::add(const T& t1, const T& t2)
{
    return t1+t2;
}



int main(int argc, char * argv[])
{
    MyCalculator<float>* fc = new MyCalculator<float>();
    double ret = fc->add(12, 3.4);

    MyCalculator<string>* dc = new MyCalculator<string>();
    std::string s = dc->add("hello", " world");
    
    std::cout << ret << std::endl;
    std::cout << s << std::endl;

    return 0;
}
