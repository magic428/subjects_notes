#include "singleton.h"
#include <iostream>

namespace magic {
// Make sure each thread can have different values.
static Singleton* singleton_instance_;

Singleton& Singleton::get_instance()
{
    if(!singleton_instance_)
        singleton_instance_ = new Singleton();
        
    return *singleton_instance_;
}

int demo()
{
    Singleton slt = Singleton::get_instance();
    Singleton slt_2 = Singleton::get_instance();
    Singleton slt_3 = Singleton::get_instance();
    
    std::cout << "res1: " << (&slt) << std::endl;
    std::cout << "res2: " << (&slt_2) << std::endl;
    std::cout << "res3: " << (&slt_3) << std::endl;

    return 0;
}

}