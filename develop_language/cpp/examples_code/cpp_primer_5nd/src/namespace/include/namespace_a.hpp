#ifndef _IMAGE_PROCESS_H_
#define _IMAGE_PROCESS_H_

#include <iostream>
#include <string>


namespace my_np {

void print_func();
    
class Blob{
public:
    // int count() {}  //错误
    int count() const {}  //正确
};

void getCount(const Blob& blob);

}

void print_func();
#endif