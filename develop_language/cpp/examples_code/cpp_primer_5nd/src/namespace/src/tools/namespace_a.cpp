#include "namespace_a.hpp"

namespace my_np {

void print_func()
{
    std::cout << "namespace my_np" << std::endl;
}

void getCount(const Blob& blob){
    blob.count();
}


}// namespace my_np

void print_func()
{
    std::cout << "namespace global" << std::endl;
}