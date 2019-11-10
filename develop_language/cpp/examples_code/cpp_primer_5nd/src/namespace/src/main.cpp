#include "namespace_a.hpp"
#include "namespace_b.hpp"


// namespace my_np{

//     void test(){  my_np::print_func();}
// };

// using my_np::print_func;


int main(int argc, char **argv)
{
    const my_np::Blob b;
    my_np::getCount(b);
    
    ::print_func();
    my_np::print_func();

    return 0;
}
