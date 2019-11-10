#include <boost/thread/thread.hpp>
#include <boost/thread/once.hpp>
#include <iostream>

int i = 0;
boost::once_flag flag = BOOST_ONCE_INIT;  

void init()
{
    ++i;
}

void thread()
{
    boost::call_once(&init, flag);
}

int main(int argc, char* argv[])
{
    boost::thread thrd1(&thread);
    boost::thread thrd2(&thread);
    thrd1.join();
    thrd2.join();
    std::cout << i << std::endl;
    return 0;
}