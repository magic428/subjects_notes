
#include "hello.h"
#include "helloclass.h"

// #define BOOST_NO_CXX11_SCOPED_ENUMS
// #include <boost/filesystem.hpp>
// #undef BOOST_NO_CXX11_SCOPED_ENUMS
// #include "boost/algorithm/string.hpp"

// using namespace boost::filesystem;
// namespace bfs = boost::filesystem;

int main()
{
    hello("hello,cmake");
	test::helloclass example;
	example.show();
    return 0;
}
