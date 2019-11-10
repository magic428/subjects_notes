#ifndef _MAGIC_SINGLETON_HEADER_
#define _MAGIC_SINGLETON_HEADER_

namespace magic {

class Singleton{
public:
    static Singleton &get_instance();

private:
    Singleton() { }
};

}
#endif