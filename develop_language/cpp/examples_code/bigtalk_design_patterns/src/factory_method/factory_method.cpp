#include "factory_method.h"

namespace magic {

int demo()
{
    AddFactory af;
    Operator * of = af.createOperator();
    of->set_number_a(2.1);
    of->set_number_b(5.1);
    double res = of->get_result();

    std::cout << "res = " << res << std::endl;
    return 0;
}

}
