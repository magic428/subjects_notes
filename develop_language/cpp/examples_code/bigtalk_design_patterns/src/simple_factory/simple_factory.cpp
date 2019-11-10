#include "simple_factory.h"

namespace magic {

Operator* OperatorFactory::getOperator(const char s_oper)
{
    Operator *oper;  // 内存泄露  

    switch(s_oper) {
        case ('+') : 
            oper = new Add();
            break;
        case ('-') : 
            oper = new Subtract();
            break;
        case ('*') : 
            oper = new Multiply();
            break;
        case ('/') : 
            oper = new Divide();
            break;
        default:
            std::cout << "unknown operator!" << std::endl;
            break;
    }

    return oper;
}

int demo()
{
    Operator *oper;
    oper = OperatorFactory::getOperator('+');
    oper->set_number_a(2.5);
    oper->set_number_b(4.1);

    double res = oper->get_result();

    std::cout << "res = " << res << std::endl;

    return 0;
}

}