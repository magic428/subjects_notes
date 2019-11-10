#include "const_enum_inline.h"

#include <iostream>
#include <string>

void displayMesg(const std::string &msg)
{
    std::cout << msg;
}

int main()
{
    displayMesg("=============================================\n");
    displayMesg("========= const_enum_inline example =========\n");
    displayMesg("=============================================\n\n");

    GamePlayer gp;
    DisplayScreen ds;
    callWithMax<int>(5, 6);
    
    return 0;
}