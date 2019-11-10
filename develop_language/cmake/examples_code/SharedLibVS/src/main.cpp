#include <iostream>

#include "dll_lib.h"

using namespace std;

int main()
{
	std::cout << "Sum: " << sum(2, 4) << std::endl;
	
	std::string desc("dll test detector");
    Detector* detector = Detector::createNew(desc, 100, 200, 3, 1);
    int width = detector->get_net_width();
    
    std::cout << "Net width: " << width << std::endl;
    std::cout << "maxSize: " << detector->maxSize << std::endl;

    getchar();
    return 0;
}  