/**
 * The main function is an example of video dehazing 
 * 
 * The core algorithm is in "dehazing.cpp," "guidedfilter.cpp," and 
 * "transmission.cpp".  You may modify the code to improve the results.    
 * 
 * The detailed description of the algorithm is presented
 * in "http://mcl.korea.ac.kr/projects/dehazing".     
 * 
 * Last updated: 2013-02-14
 * Author: Jin-Hwan, Kim.
 */

#include "box_filtering.h"
#include <ctime>
#include <sstream>
#include <iomanip>      // std::setfill, std::setw

using namespace tdmc;

int main_(int argc, char** argv)
{
    // std::string name = std::string(argv[1]);
    // cv::Mat image = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat image = cv::Mat::ones(400, 600, CV_8UC1);
    if ( !image.data ) {

        std::cout << argv[1] << std::endl;
        return -1;
    }

    BoxFilter bf(image.rows, image.cols, 1, 1);
    
    time_t start_t = clock();
    
    bf.box_filter(image.data);
    
    std::cout << (float)(clock()-start_t)/CLOCKS_PER_SEC << "secs" << std::endl;

    return 0;
}


int main()
{
    cv::Mat image = cv::Mat::ones(600, 800, CV_8UC1);

    // cv::imshow("src", image);
    uchar * image_ptr = image.data;

    double duration = static_cast<double>(cv::getTickCount());
    
    for(int k = 0; k < 2000; k++ ) {
    
        int i = rand() % image.cols;
        int j = rand() % image.rows;

        image_ptr[j*image.cols + i] = 255;

    }

    cv::imshow("dst", image);

    for (int i = 0; i < 10; i++) {

        std::stringstream ss;
        ss << "bikeOut" << std::setfill('0') << std::setw(3) << i << ".jpg"; 
        std::cout << ss.str() << std::endl;
    }


    cv::waitKey(0);
}

