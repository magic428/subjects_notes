/**
 *   The original box filter from the paper which named 
 *   dehazing video 
 * 
 *   
 * 
 *   Last updated: 2018-07-06
 *   Author: GuoZhijie, Klm.
 */
#include "box_filtering.h"

namespace tdmc{

BoxFilter::BoxFilter()
{
    sum_ = 0;
    sum_square_ = 0;
    cum_ = 0;
    cum_square_ = 0;
}


BoxFilter::BoxFilter( int height, int width, 
                      int w_height, int w_width ) 
{
    height_ = height;
    width_ = width;

    win_width_ = w_width;
    win_height_ = w_height;
    r_ = w_height; 

    sum_ = new int[height_ * width_];
    sum_square_ = new int[height_ * width_];
    cum_ = new int[height_ * width_];
    cum_square_ = new int[height_ * width_];
}


BoxFilter::~BoxFilter()
{
    if(sum_)  delete [] sum_;
    if(sum_square_)  delete [] sum_square_;
    if(cum_)  delete [] cum_;
    if(cum_square_)  delete [] cum_square_;
}


void BoxFilter::box_filter(uchar* img)
{
    memset( sum_, 0, height_ * width_ * sizeof(int) );
    memset( sum_square_, 0, height_ * width_ * sizeof(int) );
    memset( cum_, 0, height_ * width_ * sizeof(int) );
    memset( cum_square_, 0, height_ * width_ * sizeof(int) );

    // Cumlative along Y axis,  we have to deal with head and end seperatelly
    // or it will out of index
    for ( int i = 0; i < width_; i++ ) 
        cum_[i] = img[i];

    for ( int i = width_; i < width_ * height_; i++ ) {

        // Cumlative elements in previous lines
        cum_[i] = cum_[i - width_] + img[i];
    }

    // Difference along Y 
    for ( int i = 0; i < (r_+1) * width_; i++ )
        sum_[i] = cum_[i + r_*width_];
        // sum_[i] = cum_[i];
    
    for ( int i = (r_+1) * width_; i < (height_ - r_)*width_; i++ )
        sum_[i] = cum_[i + r_*width_] - cum_[i - (r_+1)*width_];

    for ( int j = height_ - r_; j < height_; j++)
        for ( int i = 0; i < width_; i++)
            sum_[j*width_ + i] = cum_[(height_-1)*width_ + i] - cum_[(j-r_-1)*width_ + i];

    // Cumlative along X axis
    for (int i = 0; i < height_*width_; i+=width_)
        cum_[i] = sum_[i];
    for (int j = 0; j < height_; j++)
        for (int i = 1; i < width_; i++)
            cum_[j*width_+i] = cum_[j*width_ + i -1] + sum_[j*width_ + i];

    // difference over X axis, we have to deal with head and end seperatelly
    // or it will out of index
    for ( int j = 0; j < height_; j++)
        for (int i = 0; i < r_+1; i++)
            sum_[j*width_ + i] = cum_[j*width_ + i + r_];

    for ( int j = 0; j < height_; j++)
        for (int i = r_+1; i < width_ - r_; i++)
            sum_[j*width_ + i] = cum_[j*width_ + i + r_] - cum_[j*width_ + i - r_ - 1];

    for ( int j = 0; j < height_; j++)
        for (int i = width_ - r_; i < width_; i++)
            sum_[j*width_ + i] = cum_[j*width_ - 1] - cum_[j*width_ - r_ - 1];
        

    for ( int i = 0; i < width_ * height_; i++ ){

        if ( (i + 1) % 10 == 0)
            std::cout << sum_[i] << ", ";
        
        if ( (i + 1) % width_ == 0)
              std::cout << std::endl;
    }

    std::cout << std::endl;
}

}