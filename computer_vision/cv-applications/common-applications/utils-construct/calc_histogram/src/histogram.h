#if !defined HISTOGRAM
#define HISTOGRAM

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Histogram1D {

  private:

    int histSize[1];
    float hranges[2];
    const float* ranges[1];
    int channels[1];

  public:

    Histogram1D() {

        // Prepare arguments for 1D histogram
        histSize[0]= 256;
        hranges[0]= 0.0;
        hranges[1]= 255.0;
        ranges[0]= hranges; 
        channels[0]= 0; // by default, we look at channel 0
    }

    // Sets the channel on which histogram will be calculated.
    // By default it is channel 0.
    void setChannel(int c) {

        channels[0]= c;
    }

    // Gets the channel used.
    int getChannel() {

        return channels[0];
    }

    // Sets the range for the pixel values.
    // By default it is [0,255]
    void setRange(float minValue, float maxValue) {

        hranges[0]= minValue;
        hranges[1]= maxValue;
    }

    // Gets the min pixel value.
    float getMinValue() {

        return hranges[0];
    }

    // Gets the max pixel value.
    float getMaxValue() {

        return hranges[1];
    }

    // Sets the number of bins in histogram.
    // By default it is 256.
    void setNBins(int nbins) {

        histSize[0]= nbins;
    }

    // Gets the number of bins in histogram.
    int getNBins() {

        return histSize[0];
    }

    // Computes the 1D histogram.
    cv::MatND getHistogram(const cv::Mat &image) {

        cv::MatND hist;

        // Compute histogram
        cv::calcHist(&image, 
            1,			// histogram of 1 image only
            channels,	// the channel used
            cv::Mat(),	// no mask is used
            hist,		// the resulting histogram
            1,			// it is a 1D histogram
            histSize,	// number of bins
            ranges		// pixel value range
        );

        return hist;
    }

    // Computes the 1D histogram and returns an image of it.
    cv::Mat getHistogramImage(const cv::Mat &image){

        // Compute histogram first
        cv::MatND hist= getHistogram(image);

        // Get min and max bin values
        double maxVal=0;
        double minVal=0;
        cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);

        // Image on which to display histogram
        cv::Mat histImg(histSize[0], histSize[0], CV_8U,cv::Scalar(255));

        // set highest point at 90% of nbins
        int hpt = static_cast<int>(0.9*histSize[0]);

        // Draw vertical line for each bin 
        for( int h = 0; h < histSize[0]; h++ ) {

            float binVal = hist.at<float>(h);
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            cv::line(histImg,cv::Point(h,histSize[0]),cv::Point(h,histSize[0]-intensity),cv::Scalar::all(0));
        }

        return histImg;
    }

    // Equalizes the source image.
    cv::Mat equalize(const cv::Mat &image) {

        cv::Mat result;
        cv::equalizeHist(image,result);

        return result;
    }

    // Stretches the source image.
    cv::Mat stretch(const cv::Mat &image, int minValue=0) {

        // Compute histogram first
        cv::MatND hist= getHistogram(image);

        // find left extremity of the histogram
        int imin= 0;
        for( ; imin < histSize[0]; imin++ ) {
            std::cout<<hist.at<float>(imin)<<std::endl;
            if (hist.at<float>(imin) > minValue)
                break;
        }
        
        // find right extremity of the histogram
        int imax= histSize[0]-1;
        for( ; imax >= 0; imax-- ) {

            if (hist.at<float>(imax) > minValue)
                break;
        }

        // Create lookup table
        int dims[1]={256};
        cv::MatND lookup(1,dims,CV_8U);

        for (int i=0; i<256; i++) {
        
            if (i < imin) lookup.at<uchar>(i)= 0;
            else if (i > imax) lookup.at<uchar>(i)= 255;
            else lookup.at<uchar>(i)= static_cast<uchar>(255.0*(i-imin)/(imax-imin)+0.5);
        }

        // Apply lookup table
        cv::Mat result;
        result= applyLookUp(image,lookup);

        return result;
    }

    // Applies a lookup table transforming an input image into a 1-channel image
    cv::Mat applyLookUp(const cv::Mat& image, const cv::MatND& lookup) {

        // Set output image (always 1-channel)
        cv::Mat result(image.rows,image.cols,CV_8U);
        cv::Mat_<uchar>::iterator itr= result.begin<uchar>();

        // Iterates over the input image
        cv::Mat_<uchar>::const_iterator it= image.begin<uchar>();
        cv::Mat_<uchar>::const_iterator itend= image.end<uchar>();

        // Applies lookup to each pixel
        for ( ; it!= itend; ++it, ++itr) {

            *itr= lookup.at<uchar>(*it);
        }

        return result;
    }

    int GetThreshold(cv::MatND& hist)
    {
        float* HistGram = (float*)hist.data;
        int X, Iter = 0;
        int MeanValueOne, MeanValueTwo, SumOne, SumTwo, SumIntegralOne, SumIntegralTwo;
        int MinValue, MaxValue;
        int Threshold, NewThreshold;

        for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;

        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

        Threshold = MinValue;
        NewThreshold = (MaxValue + MinValue) >> 1;
        while (Threshold != NewThreshold)    // 当前后两次迭代的获得阈值相同时，结束迭代    
        {
            SumOne = 0; SumIntegralOne = 0;
            SumTwo = 0; SumIntegralTwo = 0;
            Threshold = NewThreshold;
            for (X = MinValue; X <= Threshold; X++)         //根据阈值将图像分割成目标和背景两部分，求出两部分的平均灰度值      
            {
                SumIntegralOne += HistGram[X] * X;
                SumOne += HistGram[X];
            }
            MeanValueOne = SumIntegralOne / SumOne;
            for (X = Threshold + 1; X <= MaxValue; X++)
            {
                SumIntegralTwo += HistGram[X] * X;
                SumTwo += HistGram[X];
            }
            MeanValueTwo = SumIntegralTwo / SumTwo;
            NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //求出新的阈值
            Iter++;
            if (Iter >= 1000) return -1;
        }

        return Threshold;
    }
};


#endif
