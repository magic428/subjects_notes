#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <map>

using namespace cv;
using namespace std;

template<typename T>T sqr(T x){return x*x;}

template<class T, int D> inline T vecSqrDist(const Vec<T, D> &v1, const Vec<T, D> &v2) {T s = 0; for (int i=0; i<D; i++) s += sqr(v1[i] - v2[i]); return s;} // out of range risk for T = byte, ...

const int n_default_colors[3] = {12, 12, 12};

int Quantize(cv::Mat& img3f, cv::Mat &color_q,
    double ratio = 0.95, const int n_colors[3] = n_default_colors);

/**
 * \brief: Quantize Color numbers in RGB Color Space
 *         Default: quantize the color number to 12 with each channel.  
 * 
 * 
 * \param: img3f - src image, normalized to [0, 1]
 *         n_colors - color numbers (3 channels) after quantize
 *         ratio - significant colors' pixels percentage, default = 0.95
 * 
 * \return: color_q - Finally quantized image
 * 
*/
int Quantize(cv::Mat& img3f, cv::Mat &color_q, double ratio, const int n_colors[3])
{
    CV_Assert(img3f.data != NULL);

    int rows = img3f.rows, cols = img3f.cols;
    int color_bins_quant[3] = {n_colors[0], n_colors[1], n_colors[2]};
    // color bins weights of channels
    int weights[3] = {n_colors[0] * n_colors[0], n_colors[0], 1};  // 12^2, 12, 1

    cv::Mat idx1i = cv::Mat::zeros(img3f.size(), CV_32S);
        
    if (img3f.isContinuous() && idx1i.isContinuous()){
        cols *= rows;
        rows = 1;
    }

    // First Step: reduce the color bins to fixed bins, 12 
    // Build color pallet, get every quantized_color's number
    std::map<int, int> pallet;   // (quantized_color, num) pairs in pallet
    int color_val = 0;
    for (int y = 0; y < rows; y++)
    {
        const float* imgData = img3f.ptr<float>(y);
        int* idx = idx1i.ptr<int>(y);
        for (int x = 0; x < cols; x++, imgData += 3)
        {
            // color after quantized, eg: 256^3 -> 256 * 256 * 256
            color_val = (int)(imgData[0]*color_bins_quant[0])*weights[0] + 
                     (int)(imgData[1]*color_bins_quant[1])*weights[1] + 
                     (int)(imgData[2]*color_bins_quant[2]);
            pallet[color_val] ++;

            idx[x] = color_val;  // fill Mat: idx1i
        }
    }
    std::cout << "color reduced first step: " << pallet.size() << std::endl;
 
    // Second Step: Reduce more by removing colors rarely appear
    // Find significant colors, this will cover 95% pixels in image 
    int maxNum = 0;  // colors number left finally, 3 channles total: [10 - 256]
    {
        std::vector<std::pair<int, int>> num; // (num, color) pairs in num
        num.reserve(pallet.size());
        for (std::map<int, int>::iterator it = pallet.begin(); it != pallet.end(); it++)
            num.push_back(std::pair<int, int>(it->second, it->first)); // (color, num) pairs in pallet
        std::sort(num.begin(), num.end(), std::greater<pair<int, int>>()); // sort with key

        // remove the colors appear rarely
        maxNum = (int)num.size();
        int maxDropNum = cvRound(rows * cols * (1-ratio));
        for (int crnt = num[maxNum-1].first; crnt < maxDropNum && maxNum > 1; maxNum--)
            crnt += num[maxNum - 2].first;

        std::cout << "color reduced second step: " << maxNum << std::endl;
        
        maxNum = std::min(maxNum, 256); // To avoid very rarely case
        if (maxNum <= 10)
            maxNum = std::min(10, (int)num.size());

        pallet.clear();
        // Beacuse "maxNum" is the total colors number left finally, and "num" has been sorted
        // "pallet" contains (color, color_index) pairs
        for (int i = 0; i < maxNum; i++)
            pallet[num[i].second] = num[i].second; 

        // RGB color after quantized
        std::vector<cv::Vec3i> color3i(num.size());
        for (unsigned int i = 0; i < num.size(); i++)
        {
            color3i[i][0] = num[i].second / weights[0];
            color3i[i][1] = num[i].second % weights[0] / weights[1];
            color3i[i][2] = num[i].second % weights[1];
        }

        // reassign the 5% pixels colors to its nearest color which left in histogram bins
        for (unsigned int i = maxNum; i < num.size(); i++)
        {
            int similar_idx = 0, similar_val = INT_MAX;
            // remained bins
            for (int j = 0; j < maxNum; j++)
            {
                int dist_ij = vecSqrDist<int, 3>(color3i[i], color3i[j]);
                if (dist_ij < similar_val)
                    similar_val = dist_ij, similar_idx = j;
            }
            pallet[num[i].second] = pallet[num[similar_idx].second];
        }
    }

    // Finally quantized image
    color_q = cv::Mat::zeros(img3f.size(), CV_8UC3);  
    for (int y = 0; y < rows; y++)
    {
        const float* imgData = img3f.ptr<float>(y);
        int* idx = idx1i.ptr<int>(y);
        cv::Vec3b* q = color_q.ptr<cv::Vec3b>(y);
        for (int x = 0; x < cols; x++, imgData += 3)
        {
            // quantized image, * 21.3 to make image color bright
            q[x][0] = (uchar)(pallet[idx[x]] / weights[0]*21.3); 
            q[x][1] = (uchar)(pallet[idx[x]] % weights[0] / weights[1]*21.3);
            q[x][2] = (uchar)(pallet[idx[x]] % weights[1]*21.3);
        }
    }

    return 0;
}



int main(int argc, char **argv)
{
    cv::Mat image = cv::imread(argv[1]);
    cv::Mat color_q;
    cv::Mat img3f;

    image.convertTo(img3f, CV_32FC3, 1.0/255);
    
    Quantize(img3f, color_q);
     
    cv::imshow("quant", color_q);
    cv::waitKey();

    return 0;
}