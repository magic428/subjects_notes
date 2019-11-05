// #include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <omp.h>
#include <emmintrin.h>
#include <cstring>
#include <cstdlib>

#define CLIP(x) ((x)<(0)?0:((x)>(255)?(255):(x)))
#define CLIP_Z(x) ((x)<(0)?0:((x)>(1.0f)?(1.0f):(x)))
#define CLIP_TRS(x) ((x)<(0.1f)?0.1f:((x)>(1.0f)?(1.0f):(x)))

using namespace std;
using namespace cv;

// typedef uchar unsigned char;

namespace tdmc{

class BoxFilter{

public:

    BoxFilter();
    BoxFilter(int height, int width, int w_height, int w_width);
    ~BoxFilter();

    /**
     *   \brief: cummulative function for calculating the integral image 
     *           (It may apply other arraies.)
     * 
     *   \param:
     *       pfInArray - input array
     *       nR - radius of filter window
     *       nWid - width of array
     *       nHei - height of array
     * 
     *   \return:
     *       fOutArray - output array (integrated array)
     */
    void box_filter(uchar* img);

protected:
    
    uchar *image_ptr_;
    int height_;        // image heigth
    int width_;         // image width
    int win_height_;     // window heigth
    int win_width_;     // window width
    int r_;     // window radius

    int *sum_;           // sum of pixels
    int *sum_square_;    // square sum of pixels
    int *cum_;           // cumulative of pixels
    int *cum_square_;    // square cumulative of pixels


}; // end of class BoxFilter

} // end of namespace tdmc





class dehazing
{
public:
    dehazing();

    /** 
     *   \param: 
     *       nW - width of input image
     *       nH - height of input image
     *       bPrevFlag - boolean for temporal cohenrence of video dehazing
     *       bPosFlag - boolean for postprocessing.
     */
    dehazing(int nW, int nH, bool bPrevFlag, bool bPosFlag);
    
    /** 
     *   \param: 
     *       nW - width of input image
     *       nH - height of input image
     *       nTBLockSize - block size for transmission estimation 
     *       bPrevFlag - boolean for temporal cohenrence of video dehazing
     *       bPosFlag - boolean for postprocessing
     *       fL1 - information loss cost parameter (regulating)
     *       fL2 - temporal coherence paramter
     *       nGBlock - guided filter block size
    */
    dehazing(int nW, int nH, int nTBlockSize, bool bPrevFlag, bool bPosFlag, float fL1, float fL2, int nGBlock);
    ~dehazing(void);
    
    /**
     *   \brief: haze removal process
     * 
     *   \param:
     *       imInput - input image
     *       nFrame - frame number
     * 
     *   \return:
     *       imOutput - output image
     */
    void HazeRemoval(IplImage* imInput, IplImage* imOutput, int nFrame);

    /**
     *   \brief: haze removal process for a single image
     * 
     *   \param:
     *       imInput - input image
     *   \return:
     *       imOutput - output image
     */
    void ImageHazeRemoval(IplImage* imInput, IplImage* imOutput); 

    /**
     *   \brief: chnage labmda values
     *   \param:
     *       fLambdaLoss - new weight coefficient for loss cost
     *       fLambdaTemp - new weight coefficient for temporal cost
     */
    void LambdaSetting(float fLambdaLoss, float fLambdaTemp);

    void DecisionUse(bool bChoice);
    void TransBlockSize(int nBlockSize);

    /**
     *   \brief: change the block size of guided filter
     *   \param: 
     *       nBlockSize - new block size
     */
    void FilterBlockSize(int nBlockSize);

    /**
     *   \brief: Specify searching range (block shape) by user
     *   \param:
     *       pointTopLeft - the top left point of searching block
     *       pointBottomRight - the bottom right point of searching block.
     *   \return:
     *       m_nTopLeftX - integer x point
     *       m_nTopLeftY - integer y point
     *       m_nBottomRightX - integer x point
     *       m_nBottomRightY - integer y point.
     */
    void AirlightSerachRange(Point pointTopLeft, Point pointBottomRight);

    /**
     *   \brief: change the step size of guided filter
     *   \param:
     *       nStepSize - new step size
     */
    void SetFilterStepSize(int nStepsize);

    /**
     *   \brief: change boolean value
     * 
     *   \param
     *       bPrevFlag - flag
    */
    void PreviousFlag(bool bPrevFlag);

    /**
     *   \brief: change the gaussian sigma value 
     * 
     *   \param:
     *       nSigma
     */
    void FilterSigma(float nSigma);

    /**
     *   \brief: Decision function for re-estimation of atmospheric light
     *       in this file, we just implement the decision function and don't 
     *       apply the decision algorithm in the dehazing function.
     *   \param:
     *       imSrc1 - first frame 
     *       imSrc2 - second frame
     *       nThreshold - threshold value
     *   \return:
     *       boolean value 
     */
    bool Decision(IplImage* imInput, IplImage* imOutput, int nThreshold);

    /**
     *   \return: air light value 
     */
    int* GetAirlight();

    /**
     *   \return: get y image array
     */
    int* GetYImg();

    /**
     *   \return: get refined transmission array
     */
    float* GetTransmission();
    
private:
    void DownsampleImage();
    void DownsampleImageColor();
    void UpsampleTransmission();
    void MakeExpLUT();
    void GuideLUTMaker();
    void GammaLUTMaker(float fParameter);
    void IplImageToInt(IplImage* imInput);
    void IplImageToIntColor(IplImage* imInput);
    void IplImageToIntYUV(IplImage* imInput);
    
    // dehazing.cpp
    /**
     *   \brief: estimate the atmospheric light value in a hazy image.
     *           
     *      It divides the hazy image into 4 sub-block and selects the optimal block, 
     *      which has minimum std-dev and maximum average value.
     * 
     *      *Repeat* the dividing process until the size of sub-block is smaller than 
     *      pre-specified threshold value(area = 200). Then, We select the most similar value to
     *      the pure white.
     * 
     *      THIS IS A RECURSIVE FUNCTION.
     * 
     *   \param: 
     *       imInput - input image (cv iplimage)
     * 
     *   \return:
     *       m_anAirlight: estimated atmospheric light value
     */
    void AirlightEstimation(IplImage* imInput);
    /*
    *   \brief: Dehazed the image using estimated transmission and atmospheric light.
    * 
    *   \param: 
    *       imInput - Input hazy image.
    * 
    *   \return:
    *       imOutput - Dehazed image.
    */
    void RestoreImage(IplImage* imInput, IplImage* imOutput);
    /**
     *   \brief: deblocking for blocking artifacts of mpeg video sequence.
     * 
     *   \param: 
     *       imInput - Input hazy frame.
     * 
     *   \return:
     *       imOutput - Dehazed frame.
     */
    void PostProcessing(IplImage* imInput, IplImage* imOutput);

    // Transmission.cpp
    /**
     *   \brief: Estiamte the transmission in the frame.
     * 
     *   \param:
     *       pnImageY - Y channel of Current frame 
     *       pfTransmission - Transmission of Current Frame
     *       pnImageYP - Y channel of Previous frame
     *       pfTransmissionP - Transmission of Previous frame
     *       nFrame - frame no.
     *       nWid - frame width(320, downSampled)
     *       nHei - frame height(240, downSampled).
     *       
     *   \return:
     *       m_pfTransmission
     */
    void TransmissionEstimation(int *pnImageY, float *pfTransmission, int *pnImageYP, float *pfTransmissionP, int nFrame, int nWid, int nHei);
    /**
     *   \brief: Estiamte the transmission in the block. 
     *       The algorithm use exhaustive searching method(穷举搜索) and its step size
     *       is sampled to 0.1
     * 
     *       This is the Static Image Dehazing algorithm.  
     * 
     *   \param:
     *       nStartx - top left point of a block
     *       nStarty - top left point of a block
     *       nWid - frame width
     *       nHei - frame height.
     *   
     *   \return:
     *       fOptTrs
     */
    float NFTrsEstimation(int *pnImageY, int nStartX, int nStartY, int nWid, int nHei);
    /**
     *  \brief: Estiamte the transmission in the block. 
     *      The algorithm use exhaustive searching method and its step size
     *      is sampled to 0.1.
     *      
     *      The previous frame information is used to estimate current frame transmission.
     *
     *  \param:
     *      pnImageY - Y channel of Current frame 
     *      pnImageYP - Y channel of Previous frame
     *      pfTransmissionP - Transmission of Previous frame
     *      nStartx - top left point of a block
     *      nStarty - top left point of a block
     *      nWid - frame width
     *      nHei - frame height.
     *
     *  \return:
     *       fOptTrs
     */
    float NFTrsEstimationP(int *pnImageY, int *pnImageYP, float *pfTransmissionP, int nStartX, int nStartY, int nWid, int nHei);
    /**
     *   \brief: Estiamte the transmission in the frame(Color)
     *                Specified size.
     *   \param:
     *       nFrame - frame no.
     *       nWid - frame width
     *       nHei - frame height.
     *   \return:
     *       m_pfTransmission
     */
    void TransmissionEstimationColor(int *pnImageR, int *pnImageG, int *pnImageB, float *pfTransmission,int *pnImageRP, int *pnImageGP, int *pnImageBP, float *pfTransmissionP,int nFrame, int nWid, int nHei);
    /**
     *   Function: NFTrsEstimationP(COLOR)
     *   \brief: Estiamte the transmission in the block. 
     *       The algorithm use exhaustive searching method and its step size
     *       is sampled to 0.1.
     *       The previous frame information is used to estimate transmission.
     * 
     *   \param:
     *       nStartx - top left point of a block
     *       nStarty - top left point of a block
     *       nWid - frame width
     *       nHei - frame height.
     * 
     *   \return:
     *       fOptTrs
     */
    float NFTrsEstimationPColor(int *pnImageR, int *pnImageG, int *pnImageB, int *pnImageRP, int *pnImageGP, int *pnImageBP, float *pfTransmissionP, int nStartX, int nStartY, int nWid, int nHei); 
    /**
     *   \brief: Estiamte the transmission in the block. (COLOR)
     *       The algorithm use exhaustive searching method and its step size
     *       is sampled to 0.1
     * 
     *   \param:
     *       nStartx - top left point of a block
     *       nStarty - top left point of a block
     *       nWid - frame width
     *       nHei - frame height.
     * 
     *   \return:
     *       fOptTrs
     */
    float NFTrsEstimationColor(int *pnImageR, int *pnImageG, int *pnImageB, int nStartX, int nStartY, int nWid, int nHei);
    
    
    // guided filter.cpp
    void CalcAcoeff(float* pfSigma, float* pfCov, float* pfA1, float* pfA2, float* pfA3, int nIdx);
    void BoxFilter(float* pfInArray, int nR, int nWid, int nHei, float*& fOutArray);
    void BoxFilter(float* pfInArray1, float* pfInArray2, float* pfInArray3, int nR, int m_nWid, int m_nHei, float*& pfOutArray1, float*& pfOutArray2, float*& pfOutArray3);

    void GuidedFilterY(int nW, int nH, float fEps);
    void GuidedFilter(int nW, int nH, float fEps);
    void GuidedFilterShiftableWindow(float fEps);
    
    void FastGuidedFilterS();
    void FastGuidedFilter();

    // 320*240 size
    float* m_pfSmallY;      // Y image
    float* m_pfSmallPk_p;    // (Y image) - (mean of Y image)
    float* m_pfSmallNormPk; //Normalize된 Y image
    float* m_pfSmallInteg;    //Gaussian weight가 적용된 transmission 결과
    float* m_pfSmallDenom;    //Gaussian weight가 저장된 행렬

    int* m_pnSmallYImg;    // downSampled image, Y channel

    int* m_pnSmallRImg;    // downSampled image, R channel
    int* m_pnSmallGImg;    // downSampled image, G channel
    int* m_pnSmallBImg;    // downSampled image, B channel

    float* m_pfSmallTransP; // 이전 프레임의 transmission 영상
    float* m_pfSmallTrans;    //초기 transmission 영상
    float* m_pfSmallTransR; //정련된 transmission 영상
    int* m_pnSmallYImgP;    //이전 프레임의 Y채널

    int* m_pnSmallRImgP;    //이전 프레임의 Y채널
    int* m_pnSmallGImgP;    //이전 프레임의 Y채널
    int* m_pnSmallBImgP;    //이전 프레임의 Y채널

    //Original size
    float* m_pfY;        //Y image
    float* m_pfPk_p;     //(Y image) - (mean of Y image)
    float* m_pfNormPk;     //Normalize된 Y image
    float* m_pfInteg;     //Gaussian weight가 적용된 transmission 결과
    float* m_pfDenom;     //Gaussian weight가 저장된 행렬
    
    int* m_pnYImg;     //입력 영상의 Y채널
    int* m_pnYImgP;     //입력 영상의 Y채널

    int* m_pnRImg;     //입력 영상의 Y채널
    int* m_pnGImg;     //입력 영상의 Y채널
    int* m_pnBImg;     //입력 영상의 Y채널

    int* m_pnRImgP;     //입력 영상의 Y채널
    int* m_pnGImgP;     //입력 영상의 Y채널
    int* m_pnBImgP;     //입력 영상의 Y채널

    float* m_pfTransmission; //초기 transmission
    float* m_pfTransmissionP; //초기 transmission
    float* m_pfTransmissionR; //정련된 transmission 영상
    
    //////////////////////////////////////////////////////////////////////////
    int    m_nStepSize;    //Guided filter의 step size;
    float* m_pfGuidedLUT;    //Guided filter 내의 gaussian weight를 위한 LUT
    float m_fGSigma;     //Guided filter 내의 gaussian weight에 대한 sigma

    int    m_anAirlight[3]; // A, atmospheric light value
    uchar m_pucGammaLUT[256]; // Gamma 校正 LUT
    float m_pfExpLUT[256]; // Transmission 계산시, 픽셀 차이에 대한 weight용 LUT

    int    m_nAirlight;    //안개값(grey)
    
    bool m_bPreviousFlag; //이전 프레임 이용 여부
    float m_fLambda1;     //Loss cost
    float m_fLambda2;     //Temporal cost

    int    m_nWid;        //너비
    int    m_nHei;        //높이

    int    m_nTBlockSize;    // Block size for transmission estimation
    int    m_nGBlockSize;    // Block size for guided filter

    //Airlight search range
    int    m_nTopLeftX;        
    int    m_nTopLeftY;
    int    m_nBottomRightX;
    int    m_nBottomRightY;

    bool m_bPostFlag;    // Flag for post processing(deblocking)
    // function.cpp
    

};