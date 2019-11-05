OpenCV图像去噪

图像平滑算法
程序分析及结果
图像平滑算法

图像平滑与图像模糊是同一概念，主要用于图像的去噪。平滑要使用滤波器，为不改变图像的相位信息，

一般使用线性滤波器，其统一形式如下：

其中h称为滤波器的核函数，说白了就是权值。不同的核函数代表不同的滤波器，有不同的用途。

在图像处理中，常见的滤波器包括：

归一化滤波器（Homogeneous blur）

也是均值滤波器，用输出像素点核窗口内的像素均值代替输出点像素值。

高斯滤波器（Guassian blur）

是实际中最常用的滤波器，高斯滤波是将输入数组的每一个像素点与 高斯内核 卷积将卷积和当作输出像

素值。高斯核相当于对输出像素的邻域赋予不同的权值，输出像素点所在位置的权值最大（对应高斯函数

的均值位置）。二维高斯函数为，

中值滤波器（median blur）

中值滤波将图像的每个像素用邻域(以当前像素为中心的正方形区域)像素的中值代替。对椒盐噪声最有效

的滤波器，去除跳变点非常有效。

双边滤波器（Bilatrial blur）

为避免滤波器平滑图像去噪的同时使边缘也模糊，这种情况下使用双边滤波器。关于双边滤波器的解释参

见http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html

下面的程序将先给标准Lena图像添加椒盐噪声，分别使用4种不同的滤波器进行平滑操作，请注意观察不

同滤波器对椒盐噪声的去噪效果！

程序分析及结果

/*
 * FileName : image_smoothing.cpp
 * Author   : xiahouzuoxin @163.com
 * Version  : v1.0
 * Date     : Wed 17 Sep 2014 08:30:25 PM CST
 * Brief    : 
 * 
 * Copyright (C) MICL,USTB
 */
#include "cv.h"
#include "imgproc/imgproc.hpp"
#include "highgui/highgui.hpp"


using namespace std;
using namespace cv;

const int MAX_KERNEL_LENGTH = 10;

const char *wn_name = "Smoothing";


static void salt(Mat &I, int n);
static void disp_caption(const char *wn_name, Mat src, const char *caption);
static void disp_image(const char *wn_name, Mat I);

/*
 * @brief   
 * @inputs  
 * @outputs 
 * @retval  
 */
int main(int argc, char *argv[])
{
    if (argc<2) {
        cout<<"Usage: ./image_smoothing [file name]"<<endl;
        return -1;
    }

    Mat I = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    salt(I, 6000);
    imshow(wn_name, I);
    waitKey(0);

    Mat dst;  // Result

    /* Homogeneous blur */
    disp_caption(wn_name, I, "Homogeneous blur");
    for (int i=1; i<MAX_KERNEL_LENGTH; i+=2) {
        blur(I, dst, Size(i, i), Point(-1,-1));
        disp_image(wn_name, dst);
    }

    /* Guassian blur */
    disp_caption(wn_name, I, "Gaussian blur");
    for (int i=1; i<MAX_KERNEL_LENGTH; i+=2) {
        GaussianBlur(I, dst, Size(i, i), 0, 0);
        disp_image(wn_name, dst);
    }

    /* Median blur */
    disp_caption(wn_name, I, "Median blur");
    for (int i=1; i<MAX_KERNEL_LENGTH; i+=2) {
        medianBlur(I, dst, i);
        disp_image(wn_name, dst);
    }

    /* Bilatrial blur */
    disp_caption(wn_name, I, "Bilatrial blur");
    for (int i=1; i<MAX_KERNEL_LENGTH; i+=2) {
        bilateralFilter(I, dst, i, i*2, i/2);
        disp_image(wn_name, dst);
    }
    waitKey(0);

    return 0;
}

/*
 * @brief   显示提示文字（滤波方法）
 * @inputs  
 * @outputs 
 * @retval  
 */
static void disp_caption(const char *wn_name, Mat src, const char *caption)
{
    Mat dst = Mat::zeros(src.size(), src.type());

    putText(dst, caption, Point(src.cols/4, src.rows/2), CV_FONT_HERSHEY_COMPLEX, 1, Scalar

(255,255,255));

    imshow(wn_name, dst);
    waitKey(0);
}

/*
 * @brief   显示图像
 * @inputs  
 * @outputs 
 * @retval  
 */
static void disp_image(const char *wn_name, Mat I)
{
    imshow(wn_name, I);
    waitKey(1000);
}

/*
 * @brief   添加椒盐噪声
 * @inputs  
 * @outputs 
 * @retval  
 */
static void salt(Mat &I, int n=3000)
{
    for (int k=0; k<n; k++) {
        int i = rand() % I.cols;
        int j = rand() % I.rows;


        if (I.channels()) {
            I.at<uchar>(j,i) = 255;
        } else {
            I.at<Vec3b>(j,i)[0] = 255;
            I.at<Vec3b>(j,i)[1] = 255;
            I.at<Vec3b>(j,i)[2] = 255;
        }
    }
}
上面程序的逻辑非常清晰：

读入灰度图，并添加椒盐噪声（6000个噪声点）：

Mat I = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
salt(I, 6000);
LenaNoise

disp_caption和disp_image函数分别是用于显示提示文字和平滑过程中的变化图像的，平滑过程中图像的

变化如下图：

blur

注意观察上面的图，中值滤波（Median Blur）对椒盐噪声的效果最好！

四种滤波方法分别使用到4个OpenCV函数，这些函数的声明都在imgproc.hpp中，这些函数的前2个参数都

是原图像和滤波后图像。

归一化滤波器blur的第3个参数为滤波核窗口的大小，Size(i,i)表示ixi大小的窗口。

高斯滤波器GaussianBlur第3个参数也是滤波核窗口的大小，第4、第5个参数分辨表示x方向和y方向的δ
。

中值滤波器medianBlur第3个参数是滤波器的长度，该滤波器的窗口为正方形。

双边滤波器的函数原型如下：

//! smooths the image using bilateral filter
CV_EXPORTS_W void bilateralFilter( InputArray src, OutputArray dst, int d,
                             double sigmaColor, double sigmaSpace,
                             int borderType=BORDER_DEFAULT );
本程序使用的Makefile文件为：

 TARG=image_smoothing
 SRC=image_smoothing.cpp
 LIB=-L/usr/local/lib/
 INC=-I/usr/local/include/opencv/ -I/usr/local/include/opencv2
 CFLAGS=

 $(TARG):$(SRC)
     g++ -g -o $@ ${CFLAGS} $(LIB) $(INC) \
         -lopencv_core -lopencv_highgui -lopencv_imgproc \
         $^

 .PHONY:clean

 clean:
     -rm $(TARG) tags -f

========
图像代数运算：平均值去噪，减去背景



代数运算，就是对两幅图像的点之间进行加、减、乘、除的运算。四种运算相应的公式为：

代数运算中比较常用的是图像相加和相减。图像相加常用来求平均值去除addtive噪声或者实现二次曝光

（double-exposure）。图像相减用于减去背景或周期噪声，污染等。

图像相加

OpenCV中提供了相加的函数

void cvAcc(   
           const CvArr* image,//输入图像  
           CvArr* sum,  //累积图像   
           const CvArr* mask=NULL//可选的运算  
 );  
我们还需要用到一个线性变换转换函数来对相加的结果求平均

void cvConvertScale(   
        const CvArr* src, //输入数组  
        CvArr* dst,//输出数组  
        double scale=1,//比例  
        double shift=0 //缩放比例，可选  
);  
#define cvCvtScale cvConvertScale  
#define cvScale  cvConvertScale  
#define cvConvert( src, dst )  cvConvertScale( (src), (dst), 1, 0 )  

实践：平均值去噪
我们用NASA的一段幸运团的视频做实验，截取视频的某几个连续帧求平均值：

int main()  
{  
    CvCapture* capture=cvCaptureFromFile("media.avi");  
    IplImage* frame=  NULL;  
    IplImage * imgsum =NULL;  
      
    int start=301;  
    int end=304;  
    cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, start);  
      
    int count = start;  
    while( cvGrabFrame(capture) && count <= end )  
    {  
        frame = cvRetrieveFrame(capture);// 获取当前帧  
        if(imgsum==NULL){  
            imgsum=cvCreateImage(cvGetSize(frame),IPL_DEPTH_32F,3);  
            cvZero(imgsum);  
        }  
        cvAcc(frame,imgsum);  
  
        char testname[100];  
        sprintf(testname,"%s%d%s","image",count,".jpg");  
        cvShowImage(testname,frame);  
        cvSaveImage(testname,frame);  
          
        count++;  
    }  
    IplImage * imgavg = cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);  
    cvConvertScale(imgsum,imgavg,1.0/4.0);  
      
    cvShowImage("imageavg",imgavg);  
    cvSaveImage("imageavg_4.jpg",imgavg);  
      
    cvWaitKey(0);  
    cvReleaseCapture(&capture);  
    return 0;  
}  
以下从左到右分别是连续两帧、四帧、八帧、十六帧求均值的结果：
   


实践：图像二次曝光
曝光和去噪是一样的，也是对几幅图像求平均


//通过求平均二次曝光  
int main()  
{  
    IplImage* image1=  cvLoadImage("psu3.jpg");  
    IplImage* image2=  cvLoadImage("psu4.jpg");  
      
    IplImage * imgsum =cvCreateImage(cvGetSize(image1),IPL_DEPTH_32F,3);  
    cvZero(imgsum);  
    cvAcc(image1,imgsum);  
    cvAcc(image2,imgsum);  
  
    IplImage * imgavg = cvCreateImage(cvGetSize(image1),IPL_DEPTH_8U,3);  
    cvConvertScale(imgsum,imgavg,1.0/2.0);  
  
    cvShowImage("imageavg",imgavg);  
    cvSaveImage("avg.jpg",imgavg);  
  
    cvWaitKey(0);  
    cvReleaseImage(&image1);  
    cvReleaseImage(&image2);  
    cvReleaseImage(&imgsum);  
    cvReleaseImage(&imgavg);  
    return 0;  
}  
下图是对同学街舞截图的“二次曝光”效果：


图像相减

OpenCV中用cvAbsDiff函数计算两数组的差的绝对值

void cvAbsDiff(   
        const CvArr* src1,//第一个输入数组  
        const CvArr* src2,//第二个输入数组  
        CvArr* dst//输出数组  
);  

实践：减去背景
减去背景是通过两幅图像代数相减，可以判断出前景区域和运动区域，这是最简单（很多时候也是效果很
好的）运动检测方法。

//减去背景  
int main()  
{  
    IplImage* pFrame = NULL;   
    IplImage* pFrImg = NULL;  
    IplImage* pBkImg = NULL;  
  
    CvMat* pFrameMat = NULL;  
    CvMat* pFrMat = NULL;  
    CvMat* pBkMat = NULL;  
  
    CvCapture* pCapture = NULL;  
  
    int nFrmNum = 0;  
  
    //创建窗口  
    cvNamedWindow("video", 1);  
    cvNamedWindow("background",1);  
    cvNamedWindow("foreground",1);  
  
    pCapture = cvCaptureFromFile("media.avi");  
    while(pFrame = cvQueryFrame( pCapture ))  
    {  
        nFrmNum++;  
  
        //如果是第一帧，需要申请内存，并初始化  
        if(nFrmNum == 1)  
        {  
            pBkImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);  
            pFrImg = cvCreateImage(cvSize(pFrame->width, pFrame->height),  IPL_DEPTH_8U,1);  
  
            pBkMat = cvCreateMat(pFrame->height, pFrame->width, CV_32FC1);  
            pFrMat = cvCreateMat(pFrame->height, pFrame->width, CV_32FC1);  
            pFrameMat = cvCreateMat(pFrame->height, pFrame->width, CV_32FC1);  
  
            //转化成单通道图像再处理  
            cvCvtColor(pFrame, pBkImg, CV_BGR2GRAY);  
            cvCvtColor(pFrame, pFrImg, CV_BGR2GRAY);  
  
            cvConvert(pFrImg, pFrameMat);  
            cvConvert(pFrImg, pFrMat);  
            cvConvert(pFrImg, pBkMat);  
        }  
        else  
        {  
            cvCvtColor(pFrame, pFrImg, CV_BGR2GRAY);  
            cvConvert(pFrImg, pFrameMat);  
            //当前帧跟背景图相减  
            cvAbsDiff(pFrameMat, pBkMat, pFrMat);  
            //二值化前景图  
            cvThreshold(pFrMat, pFrImg, 60, 255.0, CV_THRESH_BINARY);  
            //更新背景  
            cvRunningAvg(pFrameMat, pBkMat, 0.003, 0);  
            //将背景转化为图像格式，用以显示  
            cvConvert(pBkMat, pBkImg);  
  
            cvShowImage("video", pFrame);  
            cvShowImage("background", pBkImg);  
            cvShowImage("foreground", pFrImg);  
  
            if( cvWaitKey(2) >= 0 )  
                break;  
        }  
    }  
    cvDestroyWindow("video");  
    cvDestroyWindow("background");  
    cvDestroyWindow("foreground");  
    cvReleaseImage(&pFrImg);  
    cvReleaseImage(&pBkImg);  
    cvReleaseMat(&pFrameMat);  
    cvReleaseMat(&pFrMat);  
    cvReleaseMat(&pBkMat);  
    cvReleaseCapture(&pCapture);  
    return 0;  
}  
效果图：


========


opencv连通域去噪  



//Find_Connected_Component参数说明：


       /*mask———一副灰度图


       polygon1_hull0———用多边形拟合选1，用凸包拟合选0


       scale———设置不被删除的连通轮廓大小


       num————连通轮廓的最大数目


       bbs——指向连通轮廓的外接矩形


       center——指向连通轮廓的中心*/     

     


#define CVCONTOUR_APPROX_LEVEL 2//数越大连通区域的边界越简单


#define CVCLOSE_ITR 1                   //图像形态学运算的次数


void Find_Connected_Component(IplImage *mask,int polygon1_hull0=1,float scale=4


                                                   ,int *num=0,CvRect *bbs=0,CvPoint 


*centers=0)//连通域去噪声


{


       static CvMemStorage* mem_storage=0;


       static CvSeq* contours=0;



       //先开操作，后闭操作


       cvMorphologyEx(mask,mask,0,0,CV_MOP_OPEN,CVCLOSE_ITR);


       cvMorphologyEx(mask,mask,0,0,CV_MOP_CLOSE,CVCLOSE_ITR);



       //找到所有轮廓


       if(!mem_storage)


       {


              mem_storage=cvCreateMemStorage(0);


       }


       else{


              cvClearMemStorage(mem_storage);


       }

 
       CvContourScanner scanner=cvStartFindContours(mask,mem_storage,sizeof


(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);



       //丢弃太小的轮廓，用多边形或凸包拟合剩下的轮廓


       CvSeq* c;


       int numCont=0;


       while(c=cvFindNextContour(scanner))

       {

              double len=cvContourPerimeter(c);//连通域周长


              double q=(mask->height+mask->width)/scale;


              //cout<<len<<"   ";


              if(len<q){cvSubstituteContour(scanner,NULL);}


              else{

                     CvSeq* c_new;

                     if(polygon1_hull0)

                     {

                            //多边形拟合

                            c_new=cvApproxPoly(c,sizeof

(CvContour),mem_storage,CV_POLY_APPROX_DP,CVCONTOUR_APPROX_LEVEL,0);

                     }


                     else{


                            //凸包拟合


                            c_new=cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);


                     }

                     cvSubstituteContour(scanner,c_new);

                     numCont++;

              }


       }

       contours=cvEndFindContours(&scanner);


       const CvScalar CVX_WHITE=CV_RGB(0xff,0xff,0xff);//白色


       const CvScalar CVX_BLACK=CV_RGB(0x00,0x00,0x00);              //黑色


        //重绘连通区域


       cvZero(mask);


       IplImage *maskTemp;


       int numFilled=0;


       if(num!=NULL)


       {


              int N=*num,i=0;


              CvMoments moments;


              double M00,M01,M10;


              maskTemp=cvCloneImage(mask);


              for(i=0,c=contours;c!=NULL;c=c->h_next,i++)


              {


                     if(i<N)


                     {


                            cvDrawContours(maskTemp,c,CVX_WHITE,CVX_WHITE,-1,CV_FILLED,8);


                            //找中心


                            if(centers){


                                   cvMoments(maskTemp,&moments,1);


                                   M00=cvGetSpatialMoment(&moments,0,0);


                                   M10=cvGetSpatialMoment(&moments,1,0);


                                   M01=cvGetSpatialMoment(&moments,0,1);


                                   centers[i].x=(int)(M10/M00);


                                   centers[i].y=(int)(M01/M00);


                            }


                            if(bbs!=NULL){


                                   bbs[i]=cvBoundingRect(c);


                            }


                            cvZero(maskTemp);


                            numFilled++;


                     }


                     //画区域


                     cvDrawContours(mask,c,CVX_WHITE,CVX_WHITE,-1,CV_FILLED,8);


              }


 


       //*num=numFilled;


       cvReleaseImage(&maskTemp);


       }


       else


       {


                     for(c=contours;c!=NULL;c=c->h_next)


                            cvDrawContours(mask,c,CVX_WHITE,CVX_WHITE,-1,CV_FILLED,8);


 


       }


       *num=numCont;


}


========

OpenCV单kinect多帧静止场景的深度图像去噪


老板kinect去噪的任务下达已经有半个多月了，前期除了看了几天文献之外就打酱油了，好像每天都很忙


，可是就是不知道在忙什么。这几天为了交差，就胡乱凑了几段代码，得到一个结果，也知道不行，先应


付一下，再图打算。


程序思想很简单，先对静止的场景连续采样若干帧，然后对所有点在时间域取中值，对取完中值之后的无


效点在空间域取最近邻，勉强将黑窟窿填上了。由于代码较长，现在奉上关键的几个片段：


#include<cv.h>
#include<highgui.h>
#include<iostream>
using namespace std;


#ifndef _DENOISE
#define _DENOISE


const int nFrames = 9;   // number of consecutive frames
const int width = 640;   // frame width
const int height = 480;  // frame height


class kinectDenoising
{
private:                       
IplImage* denoisedImage;
IplImage* frameSet[nFrames];
unsigned int numOfFrames;
CvRect imageROI;
public:
kinectDenoising();
~kinectDenoising();
void addFrame(IplImage* img); 
void setImageROI(bool isUpdate = true);
void medianFiltering();
void nearestFiltering();
void updateFrameSet(IplImage* img);
void showDenoiedImage(const char* window);
void showCurrentImage(const char* window);
};


void insertSort(unsigned short* data,int& len,unsigned short newData);


#endif
这是定义的头文件，装模作样的写了一个类，在构造函数里面，除了对denoisedImage分配内存之外其他


都置0，析构函数需要释放denoisedImage和frameSet数组的内存。numOfFrames本来设计为frameSet中的


图像的帧数，结果由于偷懒就用了一个定长的数组。


void kinectDenoising::setImageROI(bool isUpdate)
{
if(!isUpdate) 
{
imageROI = cvRect(22,44,591,434);
}
else
{
IplImage* image8u = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
IplImage* bitImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);


// cvThreshold can only handle images of 8UC1 or 32FC1
cvConvertScale(frameSet[0],image8u,255.0/4096.0);
cvThreshold(image8u,bitImage,0,1,CV_THRESH_BINARY);


        // the two mats rowReduced and colReduced have to be CV_32SC1 type
   // for function cvReduce() seems not to suitable for 16U type and 
   // 8U type doesn't have enough room for the result.
   CvMat* rowReduced = cvCreateMat(1,bitImage->width,CV_32FC1);
   // bitImage->width represents number of cols, while bitImage->height stands for 


rows
   CvMat* colReduced = cvCreateMat(bitImage->height,1,CV_32FC1);


   cvReduce(bitImage,rowReduced,0,CV_REDUCE_SUM);
   cvReduce(bitImage,colReduced,1,CV_REDUCE_SUM);


// compute imageROI.x
   for(int i=0;i<rowReduced->cols;i++)
   {
float temp = CV_MAT_ELEM(*rowReduced,float,0,i);
   if(temp>bitImage->height/3)
   {
imageROI.x = i;
   break;
   }
   }


// computer imageROI.width
   for(int i=rowReduced->cols;i>0;i--)
   {
float temp = CV_MAT_ELEM(*rowReduced,float,0,i-1);
if(temp>bitImage->height/3)
   {
imageROI.width = i-imageROI.x;
    break;
}
        }


// compute imageROI.y
   for(int i=0;i<colReduced->rows;i++)
    {
    float temp = CV_MAT_ELEM(*colReduced,float,i,0);
    if(temp>bitImage->height/3)
    {
    imageROI.y = i;
    break;
    }
    }


// compute imageROI.height
    for(int i=colReduced->rows;i>0;i--)
    {
    float temp = CV_MAT_ELEM(*colReduced,float,i-1,0);
    if(temp>bitImage->height/3)
    {
    imageROI.height = i-imageROI.y;
    break;
    }
    }


// set memory free
cvReleaseImage(&bitImage);
cvReleaseImage(&image8u);
cvReleaseMat(&rowReduced);
cvReleaseMat(&colReduced);
}
}
这是计算深度图像的滤波范围。由于深度图像和彩色图像的视点不一致，导致了将深度图像映射到彩色图


像上时有效像素会缩小，典型的现象就是在深度图像的四周会出现黑色的区域。这个函数就是用来将四周


的黑色框框去掉。用OpenCV的投影的方法。由于cvReduce()函数要进行累积和的计算，为了不使数据溢出


，目标数组应该用32位的浮点型（此函数只支持8位unsigned char型和32位float型）。


void kinectDenoising::medianFiltering()
{
// set result image zero
cvSetZero(denoisedImage);


unsigned short data[nFrames];
int total;
for(int i=imageROI.y;i<imageROI.y+imageROI.height;i++)
{
unsigned short* denoisedImageData = (unsigned short*)(denoisedImage-


>imageData+denoisedImage->widthStep*i);
for(int j=imageROI.x;j<imageROI.x+imageROI.width;j++)
{
total = 0;
for(int k=0;k<nFrames;k++)
{
insertSort(data,total,CV_IMAGE_ELEM(frameSet[k],unsigned 


short,i,j));
}
if(total != 0)
{
denoisedImageData[j] = data[total/2];
}
}
}
}
中值滤波，统计有效点并排序，然后取中值。insertSort()函数用来将值按从小到大的顺序进行插入，鉴


于篇幅的关系，就不贴出来了。


void kinectDenoising::nearestFiltering()
{
CvPoint topLeft,downRight;
IplImage* tempImage = cvCloneImage(denoisedImage);
for(int i=imageROI.y;i<imageROI.y+imageROI.height;i++)
{
unsigned short* data = (unsigned short*)(denoisedImage->imageData


+denoisedImage->widthStep*i);
for(int j=imageROI.x;j<imageROI.x+imageROI.width;j++)
{
for(int k=1;data[j]==0;k++)
{
topLeft = cvPoint(j-k,i-k);    // j为行数 i为列数
downRight = cvPoint(j+k,i+k);
for(int m=topLeft.x;(m<=downRight.x) && (data[j]==0);m++)
{
if(m<0) continue;
if(m>=width) break;
if(topLeft.y>=0)
{
unsigned short temp = CV_IMAGE_ELEM


(tempImage,unsigned short,topLeft.y,m);
    if(temp > 0)
    {
    data[j] = temp;
break;
    }
}
if(downRight.y < height)
{
unsigned short temp = CV_IMAGE_ELEM


(tempImage,unsigned short,downRight.y,m);
    if(temp > 0)
{
    data[j] = temp;
break;
    }
} 
}


for(int m=topLeft.y;(m<downRight.y) && (data[j]==0);m++)
{
if(m<0) continue;
if(m>=height) break;
if(topLeft.x>0)
{
unsigned short temp = CV_IMAGE_ELEM


(tempImage,unsigned short,m,topLeft.x);
if(temp > 0)
    {
    data[j] = temp;
break;
    }
}


if(downRight.x<width)
{
unsigned short temp = CV_IMAGE_ELEM


(tempImage,unsigned short,m,downRight.x);
if(temp > 0)
    {
    data[j] = temp;
break;
    }
}
}
}
}
}
cvReleaseImage(&tempImage);
}
最后是中值滤波，从最内层开始，一层层往外扩，直到找到有效值为止。


运行结果：


源图像：


结果图像：


附注：本来这个程序是在8位图像上进行的。先取得16位的unsigned short型深度图像，然后通过


cvConvertScale()函数将其转化为8位的unsigned char型，结果在进行去噪的时候怎么都不对，将


unsigned char型的数据放到matlab中一看，发现在unsigned short型数据中为0值的像素莫名其妙的在


unsigned char型里有了一个很小的值（比如说1, 2, 3, 4, 5什么的，就是不为0）。很奇怪，不知道


OpenCV中是怎么搞的。看来还是源数据靠谱，于是将其改为16位的unsigned short型，结果形势一片大好


。


http://blog.csdn.net/chenli2010/article/details/7006573


========


视频图像去模糊常用处理方法



随着“平安城市”的广泛建设，各大城市已经建有大量的视频监控系统，虽然监控系统己经广泛地存在于


银行、商场、车站和交通路口等公共场所，但是在公安工作中，由于设备或者其他条件的限制，案情发生


后的图像回放都存在图像不清晰，数据不完整的问题，无法为案件的及时侦破提供有效线索。经常出现嫌


疑人面部特征不清晰，难以辨认，嫌疑车辆车牌模糊无法辨认等问题。这给公安部门破案、法院的取证都


带来了极大的麻烦。随着平安城市的推广、各地各类监控系统建设的进一步推进，此类问题会越来越突出


。


一．模糊图像产生的原因
1.  系统自身因素
    （1）镜头聚焦不当、摄像机故障等。
    （2）传输太远、视频线老化
    （3）光学镜头的极限分辨率和摄像机不匹配导致的模糊;
    （4）相机分辨率低，欠采样成像。
2.　自然环境
    （1）摄像机罩或镜头受脏污、受遮挡等。
    （2）大雾，沙尘、雨雪等环境影响等。
3.  人为环境
    （1）环境电磁干扰;
    （2）视频压缩算法、传输带宽导致的模糊。
    （3）运动目标高速运动导致的运动模糊等;


二.　模糊图像常用的处理方法
          对于模糊图像处理技术，国内大学和科研机构在多年以前就在研究这些理论和应用，相关文献


也发布了不少，已经取得了一些很好的应用。当前有很多软件已经有了相当成熟的一套模糊图像恢复方法


，在美国FBI及其他执法机构中已有多年实际应用，其恢复出的图像可以直接当作法庭证据使用，可见模


糊图像处理技术已经取得了相当的实际应用。


          从技术方面来向，模糊图像处理方法主要分为三大类，分别是图像增强、图像复原和超分辨率


重构。
2.1  图像增强
          增强图象中的有用信息，它可以是一个失真的过程，其目的是要改善图像的视觉效果，针对给


定图像的应用场合，有目的地强调图像的整体或局部特性，将原来不清晰的图像变得清晰或强调某些感兴


趣的特征，扩大图像中不同物体特征之间的差别，抑制不感兴趣的特征，使之改善图像质量、丰富信息量


，加强图像判读和识别效果，满足某些特殊分析的需要。


           图像增强技术根据增强处理过程所在的空间不同，可分为基于空域的算法和基于频域的算法


两大类。


          前者把图像看成一种二维信号，对其进行基于二维傅里叶变换的信号增强。采用低通滤波（即


只让低频信号通过）法，可去掉图中的噪声；采用高通滤波法，则可增强边缘等高频信号，使模糊的图片


变得清晰。具有代表性的空间域算法有局部求平均值法和中值滤波（取局部邻域中的中间像素值）法等，


它们可用于去除或减弱噪声。


          基于空域的算法分为点运算算法和邻域去噪算法。点运算算法即灰度级校正、灰度变换和直方


图修正等，目的或使图像成像均匀，或扩大图像动态范围，扩展对比度。邻域增强算法分为图像平滑和锐


化两种。平滑一般用于消除图像噪声，但是也容易引起边缘的模糊。常用算法有均值滤波、中值滤波。锐


化的目的在于突出物体的边缘轮廓，便于目标识别。常用算法有梯度法、算子、高通滤波、掩模匹配法、


统计差值法等。


2.1.1  图像增强的几个方面及方法


          1.对比度变换:线性变换、非线性变换

          2.空间滤波：图像卷积运算、平滑、锐化

          3.彩色变换：单波段彩色变换、多波段彩色运算、HIS


          4.多光谱变换：K-L变换、K-T变换

          5.图像运算：插值运算、比值运算


2.1.2  图像增强的应用概况


          图像增强的方法分为空域法和频域法两种，空域法是对图像中的像素点进行操作，用公式描述


如下：


                                        g(x,y)=f(x,y)*h(x,y)


          其中是f(x,y)原图像；h(x,y)为空间转换函数；g(x,y)表示进行处理后的图像。


          频域法是间接的处理方法，是先在图像的频域中对图像的变换值进行操作，然后变回空域。例

如,先对图像进行傅里叶变化到频域，再对图像的频谱进行某种滤波修正，最后将修正后的图像进行傅里

叶反变化到空域，以此增强图像。

          很多传统图像算法都可以减轻图像的模糊程度， 比如图像滤波、几何变换、对比度拉伸、直

方图均衡、空间域锐化、亮度均匀化、形态学、颜色处理等。单个来讲，这些算法比较成熟，相对简单。

但是对于一个具体的模糊图像，往往需要上面的一种或者多种算法组合，配合不同的参数才能达到理想的

效果。

          这些算法和参数的组合进一步发展为具体的增强算法，比如“图像去雾”算法（可参考何恺明

经典去雾算法）、“图像去噪”算法、“图像锐化”算法、“图像暗细节增强”算法等。


2.2  图像复原
2.2.1  图像复原概述


          在图像的获取、传输以及保存过程中，由于各种因素，如大气的湍流效应、摄像设备中光学系


统的衍射、传感器特性的非线性、光学系统的像差、成像设备与物体之间的相对运动、感光胶卷的非线性


及胶片颗粒噪声以及电视摄像扫描的非线性等所引起的几何失真，都难免会造成图像的畸变和失真。通常


，称由于这些因素引起的质量下降为图像退化。


            早期的图像复原是利用光学的方法对失真的观测图像进行校正，而数字图像复原技术最早则


是从对天文观测图像的后期处理中逐步发展起来的。其中一个成功例子是NASA的喷气推进实验室在1964年


用计算机处理有关月球的照片。照片是在空间飞行器上用电视摄像机拍摄的，图像的复原包括消除干扰和


噪声，校正几何失真和对比度损失以及反卷积。另一个典型的例子是对肯尼迪遇刺事件现场照片的处理。


由于事发突然，照片是在相机移动过程中拍摄的，图像复原的主要目的就是消除移动造成的失真。


          早期的复原方法有：非邻域滤波法，最近邻域滤波法以及效果较好的维纳滤波和最小二乘滤波


等。目前国内外图像复原技术的研究和应用主要集中于诸如空间探索、天文观测、物质研究、遥感遥测、


军事科学、生物科学、医学影象、交通监控、刑事侦察等领域。如生物方面，主要是用于生物活体细胞内


部组织的三维再现和重构，通过复原荧光显微镜所采集的细胞内部逐层切片图，来重现细胞内部构成；医


学方面，如对肿瘤周围组织进行显微观察，以获取肿瘤安全切缘与癌肿原发部位之间关系的定量数据；天


文方面，如采用迭代盲反卷积进行气动光学效应图像复原研究等。


 2.2.2  图像退化模型


          图像复原问题的有效性关键之一取决于描述图像退化过程模型的精确性。要建立图像的退化模


型，则首先必须了解、分析图像退化的机理并用数学模型表现出来。在实际的图像处理过程中，图像均需


以数字离散函数表示，所以必须将退化模型离散化。


2.2.3  几种较经典的复原方法介绍

          图像复原算法有线性和非线性两类。线性算法通过对图像进行逆滤波来实现反卷积，这类方法

方便快捷，无需循环或迭代，直接可以得到反卷积结果，然而，它有一些局限性，比如无法保证图像的非

负性。而非线性方法通过连续的迭代过程不断提高复原质量，直到满足预先设定的终止条件，结果往往令

人满意。但是迭代程序导致计算量很大，图像复原时耗较长，有时甚至需要几个小时。所以实际应用中还

需要对两种处理方法综合考虑，进行选择。


          1）维纳滤波法


          维纳滤波法是由Wiener首先提出的，应用于一维信号处理，取得了很好的效果。之后，维纳滤

波法被用于二维信号处理，也取得了不错的效果，尤其在图像复原领域，由于维纳滤波计算量小，复原效

果好，从而得到了广泛的应用和发展。

          2）正则滤波法

          另一个容易实现线性复原的方法称为约束的最小二乘方滤波，在IPT中称为正则滤波，并且通

过函数deconvreg来实现。

          3）Lucy-Richardson算法

          L-R算法是一种迭代非线性复原算法，它是从最大似然公式印出来的，图像用泊松分布加以模

型化的。当迭代收敛时模型的最大似然函数就可以得到一个令人满意的方程。

          4）盲去卷积

          在图像复原过程中，最困难的问题之一是，如何获得PSF的恰当估计。那些不以PSF为基础的图


像复原方法统称为盲区卷积。它以MLE为基础的，即一种用被随机噪声所干扰的量进行估计的最优化策略


。工具箱通过函数deconvblind来执行盲区卷积。


2.2.4  图像复原与图像增强


           图像复原与图像增强技术一样，也是一种改善图像质量的技术。图像复原是根据图像退化的

先验知识建立一个退化模型，以此模型为基础，采用各种逆退化处理方法进行恢复，改善图像质量。

           图像复原和图像增强是有区别的，二者的目的都是为了改善图像的质量。但图像增强不考虑

图像是如何退化的，只有通过试探各种技术来增强图像的视觉效果，而图像复原就完全不同，需知道图像

退化过程的先验知识，据此找出一种相应的逆过程方法，从而得到复原的图像。图像复原主要取决于对图

像退化过程的先验知识所掌握的精确程度。


          对由于离焦、运动、大气湍流等原因引起的图像模糊，图像复原的方法效果较好，常用的算法

包括维纳滤波算法、小波算法、基于训练的方法等。在知道退化模型的情况下，相对图像增强来说，图像

复原可以取得更好的效果。


2.3 图像超分辨率重构
          现有的监控系统主要目标为宏观场景的监视，一个摄像机，覆盖很大的一个范围，导致画面中

目标太小，人眼很难直接辨认。这类由于欠采样导致的模糊占很大比例，对于由欠采样导致的模糊需要使

用超分辨率重构的方法。


          超分辨率复原是通过信号处理的方法，在提高图像的分辨率的同时改善采集图像质量。其核心

思想是通过对成像系统截止频率之外的信号高频成分估计来提高图像的分辨率。超分辨率复原技术最初只

对单幅图像进行处理，这种方法由于可利用的信息只有单幅图像，图像复原效果有着固有的局限。序列图

像的超分辨率复原技术旨在采用信号处理方法通过对序列低分辨率退化图像的处理来获得一幅或者多幅高

分辨率复原图像。由于序列图像复原可利用帧间的额外信息，比单幅复原效果更好，是当前的研究热点。

          序列图像的超分辨率复原主要分为频域法和空域法两大类，频域方法的优点是:理论简单,运算

复杂度低，缺点是:只局限于全局平移运动和线性空间不变降质模型,包含空域先验知识的能理有限。

          空域方法所采用的观测模型涉及全局和局部运动、空间可变模糊点扩散函数、非理想亚采样等

,而且具有很强的包含空域先验约束的能力。常用的空域法有非均匀插值法、迭代反投影方法(IBP)、凸集

投影法(POCS)、最大后验估计法(MAP)、最大似然估计法(ML)、滤波器法等，其中，MAP和POCS法研究较多

，发展空间很大。


三：模糊图像处理的关键和不足
          虽然很多模糊图像的处理方法在实际应用中取得了很好的效果，但是当前仍然有一些因素制约

着模糊图像处理的进一步发展，主要如下：


          1、 算法的高度针对性;

          绝大部分的模糊图像处理算法只适用于特定图像，而算法本身无法智能决定某个算法模块的开

启还是关闭。举例来说，对于有雾的图像，“去雾算法”可以取得很好的处理效果，但是作用于正常图像

，反而导致图像效果下降，“去雾算法”模块的打开或者关闭需要人工介入。


          2、 算法参数复杂性;

          模糊图像处理里面所有的算法都会包含大量的参数，这些参数的选择需要和实际的图像表现相

结合，直接决定最终的处理效果。目前算法还没有办法智能选择这些最优参数。

          3、 算法流程的经验性;


          由于实际图像很复杂，需要处理多种情况，这就需要一个算法处理流程，对于一个具体的模糊

视频，采用什么样的处理流程很难做到自动选择，需要人工选择一个合适的方法，只能靠人的经验。


四：实践和总结
          由于环境、线路、镜头、摄像机等影响，监控系统建成运营一段时间后，都会出现一部分的视

频模糊不清的问题。前面提到了针对模糊图像的各种处理算法，虽然这些算法都取得了一些较好的处理效

果，但是再好的算法都是一种后期的补救措施。如果能及时发现监控系统中图像的各种问题，并及时维修

，必然会起到事半功倍的效果。


          利用先进的视频诊断技术，开发出适用于各种需求场景的视频质量诊断系统。它能够对视频图

像出现的模糊、噪声、亮度异常和视频丢失等低质视频以及常见摄像机故障问题进行诊断，有效预防因硬

件问题导致的图像质量低下所带来的损失。从几路视频到几百上千、上万路视频，均可高效的进行检测，

自动生成检测报告，提供及时且精准的维护信息，第一时间从根源上解决图像模糊的问题。

          总体来说，对于不同种类的模糊问题，要区别对待。对于由镜头离焦、灰尘遮挡、线路老化、

摄像机故障等造成的模糊或者图像质量下降，在视频诊断系统的帮助下，一定要及时维修，从源头上解决

问题。对于低光照等优先选择日夜两用型高感光度摄像机，对于雨雾、运动和前采样等造成的图像质量下

降，可以借助于“视频增强服务器”包含的各种模糊图像处理算法，提升图像质量。


 后记


        Single-Image Super-Resolution for anime/fan-art using Deep Convolutional Neural 


Networks.
waifu2x是采用了最新锐的人工智能技术“Deep Convolutional Neural Networks”开发的网络服务。


       名字来源于海外的动画粉丝们将喜欢的角色称作“waifu（即‘我老婆’）”。把缩小的锯齿状图

传到waifu2x的话，“现在给你的图是某张图缩小一半的图。求缩小前的图哦”，人工智能就会将噪点和

锯齿的部分进行补充，生成新的图。于是“扩大时的图”将不存在了，小的图变成了扩大了的图，同时还

可以去除噪点。

========
opencv 背景差分法 改进OTSU阈值去噪



/*
 *1)头文件cvaux.h的库文件需要链接到linker. cvaux210d.lib, cvaux210.lib分别是debug和release版


本. 
 *   否则有些函数会出现error:: LINK 2101 
 *2)cvAdaptiveThreshold, cvThreshold的src和dst图像必须是同类型的单通道图像
 *3)重要: 函数退出之后,函数中的动态变量会随着栈的退出全部清空.
 *   要保存上次操作的结果,则在函数内声明为静态变量.或者在要调用的函数里先声明
 *4)cvAdaptiveThreshold()的处理结果不好,此函数试图找出所有的物体轮廓.用在背景差分中,会找到不


想要的物体
 *5)当没有前景物体时,OTSU算法会把路面显示出来.因为阈值是自动调整的.解决办法,做一个阈值筛选
 *6)VedioControl() 未实现.
 *    将cvWaitKey()封装到VedioControl()中, 如果不触发按键,VedioControl()不会退出.造成无法自


动播放
 *
 *Date: 2012/4/6
 *Author: Rocky Chen
 */
#include <stdio.h>
#include "stdafx.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include "cvaux.h" 
#include "cxmisc.h"




using namespace std;




void BackgroundDiff(IplImage* SrcImg, IplImage* FroundImg, IplImage* BackgroundImg, int 


nFrmNum, int threshold_method);
void cvOtsu(IplImage *src, int *thresholdValue);
void PrintVedioInfo(CvCapture* pCapture, IplImage* img);
void VedioControl();  //未实现




 //视频控制全局变量,
// 's' 画面stop
// 'q' 退出播放
// 'p' 打印OTSU算法中找到的阈值
char ctrl = NULL; 




int main( int argc, char** argv )
{
  //声明IplImage指针
  IplImage* pFrame = NULL; 
  IplImage* pFroundImg = NULL;
  IplImage* pBackgroundImg = NULL;




  IplImage* pFroundImg_c = NULL;
  IplImage* pBackgroundImg_c = NULL;




    //大门背景建模良好  best
//  CvCapture* pCapture = cvCreateFileCapture("D:\\C++ Projects\\OpenCV_project\\test_video


\\gate_11ms_00-30s.avi"); 
//  CvCapture* pCapture = cvCreateFileCapture("D:\\C++ Projects\\OpenCV_project\\img_video\


\video.short.mjpg.avi");
  CvCapture* pCapture = cvCreateFileCapture("D:\\C++ Projects\\OpenCV_project\\img_video\


\video.long.mjpg.avi");
  int nFrmNum = 0;


  //创建窗口
  cvNamedWindow("video", 1);
  cvNamedWindow("background",1);
  cvNamedWindow("OTSU foreground",1);
  cvNamedWindow("改进的OTSU foreground",1);

  //使窗口有序排列
  cvMoveWindow("video", 30, 0);
  cvMoveWindow("background", 360, 0);
  cvMoveWindow("OTSU foreground", 690, 0);
  cvMoveWindow("改进的OTSU foreground", 690, 320);
  //逐帧读取视频
  while(pFrame = cvQueryFrame( pCapture ))
    {

 nFrmNum++;
 //视频控制
 if( (ctrl = cvWaitKey(1000/180)) =='s' )  cvWaitKey();
 else if(ctrl == 'p') cout << "Current Frame = " << nFrmNum << endl;
 else if( ctrl =='q' )
break;

 if(nFrmNum ==1)
 {  
   pBackgroundImg = cvCreateImage(cvGetSize(pFrame),  8,1);
 pFroundImg = cvCreateImage(cvGetSize(pFrame),  8,1);
 pBackgroundImg_c = cvCreateImage(cvGetSize(pFrame),  8,1); //对比算法的图像
 pFroundImg_c = cvCreateImage(cvGetSize(pFrame),  8,1);
 }

 BackgroundDiff(pFrame,pFroundImg,pBackgroundImg, nFrmNum, CV_THRESH_OTSU);  //普通OTSU
 BackgroundDiff(pFrame,pFroundImg_c,pBackgroundImg_c, nFrmNum, CV_THRESH_BINARY); //阈值筛选

后的OTSU
 //打印视频信息,画面控制
 PrintVedioInfo(pCapture, pFroundImg);
 //显示图像
 cvShowImage("video", pFrame);
 cvShowImage("background", pBackgroundImg);
 cvShowImage("OTSU foreground", pFroundImg);
 cvShowImage("改进的OTSU foreground", pFroundImg_c);
   }  //while
  
  //销毁窗口
  cvDestroyAllWindows();
  //释放图像和矩阵
  cvReleaseImage(&pFroundImg);
  cvReleaseImage(&pBackgroundImg);
  cvReleaseCapture(&pCapture);

  return 0;
}

void VedioControl()
{  

}

/*
 *输出文字到图像
 */
void PrintVedioInfo(CvCapture* pCapture, IplImage* img)
{
assert( pCapture != NULL);
double frames = cvGetCaptureProperty(pCapture, CV_CAP_PROP_POS_FRAMES);  //视频当前帧数 
 double fps = cvGetCaptureProperty(pCapture,CV_CAP_PROP_FPS); //获得视频每秒帧数
 char str[255];
 sprintf(str,"%4.2f FPS %4.2f frames",fps,frames);  // 将浮点数转化为字符串
 CvPoint location = cvPoint(20,20); // 建立字符串打印的位置
 CvScalar color = cvScalar(255,255,255);
 CvFont font;  //建立字体变量
 cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0,1.0);  //字体设置
 cvPutText(img, str, location, &font,color);  //打印文本到图像
}

/********
*背景差分函数,求前景目标
*重要: 函数退出之后,函数中的动态变量会随着栈的退出全部清空.
*要保存上次操作的结果,则在函数内声明为静态变量.或者在要调用的函数里先声明
*
********/
void BackgroundDiff(IplImage* SrcImg, IplImage* FroundImg, IplImage* BackgroundImg, int 


nFrmNum, int threshold_method = CV_THRESH_OTSU)
{
static IplImage* SrcImg_gray = NULL;//源图像的灰度图像
static IplImage* SrcImg_grayf =NULL;  //单通道浮点图像用于背景建模
static IplImage* FroundImgf = NULL;
static IplImage* BackgroundImgf = NULL;
static   IplImage* FroundImg_temp = NULL;
    if(nFrmNum == 1)
{
 
 SrcImg_gray = cvCreateImage(cvGetSize(SrcImg),  8,1);
 FroundImg_temp = cvCreateImage(cvGetSize(SrcImg),  8,1);
 BackgroundImgf = cvCreateImage(cvGetSize(SrcImg),  32,1);  //浮点图像
 FroundImgf = cvCreateImage(cvGetSize(SrcImg),  32,1);
 SrcImg_grayf = cvCreateImage(cvGetSize(SrcImg),  32,1);

 //RGB图像先转化成8位单通道图像,再转化为浮点.
 cvCvtColor(SrcImg, BackgroundImg, CV_BGR2GRAY); 
 cvCvtColor(SrcImg, FroundImg, CV_BGR2GRAY); 
 cvConvert(BackgroundImg,BackgroundImgf);
   cvConvert(FroundImg,FroundImgf);
}
else
{
 cvCvtColor(SrcImg, SrcImg_gray, CV_BGR2GRAY);  //SrcImg_gray在上次函数退出的时候被程序栈回收
   cvConvert(SrcImg_gray,SrcImg_grayf);
 //当前帧跟背景图相减
    cvAbsDiff(SrcImg_grayf, BackgroundImgf, FroundImgf);
cvConvert(FroundImgf,FroundImg_temp);  //浮点转化为整点
 //二值化前景图
 int threshold_otsu =0;
cvOtsu(FroundImg_temp, &threshold_otsu);

 if(threshold_method == CV_THRESH_OTSU)
 {
  cvThreshold(FroundImg_temp, FroundImg, 0, 255.0, CV_THRESH_OTSU); //对比自适应阈值化
 // cvAdaptiveThreshold(FroundImg_temp, FroundImg, 255.0, 0, 0, 51);  //src和dst必须同时是

8bit或浮点图像
 }
 else
 {
    cvThreshold(FroundImg_temp, FroundImg, threshold_otsu, 255.0, CV_THRESH_BINARY); 
 }
     cvSegmentFGMask( FroundImg ); //对前景做连通域分割
 //更新背景
 cvRunningAvg(SrcImg_grayf, BackgroundImgf, 0.003, 0);  //必须是浮点图像,因为会有小数出现
 cvConvert(BackgroundImgf,BackgroundImg);
}
}

/********
 *OTSU大津法
 * thresholdValue 为使类间方差最大的阈值
 * 当找到的阈值小于一个修正阈值,返回此修正阈值.防止没有前景物体时,将背景找出来
 ********/
void cvOtsu(IplImage *src, int *thresholdValue)
{  
    int deltaT = 0; //光照调节参数
uchar grayflag =1;
IplImage* gray = NULL;
if(src->nChannels != 1) //检查源图像是否为灰度图像
{
gray = cvCreateImage(cvGetSize(src), 8, 1);
cvCvtColor(src, gray, CV_BGR2GRAY);
grayflag = 0;
}
else gray = src;
uchar* ImgData=(uchar*)(gray->imageData);   
int thresholdValue_temp = 1;
    int ihist[256];   //图像直方图,256个点  
   
    int i, imgsize; //循环变量,图像尺寸
    int n, n1, n2;  //n 非零像素个数, n1 前景像素个数, n2 背景像素个数
    double m1, m2, sum, csum, fmax, sb;//m1前景灰度均值,m2背景灰度均值
    //对直方图置零   
    memset(ihist, 0, sizeof(ihist));   
    //生成直方图  
    imgsize = (gray->widthStep)*(gray->height);//图像数据总数 
    for (i=0; i<imgsize;i++)   
    {   
    ihist[((int)(*ImgData))&255]++;//灰度统计 '&255'防止指针溢出  
    ImgData++;//像素遍历
    }   
    // set up everything   
    sum=csum=0.0;   
    n=0;   
    for (i=0; i<255; i++)   
    {   
    sum+=(double)i*(double)ihist[i];  // x*f(x)质量矩   
    n+= ihist[i];   //f(x)质量 像素总数
    }

deltaT = (int)(sum/imgsize); //像素平均灰度
deltaT = deltaT>>1; //与之矫正,delatT = v*n; v=0.5
   
    if (!n)   
    {//图像全黑,输出警告
    fprintf (stderr, "NOT NORMAL thresholdValue=160\n");   
    }   
    // OTSU算法
    fmax=-1.0;   
    n1=0;   
    for (i=0; i<255; i++)   
    {   
        n1+= ihist[i];   
        if (n1==0) {continue;}
        n2=n-n1;   
        if (n2==0) {break;}   
        csum += (double)i *ihist[i];   
        m1=csum/n1;   
        m2=(sum-csum)/n2;   
        sb=(double)n1*(double)n2*(m1-m2)*(m1-m2); //计算类间方差,  公式已简化  
        if (sb>fmax)   
        {   
            fmax=sb;   
            thresholdValue_temp=i;  //找到使类间方差最大的灰度值i   
        }  
    }   
   
if(thresholdValue_temp < 20)
*thresholdValue = 20;  //阈值筛选
else *thresholdValue = thresholdValue_temp;
if( ctrl == 'p')  //ctrl  = cvWaitKey(100),且是全局变量
{
   cout << "OTSU thresholdValue = " << thresholdValue_temp<<
", Returned thresholdValue = " << *thresholdValue<<'\n'<<endl;
}
if(!grayflag) cvReleaseImage(&gray);
}  


/***********
*轮廓提取
************/
void Labeling(IplImage *src, IplImage *dst) 
{
    CvMemStorage* storage = 0;
    storage = cvCreateMemStorage(0); //开辟默认大小的空间
    CvSeq* contour=0;
    cvCopy(src,dst,0);
    cvFindContours( dst, storage, &contour, sizeof(CvContour),
              CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); //外边缘
    int num=0;
    for( ;contour!=0; contour=contour->h_next)
    {
          CvRect rect;
      rect = cvBoundingRect(contour,0);//得到目标外接矩形
          num++;
        if((rect.height + rect.width) >= 16)
        cvRectangle(src,cvPoint(rect.x,rect.y),cvPoint(rect.x+rect.width,rect.y


+rect.height),
                  CV_RGB(255, 255,255),1,8);//绘制目标外接矩形
// cvRectangle(dst,cvPoint(rect.x,rect.y),cvPoint(rect.x+rect.width,rect.y+rect.height),
 //                 CV_RGB(255, 255,255),1,8);//绘制目标外接矩形
    }
}
========
线性滤波专场：方框滤波、均值滤波与高斯滤波

http://blog.csdn.net/poem_qianmo/article/details/22745559


写作当前博文时配套使用的OpenCV版本： 2.4.8


本篇文章中，我们一起仔细探讨了OpenCV图像处理技术中比较热门的图像滤波操作。图像滤波系列文章浅

墨准备花两次更新的时间来讲，此为上篇，为大家剖析了“方框滤波“，”均值滤波“和”高斯滤波“三

种常见线性滤波操作。而作为非线性滤波的“中值滤波”和“双边滤波”，留待我们下次剖析。

先上一张精彩截图：

如果是单单想要掌握这篇文章中讲解的OpenCV线性滤波相关的三个函数：boxFilter，blur和

GaussianBlur的使用方法的话，直接看第三部分“浅出”和第四部分“实例”就行。

在以后写的OpenCV系列文章中，浅墨暂且准备将每篇博文中知识点都分成原理、深入、浅出和实例四大部

分来讲解，

第一部分为和图像处理中线性滤波相关的理论，第二部分“深入”部分主要深入OpenCV内部，带领大家领

略OpenCV的开源魅力，进行OpenCV相关源码的剖析，做到对OpenCV理解深刻，做一个高端大气的OpenCV使

用者。 第三部分“浅出”主要教会大家如何快速上手当前文章中介绍的相关OpenCV API函数。 而在第四

部分，浅墨会为大家准备一个和本篇文章相关的详细注释的综合实例程序。

给出本篇万字文章的结构脉络：

一、理论——相关图像处理概念介绍

二、深入——OpenCV源码讲解

三、浅出——API函数讲解

四、实例——详细注释的博文配套程序

一、理论与概念讲解


1.关于平滑处理

“平滑处理“（smoothing）也称“模糊处理”（bluring），是一项简单且使用频率很高的图像处理方法

。平滑处理的用途有很多，最常见的是用来减少图像上的噪点或者失真。在涉及到降低图像分辨率时，平

滑处理是非常好用的方法。

2.滤波与滤波器

首先看一下滤波的概念，滤波是将信号中特定波段频率滤除的操作，是抑制和防止干扰的一项重要措施。

而滤波器就是建立的一个数学模型，通过这个模型来将图像数据进行能量转化，能量低的就排除掉，噪声

就是属于低能量部分。

一种形象的比喻法是：我们可以把滤波器想象成一个包含加权系数的窗口，当使用这个滤波器平滑处理图

像时，就把这个窗口放到图像之上，透过这个窗口来看我们得到的图像。

滤波器的种类有很多， 在新版本的OpenCV中，提供了如下五种常用的图像平滑处理操作方法，且他们分

别被封装在单独的函数中，使用起来非常方便：

方框滤波——boxblur函数
均值滤波——blur函数
高斯滤波——GaussianBlur函数
中值滤波——medianBlur函数
双边滤波——bilateralFilter函数
今天我们要讲解的是作为线性滤波的方框滤波，均值滤波和高斯滤波。两种非线性滤波操作——中值滤波

和双边滤波，我们留待下次讲解。

3.对线性滤波器的简介


线性滤波器：线性滤波器经常用于剔除输入信号中不想要的频率或者从许多频率中选择一个想要的频率。


几种常见的线性滤波器：


允许低频率通过的 低通滤波器 。
允许高频率通过的 高通滤波器 。
允许一定范围频率通过的 带通滤波器 。
阻止一定范围频率通过并且允许其它频率通过的 带阻滤波器 。
允许所有频率通过、仅仅改变相位关系的 全通滤波器 。
阻止一个狭窄频率范围通过的特殊 带阻滤波器 ， 陷波滤波器 （Band-stop filter）。


4.关于滤波和模糊

关于滤波和模糊，大家往往在初次接触的时候会弄混淆，“一会儿说滤波，一会儿又说模糊，什么玩意儿啊”。


我们上文已经提到过，滤波是将信号中特定波段频率滤除的操作，是抑制和防止干扰的一项重要措施。

为了方便说明，就拿我们经常用的高斯滤波来作例子吧。 我们知道，滤波可分低通滤波和高通滤波两种

。 而高斯滤波是指用高斯函数作为滤波函数的滤波操作 ，至于是不是模糊，要看是高斯低通还是高斯高

通，低通就是模糊，高通就是锐化。

其实说白了是很简单的，对吧：

高斯滤波是指用高斯函数作为滤波函数的滤波操作。

高斯模糊就是高斯低通滤波。

5.线性滤波


线性滤波是一种常用的邻域算子，像素的输出值取决于输入像素的加权和，具体过程如图。


 


图注：邻域滤波（卷积）：左边图像与中间图像的卷积产生右边图像。目标图像中蓝色标记的像素是利用


原图像中红色标记的像素计算得到的。


线性滤波处理的输出像素值   是输入像素值   的加权和 :
 


其中的加权和为 ，我们称其为“核”，滤波器的加权系数，即滤波器的“滤波系数”。


上面的式子可以简单写作：
  


其中f表示输入像素值，h表示加权系数“核“，g表示输出像素值


在新版本的OpenCV中，提供了如下三种常用的线性滤波操作，他们分别被封装在单独的函数中，使用起来


非常方便：


方框滤波——boxblur函数


均值滤波——blur函数


高斯滤波——GaussianBlur函数


下面我们来对他们进行一一介绍。


6.方框滤波（box Filter）


方框滤波（box Filter）被封装在一个名为boxblur的函数中，即boxblur函数的作用是使用方框滤波器（


box filter）来模糊一张图片，从src输入，从dst输出。


函数原型如下：


C++: void boxFilter(InputArray src,OutputArray dst, int ddepth, Size ksize, Point 


anchor=Point(-1,-1), boolnormalize=true, int borderType=BORDER_DEFAULT )
参数详解：


第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处


理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为CV_8U, CV_16U, CV_16S, 


CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。
第三个参数，int类型的ddepth，输出图像的深度，-1代表使用原图深度，即src.depth()。
第四个参数，Size类型（对Size类型稍后有讲解）的ksize，内核的大小。一般这样写Size( w,h )来表示


内核的大小( 其中，w 为像素宽度， h为像素高度)。Size（3,3）就表示3x3的核大小，Size（5,5）就表


示5x5的核大小
第五个参数，Point类型的anchor，表示锚点（即被平滑的那个点），注意他有默认值Point(-1,-1)。如


果这个点坐标是负值的话，就表示取核的中心为锚点，所以默认值Point(-1,-1)表示这个锚点在核的中心


。
第六个参数，bool类型的normalize，默认值为true，一个标识符，表示内核是否被其区域归一化


（normalized）了。
第七个参数，int类型的borderType，用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT


，我们一般不去管它。
boxFilter（）函数方框滤波所用的核为：
  
其中：


  其中f表示原图，h表示核，g表示目标图，当normalize=true的时候，方框滤波就变成了我们熟悉的均


值滤波。也就是说，均值滤波是方框滤波归一化（normalized）后的特殊情况。其中，归一化就是把要处


理的量都缩放到一个范围内,比如(0,1)，以便统一处理和直观量化。


而非归一化（Unnormalized）的方框滤波用于计算每个像素邻域内的积分特性，比如密集光流算法


（dense optical flow algorithms）中用到的图像倒数的协方差矩阵（covariance matrices of image 


derivatives）


如果我们要在可变的窗口中计算像素总和，可以使用integral()函数。


7.均值滤波


均值滤波，是最简单的一种滤波操作，输出图像的每一个像素是核窗口内输入图像对应像素的像素的平均


值( 所有像素加权系数相等)，其实说白了它就是归一化后的方框滤波。


我们在下文进行源码剖析时会发现，blur函数内部中其实就是调用了一下boxFilter。


下面开始讲均值滤波的内容吧。


1）均值滤波的理论简析


均值滤波是典型的线性滤波算法，主要方法为邻域平均法，即用一片图像区域的各个像素的均值来代替原


图像中的各个像素值。一般需要在图像上对目标像素给出一个模板（内核），该模板包括了其周围的临近


像素（比如以目标像素为中心的周围8（3x3-1）个像素，构成一个滤波模板，即去掉目标像素本身）。再


用模板中的全体像素的平均值来代替原来像素值。即对待处理的当前像素点（x，y），选择一个模板，该


模板由其近邻的若干像素组成，求模板中所有像素的均值，再把该均值赋予当前像素点（x，y），作为处


理后图像在该点上的灰度个g（x，y），即个g（x，y）=1/m ∑f（x，y） ，其中m为该模板中包含当前像


素在内的像素总个数。


2）均值滤波的缺陷


均值滤波本身存在着固有的缺陷，即它不能很好地保护图像细节，在图像去噪的同时也破坏了图像的细节


部分，从而使图像变得模糊，不能很好地去除噪声点。


3）在OpenCV中使用均值滤波——blur函数


blur函数的作用是，对输入的图像src进行均值滤波后用dst输出。


blur函数文档中，给出的其核是这样的：


这个内核一看就明了，就是在求均值，即blur函数封装的就是均值滤波。


 blur 函数的原型：


C++: void blur(InputArray src, OutputArraydst, Size ksize, Point anchor=Point(-1,-1), int 


borderType=BORDER_DEFAULT )
第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处


理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为CV_8U, CV_16U, CV_16S, 


CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。比如可以用


Mat::Clone，以源图片为模板，来初始化得到如假包换的目标图。
第三个参数，Size类型（对Size类型稍后有讲解）的ksize，内核的大小。一般这样写Size( w,h )来表示


内核的大小( 其中，w 为像素宽度， h为像素高度)。Size（3,3）就表示3x3的核大小，Size（5,5）就表


示5x5的核大小
第四个参数，Point类型的anchor，表示锚点（即被平滑的那个点），注意他有默认值Point(-1,-1)。如


果这个点坐标是负值的话，就表示取核的中心为锚点，所以默认值Point(-1,-1)表示这个锚点在核的中心


。
第五个参数，int类型的borderType，用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT


，我们一般不去管它。
8.高斯滤波


1）高斯滤波的理论简析


高斯滤波是一种线性平滑滤波，适用于消除高斯噪声，广泛应用于图像处理的减噪过程。通俗的讲，高斯


滤波就是对整幅图像进行加权平均的过程，每一个像素点的值，都由其本身和邻域内的其他像素值经过加


权平均后得到。高斯滤波的具体操作是：用一个模板（或称卷积、掩模）扫描图像中的每一个像素，用模


板确定的邻域内像素的加权平均灰度值去替代模板中心像素点的值。


大家常常说高斯滤波最有用的滤波操作，虽然它用起来，效率往往不是最高的。


高斯模糊技术生成的图像，其视觉效果就像是经过一个半透明屏幕在观察图像，这与镜头焦外成像效果散


景以及普通照明阴影中的效果都明显不同。高斯平滑也用于计算机视觉算法中的预先处理阶段，以增强图


像在不同比例大小下的图像效果（参见尺度空间表示以及尺度空间实现）。从数学的角度来看，图像的高


斯模糊过程就是图像与正态分布做卷积。由于正态分布又叫作高斯分布，所以这项技术就叫作高斯模糊。


图像与圆形方框模糊做卷积将会生成更加精确的焦外成像效果。由于高斯函数的傅立叶变换是另外一个高


斯函数，所以高斯模糊对于图像来说就是一个低通滤波操作。


  高斯滤波器是一类根据高斯函数的形状来选择权值的线性平滑滤波器。高斯平滑滤波器对于抑制服从正


态分布的噪声非常有效。一维零均值高斯函数为：
   


其中，高斯分布参数Sigma决定了高斯函数的宽度。对于图像处理来说，常用二维零均值离散高斯函数作


平滑滤波器。


二维高斯函数为：
 


2）在OpenCV中使用高斯滤波——GaussianBlur函数


GaussianBlur函数的作用是用高斯滤波器来模糊一张图片，对输入的图像src进行高斯滤波后用dst输出。 


它将源图像和指定的高斯核函数做卷积运算，并且支持就地过滤（In-placefiltering）。


C++: void GaussianBlur(InputArray src,OutputArray dst, Size ksize, double sigmaX, double 


sigmaY=0, intborderType=BORDER_DEFAULT )
第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。它可以是单独的任意通


道数的图片，但需要注意，图片深度应该为CV_8U,CV_16U, CV_16S, CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。比如可以用


Mat::Clone，以源图片为模板，来初始化得到如假包换的目标图。
第三个参数，Size类型的ksize高斯内核的大小。其中ksize.width和ksize.height可以不同，但他们都必


须为正数和奇数。或者，它们可以是零的，它们都是由sigma计算而来。
第四个参数，double类型的sigmaX，表示高斯核函数在X方向的的标准偏差。
第五个参数，double类型的sigmaY，表示高斯核函数在Y方向的的标准偏差。若sigmaY为零，就将它设为


sigmaX，如果sigmaX和sigmaY都是0，那么就由ksize.width和ksize.height计算出来。
为了结果的正确性着想，最好是把第三个参数Size，第四个参数sigmaX和第五个参数sigmaY全部指定到。
第六个参数， int类型的borderType，用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT


，我们一般不去管它。
 嗯，第一部分的理论介绍大概就是这些了，我们接着进入第二部分，OpenCV的源码剖析。


二、深入——OpenCV源码剖析


上一篇文章中我们已经和当前最新版的OpenCV的源码亲密接触过。 在这一部分中，浅墨将带领大家领略


OpenCV的开源魅力，对OpenCV中本篇文章里讲解到线性滤波函数——boxFilter，blur和GaussianBlur函


数以及周边的涉及到的源码进行适当的剖析。


这样，我们就可以对 OpenCV有一个更加深刻的理解，成为一个高端大气的OpenCV使用者。


<1>OpenCV中boxFilter函数源码解析


我们可以在OpenCV的安装路径的\sources\modules\imgproc\src下的smooth.cpp源文件的第711行找到


boxFilter函数的源代码。对应于浅墨将OpenCV 2.4.8安装在D:\Program Files\opencv的路径下，那么，


smooth.cpp文件就在 D:\ProgramFiles\opencv\sources\modules\imgproc\src路径下。


//-----------------------------------【boxFilter（）函数中文注释版源代


码】----------------------------
// 代码作用：进行box Filter滤波操作的函数
// 说明：以下代码为来自于计算机开源视觉库OpenCV的官方源代码
// OpenCV源代码版本：2.4.8
// 源码路径：…\opencv\sources\modules\imgproc\src\smooth.cpp
// 源文件中如下代码的起始行数：711行
// 中文注释by浅墨
//------------------------------------------------------------------------------------------


--------------
void cv::boxFilter( InputArray _src,OutputArray _dst, int ddepth,
        Size ksize, Point anchor,
        bool normalize, int borderType)
{
   Mat src = _src.getMat();//拷贝源图的形参Mat数据到临时变量，用于稍后的操作
int sdepth =src.depth(), cn = src.channels();//定义int型临时变量，代表源图深度的sdepth，源图


通道的引用cn
 
//处理ddepth小于零的情况
   if( ddepth < 0 )
     ddepth = sdepth;
   _dst.create( src.size(), CV_MAKETYPE(ddepth, cn) );//初始化目标图
Mat dst =_dst.getMat();//拷贝目标图的形参Mat数据到临时变量，用于稍后的操作
 
//处理 borderType不为 BORDER_CONSTANT 且normalize为真的情况
   if( borderType != BORDER_CONSTANT && normalize ){
     if( src.rows == 1 )
       ksize.height = 1;
     if( src.cols == 1 )
       ksize.width = 1;
}
 
//若之前有过HAVE_TEGRA_OPTIMIZATION优化选项的定义，则执行宏体中的tegra优化版函数并返回
#ifdef HAVE_TEGRA_OPTIMIZATION
   if ( tegra::box(src, dst, ksize, anchor, normalize, borderType) )
     return;
#endif
 
     //调用FilterEngine滤波引擎，正式开始滤波操作
   Ptr<FilterEngine> f = createBoxFilter( src.type(), dst.type(),
            ksize, anchor,normalize, borderType );
   f->apply( src, dst );
}
其中的Ptr是用来动态分配的对象的智能指针模板类。可以发现，函数的内部代码思路是很清晰的，先拷


贝源图的形参Mat数据到临时变量，定义一些临时变量，在处理ddepth小于零的情况，接着处理 


borderType不为 BORDER_CONSTANT 且normalize为真的情况，最终调用FilterEngine滤波引擎创建一个


BoxFilter，正式开始滤波操作。


这里的FilterEngine是OpenCV图像滤波功能的核心引擎，我们有必要详细剖析看其源代码。


<2>FilterEngine 类解析——OpenCV图像滤波核心引擎


FilterEngine类是OpenCV关于图像滤波的主力军类，OpenCV图像滤波功能的核心引擎。各种滤波函数比如


blur， GaussianBlur，到头来其实是就是在函数末尾处定义了一个Ptr<FilterEngine>类型的f，然后f-


>apply( src, dst )了一下而已。


这个类可以把几乎是所有的滤波操作施加到图像上。它包含了所有必要的中间缓存器。有很多和滤波相关


的create系函数的返回值直接就是Ptr<FilterEngine>。比如cv::createSeparableLinearFilter(),


 cv::createLinearFilter(),cv::createGaussianFilter(), cv::createDerivFilter(),


 cv::createBoxFilter() 和cv::createMorphologyFilter().，这里给出其中一个函数的原型吧：


Ptr<FilterEngine>createLinearFilter(int srcType, int dstType, InputArray kernel, 


Point_anchor=Point(-1,-1), double delta=0, int rowBorderType=BORDER_DEFAULT, 


intcolumnBorderType=-1, const Scalar& borderValue=Scalar() )
上面我们提到过了，其中的Ptr是用来动态分配的对象的智能指针模板类，而上面的尖括号里面的模板参


数就是FilterEngine。


使用FilterEngine类可以分块处理大量的图像，构建复杂的管线，其中就包含一些进行滤波阶段。如果我


们需要使用预先定义好的的滤波操作，cv::filter2D(), cv::erode(),以及cv::dilate()，可以选择，他


们不依赖于FilterEngine，自立自强，在自己函数体内部就实现了FilterEngine提供的功能。不像其他的


诸如我们今天讲的blur系列函数，依赖于FilterEngine引擎。


我们看下其类声明经过浅墨详细注释的源码： 


//-----------------------------------【FilterEngine类中文注释版源代


码】----------------------------
//  代码作用：FilterEngine类，OpenCV图像滤波功能的核心引擎
//  说明：以下代码为来自于计算机开源视觉库OpenCV的官方源代码
//  OpenCV源代码版本：2.4.8
//  源码路径：…\opencv\sources\modules\imgproc\include\opencv2\imgproc\imgproc.hpp
//  源文件中如下代码的起始行数：222行
//  中文注释by浅墨
//------------------------------------------------------------------------------------------


--------------
 
class CV_EXPORTS FilterEngine
{
public:
  //默认构造函数
  FilterEngine();
  //完整的构造函数。 _filter2D 、_rowFilter 和 _columnFilter之一，必须为非空
  FilterEngine(const Ptr<BaseFilter>& _filter2D,
            constPtr<BaseRowFilter>& _rowFilter,
           constPtr<BaseColumnFilter>& _columnFilter,
            int srcType, int dstType, intbufType,
            int_rowBorderType=BORDER_REPLICATE,
            int _columnBorderType=-1,
            const Scalar&_borderValue=Scalar());
  //默认析构函数
  virtual ~FilterEngine();
  //重新初始化引擎。释放之前滤波器申请的内存。
  void init(const Ptr<BaseFilter>& _filter2D,
          constPtr<BaseRowFilter>& _rowFilter,
          constPtr<BaseColumnFilter>& _columnFilter,
          int srcType, int dstType, intbufType,
          int_rowBorderType=BORDER_REPLICATE, int _columnBorderType=-1,
          const Scalar&_borderValue=Scalar());
  //开始对指定了ROI区域和尺寸的图片进行滤波操作
  virtual int start(Size wholeSize, Rect roi, int maxBufRows=-1);
   //开始对指定了ROI区域的图片进行滤波操作
  virtual int start(const Mat& src, const Rect&srcRoi=Rect(0,0,-1,-1),
               bool isolated=false, intmaxBufRows=-1);
  //处理图像的下一个srcCount行（函数的第三个参数）
  virtual int proceed(const uchar* src, int srcStep, int srcCount,
                uchar* dst, intdstStep);
  //对图像指定的ROI区域进行滤波操作，若srcRoi=(0,0,-1,-1)，则对整个图像进行滤波操作
  virtual void apply( const Mat& src, Mat& dst,
                const Rect&srcRoi=Rect(0,0,-1,-1),
                Point dstOfs=Point(0,0),
                bool isolated=false);
 
  //如果滤波器可分离，则返回true
boolisSeparable() const { return (const BaseFilter*)filter2D == 0; }
 
  //返回输入和输出行数
  int remainingInputRows() const;
intremainingOutputRows() const;
 
  //一些成员参数定义
  int srcType, dstType, bufType;
  Size ksize;
  Point anchor;
  int maxWidth;
  Size wholeSize;
  Rect roi;
  int dx1, dx2;
  int rowBorderType, columnBorderType;
  vector<int> borderTab;
  int borderElemSize;
  vector<uchar> ringBuf;
  vector<uchar> srcRow;
  vector<uchar> constBorderValue;
  vector<uchar> constBorderRow;
  int bufStep, startY, startY0, endY, rowCount, dstY;
  vector<uchar*> rows;
 
  Ptr<BaseFilter> filter2D;
  Ptr<BaseRowFilter> rowFilter;
  Ptr<BaseColumnFilter> columnFilter;
};
<3>OpenCV中size类型剖析


size类型我们也讲一下， 通过转到定义，我们可以在……\opencv\sources\modules\core\include


\opencv2\core\core.hpp路径下，对应于浅墨的OpenCV安装路径，就是在


D:\ProgramFiles\opencv\sources\modules\core\include\opencv2\core\core.hpp下，找到其原型声明


：


typedef Size_<int> Size2i;
typedef Size2i Size;
Size_ 是个模板类，在这里Size_<int>表示其类体内部的模板所代表的类型为int。


那这两句的意思，就是首先给已知的数据类型Size_<int>起个新名字，叫Size2i。


然后又给已知的数据类型Size2i起个新名字，叫Size。


所以，连起来就是，Size_<int>、Size2i、Size这三个类型名等价。


然后我们追根溯源，找到Size_模板类的定义：


//-----------------------------------【Size_类中文注释版源代码】----------------------------
//  代码作用：作为尺寸相关数据结构的Size_ 模板类
//  说明：以下代码为来自于计算机开源视觉库OpenCV的官方源代码
//  OpenCV源代码版本：2.4.8
//  源码路径：…\opencv\sources\modules\core\include\opencv2\core\core.hpp
//  源文件中如下代码的起始行数：816行
//  中文注释by浅墨
//------------------------------------------------------------------------------------------


--------------
template<typename _Tp> class Size_
{
public:
  typedef _Tp value_type;
 
  //不同的构造函数定义
  Size_();
  Size_(_Tp _width, _Tp _height);
  Size_(const Size_& sz);
  Size_(const CvSize& sz);
  Size_(const CvSize2D32f& sz);
  Size_(const Point_<_Tp>& pt);
 
  Size_& operator = (const Size_& sz);
  //区域(width*height)
  _Tp area() const;
 
  //转化另一种数据类型。
   template<typename_Tp2> operator Size_<_Tp2>() const;
 
  //转换为旧式的OpenCV类型.
  operator CvSize() const;
  operator CvSize2D32f() const;
 
  _Tp width, height; //宽度和高度，常用属性
};


可以看到Size_模板类的内部又是重载了一些构造函数以满足我们的需要，其中，我们用得最多的是如下


这个构造函数：


Size_(_Tp _width, _Tp _height);
另外，代码末尾定义了模板类型的宽度和高度：


_Tp width, height; //宽度和高度
于是我们可以用XXX. width和XXX.height来分别表示其宽度和高度。


给大家一个示例，方便理解：


Size(5, 5)；//构造出的Size宽度和高度都为5，即XXX.width和XXX.height都为5
<4>OpenCV中blur函数源码剖析


我们可以在OpenCV的安装路径的\sources\modules\imgproc\src下的smooth.cpp源文件中找到blur的源代


码。对应于浅墨将OpenCV 2.4.8安装在D:\Program Files\opencv下，那么，smooth.cpp文件就在 D:


\ProgramFiles\opencv\sources\modules\imgproc\src路径下。


一起来看一下OpenCV中blur函数定义的真面目：


//-----------------------------------【blur（）函数中文注释版源代


码】----------------------------
//     代码作用：进行blur均值滤波操作的函数
//     说明：以下代码为来自于计算机开源视觉库OpenCV的官方源代码
//     OpenCV源代码版本：2.4.8
//     源码路径：…\opencv\sources\modules\imgproc\src\smooth.cpp
//     源文件中如下代码的起始行数：738行
//     中文注释by浅墨
//------------------------------------------------------------------------------------------


--------------
 
void cv::blur(InputArray src, OutputArray dst,
          Size ksize, Point anchor, int borderType )
{
//调用boxFilter函数进行处理
   boxFilter( src, dst, -1, ksize, anchor, true, borderType );
}
可以看到在blur函数内部就是调用了一个boxFilter函数，且第六个参数为true，即我们上文所说的


normalize=true，即均值滤波是均一化后的方框滤波。


<5>OpenCV中GaussianBlur函数源码剖析


最后，我们看一下OpenCV中GaussianBlur函数源代码：


//-----------------------------------【GaussianBlur（）函数中文注释版源代


码】-----------------------
//  代码作用：封装高斯滤波的GaussianBlur（）函数
//  说明：以下代码为来自于计算机开源视觉库OpenCV的官方源代码
//  OpenCV源代码版本：2.4.8
//  源码路径：…\opencv\sources\modules\imgproc\src\smooth.cpp
//  源文件中如下代码的起始行数：832行
//  中文注释by浅墨
//------------------------------------------------------------------------------------------


--------------
 
void cv::GaussianBlur( InputArray _src,OutputArray _dst, Size ksize,
             double sigma1, doublesigma2,
             int borderType )
{
//拷贝形参Mat数据到临时变量，用于稍后的操作
  Mat src = _src.getMat();
  _dst.create( src.size(), src.type() );
Mat dst =_dst.getMat();
 
//处理边界选项不为BORDER_CONSTANT时的情况
  if( borderType != BORDER_CONSTANT )
   {
     if( src.rows == 1 )
        ksize.height = 1;
     if( src.cols == 1 )
        ksize.width = 1;
}
 
     //若ksize长宽都为1，将源图拷贝给目标图
  if( ksize.width == 1 && ksize.height == 1 )
   {
     src.copyTo(dst);
     return;
}
 
//若之前有过HAVE_TEGRA_OPTIMIZATION优化选项的定义，则执行宏体中的tegra优化版函数并返回
#ifdef HAVE_TEGRA_OPTIMIZATION
  if(sigma1 == 0 && sigma2 == 0 && tegra::gaussian(src,dst, ksize, borderType))
     return;
#endif
 
//如果HAVE_IPP&& (IPP_VERSION_MAJOR >= 7为真，则执行宏体中语句
#if defined HAVE_IPP &&(IPP_VERSION_MAJOR >= 7)
  if(src.type() == CV_32FC1 && sigma1 == sigma2 &&ksize.width == ksize.height && sigma1 != 


0.0 )
   {
     IppiSize roi = {src.cols, src.rows};
     int bufSize = 0;
     ippiFilterGaussGetBufferSize_32f_C1R(roi, ksize.width, &bufSize);
     AutoBuffer<uchar> buf(bufSize+128);
     if( ippiFilterGaussBorder_32f_C1R((const Ipp32f *)src.data,(int)src.step,
                            (Ipp32f *)dst.data, (int)dst.step,
                            roi,ksize.width, (Ipp32f)sigma1,
                            (IppiBorderType)borderType, 0.0,
                            alignPtr(&buf[0],32)) >= 0 )
        return;
   }
#endif
 
  //调动滤波引擎，正式进行高斯滤波操作
  Ptr<FilterEngine> f = createGaussianFilter( src.type(), ksize,sigma1, sigma2, borderType 


);
  f->apply( src, dst );
}
嗯，今天的源码解析就到这里吧，原理部分学完，深入OpenCV源码部分也学完，相信大家应该对OpenCV中


的线性滤波有了比较详细的认识，已经跃跃欲试想看这个几个函数用起来可以得出什么效果了。


三、浅出——线性滤波函数快速上手攻略


这一部分的内容就是为了大家能快速上手 boxFilter、blur和GaussianBlur 这三个函数所准备的。还等


什么呢，开始吧。


<1>boxFilter函数——方框滤波


boxFilter的函数作用是使用方框滤波（box filter）来模糊一张图片，由src输入，dst输出。


函数原型如下：


C++: void boxFilter(InputArray src,OutputArray dst, int ddepth, Size ksize, Point 


anchor=Point(-1,-1), boolnormalize=true, int borderType=BORDER_DEFAULT )
参数详解如下：


第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处


理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为CV_8U, CV_16U, CV_16S, 


CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。
第三个参数，int类型的ddepth，输出图像的深度，-1代表使用原图深度，即src.depth()。
第四个参数，Size类型的ksize，内核的大小。一般这样写Size( w,h )来表示内核的大小( 其中，w 为像


素宽度， h为像素高度)。Size（3,3）就表示3x3的核大小，Size（5,5）就表示5x5的核大小
第五个参数，Point类型的anchor，表示锚点（即被平滑的那个点），注意他有默认值Point(-1,-1)。如


果这个点坐标是负值的话，就表示取核的中心为锚点，所以默认值Point(-1,-1)表示这个锚点在核的中心


。
第六个参数，bool类型的normalize，默认值为true，一个标识符，表示内核是否被其区域归一化


（normalized）了。
第七个参数，int类型的borderType，用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT


，我们一般不去管它。
调用代码示范如下：


       //载入原图
       Matimage=imread("2.jpg");
       //进行均值滤波操作
       Matout;
       boxFilter(image, out, -1,Size(5, 5));


用上面三句核心代码架起来的完整程序代码：


//-----------------------------------【头文件包含部


分】---------------------------------------
//     描述：包含程序所依赖的头文件
//------------------------------------------------------------------------------------------


----
#include "opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
 
//-----------------------------------【命名空间声明部


分】---------------------------------------
//     描述：包含程序所使用的命名空间
//------------------------------------------------------------------------------------------


----- 
using namespace cv;
 
//-----------------------------------【main( )函


数】--------------------------------------------
//     描述：控制台应用程序的入口函数，我们的程序从这里开始
//------------------------------------------------------------------------------------------


-----
int main( )
{
  //载入原图
  Matimage=imread("2.jpg");
 
  //创建窗口
  namedWindow("均值滤波【原图】" );
  namedWindow("均值滤波【效果图】");
 
  //显示原图
  imshow("均值滤波【原图】", image );
 
  //进行滤波操作
  Matout;
  boxFilter(image, out, -1,Size(5, 5));
 
  //显示效果图
  imshow("均值滤波【效果图】" ,out );
 
  waitKey(0 );    
}
运行效果图（内核大小Size(5, 5)）：


  


<2>blur函数——均值滤波


blur的作用是对输入的图像src进行均值滤波后用dst输出。


函数原型如下：


C++: void blur(InputArray src, OutputArraydst, Size ksize, Point anchor=Point(-1,-1), int 


borderType=BORDER_DEFAULT )
参数详解如下：


第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。该函数对通道是独立处


理的，且可以处理任意通道数的图片，但需要注意，待处理的图片深度应该为CV_8U, CV_16U, CV_16S, 


CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。比如可以用


Mat::Clone，以源图片为模板，来初始化得到如假包换的目标图。
第三个参数，Size类型（对Size类型稍后有讲解）的ksize，内核的大小。一般这样写Size( w,h )来表示


内核的大小( 其中，w 为像素宽度， h为像素高度)。Size（3,3）就表示3x3的核大小，Size（5,5）就表


示5x5的核大小
第四个参数，Point类型的anchor，表示锚点（即被平滑的那个点），注意他有默认值Point(-1,-1)。如


果这个点坐标是负值的话，就表示取核的中心为锚点，所以默认值Point(-1,-1)表示这个锚点在核的中心


。
第五个参数，int类型的borderType，用于推断图像外部像素的某种边界模式。有默认值BORDER_DEFAULT


，我们一般不去管它。
调用代码示范：


       //载入原图
       Matimage=imread("1.jpg");
       //进行均值滤波操作
       Matout;
       blur(image, out, Size(7, 7));
为了大家的理解和应用方便，还是给出用上面三句核心代码架起来完整程序的代码：


//-----------------------------------【头文件包含部


分】---------------------------------------
//     描述：包含程序所依赖的头文件
//------------------------------------------------------------------------------------------


----
#include "opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
 
//-----------------------------------【命名空间声明部


分】---------------------------------------
//     描述：包含程序所使用的命名空间
//------------------------------------------------------------------------------------------


----- 
using namespace cv;
 
//-----------------------------------【main( )函


数】--------------------------------------------
//     描述：控制台应用程序的入口函数，我们的程序从这里开始
//------------------------------------------------------------------------------------------


-----
int main( )
{
  //载入原图
  Matimage=imread("1.jpg");
 
  //创建窗口
  namedWindow("均值滤波【原图】" );
  namedWindow("均值滤波【效果图】");
 
  //显示原图
  imshow("均值滤波【原图】", image );
 
  //进行滤波操作
  Matout;
  blur(image, out, Size(7, 7));
 
  //显示效果图
  imshow("均值滤波【效果图】" ,out );
 
  waitKey(0 );    
}
运行效果图（内核大小Size(7, 7)）：




<3>GaussianBlur函数——高斯滤波


GaussianBlur函数的作用是用高斯滤波器来模糊一张图片，对输入的图像src进行高斯滤波后用dst输出。


函数原型如下：


C++: void GaussianBlur(InputArray src,OutputArray dst, Size ksize, double sigmaX, double 


sigmaY=0, intborderType=BORDER_DEFAULT )
参数详解如下：


第一个参数，InputArray类型的src，输入图像，即源图像，填Mat类的对象即可。它可以是单独的任意通


道数的图片，但需要注意，图片深度应该为CV_8U,CV_16U, CV_16S, CV_32F 以及 CV_64F之一。
第二个参数，OutputArray类型的dst，即目标图像，需要和源图片有一样的尺寸和类型。比如可以用


Mat::Clone，以源图片为模板，来初始化得到如假包换的目标图。
第三个参数，Size类型的ksize高斯内核的大小。其中ksize.width和ksize.height可以不同，但他们都必


须为正数和奇数。或者，它们可以是零的，它们都是由sigma计算而来。
第四个参数，double类型的sigmaX，表示高斯核函数在X方向的的标准偏差。
第五个参数，double类型的sigmaY，表示高斯核函数在Y方向的的标准偏差。若sigmaY为零，就将它设为


sigmaX，如果sigmaX和sigmaY都是0，那么就由ksize.width和ksize.height计算出来。
为了结果的正确性着想，最好是把第三个参数Size，第四个参数sigmaX和第五个参数sigmaY全部指定到。
第六个参数，int类型的borderType，用于推断图像外部像素的某种边界模式。注意它有默认值


BORDER_DEFAULT。
 调用示例：


//载入原图
       Matimage=imread("1.jpg");
       //进行滤波操作
       Matout;
       blur(image, out, Size(5, 5));
用上面三句核心代码架起来的完整程序代码：


//-----------------------------------【头文件包含部


分】---------------------------------------
//     描述：包含程序所依赖的头文件
//------------------------------------------------------------------------------------------


----
#include "opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
 
//-----------------------------------【命名空间声明部


分】---------------------------------------
//     描述：包含程序所使用的命名空间
//------------------------------------------------------------------------------------------


----- 
using namespace cv;
 
//-----------------------------------【main( )函


数】--------------------------------------------
//     描述：控制台应用程序的入口函数，我们的程序从这里开始
//------------------------------------------------------------------------------------------


-----
int main( )
{
  //载入原图
  Matimage=imread("1.jpg");
 
  //创建窗口
  namedWindow("均值滤波【原图】" );
  namedWindow("均值滤波【效果图】");
 
  //显示原图
  imshow("均值滤波【原图】", image );
 
  //进行均值滤波操作
  Matout;
  GaussianBlur(image, out, Size( 3, 3 ), 0, 0 );
 
  //显示效果图
  imshow("均值滤波【效果图】" ,out );
 
  waitKey(0 );    
}
运行效果图（内核大小Size(5, 5)）：


 


四、图像线性滤波综合示例 程序


依然是每篇文章都会配给大家的一个详细注释的博文配套示例程序，把这篇文章中介绍的知识点以代码为


载体，展现给大家。


这个示例程序中可以用轨迹条来控制三种线性滤波的核参数值，通过滑动滚动条，就可以控制图像在三种


线性滤波下的模糊度，有一定的可玩性。废话不多说，上代码吧：


//-----------------------------------【程序说


明】----------------------------------------------
//     程序名称:：【OpenCV入门教程之八】线性滤波专场：方框滤波、均值滤波与高斯滤波 配


套源码
//     开发所用OpenCV版本：2.4.8
//     2014年3月31 日 Create by 浅墨
//------------------------------------------------------------------------------------------


------
 
//-----------------------------------【头文件包含部


分】---------------------------------------
//     描述：包含程序所依赖的头文件
//------------------------------------------------------------------------------------------


----
#include <opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
 
//-----------------------------------【命名空间声明部


分】---------------------------------------
//     描述：包含程序所使用的命名空间
//------------------------------------------------------------------------------------------


----- 
using namespace std;
using namespace cv;
 
 
//-----------------------------------【全局变量声明部


分】--------------------------------------
//     描述：全局变量声明
//------------------------------------------------------------------------------------------


-----
Matg_srcImage,g_dstImage1,g_dstImage2,g_dstImage3;//存储图片的Mat类型
int g_nBoxFilterValue=3;  //方框滤波参数值
int g_nMeanBlurValue=3;  //均值滤波参数值
int g_nGaussianBlurValue=3;  //高斯滤波参数值
 
 
//-----------------------------------【全局函数声明部


分】--------------------------------------
//     描述：全局函数声明
//------------------------------------------------------------------------------------------


-----
//四个轨迹条的回调函数
static void on_BoxFilter(int, void *);     //均值滤波
static void on_MeanBlur(int, void *);    //均值滤波
static void on_GaussianBlur(int, void *);      //高斯滤波
 
 
 
//-----------------------------------【main( )函


数】--------------------------------------------
//     描述：控制台应用程序的入口函数，我们的程序从这里开始
//------------------------------------------------------------------------------------------


-----
int main(  )
{
  //改变console字体颜色
  system("color5E"); 
 
  //载入原图
  g_srcImage= imread( "1.jpg", 1 );
  if(!g_srcImage.data ) { printf("Oh，no，读取srcImage错误~！\n"); return false; }
 
  //克隆原图到三个Mat类型中
  g_dstImage1= g_srcImage.clone( );
  g_dstImage2= g_srcImage.clone( );
  g_dstImage3= g_srcImage.clone( );
 
  //显示原图
  namedWindow("【<0>原图窗口】", 1);
  imshow("【<0>原图窗口】",g_srcImage);
 
 
  //=================【<1>方框滤波】==================
  //创建窗口
  namedWindow("【<1>方框滤波】", 1);
  //创建轨迹条
  createTrackbar("内核值：", "【<1>方框滤波】",&g_nBoxFilterValue, 40,on_BoxFilter );
  on_MeanBlur(g_nBoxFilterValue,0);
  imshow("【<1>方框滤波】", g_dstImage1);
  //================================================
 
  //=================【<2>均值滤波】==================
  //创建窗口
  namedWindow("【<2>均值滤波】", 1);
  //创建轨迹条
  createTrackbar("内核值：", "【<2>均值滤波】",&g_nMeanBlurValue, 40,on_MeanBlur );
  on_MeanBlur(g_nMeanBlurValue,0);
  //================================================
 
  //=================【<3>高斯滤波】=====================
  //创建窗口
  namedWindow("【<3>高斯滤波】", 1);
  //创建轨迹条
  createTrackbar("内核值：", "【<3>高斯滤波】",&g_nGaussianBlurValue, 40,on_GaussianBlur );
  on_GaussianBlur(g_nGaussianBlurValue,0);
  //================================================
 
 
  //输出一些帮助信息
  cout<<endl<<"\t嗯。好了，请调整滚动条观察图像效果~\n\n"
    <<"\t按下“q”键时，程序退出~!\n"
    <<"\n\n\t\t\t\tby浅墨";
 
  //按下“q”键时，程序退出
  while(char(waitKey(1))!= 'q') {}
 
  return0;
}
 
 
//-----------------------------【on_BoxFilter( )函数】------------------------------------
//     描述：方框滤波操作的回调函数
//------------------------------------------------------------------------------------------


-----
static void on_BoxFilter(int, void *)
{
  //方框滤波操作
  boxFilter(g_srcImage, g_dstImage1, -1,Size( g_nBoxFilterValue+1, g_nBoxFilterValue+1));
  //显示窗口
  imshow("【<1>方框滤波】", g_dstImage1);
}
 
 
//-----------------------------【on_MeanBlur( )函数】------------------------------------
//     描述：均值滤波操作的回调函数
//------------------------------------------------------------------------------------------


-----
static void on_MeanBlur(int, void *)
{
  //均值滤波操作
  blur(g_srcImage, g_dstImage2, Size( g_nMeanBlurValue+1, g_nMeanBlurValue+1),Point(-1,-1));
  //显示窗口
  imshow("【<2>均值滤波】", g_dstImage2);
}
 
 
//-----------------------------【ContrastAndBright( )函


数】------------------------------------
//     描述：高斯滤波操作的回调函数
//------------------------------------------------------------------------------------------


-----
static void on_GaussianBlur(int, void *)
{
  //高斯滤波操作
  GaussianBlur(g_srcImage, g_dstImage3, Size( 


g_nGaussianBlurValue*2+1,g_nGaussianBlurValue*2+1 ), 0, 0);
  //显示窗口
  imshow("【<3>高斯滤波】", g_dstImage3);
}
最后是一些运行截图，原图：
  
 方框滤波：
 
均值滤波：
 
高斯滤波：
 
========
OpenCV基础篇之图像的DFT频域变换

程序及分析


/*
 * FileName : fft2.cpp
 * Author   : xiahouzuoxin @163.com
 * Version  : v1.0
 * Date     : Wed 30 Jul 2014 09:42:12 PM CST
 * Brief    : 
 * 
 * Copyright (C) MICL,USTB
 */


#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "imgproc/imgproc.hpp"


using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    if (argc < 2) {
        cout<<"Usage:./fft2 [image name]"<<endl;
        return -1;
    }


    // Read as grayscale image
    Mat image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    if (!image.data) {
        cout << "Read image error"<<endl;
        return -1;
    }


    Mat padded;
    int m = getOptimalDFTSize(image.rows);  // Return size of 2^x that suite for FFT
    int n = getOptimalDFTSize(image.cols);
    // Padding 0, result is @padded
    copyMakeBorder(image, padded, 0, m-image.rows, 0, n-image.cols, BORDER_CONSTANT, 


Scalar::all(0));


    // Create planes to storage REAL part and IMAGE part, IMAGE part init are 0
    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);


    dft(complexI, complexI);


    // compute the magnitude and switch to logarithmic scale
    split(complexI, planes);
    magnitude(planes[0], planes[0], planes[1]);
    Mat magI = planes[0];


    // => log(1+sqrt(Re(DFT(I))^2+Im(DFT(I))^2))
    magI += Scalar::all(1);
    log(magI, magI);


    // crop the spectrum
    magI = magI(Rect(0, 0, magI.cols & (-2), magI.rows & (-2)));
    Mat _magI = magI.clone();
    normalize(_magI, _magI, 0, 1, CV_MINMAX);


    // rearrange the quadrants of Fourier image so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;


    Mat q0(magI, Rect(0,0,cx,cy));    // Top-Left
    Mat q1(magI, Rect(cx,0,cx,cy));   // Top-Right
    Mat q2(magI, Rect(0,cy,cx,cy));   // Bottom-Left
    Mat q3(magI, Rect(cx,cy,cx,cy));  // Bottom-Right


    // exchange Top-Left and Bottom-Right
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);


    // exchange Top-Right and Bottom-Left
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);


    normalize(magI, magI, 0, 1, CV_MINMAX);


    imshow("Input image", image);
    imshow("Spectrum magnitude before shift frequency", _magI);
    imshow("Spectrum magnitude after shift frequency", magI);
    waitKey();


    return 0;
}
本程序的作用是：将图像从空间域转换到频率域，并绘制频域图像。


二维图像的DFT（离散傅里叶变换），


图像的频域表示的是什么含义呢？又有什么用途呢？图像的频率是表征图像中灰度变化剧烈程度的指标，


是灰度在平面空间上的梯度。图像的边缘部分是突变部分，变化较快，因此反应在频域上是高频分量；图


像的噪声大部分情况下是高频部分；图像大部分平缓的灰度变化部分则为低频分量。也就是说，傅立叶变


换提供另外一个角度来观察图像，可以将图像从灰度分布转化到频率分布上来观察图像的特征。


频域在图像处理中，就我所知的用途主要在两方面：图像压缩和图像去噪。关于这两点将在下面给出图片


DFT的变换结果后说明。


有关DFT的更多性质请参考胡广书教授的《数字信号处理》教材。


请注意读图片的函数与之前有所不同：


Mat image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
CV_LOAD_IMAGE_GRAYSCALE参数表示将原图像转换为灰度图后读入，这是因为后面的DFT变换都是基于二维


信号的，而彩色图像是三维信号。当然，也可以对RGB每一通道都进行DFT运算。


DFT算法的原理要求输入信号的长度最好为2^n，这样可以使用快速傅里叶变换算法（FFT算法）进行加速


。所以程序中使用


copyMakeBorder(image, padded, 0, m-image.rows, 0, n-image.cols, BORDER_CONSTANT, 


Scalar::all(0));
填充0使横纵长度都为2^n。


对于一维信号，原DFT直接运算的复杂度是O(N^2)，而快速傅里叶变换的复杂度降低到O(Nlog2(N))，假设


N为512，足足提高了512/9≈57倍。


由DFT的性质知，输入为实信号（图像）的时候，频域输出为复数，因此将频域信息分为幅值和相位。频


域的幅值高的代表高频分量，幅值低的地方代表低频分量，因此程序中使用


// => log(1+sqrt(Re(DFT(I))^2+Im(DFT(I))^2))
magI += Scalar::all(1);
log(magI, magI);


// crop the spectrum
magI = magI(Rect(0, 0, magI.cols & (-2), magI.rows & (-2)));
Mat _magI = magI.clone();
normalize(_magI, _magI, 0, 1, CV_MINMAX);
进行log幅值计算及归一化幅值（归一化目的主要是方便将频域通过图像的形式进行显示）。


关于频域中心平移：将图像的高频分量平移到图像的中心，便于观测。


int cx = magI.cols/2;
int cy = magI.rows/2;


Mat q0(magI, Rect(0,0,cx,cy));    // Top-Left
Mat q1(magI, Rect(cx,0,cx,cy));   // Top-Right
Mat q2(magI, Rect(0,cy,cx,cy));   // Bottom-Left
Mat q3(magI, Rect(cx,cy,cx,cy));  // Bottom-Right


// exchange Top-Left and Bottom-Right
Mat tmp;
q0.copyTo(tmp);
q3.copyTo(q0);
tmp.copyTo(q3);


// exchange Top-Right and Bottom-Left
q1.copyTo(tmp);
q2.copyTo(q1);
tmp.copyTo(q2);
其原理就是将左上角的频域和右下角的互换，右上角和左下角互换。


请注意：频域点和空域点的坐标没有一一对应的关系，两者的关系只是上面的DFT公式所见到的。


本程序因为使用到图像处理相关的函数，所以包含了头文件imgproc/imgproc.hpp，该文件位于opencv安


装目录的include/opencv2/目录下，在编写Makefile时也要增加相关的头文件路径和库，本程序使用的


Makefile如下：


TARG=fft2
SRC=fft2.cpp
LIB=-L/usr/local/lib/
INC=-I/usr/local/include/opencv/ -I/usr/local/include/opencv2
CFLAGS=


$(TARG):$(SRC)
  g++ -g -o $@ ${CFLAGS} $(LIB) $(INC) \
      -lopencv_core -lopencv_highgui -lopencv_imgproc \
      $^


.PHONY:clean


clean:
  -rm $(TARG) tags -f
其中Makefile中的\表示换行（反斜杠后不能再有任何字符，包括空格），如上库增加了-


lopencv_imgproc，头文件路径增加了-I/usr/local/include/opencv2。


效果


dft


上图从左到右分别是：原始灰度图、频域平移前的频域图像、频域中心平移后的频域图像。


提到图像频域变换的用途：压缩和去噪。压缩的原理就是在频域中，大部分频域的值为0（或接近0，可以


进行有损压缩，如jpeg图像），只要压缩频域中的少数非0值即可达到图片压缩的目的。去噪则是通过频


域的滤波实现，因为噪声大部分情况下体现为高频信号，使用低通滤波器即可滤除高频噪声（当然，也会


带来损失，那就是边缘会变得模糊（之前说过，边缘也是高频信号））。


========
一个老话题，图像去噪

请教，想通过频域滤波器来滤除噪声，为此想对噪声图像进行统计特性的分析与获取，再服务到滤波器上


。如何完成服务于滤波器的噪声图像的统计特性的分析与获取？对滤波器的设计如何考虑？谢谢，


图像去噪是图像处理的一个基本问题，真的老话题，恐怕不是一两句话就可以讲清楚的。图像处理方面的


书都有介绍。


应用频域滤波器滤除噪声的先决条件是：噪声频谱与图像频谱可以差别开，如用低通滤波器滤除高频噪声


，用高通滤波器滤除低频噪声。
噪声可以分为有限的几种：椒盐噪声、高斯噪声、泊松噪声、1/f噪声、纹理噪声等等。对不同类型的噪


声有不同的滤波器来处理才能获得较好的性能。实际噪声也可能是上述几种噪声的组合。
设计图像噪声滤波器的步骤：
（1）确定噪声的类型：观察或实验
（2）设计、改进滤波器 可用峰值信噪比PSNR等来定量对比，也可结合人眼观察的主观效果




我不知道如何能：
（1）确定噪声的类型？经验？
        即便认为是高斯噪声，但我已经用相应的均值滤波（matlab）效果并不好？
（2）设计、改进滤波器：我不会设计滤波器，因为我不知道应该获取噪声图像的哪些统计特性，才能设


计它？


我想获得更多的细节，


还有，一般的图像的噪声频谱与图像频谱应该是可以差别吧。我认为只是效果的问题，难道不是吗？


我感觉去噪这个问题根据情况而定，就那么多方法，你可以根据你自己的情况找到一种适合的方法


是的，我查了许多文献，方法是很多，但在实际应用中效果并没有常用案例图（加入常见特定某种或某几


种噪声）的去噪效果好。目前我处理的的红外摄取的图像中如果噪声处理不好，会影响后期效果的处理（


分维计算）。
最近半个月我接触使用了OPENCV，它让新手上手很快，对图像元素处理也很方便。


========