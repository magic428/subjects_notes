#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

Mat image,imageHSV,imageHist,imageNorm,imagecalcBack;
Mat imagegirl,imagegirlHSV;

int histSize=1;
float histR[]={0,255};
const float *histRange=histR;

int channels[]={0,1};

void TrackBarFun(int ,void(*));

int main(int argc,char *argv[])
{
	image=imread(argv[1]);
	imagegirl=imread(argv[2]);
	cvtColor(imagegirl,imagegirlHSV,CV_RGB2HSV);
	if(!image.data)
	{
		return -1;
	}
	cvtColor(image,imageHSV,CV_RGB2HSV);
	namedWindow("HSV");
	createTrackbar("bins控制", "HSV", &histSize, 100,TrackBarFun);	
	TrackBarFun(0,0);
	waitKey();
}
void TrackBarFun(int histSize,void(*))
{
	if(histSize==0)
	{
		histSize=1;
	}
	calcHist(&imageHSV,2,channels,Mat(),imageHist,1,&histSize,&histRange,true,false);
	normalize(imageHist,imageNorm,0,255,NORM_MINMAX,-1,Mat());
	calcBackProject(&imagegirlHSV,2,channels,imageNorm,imagecalcBack,&histRange,1,true);	
	imshow("Source",imagegirl);
	imshow("HSV",imagegirlHSV);
	imshow("CalcBack",imagecalcBack);
}