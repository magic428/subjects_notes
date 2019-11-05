/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 4 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
using namespace std;

// #include "cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "histogram.h"

bool IsDimodal(double* HistGram)       // 检测直方图是否为双峰的
{
	// 对直方图的峰进行计数，只有峰数位2才为双峰 
	int Count = 0;
	for (int Y = 1; Y < 255; Y++)
	{
		if (HistGram[Y - 1] < HistGram[Y] && HistGram[Y + 1] < HistGram[Y])
		{
			Count++;
			if (Count > 2) return false;
		}
	}
	if (Count == 2)
		return true;
	else
		return false;
}

int GetIntermodesThreshold(int* HistGram)
{
	int Y, Iter = 0, Index;
	double* HistGramC = new double[256];           // 基于精度问题，一定要用浮点数来处理，否则得不到正确的结果
	double* HistGramCC = new double[256];          // 求均值的过程会破坏前面的数据，因此需要两份数据
	for (Y = 0; Y < 256; Y++)
	{
		HistGramC[Y] = HistGram[Y];
		HistGramCC[Y] = HistGram[Y];
	}
	// 通过三点求均值来平滑直方图
	while (IsDimodal(HistGramCC) == false)                                                  // 判断是否已经是双峰的图像了      
	{
		HistGramCC[0] = (HistGramC[0] + HistGramC[0] + HistGramC[1]) / 3;                   // 第一点
		for (Y = 1; Y < 255; Y++)
			HistGramCC[Y] = (HistGramC[Y - 1] + HistGramC[Y] + HistGramC[Y + 1]) / 3;       // 中间的点
		HistGramCC[255] = (HistGramC[254] + HistGramC[255] + HistGramC[255]) / 3;           // 最后一点
		memcpy(HistGramCC, HistGramC, 256 * sizeof(double));         // 备份数据，为下一次迭代做准备
		Iter++;
		if (Iter >= 10000) return -1;                                                       // 似乎直方图无法平滑为双峰的，返回错误代码
	}
// 阈值为两峰值的平均值
	int* Peak = new int[2];
	for (Y = 1, Index = 0; Y < 255; Y++)
		if (HistGramCC[Y - 1] < HistGramCC[Y] && HistGramCC[Y + 1] < HistGramCC[Y]) Peak[Index++] = Y - 1;
	return ((Peak[0] + Peak[1]) / 2);
}



int GetIterativeBestThreshold(int* HistGram)
{
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


int main()
{
	// Read input image
	cv::Mat image= cv::imread("../0012_ARC.png",0);
	if (!image.data)
		return 0; 

    // Display the image
	cv::namedWindow("Image");
	cv::imshow("Image",image);

	// The histogram object
	Histogram1D h;

    // Compute the histogram
	cv::MatND histo= h.getHistogram(image);
	int thresh = h.GetThreshold(histo);

	// Loop over each bin
	for (int i=0; i<256; i++) 
		cout << "Value " << i << " = " << histo.at<float>(i) << endl;  

	// Display a histogram as an image
	cv::namedWindow("Histogram");
	cv::imshow("Histogram",h.getHistogramImage(image));

	// creating a binary image by thresholding at the valley
	cv::Mat thresholded;
	cout << "thresh = " << thresh/255.0 << endl;  
	cv::threshold(image,thresholded,thresh,255,cv::THRESH_BINARY);
 
	// Display the thresholded image
	cv::namedWindow("Binary Image");
	cv::imshow("Binary Image",thresholded);
	cv::imwrite("binary.bmp",thresholded);

#if 0
	// Equalize the image
	cv::Mat eq= h.equalize(image);

	// Show the result
	cv::namedWindow("Equalized Image");
	cv::imshow("Equalized Image",eq);

	// Show the new histogram
	cv::namedWindow("Equalized Histogram");
	cv::imshow("Equalized Histogram",h.getHistogramImage(eq));

	// Stretch the image ignoring bins with less than 5 pixels
	cv::Mat str= h.stretch(image,5);

	// Show the result
	cv::namedWindow("Stretched Image");
	cv::imshow("Stretched Image",str);

	// Show the new histogram
	cv::namedWindow("Stretched Histogram");
	cv::imshow("Stretched Histogram",h.getHistogramImage(str));

	// Create an image inversion table
	// uchar lookup[256];
	
	// for (int i=0; i<256; i++) {
		
	// 	lookup[i]= 255-i;
	// }

	// // Apply lookup and display negative image
	// cv::namedWindow("Negative image");
	// cv::imshow("Negative image",h.applyLookUp(image, lookup));
#endif
	cv::waitKey();
	return 0;
}

