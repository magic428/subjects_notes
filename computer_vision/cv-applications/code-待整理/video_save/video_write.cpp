#include <opencv2/highgui/highgui.hpp>    
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/core/core.hpp>   
#include <iostream>
  
using namespace cv;  
using namespace std;  
  
int main(int argc,char *argv[])    
{    
    VideoCapture videoInput(argv[1]);  
    if(!videoInput.isOpened()) {  
        return -1;  
        cout << "open camera failed!" << endl;
    }  

    float fpsInput=24; //获取帧率  
    float pauseInput=1000/fpsInput;  //设置帧间隔  
    Mat frame;  
    Size videoSize=Size(videoInput.get(CV_CAP_PROP_FRAME_WIDTH)/2,videoInput.get(CV_CAP_PROP_FRAME_HEIGHT)/2);  
    videoSize=Size(videoInput.get(CV_CAP_PROP_FRAME_WIDTH)/2,videoInput.get(CV_CAP_PROP_FRAME_HEIGHT)/2);  
    std::cout << videoSize << std::endl;
  
    // string videoPath1="videoRecordPIM1.avi";  
    // int fourcc1=CV_FOURCC('P','I','M','1');  
    // VideoWriter videoOutput1(videoPath1,fourcc1,fpsInput,videoSize,true);  
  
    // string videoPath2="videoRecordMJPG.avi";  
    // int fourcc2=CV_FOURCC('M','J','P','G');  
    // VideoWriter videoOutput2(videoPath2,fourcc2,fpsInput,videoSize,true);  
  
    string videoPath3="videoRecordMP42.avi";  
    // int fourcc3=CV_FOURCC('M', 'P', '4', '2');  
    int fourcc3=CV_FOURCC('D', 'I', 'V', 'X');  
    VideoWriter videoOutput3(videoPath3,fourcc3,fpsInput,videoSize,true);  
  
    // string videoPath4="videoRecordDIV3.avi";  
    // int fourcc4=CV_FOURCC('D', 'I', 'V', '3');  
    // VideoWriter videoOutput4(videoPath4,fourcc4,fpsInput,videoSize,true);  
  
    // string videoPath5="videoRecordDIVX.avi";  
    // int fourcc5=CV_FOURCC('D', 'I', 'V', 'X');  
    // VideoWriter videoOutput5(videoPath5,fourcc5,fpsInput,videoSize,true);     
  
    // string videoPath8="videoRecordFLV1.avi";  
    // int fourcc8=CV_FOURCC('F', 'L', 'V', '1');  
    // VideoWriter videoOutput8(videoPath8,fourcc8,fpsInput,videoSize,true);  
  
    // if(!videoOutput1.isOpened()) {  
    //     return -1;  
    // }  
    // if(!videoOutput2.isOpened()) {  
    //     return -1;  
    // }  
    if(!videoOutput3.isOpened()) {  
        return -1;  
    }  
    // if(!videoOutput4.isOpened()) {  
    //     return -1;  
    // }  
    // if(!videoOutput5.isOpened()) {  
    //     return -1;  
    // }  
      
    // if(!videoOutput8.isOpened()) {  
    //     return -1;  
    // }  
  
    while(true) {  
        videoInput >> frame;  
        // videoOutput3 << frame;  
        videoOutput3.write(frame.clone());

        imshow("Video", frame);  
        if(frame.empty() || waitKey(10)==27) {  
            break;  
        }  

        // videoOutput1<<frame;  
        // videoOutput2<<frame;  
        // videoOutput4<<frame;  
        // videoOutput5<<frame;  
        // videoOutput8<<frame;  
        // waitKey();  
    }  
    videoInput.release();
    videoOutput3.release();

    return 0;    
}  