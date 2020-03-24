#ifndef _STREAM_FETCH_H__
#define _STREAM_FETCH_H__
/**
 * rtsp video stream prefetch thread
 * 
 * Last updated: 2019-10-09
 * Author: Zhijie Guo. 
 */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <map>
#include <iostream>
#include <stdio.h>

extern "C"
{
    #include "libavcodec/avcodec.h"
    #include "libavformat/avformat.h"
    #include "libavutil/pixfmt.h"
    #include "libswscale/swscale.h"
        
    #include <libavutil/mathematics.h>
    #include <libavutil/time.h>
}


namespace tdmc
{
    using namespace std;
    
    class Stream_Fetch {
    public:
        Stream_Fetch() {}
        Stream_Fetch::Stream_Fetch(const std::string & end_point);
        ~Stream_Fetch() {} 

        void do_task();
        void fetch_frames();
        bool get_one_frame(cv::Mat & frame);
        size_t frame_size() { return frame_buffer_.size();  }

        void clear_queue() {
            queue<cv::Mat> empty;
            swap(empty, frame_buffer_);
        }


    private:
        std::queue<cv::Mat> frame_buffer_;
        std::string rtsp_end_point_;
        bool demo_done_;
        cv::VideoCapture cap_;

        std::thread t_;
        std::mutex m_;
    };
    
} // namespace tdmc


#endif