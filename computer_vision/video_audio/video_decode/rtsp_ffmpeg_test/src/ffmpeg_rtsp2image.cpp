/**
 * Convert rtsp video stream to cv::Mat image
 * 
 * Last updated: 2019-10-10
 * Author: Zhijie Guo. 
 */
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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <getopt.h>

using namespace cv;
using namespace std;

#define USAGE   "rtsp2img -i <rtsp url> -n <number of frames you want to save>"
#define OPTS    "i:n:h"

static void print_usage()
{
    printf("Usage: %s\n", USAGE);
    return;
}

int main(int argc, char **argv)
{
	AVFormatContext *pFormatCtx;
	AVCodecContext *pCodecCtx;
	AVCodec *pCodec;
	AVFrame *pFrame, *pFrameRGB;
	AVPacket *packet;
	uint8_t *out_buffer;

	static struct SwsContext *img_convert_ctx;

	/// rtsp地址，可根据实际情况修改
	char rtsp_endpoint[256] = { 0 };
	//char rtsp_endpoint[] = "rtsp://admin:admin123@192.168.1.164:554/h264/ch1/main/av_stream";
	//char rtsp_endpoint[] = "rtsp://admin:root1234@192.168.1.64:554/h264/ch1/main/av_stream";
	int videoStream, i, numBytes;
	int ret, got_picture;
	int opt, frames_count = -1;

	while ((opt = getopt(argc, argv, OPTS)) != -1) {

        switch (opt) {
            case 'i':
                strcpy(rtsp_endpoint, optarg);
				printf("rtsp_endpoint: %s.\n", optarg);
                break;
            case 'n':
                frames_count = atoi(optarg);
                if (frames_count < 0) {
                    print_usage();
                    return -1;
                }
                printf("frames_count = %d\n", frames_count);
                break; 
			case 'h':
            default:
                print_usage();
                return -1;
        }
    }

    if (strlen(rtsp_endpoint) == 0  || frames_count < 0) {
        print_usage();
        return -1;
    }

	avformat_network_init();   ///初始化FFmpeg网络模块，2017.8.5---lizhen
	av_register_all();         //初始化FFMPEG  调用了这个才能正常适用编码器和解码器

	 //Allocate an AVFormatContext.
	pFormatCtx = avformat_alloc_context();

	///2017.8.5---lizhen
	AVDictionary *avdic = NULL;
	char option_key[] = "rtsp_transport";
	char option_value[] = "tcp";
	av_dict_set(&avdic, option_key, option_value, 0);
	char option_key2[] = "max_delay";
	char option_value2[] = "100";
	av_dict_set(&avdic, option_key2, option_value2, 0);
	
	if (avformat_open_input(&pFormatCtx, rtsp_endpoint, NULL, &avdic) != 0) {
		printf("can't open the file. \n");
		return -1;
	}

	if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
		printf("Could't find stream infomation.\n");
		return -1;
	}

	videoStream = -1;

	///循环查找视频中包含的流信息，直到找到视频类型的流
	///便将其记录下来 保存到videoStream变量中
	///这里我们现在只处理视频流  音频流先不管他
	for (i = 0; i < pFormatCtx->nb_streams; i++) {
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			videoStream = i;
		}
	}

	///如果videoStream为-1 说明没有找到视频流
	if (videoStream == -1) {
		printf("Didn't find a video stream.\n");
		return -1;
	}

	///查找解码器
	pCodecCtx = pFormatCtx->streams[videoStream]->codec;
	pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
	///2017.8.9---lizhen
	pCodecCtx->bit_rate = 0;   //初始化为0
	pCodecCtx->time_base.num = 1;  //下面两行：一秒钟25帧
	pCodecCtx->time_base.den = 10;
	pCodecCtx->frame_number = 1;  //每包一个视频帧

	if (pCodec == NULL) {
		printf("Codec not found.\n");
		return -1;
	}

	///打开解码器
	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
		printf("Could not open codec.\n");
		return -1;
	}

	pFrame = av_frame_alloc();
	pFrameRGB = av_frame_alloc();

	///这里我们改成了将解码后的YUV数据转换成RGB32
	//img_convert_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height,
	//	pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height,
	//	AV_PIX_FMT_RGB32, SWS_BICUBIC, NULL, NULL, NULL);
	img_convert_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height,
		pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height,
		AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);

	// 保存BGR，opencv中是按BGR来保存的
	//numBytes = avpicture_get_size(AV_PIX_FMT_RGB32, pCodecCtx->width, pCodecCtx->height);
	numBytes = avpicture_get_size(AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
	out_buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
	avpicture_fill((AVPicture *)pFrameRGB, out_buffer, AV_PIX_FMT_BGR24,
		pCodecCtx->width, pCodecCtx->height);

	int y_size = pCodecCtx->width * pCodecCtx->height;

	packet = (AVPacket *)malloc(sizeof(AVPacket)); //分配一个packet
	av_new_packet(packet, y_size); //分配packet的数据

	//opencv
	cv::Mat pCvMat;
	pCvMat.create(cv::Size(pCodecCtx->width, pCodecCtx->height), CV_8UC3);

	int idx = 0;
	std::string pic_name = "demo_";
	std::string post_fix = ".jpg";

	while (idx++ < frames_count)
	{
		printf("av_read_frame: %d.\n", idx);
		if (av_read_frame(pFormatCtx, packet) < 0)
		{
			break; //这里认为视频读取完了
		}

		if (packet->stream_index == videoStream) {
			ret = avcodec_decode_video2(pCodecCtx, pFrame, &got_picture, packet);

			if (ret < 0) {
				printf("decode error.\n");
				return -1;
			}

			if (got_picture) {
				sws_scale( img_convert_ctx, (uint8_t const * const *)pFrame->data,
						   pFrame->linesize, 0, pCodecCtx->height, 
						   pFrameRGB->data, pFrameRGB->linesize);
				
				memcpy(pCvMat.data, out_buffer, numBytes);
				cv::imwrite(pic_name + std::to_string(idx) + post_fix, pCvMat);
				//cv::waitKey(500);

				// 把这个 RGB 数据用 QImage 加载
				//QImage tmpImg((uchar *)out_buffer, pCodecCtx->width, pCodecCtx->height, QImage::Format_RGB32);
				//QImage image = tmpImg.copy(); //把图像复制一份 传递给界面显示
				//emit sig_GetOneFrame(image);  //发送信号
				//
				///// 2017.8.11---lizhen
				//// 提取出图像中的R数据
				//for (int i = 0; i<pCodecCtx->width; i++)
				//{
				//	for (int j = 0; j<pCodecCtx->height; j++)
				//	{
				//		QRgb rgb = image.pixel(i, j);
				//		int r = qRed(rgb);
				//		image.setPixel(i, j, qRgb(r, 0, 0));
				//	}
				//}
				//emit sig_GetRFrame(image);
			}
		}
		av_free_packet(packet); //释放资源,否则内存会一直上升

								///2017.8.7---lizhen
		cv::waitKey(2); //停一停  不然放的太快了
	}
	av_free(out_buffer);
	av_free(pFrameRGB);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);
}
