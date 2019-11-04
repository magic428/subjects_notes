/**
 * Convert rtsp video stream to video (mp4, flv, avi et al.)
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

//#include <unistd.h>
#include <getopt.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define USAGE   "rtsp2x -i <rtsp url> -t [avi | flv | mp4] -n <number of frames you want to save>"
#define OPTS    "i:t:n:h"

static void print_usage()
{
    printf("Usage: %s\n", USAGE);
    return;
}

int main(int argc,char **argv)
{
    AVOutputFormat *ofmt = NULL;
    AVFormatContext *ifmt_ctx = NULL,*ofmt_ctx = NULL;
    AVPacket pkt;
    char in_filename[256] = {0}, out_filename[256] = {0};    
    int ret,i;
    int video_index=-1;
    int frame_index=0;
    int I_received = 0;

    int opt, frames_count = -1;

    while ((opt = getopt(argc, argv, OPTS)) != -1) {

        switch (opt) {
            case 'i':
                strcpy(in_filename, optarg);
                break;
            case 't':
                if (strcmp(optarg, "avi") == 0)
                        strcpy(out_filename, "receive.avi");
                else if (strcmp(optarg, "flv") == 0)
                        strcpy(out_filename, "receive.flv");
                else if (strcmp(optarg, "mp4") == 0) 
                        strcpy(out_filename, "receive.mp4");
                else {
                        return -1;
                }
                print_usage();
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

    if (strlen(in_filename) == 0 || strlen(out_filename) == 0 || frames_count < 0) {
        print_usage();
        return -1;
    }

    av_register_all();
    avformat_network_init();

    // 使用TCP连接打开RTSP，设置最大延迟时间
    AVDictionary *avdic=NULL;  
    char option_key[]="rtsp_transport";  
    char option_value[]="tcp";  
    av_dict_set(&avdic, option_key, option_value, 0);  
    char option_key2[]="max_delay";  
    char option_value2[]="5000000";  
    av_dict_set(&avdic,option_key2,option_value2,0); 
    
    // 打开输入流
    if((ret=avformat_open_input(&ifmt_ctx,in_filename,0,&avdic))<0)
    {
        printf("Could not open input file.\n");
        goto end;
    }
    if((ret=avformat_find_stream_info(ifmt_ctx,0))<0)
    {
        printf("Failed to retrieve input stream information\n");
        goto end;
    }

    //nb_streams代表有几路流，一般是2路：即音频和视频，顺序不一定
    for(i=0;i<ifmt_ctx->nb_streams;i++){
        
        if(ifmt_ctx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO)
        {
            //这一路是视频流，标记一下，以后取视频流都从ifmt_ctx->streams[video_index]取
            video_index=i;
            break;
        }
    }

    av_dump_format(ifmt_ctx,0,in_filename,0);

    // 打开输出流
    avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, out_filename);
    
    if(!ofmt_ctx)
    {
        printf("Could not create output context\n");
        ret = AVERROR_UNKNOWN;
        goto end;
    }
    
    ofmt = ofmt_ctx->oformat;
    for(i=0;i<ifmt_ctx->nb_streams;i++)
    {   
        // 根据输入流创建输出流
        AVStream *in_stream = ifmt_ctx->streams[i];
        AVStream *out_stream = avformat_new_stream(ofmt_ctx,in_stream->codec->codec);
        if(!out_stream)
        {
            printf("Failed allocating output stream.\n");
            ret = AVERROR_UNKNOWN;
            goto end;
        }

        // 将输出流的编码信息复制到输入流
        ret = avcodec_copy_context(out_stream->codec, in_stream->codec);
        if(ret < 0)
        {
            printf("Failed to copy context from input to output stream codec context\n");
            goto end;
        }
        out_stream->codec->codec_tag = 0;
    
        if(ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
            out_stream->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    // Dump format--------------------
    av_dump_format(ofmt_ctx, 0, out_filename, 1);
    // 打开输出文件
    if(!(ofmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, out_filename, AVIO_FLAG_WRITE);
        if(ret < 0) {
            printf("Could not open output URL '%s'", out_filename);
            goto end;
        }
    }

    // 写文件头到输出文件
    ret = avformat_write_header(ofmt_ctx,NULL);
    if(ret < 0) {
        printf("Error occured when opening output URL\n");
        goto end;
    }

    // while 循环中持续获取数据包，不管音频视频都存入文件
    while(1) {
        AVStream *in_stream,*out_stream;
        //从输入流获取一个数据包
        ret = av_read_frame(ifmt_ctx, &pkt);
        if(ret < 0)
            break;

        in_stream = ifmt_ctx->streams[pkt.stream_index];
        out_stream = ofmt_ctx->streams[pkt.stream_index];
        //copy packet
        //转换 PTS/DTS 时序
        pkt.pts = av_rescale_q_rnd(pkt.pts,in_stream->time_base,out_stream->time_base,(enum AVRounding)(AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX));
        pkt.dts = av_rescale_q_rnd(pkt.dts, in_stream->time_base, out_stream->time_base, (enum AVRounding)(AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX));  
        //printf("pts %d dts %d base %d\n",pkt.pts,pkt.dts, in_stream->time_base);
        pkt.duration = av_rescale_q(pkt.duration, in_stream->time_base, out_stream->time_base); 
        pkt.pos = -1;  

        //此 while 循环中并非所有 packet 都是视频帧，当收到视频帧时记录一下，仅此而已
        if(pkt.stream_index==video_index) {
            if ((pkt.flags & AV_PKT_FLAG_KEY) && (I_received == 0)) 
                I_received = 1;

            if (I_received == 0)
                continue;

            printf("Receive %8d video frames from input URL\n",frame_index);
            frame_index++;
        } else {
            continue;
        }

        if (frame_index == frames_count)
            break;

        // 将包数据写入到文件。
        ret = av_interleaved_write_frame(ofmt_ctx, &pkt);
        if(ret < 0) {
            /**
            当网络有问题时，容易出现到达包的先后不一致，pts 时序混乱会导致
            av_interleaved_write_frame 函数报 -22 错误。暂时先丢弃这些迟来的帧吧
            若所大部分包都没有pts时序，那就要看情况自己补上时序（比如较前一帧时序+1）再写入。
            */
            if(ret==-22){
                continue;
            } else {
                printf("Error muxing packet.error code %d\n" , ret);
                break;
            }
            
        }
        
        //av_free_packet(&pkt); //此句在新版本中已deprecated 由av_packet_unref代替
        av_packet_unref(&pkt);
    }

    //写文件尾
    av_write_trailer(ofmt_ctx);

end:
    av_dict_free(&avdic);
    avformat_close_input(&ifmt_ctx);

    // Close input
    if(ofmt_ctx && !(ofmt->flags & AVFMT_NOFILE))
        avio_close(ofmt_ctx->pb);
    avformat_free_context(ofmt_ctx);
    if(ret<0 && ret != AVERROR_EOF)
    {
        printf("Error occured.\n");
        return -1;
    }
    return 0;
}


int main_(int argc, char **argv)
{
	AVOutputFormat *ofmt = NULL;
	AVFormatContext *ifmt_ctx = NULL, *ofmt_ctx = NULL;
	AVPacket pkt;
	char in_filename[128] = { 0 }, out_filename[128] = { 0 };
	int ret, i;
	int video_index = -1;
	int frame_index = 0;
	int I_received = 0;

	int opt, frames_count = -1;

	while ((opt = getopt(argc, argv, OPTS)) != -1) {

		switch (opt) {
		case 'i':
			strcpy(in_filename, optarg);
			break;
		case 't':
			if (strcmp(optarg, "avi") == 0)
				strcpy(out_filename, "receive.avi");
			else if (strcmp(optarg, "flv") == 0)
				strcpy(out_filename, "receive.flv");
			else if (strcmp(optarg, "mp4") == 0)
				strcpy(out_filename, "receive.mp4");
			else {
				return -1;
			}
			print_usage();
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

	if (strlen(in_filename) == 0 || strlen(out_filename) == 0 || frames_count < 0) {
		print_usage();
		return -1;
	}

	av_register_all();
	avformat_network_init();

	// 使用TCP连接打开RTSP，设置最大延迟时间
	AVDictionary *avdic = NULL;
	char option_key[] = "rtsp_transport";
	char option_value[] = "tcp";
	av_dict_set(&avdic, option_key, option_value, 0);
	char option_key2[] = "max_delay";
	char option_value2[] = "5000000";
	av_dict_set(&avdic, option_key2, option_value2, 0);

	// 打开输入流
	if ((ret = avformat_open_input(&ifmt_ctx, in_filename, 0, &avdic))<0)
	{
		printf("Could not open input file.\n");
		goto end;
	}
	if ((ret = avformat_find_stream_info(ifmt_ctx, 0))<0)
	{
		printf("Failed to retrieve input stream information\n");
		goto end;
	}

	//nb_streams代表有几路流，一般是2路：即音频和视频，顺序不一定
	for (i = 0; i<ifmt_ctx->nb_streams; i++) {

		if (ifmt_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
		{
			//这一路是视频流，标记一下，以后取视频流都从ifmt_ctx->streams[video_index]取
			video_index = i;
			break;
		}
	}

	av_dump_format(ifmt_ctx, 0, in_filename, 0);

	// 打开输出流
	avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, out_filename);

	if (!ofmt_ctx)
	{
		printf("Could not create output context\n");
		ret = AVERROR_UNKNOWN;
		goto end;
	}

	ofmt = ofmt_ctx->oformat;
	for (i = 0; i<ifmt_ctx->nb_streams; i++)
	{
		// 根据输入流创建输出流
		AVStream *in_stream = ifmt_ctx->streams[i];
		AVStream *out_stream = avformat_new_stream(ofmt_ctx, in_stream->codec->codec);
		if (!out_stream)
		{
			printf("Failed allocating output stream.\n");
			ret = AVERROR_UNKNOWN;
			goto end;
		}

		// 将输出流的编码信息复制到输入流
		ret = avcodec_copy_context(out_stream->codec, in_stream->codec);
		if (ret < 0)
		{
			printf("Failed to copy context from input to output stream codec context\n");
			goto end;
		}
		out_stream->codec->codec_tag = 0;

		if (ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
			out_stream->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	}

	// Dump format--------------------
	av_dump_format(ofmt_ctx, 0, out_filename, 1);
	// 打开输出文件
	if (!(ofmt->flags & AVFMT_NOFILE)) {
		ret = avio_open(&ofmt_ctx->pb, out_filename, AVIO_FLAG_WRITE);
		if (ret < 0) {
			printf("Could not open output URL '%s'", out_filename);
			goto end;
		}
	}

	// 写文件头到输出文件
	ret = avformat_write_header(ofmt_ctx, NULL);
	if (ret < 0) {
		printf("Error occured when opening output URL\n");
		goto end;
	}

	// while 循环中持续获取数据包，不管音频视频都存入文件
	while (1) {
		AVStream *in_stream, *out_stream;
		//从输入流获取一个数据包
		ret = av_read_frame(ifmt_ctx, &pkt);
		if (ret < 0)
			break;

		in_stream = ifmt_ctx->streams[pkt.stream_index];
		out_stream = ofmt_ctx->streams[pkt.stream_index];
		//copy packet
		//转换 PTS/DTS 时序
		pkt.pts = av_rescale_q_rnd(pkt.pts, in_stream->time_base, out_stream->time_base, (enum AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
		pkt.dts = av_rescale_q_rnd(pkt.dts, in_stream->time_base, out_stream->time_base, (enum AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
		//printf("pts %d dts %d base %d\n",pkt.pts,pkt.dts, in_stream->time_base);
		pkt.duration = av_rescale_q(pkt.duration, in_stream->time_base, out_stream->time_base);
		pkt.pos = -1;

		//此 while 循环中并非所有 packet 都是视频帧，当收到视频帧时记录一下，仅此而已
		if (pkt.stream_index == video_index) {
			if ((pkt.flags & AV_PKT_FLAG_KEY) && (I_received == 0))
				I_received = 1;

			if (I_received == 0)
				continue;

			printf("Receive %8d video frames from input URL\n", frame_index);
			frame_index++;
		}
		else {
			continue;
		}

		if (frame_index == frames_count)
			break;

		// 将包数据写入到文件。
		ret = av_interleaved_write_frame(ofmt_ctx, &pkt);
		if (ret < 0) {
			/**
			当网络有问题时，容易出现到达包的先后不一致，pts 时序混乱会导致
			av_interleaved_write_frame 函数报 -22 错误。暂时先丢弃这些迟来的帧吧
			若所大部分包都没有pts时序，那就要看情况自己补上时序（比如较前一帧时序+1）再写入。
			*/
			if (ret == -22) {
				continue;
			}
			else {
				printf("Error muxing packet.error code %d\n", ret);
				break;
			}

		}

		//av_free_packet(&pkt); //此句在新版本中已deprecated 由av_packet_unref代替
		av_packet_unref(&pkt);
	}

	//写文件尾
	av_write_trailer(ofmt_ctx);

end:
	av_dict_free(&avdic);
	avformat_close_input(&ifmt_ctx);

	// Close input
	if (ofmt_ctx && !(ofmt->flags & AVFMT_NOFILE))
		avio_close(ofmt_ctx->pb);
	avformat_free_context(ofmt_ctx);
	if (ret<0 && ret != AVERROR_EOF)
	{
		printf("Error occured.\n");
		return -1;
	}
	return 0;
}


using namespace cv;
using namespace std;


int main_xx()
{
	AVFormatContext *pFormatCtx;
	AVCodecContext *pCodecCtx;
	AVCodec *pCodec;
	AVFrame *pFrame, *pFrameRGB;
	AVPacket *packet;
	uint8_t *out_buffer;

	static struct SwsContext *img_convert_ctx;

	int videoStream, i, numBytes;
	int ret, got_picture;

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
	///rtsp地址，可根据实际情况修改
	//char url[] = "rtsp://admin:admin123@192.168.1.164:554/h264/ch1/main/av_stream";
	char url[] = "rtsp://admin:root1234@192.168.1.64:554/h264/ch1/main/av_stream";

	if (avformat_open_input(&pFormatCtx, url, NULL, &avdic) != 0) {
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
	while (1)
	{
		printf("av_read_frame: %d.\n", idx);
		idx++;
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
		cv::waitKey(200); //停一停  不然放的太快了
	}
	av_free(out_buffer);
	av_free(pFrameRGB);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);
}

#if 0
int main_(int argc, char *argv[])
{
	//解码器指针
	AVCodec *pCodec;
	//ffmpeg解码类的类成员
	AVCodecContext *pCodecCtx;
	//多媒体帧，保存解码后的数据帧
	AVFrame *pAvFrame;
	//保存视频流的信息
	AVFormatContext *pFormatCtx;

	if (argc <= 1) {
		printf("need filename\n");
		return -1;
	}
	char *filename = argv[1];
	//注册库中所有可用的文件格式和编码器
	av_register_all();

	pFormatCtx = avformat_alloc_context();
	//检查文件头部
	if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) != 0) {
		printf("Can't find the stream!\n");
	}
	//查找流信息
	if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
		printf("Can't find the stream information !\n");
	}

	int videoindex = -1;
	//遍历各个流，找到第一个视频流,并记录该流的编码信息
	for (int i = 0; i < pFormatCtx->nb_streams; ++i) {
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			videoindex = i;
			break;
		}
	}
	if (videoindex == -1) {
		printf("Don't find a video stream !\n");
		return -1;
	}
	//得到一个指向视频流的上下文指针
	pCodecCtx = pFormatCtx->streams[videoindex]->codec;
	//到该格式的解码器
	pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
	if (pCodec == NULL) {
		//寻找解码器
		printf("Cant't find the decoder !\n");
		return -1;
	}
	//打开解码器
	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
		printf("Can't open the decoder !\n");
		return -1;
	}

	//分配帧存储空间
	pAvFrame = av_frame_alloc();
	//存储解码后转换的RGB数据
	AVFrame *pFrameBGR = av_frame_alloc();

	// 保存BGR，opencv中是按BGR来保存的
	int size = avpicture_get_size(AV_PIX_FMT_BGR24, pCodecCtx->width,
		pCodecCtx->height);
	uint8_t *out_buffer = (uint8_t *)av_malloc(size);
	avpicture_fill((AVPicture *)pFrameBGR, out_buffer, AV_PIX_FMT_BGR24,
		pCodecCtx->width, pCodecCtx->height);

	AVPacket *packet = (AVPacket *)malloc(sizeof(AVPacket));
	printf("-----------输出文件信息---------\n");
	av_dump_format(pFormatCtx, 0, filename, 0);
	printf("------------------------------");

	struct SwsContext *img_convert_ctx;
	img_convert_ctx =
		sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
			pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,
			NULL);

	//opencv
	cv::Mat pCvMat;
	pCvMat.create(cv::Size(pCodecCtx->width, pCodecCtx->height), CV_8UC3);

	int ret;
	int got_picture;

	Mat gray;
	CascadeClassifier cascade;
	cascade.load("haarcascade_frontalface_alt.xml");

	cvNamedWindow("RGB", 1);
	time_t t;
	while (1) {
		if (av_read_frame(pFormatCtx, packet) >= 0) {
			if (packet->stream_index == videoindex) {
				ret = avcodec_decode_video2(pCodecCtx, pAvFrame, &got_picture, packet);
				if (ret < 0) {
					printf("Decode Error.（解码错误）\n");
					return -1;
				}
				if (got_picture) {
					//YUV to RGB
					sws_scale(img_convert_ctx, (const uint8_t *const *)pAvFrame->data,
						pAvFrame->linesize, 0, pCodecCtx->height, pFrameBGR->data, pFrameBGR->linesize);

					memcpy(pCvMat.data, out_buffer, size);

					/////////////////////////////////////////////////////////////////
					vector<Rect> faces(0);

					cvtColor(pCvMat, gray, CV_BGR2GRAY);
					//改变图像大小，使用双线性差值
					//resize(gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR);
					//变换后的图像进行直方图均值化处理
					equalizeHist(gray, gray);

					time(&t);
					printf("before %s\n", ctime(&t));
					cascade.detectMultiScale(gray, faces, 2, 2,
						CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH | CV_HAAR_SCALE_IMAGE,
						Size(50, 50));
					time(&t);
					printf("after %s\n", ctime(&t));
					Mat face;
					Point text_lb;

					for (size_t i = 0; i < faces.size(); i++) {
						if (faces[i].height > 0 && faces[i].width > 0) {
							face = gray(faces[i]);
							text_lb = Point(faces[i].x, faces[i].y);

							rectangle(pCvMat, faces[i], Scalar(255, 0, 0), 1, 8, 0);
							//putText(pCvMat, "name", text_lb, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
						}
					}
#if 0
					Mat face_test;
					int predictPCA = 0;
					if (face.rows >= 120) {
						resize(face, face_test, Size(92, 112));

					}
					//Mat face_test_gray;
					//cvtColor(face_test, face_test_gray, CV_BGR2GRAY);

					if (!face_test.empty()) {
						//测试图像应该是灰度图
						predictPCA = modelPCA->predict(face_test);
					}

					cout << predictPCA << endl;
					if (predictPCA == 40) {
						string name = "LiuXiaoLong";
						putText(pCvMat, name, text_lb, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255));
					}
#endif
					/////////////////////////////////////////////////////////////////

					imshow("RGB", pCvMat);
					waitKey(1);
				}
			}
			av_free_packet(packet);
		}
		else {
			break;
		}
	}

	av_free(out_buffer);
	av_free(pFrameBGR);
	av_free(pAvFrame);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);

	sws_freeContext(img_convert_ctx);
	cvDestroyWindow("RGB");

	system("pause");
	return 0;
}

#endif