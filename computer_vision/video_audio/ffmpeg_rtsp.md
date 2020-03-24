# 基于 ffmpeg 的 rtsp 协议解码,推流,拉流方案  













## issues  

1. concealing 3342 DC 3342 AC 3342MV errors in P frame   

FFmpeg\libavcodec\error_resilience.c 文件的 ff_er_frame_end() 函数.  

```cpp
av_log( s->avctx, AV_LOG_INFO, 
        "concealing %d DC, %d AC, %d MV errors in %c frame\n",
        dc_error, ac_error, mv_error, 
        av_get_picture_type_char(s->cur_pic.f->pict_type));
```


```cpp
// image2rtsp.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include <stdio.h>  
#include "opencv2/opencv.hpp"
#define __STDC_CONSTANT_MACROS  
#include "image2rtsp.h"
using namespace cv;

extern "C"
{
#include "libavformat/avformat.h"  
#include "libavutil/mathematics.h"  
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavutil/time.h"  
};

#include<string.h>  
#include<malloc.h>  

#pragma pack(1)  

int main()
{
    typedef struct tagBITMAPFILEHEADER
    {
        unsigned short  bfType; //2 位图文件的类型，必须为“BM”  
        unsigned long bfSize; //4 位图文件的大小，以字节为单位  
        unsigned short bfReserved1; //2 位图文件保留字，必须为0  
        unsigned short bfReserved2; //2 位图文件保留字，必须为0  
        unsigned long bfOffBits; //4 位图数据的起始位置，以相对于位图文件头的偏移量表示，以字节为单位  
    } BITMAPFILEHEADER;//该结构占据14个字节。  

    typedef struct tagBITMAPINFOHEADER {
        unsigned long biSize; //4 本结构所占用字节数  
        long biWidth; //4 位图的宽度，以像素为单位  
        long biHeight; //4 位图的高度，以像素为单位  
        unsigned short biPlanes; //2 目标设备的平面数不清，必须为1  
        unsigned short biBitCount;//2 每个像素所需的位数，必须是1(双色), 4(16色)，8(256色)或24(真彩色)之一  
        unsigned long biCompression; //4 位图压缩类型，必须是 0(不压缩),1(BI_RLE8压缩类型)或2(BI_RLE4压缩类型)之一  
        unsigned long biSizeImage; //4 位图的大小，以字节为单位  
        long biXPelsPerMeter; //4 位图水平分辨率，每米像素数  
        long biYPelsPerMeter; //4 位图垂直分辨率，每米像素数  
        unsigned long biClrUsed;//4 位图实际使用的颜色表中的颜色数  
        unsigned long biClrImportant;//4 位图显示过程中重要的颜色数  
    } BITMAPINFOHEADER;//该结构占据40个字节。  

    int nWidth = 0;
    int nHeight = 0;
    int nDataLen = 0;
    int nLen;

    char csFileName[20];

    int fileI = 1;
    unsigned char *pBmpBuf;
    //for (fileI = 1; fileI <= 1; fileI++)
    {
        sprintf(csFileName, "D:/workSpace/gitwork/video_decode/image2rtsp/image2rtsp/bmp/%d.bmp", fileI);
        printf(" %s\n", csFileName);

        FILE *fp;
        if ((fp = fopen(csFileName, "rb")) == NULL)  //以二进制的方式打开文件  
        {
            return false;
        }
        char fileType[2];
        fread(&fileType, 1, 2 * sizeof(char), fp);
        if (fileType[0] != 0x42 || fileType[1] != 0x4d)
        {
            printf("0x%02x, 0x%02x\n", fileType[0], fileType[1]);
            printf("file is not .bmp file!");
            system("pause");
            return 0;
        }

        if (fseek(fp, sizeof(BITMAPFILEHEADER), 0))  //跳过BITMAPFILEHEADE  
        {
            return false;
        }
        BITMAPINFOHEADER infoHead;
        fread(&infoHead, sizeof(BITMAPINFOHEADER), 1, fp);   //从fp中读取BITMAPINFOHEADER信息到infoHead中,同时fp的指针移动  
        nWidth = infoHead.biWidth;
        nHeight = infoHead.biHeight;
        int linebyte = (nWidth * 24 / 8 + 3) / 4 * 4; //计算每行的字节数，24：该图片是24位的bmp图，3：确保不丢失像素  

                                                      //cout<<bmpwidth<<" "<<bmpheight<<endl;  
        nDataLen = linebyte*nHeight;
        pBmpBuf = new unsigned char[linebyte*nHeight];
        fread(pBmpBuf, sizeof(char), linebyte*nHeight, fp);
        fclose(fp);
    }
    printf("file ok\n");

    av_register_all();
    avformat_network_init();   ///初始化FFmpeg网络模块 
    avcodec_register_all();
    printf("width: %d, height:%d\n", nWidth, nHeight);

    AVFrame *m_pRGBFrame = new AVFrame[1];  //RGB帧数据      
    AVFrame *m_pYUVFrame = new AVFrame[1];;  //YUV帧数据    
    AVCodecContext *c = NULL;
    AVCodecContext *in_c = NULL;
    AVCodec *pCodecH264; //编码器    
    uint8_t * yuv_buff;//    

                       //查找h264编码器    
    pCodecH264 = avcodec_find_encoder(AV_CODEC_ID_H264);

    c = avcodec_alloc_context3(pCodecH264);
    //c->bit_rate = 3000000;// put sample parameters     
    c->bit_rate = 0;   //初始化为0
                       // frames per second     
    c->time_base.num = 1;  //下面两行：一秒钟25帧 //(AVRational){1,25};    
    c->time_base.den = 10;
    c->frame_number = 1;  //每包一个视频帧
    c->width = nWidth;//     
    c->height = nHeight;//     
    c->gop_size = 10; // emit one intra frame every ten frames     
    c->max_b_frames = 1;
    c->thread_count = 1;
    c->pix_fmt = AV_PIX_FMT_YUV420P;//PIX_FMT_RGB24;    

                                    //av_opt_set(c->priv_data, /*"preset"*/"libvpx-1080p.ffpreset", /*"slow"*/NULL, 0);    
                                    //打开编码器    
    if (avcodec_open2(c, pCodecH264, NULL)<0)
        printf("canot open codec\n");

    int size = c->width * c->height;

    yuv_buff = (uint8_t *)malloc((size * 3) / 2); // size for YUV 420     

                                                  //将rgb图像数据填充rgb帧    
    uint8_t * rgb_buff = new uint8_t[nDataLen];


    /*FILE *h264_f = NULL;
    const char * filename = "0_Data.h264";
    h264_f = fopen(filename, "wb");
    if (!h264_f)
    {
    printf("could not open %s\n", filename);
    exit(1);
    }*/

    //初始化SwsContext    
    SwsContext * scxt = sws_getContext(c->width, c->height, AV_PIX_FMT_BGR24, c->width, c->height, AV_PIX_FMT_YUV420P, SWS_POINT, NULL, NULL, NULL);


    //-----lbg----------
    AVOutputFormat *ofmt = NULL;
    //输入对应一个AVFormatContext，输出对应一个AVFormatContext  
    //（Input AVFormatContext and Output AVFormatContext）  
    AVFormatContext  *ofmt_ctx = NULL;
    //AVPacket pkt;
    const char *in_filename;
    //const char *out_filename = "rtsp://192.168.1.5:8554/live.sdp";//输出 URL（Output URL）[RTMP]  
    const char *out_filename = "rtsp://localhost/test";//输出 URL（Output URL）[RTMP]  
                                                       //out_filename = "rtp://233.233.233.233:6666";//输出 URL（Output URL）[UDP]  
    int ret, i;
    int videoindex = -1;
    int frame_index = 0;
    int64_t start_time = 0;

    //in_filename = "F:/FeigeDownload/007.avi";//输入URL（Input file URL）  
    //in_filename  = "shanghai03_p.h264";  

    //输出（Output）  
    avformat_alloc_output_context2(&ofmt_ctx, NULL, "rtsp", out_filename); //RTMP  
                                                                           //avformat_alloc_output_context2(&ofmt_ctx, NULL, "mpegts", out_filename);//UDP  

    if (!ofmt_ctx)
    {
        printf("Could not create output context\n");
        ret = AVERROR_UNKNOWN;
        goto end;
    }
    ofmt = ofmt_ctx->oformat;
    //for (i = 0; i < ifmt_ctx->nb_streams; i++) {
    //根据输入流创建输出流（Create output AVStream according to input AVStream）  
    //AVStream *in_stream = ifmt_ctx->streams[i];
    //AVStream *out_stream = avformat_new_stream(ofmt_ctx, in_stream->codec->codec);

    //avcodec_parameters_from_context(out_stream->codecpar, c);
    //av_stream_set_r_frame_rate(out_stream, { 1, 25 });

    AVStream *out_stream = avformat_new_stream(ofmt_ctx, pCodecH264);
    if (!out_stream) {
        printf("Failed allocating output stream\n");
        ret = AVERROR_UNKNOWN;
        goto end;
    }
    //复制AVCodecContext的设置（Copy the settings of AVCodecContext）  
    /*ret = avcodec_copy_context(out_stream->codec, in_stream->codec);
    if (ret < 0) {
    printf("Failed to copy context from input to output stream codec context\n");
    goto end;
    }*/

    ret = avcodec_copy_context(out_stream->codec, c);
    if (ofmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        //out_stream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;
        out_stream->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    //}
    //Dump Format------------------  
    av_dump_format(ofmt_ctx, 0, out_filename, 1);
    //打开输出URL（Open output URL）  
    printf("begin to avio_open...\n");
    if (!(ofmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, out_filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            printf("Could not open output URL '%s'", out_filename);
            goto end;
        }
    }
    //写文件头（Write file header）  
    ret = avformat_write_header(ofmt_ctx, NULL);
    if (ret < 0) {
        printf("Error occurred when opening output URL\n");
        goto end;
    }

    //----------------
    printf("begin to broadcast...\n");//图像编码    
    int duation = 0;
    int outbuf_size = 100000;
    uint8_t * outbuf = (uint8_t*)malloc(outbuf_size);
    int u_size = 0;
    AVPacket avpkt;
    for (int i = 1; i<10; ++i)
    {
        //AVFrame *m_pYUVFrame = new AVFrame[1];    
        //int index = i % 200 + 1;
        int index = i;

        sprintf(csFileName, "D:/workSpace/gitwork/video_decode/image2rtsp/image2rtsp/bmp/%d.bmp", index);
        printf("send : %s\n", csFileName);

        FILE *fp;
        if ((fp = fopen(csFileName, "rb")) == NULL)  //以二进制的方式打开文件  
        {
            return false;
        }
        char fileType[2];
        fread(&fileType, 1, 2 * sizeof(char), fp);
        if (fileType[0] != 0x42 || fileType[1] != 0x4d)
        {
            printf("file is not .bmp file!");
            system("pause");
            return 0;
        }

        if (fseek(fp, sizeof(BITMAPFILEHEADER), 0))  //跳过BITMAPFILEHEADE  
        {
            return false;
        }
        BITMAPINFOHEADER infoHead;
        fread(&infoHead, sizeof(BITMAPINFOHEADER), 1, fp);   //从fp中读取BITMAPINFOHEADER信息到infoHead中,同时fp的指针移动  
        nWidth = infoHead.biWidth;
        nHeight = infoHead.biHeight;
        int linebyte = (nWidth * 24 / 8 + 3) / 4 * 4; //计算每行的字节数，24：该图片是24位的bmp图，3：确保不丢失像素  

                                                      //cout<<bmpwidth<<" "<<bmpheight<<endl;  
        nDataLen = linebyte*nHeight;
        pBmpBuf = new unsigned char[linebyte*nHeight];
        fread(pBmpBuf, sizeof(char), linebyte*nHeight, fp);
        fclose(fp);
        memcpy(rgb_buff, pBmpBuf, nDataLen);


        avpicture_fill((AVPicture*)m_pRGBFrame, (uint8_t*)rgb_buff, AV_PIX_FMT_RGB24, nWidth, nHeight);

        //将 YUV buffer 填充 YUV Frame    
        avpicture_fill((AVPicture*)m_pYUVFrame, (uint8_t*)yuv_buff, AV_PIX_FMT_YUV420P, nWidth, nHeight);

        // 翻转 RGB 图像    
        m_pRGBFrame->data[0] += m_pRGBFrame->linesize[0] * (nHeight - 1);
        m_pRGBFrame->linesize[0] *= -1;
        m_pRGBFrame->data[1] += m_pRGBFrame->linesize[1] * (nHeight / 2 - 1);
        m_pRGBFrame->linesize[1] *= -1;
        m_pRGBFrame->data[2] += m_pRGBFrame->linesize[2] * (nHeight / 2 - 1);
        m_pRGBFrame->linesize[2] *= -1;


        //将 RGB 转化为 YUV    
        sws_scale(scxt, m_pRGBFrame->data, m_pRGBFrame->linesize, 0, c->height, m_pYUVFrame->data, m_pYUVFrame->linesize);

        int got_packet_ptr = 0;
        avpkt.size = 12;
        printf("outbuf_size: %d, avpkt.size: %d, got_packet_ptr:%d\n", outbuf_size, avpkt.size, got_packet_ptr);
        av_init_packet(&avpkt);
        avpkt.data = outbuf;
        avpkt.size = outbuf_size;
        u_size = avcodec_encode_video2(c, &avpkt, m_pYUVFrame, &got_packet_ptr);
        printf("outbuf_size: %d, avpkt.size: %d, got_packet_ptr:%d\n", outbuf_size, avpkt.size, got_packet_ptr);
        m_pYUVFrame->pts++;
        if (u_size == 0)
        {

            //avpkt.pts = av_rescale_q_rnd(avpkt.pts, c->time_base, out_stream->time_base, (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
            //avpkt.dts = av_rescale_q_rnd(avpkt.dts, c->time_base, out_stream->time_base, (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
            //avpkt.duration = av_rescale_q(avpkt.duration, c->time_base, out_stream->time_base);


            time_t tt = time(NULL);//这句返回的只是一个时间cuo
            tm* t = localtime(&tt);

            int64_t i = *((int64_t*)&t);


            avpkt.pts = duation;
            avpkt.dts = duation;
            avpkt.duration = 0;
            duation += 18000;
            avpkt.pos = -1;
            ret = av_interleaved_write_frame(ofmt_ctx, &avpkt);

            if (ret < 0) {
                printf("Error muxing packet %d\n", ret);
            }
            else {
                printf("muxing packet ok\n", ret);
            }
            //fwrite(avpkt.data, 1, avpkt.size, f);

        }
    }

    //fclose(h264_f);
    free(outbuf);
    avcodec_close(c);
    av_free(c);


    av_write_trailer(ofmt_ctx);
end:
    //avformat_close_input(&ifmt_ctx);

    if (ofmt_ctx && !(ofmt->flags & AVFMT_NOFILE))
        avio_close(ofmt_ctx->pb);
    avformat_free_context(ofmt_ctx);
    if (ret < 0 && ret != AVERROR_EOF) {
        printf("Error occurred.\n");
        return -1;
    }

    return 0;
}


```