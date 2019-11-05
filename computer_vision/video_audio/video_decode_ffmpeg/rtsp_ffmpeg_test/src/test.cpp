/*******************************************************************************
 * Video encoding example
 *******************************************************************************/

int main(int argc, char** argv)
{
    AVCodec *codec = NULL;
    AVCodecContext *codecCtx= NULL;
    AVFormatContext *pFormatCtx = NULL;
    AVStream * pVideoStream = NULL;
    AVFrame *picture = NULL;

    int i, x, y,            //
        ret,                 // Return value
        got_packet_ptr;     // Data encoded into packet

    // Register all formats and codecs
    avcodec_register_all();
    av_register_all();
    avformat_network_init();
    
    char filename[100] ;
    sprintf_s(filename,sizeof(filename),"%s","rtsp://192.168.1.7:8554/live.sdp");
    if(argc>1)
    {
        sprintf_s(filename,sizeof(filename),"%s",argv[1]);
    }
    printf_s("URL: %s\n", filename);
    
    // allocate context
    ret = avformat_alloc_output_context2( &pFormatCtx, NULL, "rtsp", filename );
    if ( !pFormatCtx || ret < 0 )
    {
        fprintf(stderr,"Could not allocate output context" );
    }

    pFormatCtx->flags |= AVFMT_FLAG_NOBUFFER|AVFMT_FLAG_FLUSH_PACKETS;
    pFormatCtx->max_interleave_delta = 1;
    pFormatCtx->oformat->video_codec = AV_CODEC_ID_H264;
    
    // Find the codec.
    codec = avcodec_find_encoder(pFormatCtx->oformat->video_codec);
    if (codec == NULL) {
        fprintf(stderr, "Codec not found\n");
        return -1;
    }

    // Add stream to pFormatCtx
    pVideoStream = avformat_new_stream(pFormatCtx, codec);
    if (!pVideoStream)
    {
        fprintf(stderr, "Cannot add new video stream\n");
        return -1;
    }
    
    int framerate = 10;
    pVideoStream->id = pFormatCtx->nb_streams-1;
    pVideoStream->time_base.den = framerate;
    pVideoStream->time_base.num = 1;
    
    // Set context
    codecCtx = pVideoStream->codec;
    codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    codecCtx->profile = FF_PROFILE_H264_BASELINE;
    // Resolution must be a multiple of two.
    codecCtx->width  = 320;
    codecCtx->height = 240;

    codecCtx->bit_rate = 1000000;
    codecCtx->time_base.den = framerate;
    codecCtx->time_base.num = 1;
    codecCtx->gop_size = 12; // emit one intra frame every twelve frames at most

    if (pFormatCtx->oformat->flags & AVFMT_GLOBALHEADER)
        codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

    // Open the codec.
    if (avcodec_open2(codecCtx, codec, NULL) < 0)
    {
        fprintf(stderr, "Cannot open video codec\n");
        return -1;
    }

    ret = avio_open2(&pFormatCtx->pb, filename, AVIO_FLAG_WRITE, NULL, NULL);
    if (ret < 0)
    {
        // Error "Protocol not found"
        av_strerror(ret,errbuf, ERRBUFFLEN);
        fprintf(stderr, "avio_open2() fail: %s\n", errbuf);
        //return -1;
    }

    // Write file header. (Gets stuck here)
    ret = avformat_write_header(pFormatCtx, NULL);
    if ( ret < 0 )
    {
        fprintf(stderr, "error writing header");
        return -1;
    }

    // ...more code
}
--
Ang