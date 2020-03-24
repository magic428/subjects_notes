# 基于 ffmpeg 将 rtsp 视频流保存为图片和 MP4 视频

工程项目中使用 OpenCV 的 VideoCap 获取 rtsp 数据流时总是出现花屏卡顿现象， 因此尝试使用 ffmpeg 解析 rtsp 数据流。 结果显示 ffmpeg 可以稳定地完成图像视频的转换任务。   

- 将 rtsp 视频流解析为常见格式的视频文件, 如 avi | flv | mp4；   
- 将 rtsp 视频流解析为 cv::Mat 图片格式；  
- 使用多线程实现视频流的解码和提取。  

## 1. rtsp 转为图片  


```bash
./rtsp2img[.exe] -i <rtsp url> -n <number of frames you want to save>
```

## 2. rtsp 转为视频  

```bash
.\rtsp2video[.exe] -i rtsp://admin:root1234@192.168.1.64:554/h264/ch1/main/av_stream -t flv -n 120
```

## 3. rtsp 多线程转换图片  

```bash
.\rtsp_img_thread[.exe]  
```

