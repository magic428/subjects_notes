# 去雾终端程序

关于配置文件中的参数，这里给出详细介绍。  

```conf
[video_input]
stream_file_path=D:/workSpace/data_dl/frog/coal-frog.mp4

[haze_remove]
TE_block_size=25  # 透射率估计使用的 block 大小
fL1=5.0
fL2=1.0
GF_blocks_size=40

[video_output]
output_save_path=output.avi
```


## 程序中可能出现的问题  

1. 提示找不到 conf.ini 配置文件  

确认 “配置属性 | 生成事件 | 生成后事件 | 命令行” 是否有以下内容：  

copy $(TargetDir)..\..\config\conf.ini  $(TargetDir)
copy $(TargetDir)..\..\config\conf.ini  $(TargetDir)..\

如果没有， 手动添加即可。  


2. 关于 rtsp 流产生的视频花屏卡顿问题。  

rtsp 需要以较高的频率获取数据流，如果计算机的 CPU 占用了过高， 则可能会出现这种情况。 请检查你的电脑 CPU 占用率是否过高。 

rtsp://admin:admin1234@192.168.10.108:554/cam/realmonitor?channel=1&subtype=0

![test](https://github.com/magic428/work_note/blob/master/deeplearning/Deeplearning_README.md)  