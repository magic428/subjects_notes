## 各主流摄像头的 rtsp 地址格式  

1. 海康威视  

```
rtsp://[username]:[password]@[ip]:[port]/[codec]/[channel]/[subtype]/av_stream  
```

说明：
username: 用户名。例如admin。Add-Member
password: 密码。例如12345。
ip: 为设备IP。例如 192.0.0.64。
port: 端口号默认为554，若为默认可不填写。
codec：有h264、MPEG-4、mpeg4这几种。
channel: 通道号，起始为1。例如通道1，则为ch1。
subtype: 码流类型，主码流为main，辅码流为sub。

例如，请求海康摄像机通道1的主码流，Url如下
主码流：
rtsp://admin:12345@192.0.0.64:554/h264/ch1/main/av_stream
rtsp://admin:12345@192.0.0.64:554/MPEG-4/ch1/main/av_stream
子码流：
rtsp://admin:12345@192.0.0.64/mpeg4/ch1/sub/av_stream
rtsp://admin:12345@192.0.0.64/h264/ch1/sub/av_stream

2. 大华  

```
rtsp://[username]:[password]@[ip]:[port]/cam/realmonitor?[channel]&[subtype]
```

说明:
username: 用户名。例如admin。
password: 密码。例如admin。
ip: 为设备IP。例如 10.7.8.122。
port: 端口号默认为554，若为默认可不填写。
channel: 通道号，起始为1。例如通道2，则为channel=2。
subtype: 码流类型，主码流为0（即subtype=0），辅码流为1（即subtype=1）。

例如，请求某设备的通道2的辅码流，Url如下
rtsp://admin:admin@10.12.4.84:554/cam/realmonitor?channel=2&subtype=1

3. D-Link

```
rtsp://[username]:[password]@[ip]:[port]/[channel].sdp
```

说明：
username：用户名。例如admin
password：密码。例如12345，如果没有网络验证可直接写成rtsp:// [ip]:[port]/[channel].sdp
ip：为设备IP。例如192.168.0.108。
port：端口号默认为554，若为默认可不填写。
channel：通道号，起始为1。例如通道2，则为live2。


例如，请求某设备的通道2的码流，URL如下
rtsp://admin:12345@192.168.200.201:554/live2.sdp


4. Axis（安讯士）

```
rtsp://[username]:[password]@[ip]/axis-media/media.amp?[videocodec]&[resolution]
```

说明：
username：用户名。例如admin
password：密码。例如12345，如果没有网络验证可省略用户名密码部分以及@字符。
ip：为设备IP。例如192.168.0.108。
videocodec：支持MPEG、h.264等，可缺省。
resolution：分辨率，如resolution=1920x1080，若采用默认分辨率，可缺省此参数。

例如，请求某设备h264编码的1280x720的码流，URL如下：
rtsp:// 192.168.200.202/axis-media/media.amp?videocodec=h264&resolution=1280x720 

## 各厂家rtsp地址格式如下  

一. 海康、中威摄像机

格式1

    主码流：rtsp://admin:12345@192.168.1.64:554/Streaming/Channels/1

    子码流：rtsp://admin:12345@192.168.1.64:554/Streaming/Channels/2

　　 第三码流：rtsp://admin:12345@192.168.1.64:554/Streaming/Channels/3

格式2

rtsp://admin:12345@192.168.1.64:554/ch1/main/av_stream

二. 大华

    rtsp://admin:12345@192.168.1.64:554/cam/realmonitor?channel=1&subtype=0

三. 英飞拓

球机：  
    单播和组播地址一致  
        高码流（主码流）RTSP地址：rtsp://admin:admin@192.168.1.64/1/1080p  
        低码流（子码流）RTSP地址：rtsp://admin:admin@192.168.1.64/1/D1  

半球和枪机：（亭子、车道）
    单播：
        高码流（主码流）RTSP地址：rtsp://192.168.1.64:554/1/h264major
        低码流（子码流）RTSP地址：rtsp://192.168.1.64:554/1/h264minor
    组播：
        高码流（主码流）RTSP地址：rtsp://192.168.1.64:554/1/h264major/multicast
        低码流（子码流）RTSP地址：rtsp://192.168.1.64:554/1/h264minor/multicast

四. 三星

单播：
    高码流rtsp地址：rtsp://admin:admin001@192.168.1.64:554/onvif/profile2/media.smp（720P）
    低码率rtsp地址: rtsp://admin:admin001@192.168.1.64:554/onvif/profile3/media.smp

组播：
    高码流rtsp地址：rtsp://admin:admin001@192.168.1.64:554/onvif/multicast/profile2/media.smp (720p)
    低码流rtsp地址：rtsp://admin:admin001@192.168.1.64:554/onvif/multicast/profile3/media.smp

五. 宇视：
      rtsp://admin:admin@192.168.1.64/media/video1/multicast

六. LG

单播和组播地址一致
    高码流（主码流）RTSP地址：rtsp://admin:admin@192.168.1.64:554/Master-0
    低码流（子码流）RTSP地址：rtsp://admin:admin@1192.168.1.64:554/Slave-0

七、 派尔高网络摄像机：

RTSP主码流地址为：rtsp://192.168.1.64/h264
    第一从码流RTSP地址为：rtsp://192.168.1.64/h264_2
    第二从码流RTSP地址为：rtsp://192.168.1.64/h264_3

八、 安讯士网络摄像机：

单播：
    rtsp://root:123456@192.168.1.64/axis-media/media.amp?videocodec=h264&resolution=1280x720&fps=25
组播：
    rtsp://root:123456@192.168.1.64/onvif-media/media.amp?profile=profile_1_h264&streamtype=multicast
    http://root:123456@192.168.1.64/axis-cgi/alwaysmulti.sdp?camera=1

九. 非凡
    rtsp://admin:12345@192.168.1.64:554/streaming/channels/101

十. 金三立
    rtsp://Admin:111111@192.168.1.64/stream/av0_0
