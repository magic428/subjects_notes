### 在工程中使用 live555 时出现的问题  

分两个图片进行推流后会出现问题: 中间路串流并丢失最后一路流.  

```bash
CameraJPEGDeviceSource DeConstructor:cvideo-JPEG-1-0.jpg
CameraJPEGDeviceSource DeConstructor:cvideo-JPEG-1-1.jpg
RTCPInstance::RTCPInstance error: totSessionBW parameter should not be zero!
```

为什么会打印这行信息?  

这样信息只有在切换 rtsp 协议的时候才会出现, 说明软件可以检测到这个取流客户端断开, 然后导致了 CameraJPEGDeviceSource* 指针的释放. 


以下是 stream 信号对应的 buffer 指针和事件 triger ID。  

```bash
====== signal 0, sources=0xd1923c00, id=2147483648
====== signal 1, sources=0xd1928ba0, id=1073741824

# 为什么在中间切换视频流通道的时候, 会导致两路 signal 的 ID 相同  
====== signal 0, sources=0xd1987230, id=8388608
====== signal 1, sources=0xd1987230, id=8388608

====== signal 0, sources=0xd4460090, id=1048576
====== signal 1, sources=0xd1987230, id=2097152
```


(1) VLC 断开连接会打印   

```
CameraJPEGDeviceSource DeConstructor:cvideo-JPEG-1-0.jpg
```

(2) 切换 rtsp 视频流 ip 的时候会打印  

```
CameraJPEGDeviceSource DeConstructor:cvideo-JPEG-1-0.jpg
RTCPInstance::RTCPInstance error: totSessionBW parameter should not be zero!
```

```bash
RTCPInstance::RTCPInstance error: totSessionBW parameter should not be zero!
```

重新创建一个连接的时候也会打印上面的消息.  

如果只有一个连接, 那么两个信号 ID 是同一个值;  

如果有两个连接, 那么两个信号 ID 会不相同.  

```bash
====== signal 0, sources=0x9f505cf0, id=262144
====== signal 1, sources=0x9f50d060, id=524288
```

例子:  

```bash
# 初始化  
CameraJPEGDeviceSource: TriggerId=2147483648
CameraJPEGDeviceSource: TriggerId=1073741824

====== Signal 0, device addr=0xc0db6930, TriggerId=2147483648
====== Signal 1, device addr=0xc0d9fa30, TriggerId=1073741824

## 连接 camera 2-0 后  

====== Signal 0, device addr=0xc0db6930, TriggerId=2147483648
====== Signal 1, device addr=0xc0d96050, TriggerId=268435456

## 断开后重新连接 camera1-0 和 camera2-0  
====== Signal 0, device addr=0xc0d96050, TriggerId=134217728
====== Signal 1, device addr=0xc0d95700, TriggerId=67108864
```
