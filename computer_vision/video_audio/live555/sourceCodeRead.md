## live555 源码  

signalNewFrameData() 之后会触发什么操作?  

CameraJPEGDeviceSource::eventTriggerId  
CameraJPEGDeviceSource  

到底扮演了什么样的角色.   

Note: Also, if you want to have  multiple device threads, each one using a different 'event trigger id', then you will need to make "eventTriggerId" a non-static member variable of "DeviceSource".  

fJpegFrameParser: 负责解析 JPEG 格式. 

taskScheduler 是事件处理的核心？   

### 1. 数据流向是什么? 

提交数据的是 CameraThread (关联的设备和事件为 CameraJPEGDeviceSource 及其 eventTriggerId).  

监听数据的一定是 rtsp server, 因此提交的数据到达 socket 后肯定被 server 收集到之后再转发. 

**RTSP server 在干啥?**  

turnOnBackgroundReadHandling()  

```cpp
rtspServer->addServerMediaSession(sms);
```

何为 ServerMediaSession? 每个推流的线程均需要告诉服务器推流的 stream name, 服务器会根据这个 stream name 创建一个对应的会话供其推流.  

@addServerMediaSession() 其实就是每一个将 'stream name' 映射到 "ServerMediaSession" 对象.  

```cpp
ourScheduler->triggerEvent( CameraJPEGDeviceSource::eventTriggerId, 
                            ourDevice );

SingleStep()->fTriggeredEventHandlers()
```

trigger 处理函数从哪来的?   

对于本程序, 处理函数就是 CameraJPEGDeviceSource::deliverFrame().  

```cpp
memmove(fTo, newFrameDataStart, fFrameSize);
/// After delivering the data, inform the reader that it is now available:
FramedSource::afterGetting(this);
```

可以看出 fAfterGettingFunc 应该就是 server 获取数据的函数.  

```cpp
// server
->RTSPServer()
->incomingConnectionHandlerRTSP()
->incomingConnectionHandler()

// client 
->createNewClientConnection()
->RTSPClientConnection()
->incomingRequestHandler()
->incomingRequestHandler1()
->handleRequestBytes()
->RTSPServer::RTSPClientSession::handleCmd_PLAY()  
->OnDemandServerMediaSubsession::startStream()
->MediaSink::startPlaying()
->MultiFramedRTPSink::continuePlaying()
->MultiFramedRTPSink::getNextFrame()
->MultiFramedRTPSink::afterGettingFrame();
```
最接近的一个发 rtsp 包函数为 RTPInterface::sendPacket(), 这个函数使用的是 UDP.   

那么如何设置 TCP 传输呢? 在 fRTPInterface.sendPacket() 之前调用: RTPInterface::setStreamSocket() 函数设置 TCP 即可. 如 setStreamSocket(684, 1); 
可以从 OnDemandServerMediaSubsession::startStream() 入手, 添加 setStreamSocket() 函数设置.  

### 2. 如何选择进行推流的网卡?   

- 手动设置一个接收推流的 IP 地址: @ReceivingInterfaceAddr;  
- 自动选择本地网卡 @ReceivingInterfaceAddr=INADDR_ANY && @clientSocket = -1;  
- 自动生成一个虚拟的 IP 地址: @ReceivingInterfaceAddr=INADDR_ANY && @clientSocket > 0;  

@ourIPAddress() 函数可以自动获取本地网卡地址, NetAddressList 可以获取本地所有可用的网卡.  

```cpp
// Try to resolve "hostname" to an IP address:
NetAddressList addresses(hostname);
NetAddressList::Iterator iter(addresses);
```

目前, 创建 socket 时系统会选择一张可用网卡, 因此对于多网卡情形, 有时无法按照用户意图绑定特定的网卡.  

**解决方案**:  

```cpp
// specify input interface, which means there are multiple IP interfaces. 
ReceivingInterfaceAddr = addr;
```
