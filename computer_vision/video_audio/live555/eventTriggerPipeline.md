# live555 源码阅读 - Event Trigger  

> taskScheduler().doEventLoop() --> SingleStep()  

## 从推流 Push 的用户角度来看  

用户在准备好一帧 JPEG 图片后, 需要调用 signalNewFrameData() 函数通知 RTSP 服务器当前有一帧数据到来.   

signalNewFrameData() 函数只是完成信号上报工作, 并不执行其他复杂功能.  

- 首先, 记录客户端数据 clientData;  
- 然后, 将该 eventTriggerId 标记为待处理状态. (在最后将其标记为待处理状态, 是否会由于竟态导致标记失败?)  

**那么之后 RTSP 会进行什么操作呢?**  

所有的推流线程启动完毕之后, 就会启动 doEventLoop() 函数. 该函数会重复处理可读的 sockets 和到期需要处理的 events.  

SingleStep() 函数的处理流程:  

- select 监听 socket 上的数据;   
- 从上一个处理过的 socket num (fLastHandledSocketNum) 开始处理可读 socket; 具体处理函数为 (*handler->handlerProc)(handler->clientData, resultConditionSet);
- 然后处理刚刚被触发的事件. 在 socket handler 之后处理被触发的事件, 是为了避免事件 handler 修改了 readable sockets set;  
- 最后 handleAlarm() 处理 "delayed event".  

**TODO: (*handler->handlerProc) 具体是哪个函数?**

```cpp


```




到底扮演了什么样的角色.   

Note: Also, if you want to have  multiple device threads, each one using a different 'event trigger id', then you will need to make "eventTriggerId" a non-static member variable of "DeviceSource".  

```cpp
ourScheduler->triggerEvent( CameraJPEGDeviceSource::eventTriggerId, 
                            ourDevice );

SingleStep() --> fTriggeredEventHandlers[i]()
```

那么处理 trigger 的函数从哪来的?   

对于本程序, 处理函数就是 CameraJPEGDeviceSource::deliverFrame().  



## 从拉流 Pull 的用户角度来看  

一般来说, 可以使用 VLC 等拉流客户端.  

**重新连接和断开连接时会发生什么?**  

这应该是 RTSP server 的负责部分.  


