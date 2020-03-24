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

SingleStep() --> fTriggeredEventHandlers[i]()
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
--> RTSPServer()
 // Arrange to handle connections from others:
env.taskScheduler().turnOnBackgroundReadHandling(fRTSPServerSocket,
                           (TaskScheduler::BackgroundHandlerProc*)&incomingConnectionHandlerRTSP, this);
--> incomingConnectionHandlerRTSP()
--> incomingConnectionHandler()

// client 
--> createNewClientConnection()
--> RTSPClientConnection()
--> incomingRequestHandler()
--> incomingRequestHandler1()
--> handleRequestBytes()
--> RTSPServer::RTSPClientSession::handleCmd_withinSession()  
--> RTSPServer::RTSPClientSession::handleCmd_PLAY()  
--> OnDemandServerMediaSubsession::startStream()
--> MediaSink::startPlaying()
--> MultiFramedRTPSink::continuePlaying()
--> MultiFramedRTPSink::getNextFrame()
--> MultiFramedRTPSink::afterGettingFrame();
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



## RTSP Server  

它的主要任务有两个:  

- 接收 Pushser 推流客户端的视频帧数据;  
- 给 Puller 拉流客户端下发视频帧数据.  

**关键的数据结构: RTSPServer**.  

1) 单例模式的 RTSP Server.  

创建一个 socket, 然后使用它初始化一个 RTSPServer 实例.  

```cpp
RTSPServer* 
RTSPServer::createNew( UsageEnvironment& env, 
    Port ourPort, UserAuthenticationDatabase* authDatabase,
    unsigned reclamationTestSeconds) 
{
    int ourSocket = setUpOurSocket(env, ourPort);
    if (ourSocket == -1) return NULL;

    return new RTSPServer(env, ourSocket, ourPort, authDatabase, reclamationTestSeconds);
}
```

setUpOurSocket() 函数流程如下:  

- 调用 socket(AF_INET, type, 0) 创建一个 socket;  
- 然后设置其发送缓存的大小为 50*1024;  
- 调用 listen() 函数来支持多路 rtsp 客户端连接;  
- 获取 socket 接口, 可以使用用户自定义的端口.  

2) RTSPServer 构造函数  

- 初始化类成员变量;  
- 配置忽略 Socket 的 signal pipeline;  
- 设置后台处理的 handler 为 incomingConnectionHandlerRTSP() 函数.  

```cpp
RTSPServer::RTSPServer(UsageEnvironment& env,
               int ourSocket, Port ourPort,
               UserAuthenticationDatabase* authDatabase,
               unsigned reclamationTestSeconds)
  : Medium(env), fRTSPServerPort(ourPort), 
    fRTSPServerSocket(ourSocket), fHTTPServerSocket(-1), fHTTPServerPort(0),
    fServerMediaSessions(HashTable::create(STRING_HASH_KEYS)),
    fClientConnections(HashTable::create(ONE_WORD_HASH_KEYS)),
    fClientConnectionsForHTTPTunneling(NULL), // will get created if needed
    fClientSessions(HashTable::create(STRING_HASH_KEYS)),
    fPendingRegisterRequests(HashTable::create(ONE_WORD_HASH_KEYS)), fRegisterRequestCounter(0),
    fAuthDB(authDatabase), fReclamationTestSeconds(reclamationTestSeconds),
    fAllowStreamingRTPOverTCP(True) 
{
    // so that clients on the same host that are killed don't also kill us
    ignoreSigPipeOnSocket(ourSocket); 

    // Arrange to handle connections from others:
    env.taskScheduler().turnOnBackgroundReadHandling(fRTSPServerSocket,
                            (TaskScheduler::BackgroundHandlerProc*)&incomingConnectionHandlerRTSP, this);
}
```

```cpp
void BasicTaskScheduler::setBackgroundHandling(
    int socketNum, int conditionSet, 
    BackgroundHandlerProc* handlerProc, void* clientData) 
{
    if (socketNum < 0) return;
#if !defined(__WIN32__) && !defined(_WIN32) && defined(FD_SETSIZE)
    if (socketNum >= (int)(FD_SETSIZE)) return;
#endif

    FD_CLR((unsigned)socketNum, &fReadSet);
    FD_CLR((unsigned)socketNum, &fWriteSet);
    FD_CLR((unsigned)socketNum, &fExceptionSet);
    
    if (conditionSet == 0) {
        fHandlers->clearHandler(socketNum);
        if (socketNum+1 == fMaxNumSockets) {
            --fMaxNumSockets;
        }
    } else {
        fHandlers->assignHandler(socketNum, conditionSet, handlerProc, clientData);
        if (socketNum+1 > fMaxNumSockets) {
            fMaxNumSockets = socketNum+1;
        }

        if (conditionSet&SOCKET_READABLE) 
            FD_SET((unsigned)socketNum, &fReadSet);
        if (conditionSet&SOCKET_WRITABLE) 
            FD_SET((unsigned)socketNum, &fWriteSet);
        if (conditionSet&SOCKET_EXCEPTION) 
            FD_SET((unsigned)socketNum, &fExceptionSet);
    }
}

void HandlerSet::assignHandler(int socketNum, 
    int conditionSet, 
    TaskScheduler::BackgroundHandlerProc* handlerProc, 
    void* clientData) 
{
    // First, see if there's already a handler for this socket:
    HandlerDescriptor* handler = lookupHandler(socketNum);

    if (handler == NULL) { // No existing handler, so create a new descr:
        handler = new HandlerDescriptor(fHandlers.fNextHandler);
        handler->socketNum = socketNum;
    }

    handler->conditionSet = conditionSet;
    handler->handlerProc = handlerProc;
    handler->clientData = clientData;
}
```

最终的效果只是将处理函数 incomingConnectionHandlerRTSP 绑定到 RTSP Server 而已. 后续应该会有 loop 函数进行异步处理.  

incomingConnectionHandlerRTSP() 函数处理连接到 RTSPServer 取流客户端请求, 是 RTSP Server 的重点.  

```cpp
// incomingConnectionHandlerRTSP() 
// --> incomingConnectionHandlerRTSP1() 
// --> incomingConnectionHandler(fRTSPServerSocket)

void RTSPServer::incomingConnectionHandlerRTSP(void* instance, int /*mask*/) 
{
  RTSPServer* server = (RTSPServer*)instance;
  server->incomingConnectionHandlerRTSP1();
}

void RTSPServer::incomingConnectionHandlerRTSP1() 
{
  incomingConnectionHandler(fRTSPServerSocket);
}

void RTSPServer::incomingConnectionHandler(int serverSocket) 
{
    struct sockaddr_in clientAddr;
    SOCKLEN_T clientAddrLen = sizeof clientAddr;
    int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrLen);
    if (clientSocket < 0) {
        int err = envir().getErrno();
        if (err != EWOULDBLOCK) {
            envir().setResultErrMsg("accept() failed: ");
        }
        return;
    }
    makeSocketNonBlocking(clientSocket);
    increaseSendBufferTo(envir(), clientSocket, 50*1024);

#ifdef DEBUG
    envir() << "accept()ed connection from " << AddressString(clientAddr).val() << "\n";
#endif

    // Create a new object for handling this RTSP connection:
    (void)createNewClientConnection(clientSocket, clientAddr);
}

```

一旦某个 VLC 取流客户端连接到 RTSP Server 时, 服务器就会调用 handlerProc() ( 即, incomingConnectionHandler() ) 创建一个 RTSPClientConnection 客户端连接对象, 以处理客户端请求.  

真正的客户端请求处理函数为 incomingRequestHandler().  








接下来关于 RTSPServer 的流程是: 


```cpp
rtspServer->addServerMediaSession(sms);
announceStream(rtspServer, sms, g_rtsp_server_ip, streamName, inputFileName);
```



## RTSP Client 客户端 

```cpp
RTSPServer::RTSPClientConnection::
RTSPClientConnection( RTSPServer& ourServer, 
    int clientSocket, struct sockaddr_in clientAddr)
  : fOurServer(ourServer), fIsActive(True),
    fClientInputSocket(clientSocket),
    fClientOutputSocket(clientSocket), fClientAddr(clientAddr),
    fRecursionCount(0), fOurSessionCookie(NULL) 
{
    // Add ourself to our 'client connections' table:
    fOurServer.fClientConnections->Add((char const*)this, this);

    // Arrange to handle incoming requests:
    resetRequestBuffer();
    envir().taskScheduler().setBackgroundHandling(
        fClientInputSocket, SOCKET_READABLE|SOCKET_EXCEPTION,
        (TaskScheduler::BackgroundHandlerProc*)&incomingRequestHandler, 
        this);
}

void RTSPServer::RTSPClientConnection::
incomingRequestHandler(void* instance, int /*mask*/) 
{
    RTSPClientConnection* session = (RTSPClientConnection*)instance;
    session->incomingRequestHandler1();
}

void RTSPServer::RTSPClientConnection::
incomingRequestHandler1() 
{
    struct sockaddr_in dummy; // 'from' address, meaningless in this case

    int bytesRead = readSocket(envir(), fClientInputSocket, &fRequestBuffer[fRequestBytesAlreadySeen], fRequestBufferBytesLeft, dummy);
    handleRequestBytes(bytesRead);
}

// parseRTSPRequestString() 
// handleCmd_OPTIONS();
// RTSPServer::RTSPClientSession::handleCmd_withinSession()  
// RTSPServer::RTSPClientSession::handleCmd_PLAY()  


void RTSPServer::RTSPClientConnection::
handleRequestBytes(int newBytesRead) 
{
    ...
    // Parse the request string into command name and 'CSeq', 
    // then handle the command:
    fRequestBuffer[fRequestBytesAlreadySeen] = '\0';
    Boolean parseSucceeded = parseRTSPRequestString(
                (char*)fRequestBuffer, fLastCRLF+2 - fRequestBuffer,
                cmdName, sizeof cmdName,
                urlPreSuffix, sizeof urlPreSuffix,
                urlSuffix, sizeof urlSuffix,
                cseq, sizeof cseq,
                sessionIdStr, sizeof sessionIdStr,
                contentLength);
    fLastCRLF[2] = '\r'; // restore its value
    Boolean playAfterSetup = False;
    if (parseSucceeded) {
     
        if (strcmp(cmdName, "OPTIONS") == 0) {
            // If the "OPTIONS" command included a "Session:" id for a session that 
            // doesn't exist, then treat this as an error:
            if (requestIncludedSessionId && clientSession == NULL) {
                handleCmd_sessionNotFound();
            } else {
                // Normal case:
                handleCmd_OPTIONS();
            }
        } else if (urlPreSuffix[0] == '\0' && urlSuffix[0] == '*'
                && urlSuffix[1] == '\0') {
            // The special "*" URL means: an operation on the entire server.  
            // This works only for GET_PARAMETER and SET_PARAMETER:
            if (strcmp(cmdName, "GET_PARAMETER") == 0) {
                handleCmd_GET_PARAMETER((char const*)fRequestBuffer);
            } else if (strcmp(cmdName, "SET_PARAMETER") == 0) {
                handleCmd_SET_PARAMETER((char const*)fRequestBuffer);
            } else {
                handleCmd_notSupported();
            }
        } else if (strcmp(cmdName, "DESCRIBE") == 0) {
            handleCmd_DESCRIBE(urlPreSuffix, urlSuffix, (char const*)fRequestBuffer);
        } else if (strcmp(cmdName, "SETUP") == 0) {
            Boolean areAuthenticated = True;

            if (!requestIncludedSessionId) {
                // No session id was present in the request.  So create 
                // a new "RTSPClientSession" object for this request.  
                // Choose a random (unused) 32-bit integer for the session id
                // (it will be encoded as a 8-digit hex number).  (We avoid choosing session id 0,
                // because that has a special use (by "OnDemandServerMediaSubsession").)

                // But first, make sure that we're authenticated to perform this command:
                char urlTotalSuffix[RTSP_PARAM_STRING_MAX];
                urlTotalSuffix[0] = '\0';
                if (urlPreSuffix[0] != '\0') {
                    strcat(urlTotalSuffix, urlPreSuffix);
                    strcat(urlTotalSuffix, "/");
                }
                strcat(urlTotalSuffix, urlSuffix);
                if (authenticationOK("SETUP", urlTotalSuffix, (char const*)fRequestBuffer)) {
                    u_int32_t sessionId;
                    do {
                        sessionId = (u_int32_t)our_random32();
                        sprintf(sessionIdStr, "%08X", sessionId);
                    } while (sessionId == 0 || fOurServer.fClientSessions->Lookup(sessionIdStr) != NULL);
                    clientSession = fOurServer.createNewClientSession(sessionId);
                    fOurServer.fClientSessions->Add(sessionIdStr, clientSession);
                } else {
                    areAuthenticated = False;
                }
            }
            if (clientSession != NULL) {
                clientSession->handleCmd_SETUP(this, urlPreSuffix, urlSuffix, (char const*)fRequestBuffer);
                playAfterSetup = clientSession->fStreamAfterSETUP;
            } else if (areAuthenticated) {
                handleCmd_sessionNotFound();
            }
        } else if ( strcmp(cmdName, "TEARDOWN") == 0
                    || strcmp(cmdName, "PLAY") == 0
                    || strcmp(cmdName, "PAUSE") == 0
                    || strcmp(cmdName, "GET_PARAMETER") == 0
                    || strcmp(cmdName, "SET_PARAMETER") == 0) {
            if (clientSession != NULL) {
            clientSession->handleCmd_withinSession(this, cmdName, urlPreSuffix, urlSuffix, (char const*)fRequestBuffer);
            } else {
            handleCmd_sessionNotFound();
            }
        } else if (strcmp(cmdName, "REGISTER") == 0) {
            // Because - unlike other commands - an implementation of this command needs
            // the entire URL, we re-parse the command to get it:
            char* url = strDupSize((char*)fRequestBuffer);
            if (sscanf((char*)fRequestBuffer, "%*s %s", url) == 1) {
            // Check for special command-specific parameters in a "Transport:" header:
            Boolean reuseConnection, deliverViaTCP;
            char* proxyURLSuffix;
            parseTransportHeaderForREGISTER((const char*)fRequestBuffer, reuseConnection, deliverViaTCP, proxyURLSuffix);

            handleCmd_REGISTER(url, urlSuffix, (char const*)fRequestBuffer, reuseConnection, deliverViaTCP, proxyURLSuffix);
            delete[] proxyURLSuffix;
            } else {
            handleCmd_bad();
            }
            delete[] url;
        } else {
            // The command is one that we don't handle:
            handleCmd_notSupported();
        }
    } else {
      // The request was not (valid) RTSP, but check for a special case: 
      // HTTP commands (for setting up RTSP-over-HTTP tunneling):
      //...
    }
    
    // 解析完之后给客户端发送一个响应
    send(fClientOutputSocket, (char const*)fResponseBuffer, 
        strlen((char*)fResponseBuffer), 0);
    
    if (playAfterSetup) {
        // The client has asked for streaming to commence now, rather than after a
        // subsequent "PLAY" command.  So, simulate the effect of a "PLAY" command:
        clientSession->handleCmd_withinSession(this, "PLAY", 
            urlPreSuffix, urlSuffix, (char const*)fRequestBuffer);
    }
    
    // Check whether there are extra bytes remaining in the buffer, after the end of the request (a rare case).
    // If so, move them to the front of our buffer, and keep processing it, because it might be a following, pipelined request.
    unsigned requestSize = (fLastCRLF+4-fRequestBuffer) + contentLength;
    numBytesRemaining = fRequestBytesAlreadySeen - requestSize;
    resetRequestBuffer(); // to prepare for any subsequent request
    
    if (numBytesRemaining > 0) {
      memmove(fRequestBuffer, &fRequestBuffer[requestSize], numBytesRemaining);
      newBytesRead = numBytesRemaining;
    }
  } while (numBytesRemaining > 0);
  
  --fRecursionCount;
  if (!fIsActive) {
    if (fRecursionCount > 0) closeSockets(); else delete this;
    // Note: The "fRecursionCount" test is for a pathological situation where we reenter the event loop and get called recursively
    // while handling a command (e.g., while handling a "DESCRIBE", to get a SDP description).
    // In such a case we don't want to actually delete ourself until we leave the outermost call.
  }
}

void RTSPServer::RTSPClientSession::
handleCmd_withinSession(
              RTSPServer::RTSPClientConnection* ourClientConnection,
              char const* cmdName,
              char const* urlPreSuffix, char const* urlSuffix,
              char const* fullRequestStr) 
{
    // This will either be:
    // - a non-aggregated operation, if "urlPreSuffix" is the session (stream)
    //   name and "urlSuffix" is the subsession (track) name, or
    // - an aggregated operation, if "urlSuffix" is the session (stream) name,
    //   or "urlPreSuffix" is the session (stream) name, and "urlSuffix" is empty,
    //   or "urlPreSuffix" and "urlSuffix" are both nonempty, but when concatenated, 
    // (with "/") form the session (stream) name.
    // Begin by figuring out which of these it is:
    ServerMediaSubsession* subsession;

    if (fOurServerMediaSession == NULL) { // There wasn't a previous SETUP!
        ourClientConnection->handleCmd_notSupported();
        return;
    } else if ( urlSuffix[0] != '\0' 
                && strcmp(fOurServerMediaSession->streamName(), urlPreSuffix) == 0) {
        // Non-aggregated operation.
        // Look up the media subsession whose track id is "urlSuffix":
        ServerMediaSubsessionIterator iter(*fOurServerMediaSession);
        while ((subsession = iter.next()) != NULL) {
            if (strcmp(subsession->trackId(), urlSuffix) == 0) break; // success
        }
        if (subsession == NULL) { // no such track!
            ourClientConnection->handleCmd_notFound();
            return;
        }
    } else if ( strcmp(fOurServerMediaSession->streamName(), urlSuffix) == 0 
                || ( urlSuffix[0] == '\0' 
                     && strcmp(fOurServerMediaSession->streamName(), urlPreSuffix) == 0)) {
        // Aggregated operation
        subsession = NULL;
    } else if (urlPreSuffix[0] != '\0' && urlSuffix[0] != '\0') {
        // Aggregated operation, if <urlPreSuffix>/<urlSuffix> is the session (stream) name:
        unsigned const urlPreSuffixLen = strlen(urlPreSuffix);
        if (strncmp(fOurServerMediaSession->streamName(), urlPreSuffix, urlPreSuffixLen) == 0 &&
            fOurServerMediaSession->streamName()[urlPreSuffixLen] == '/' &&
            strcmp(&(fOurServerMediaSession->streamName())[urlPreSuffixLen+1], urlSuffix) == 0) {
                subsession = NULL;
        } else {
            ourClientConnection->handleCmd_notFound();
            return;
        }
    } else { // the request doesn't match a known stream and/or track at all!
        ourClientConnection->handleCmd_notFound();
        return;
    }

    if (strcmp(cmdName, "TEARDOWN") == 0) {
        handleCmd_TEARDOWN(ourClientConnection, subsession);
    } else if (strcmp(cmdName, "PLAY") == 0) {
        handleCmd_PLAY(ourClientConnection, subsession, fullRequestStr);
    } else if (strcmp(cmdName, "PAUSE") == 0) {
        handleCmd_PAUSE(ourClientConnection, subsession);
    } else if (strcmp(cmdName, "GET_PARAMETER") == 0) {
        handleCmd_GET_PARAMETER(ourClientConnection, subsession, fullRequestStr);
    } else if (strcmp(cmdName, "SET_PARAMETER") == 0) {
        handleCmd_SET_PARAMETER(ourClientConnection, subsession, fullRequestStr);
    }
}
```

handleCmd_withinSession() 是根据 stream name 来处理对应的 rtsp 
