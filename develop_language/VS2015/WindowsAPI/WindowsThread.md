# 线程的使用

## 线程的创建

 

## 信号量
在开发软件的过程中，多线程的程序往往需要实现相互通讯，比如这种场景：几个线程添加一个消息到队列里，而另一个线程在睡眠时，就需要唤醒那个线程来处理事情。这时，就需要使用到信号量来进行同步。`CreateSemaphore`是创建信号量，`ReleaseSemaphore`是增加信号量。

1. 函数声明
函数`CreateSemaphore`和`ReleaseSemaphore`的声明如下：
```cpp
WINBASEAPI
__out
HANDLE
WINAPI
CreateSemaphoreA(
    __in_opt LPSECURITY_ATTRIBUTES lpSemaphoreAttributes,
    __in     LONG lInitialCount,
    __in     LONG lMaximumCount,
    __in_opt LPCSTR lpName
    );
   
WINBASEAPI
__out
HANDLE
WINAPI
CreateSemaphoreW(
    __in_opt LPSECURITY_ATTRIBUTES lpSemaphoreAttributes,
    __in     LONG lInitialCount,
    __in     LONG lMaximumCount,
    __in_opt LPCWSTR lpName
    );

#ifdef UNICODE
#define CreateSemaphore CreateSemaphoreW
#else
#define CreateSemaphore CreateSemaphoreA
#endif // !UNICODE
 
WINAPI
ReleaseSemaphore(
    __in      HANDLE hSemaphore,
    __in      LONG lReleaseCount,
    __out_opt LPLONG lpPreviousCount
    );
```
**对于创建信号量**：
`lpSemaphoreAttributes`是信号量的安全属性。
`lInitialCount`是初始化的信号量。
`lMaximumCount`是允许信号量增加到最大值。
`lpName`是信号量的名称。
**对于释放信号量**：
`hSemaphore`是要增加的信号量句柄。
`lReleaseCount`是增加的计数。
`lpPreviousCount`是增加前的数值返回。
2. 信号量使用实例
调用函数的例子如下：
```cpp
 //线程运行函数。
 //在这里可以使用类里的成员，也可以让派生类实现更强大的功能。
 DWORD CThreadSemaphore::Run(void)
 {
  //输出到调试窗口。
  ::OutputDebugString(_T("Run()线程函数运行/r/n"));     
 
  //
  const LONG cMax = 10;
   m_hSemaphore = CreateSemaphore(
        NULL,   // 缺省的安全属性。
        0,   // 初始化为0个信号量。
        cMax,   // 最大为10个信号量。
        NULL); // 不命名。
 
  if (m_hSemaphore == NULL)
  {
         return -1;
  }
 
  //
  const int nMaxObjs = 2;
  HANDLE hWaitObjects[nMaxObjs] = {m_hEventExit,m_hSemaphore};
 
  //线程循环。
  for (;;)
  {
         DWORD dwRet = WaitForMultipleObjects(nMaxObjs,hWaitObjects,FALSE,INFINITE);
         if (dwRet == WAIT_TIMEOUT)
         {
               //可以继续运行。                
               TCHAR chTemp[128];
               wsprintf(chTemp,_T("CThreadSemaphore::Run() ThreadID=%d/r/n"),m_dwThreadID);
               ::OutputDebugString(chTemp);
 
               //目前没有做什么事情，就让线程释放一下CPU。
               Sleep(10);
         }
         else if (dwRet == WAIT_OBJECT_0)
         {
               //退出线程。
               ::OutputDebugString(_T("Run() 退出线程/r/n"));
               break;
         }
         else if (dwRet == WAIT_OBJECT_0+1)
         {
               //可以继续运行。                
               TCHAR chTemp[128];
               wsprintf(chTemp,_T("CThreadSemaphore::Run() Semaphore,ThreadID=%d/r/n"),m_dwThreadID);
               ::OutputDebugString(chTemp);
 
               //
 
         }
         else if (dwRet == WAIT_ABANDONED)
         {
               //出错。
               ::OutputDebugString(_T("Run() 线程出错/r/n"));
               return -1;
         }
  }
 
  //
  if (m_hSemaphore)
  {
         CloseHandle(m_hSemaphore);
          m_hSemaphore = NULL;
  }
 
  return 0;
 }
 
第11行就是创建信号量。
第29行等信号量事件和退出事件。
 

 
  //
  //增加信号量
  //蔡军生 2007/10/10 QQ:9073204 深圳
  //
  void IncSemaphore(void)
  {
         if (m_hSemaphore)
         {
              if (!ReleaseSemaphore(
                    m_hSemaphore, // 要增加的信号量。
                   1,           // 增加1.
                   NULL) )      // 不想返回前一次信号量。
               {
 
               }
         }
  }
  
```


```
#include <iostream>
#include <process.h>
#include <Windows.h>

using namespace std;

void callBack(void * userdata)
{
	cout << "callBack executing... userdata=" << *((int*)userdata) << endl;
	cout << endl;

}

void callBack2(void * userdata)
{
	cout << "callBack executing2... userdata=" << *((int*)userdata) << endl;
}

int main()
{
	int arg = 10;
	int arg2 = 15;
	_beginthread(callBack, 0, (void *)&arg);
	Sleep(500);
	_beginthread(callBack2, 0, (void *)&arg2);
	
	while (1){
		Sleep(10);
	}

	return 0;
}

```







































## 使用实例
```cpp
#include <iostream>
#include <process.h>
#include <Windows.h>

using namespace std;

void callBack(void * userdata)
{
	cout << "callBack executing... userdata=" << *((int*)userdata) << endl;
	cout << endl;

}

void callBack2(void * userdata)
{
	cout << "callBack executing2... userdata=" << *((int*)userdata) << endl;
}

int main()
{
	int arg = 10;
	int arg2 = 15;
	_beginthread(callBack, 0, (void *)&arg);
	Sleep(500);
	_beginthread(callBack2, 0, (void *)&arg2);
	
	while (1){
		Sleep(10);
	}

	return 0;
}
```