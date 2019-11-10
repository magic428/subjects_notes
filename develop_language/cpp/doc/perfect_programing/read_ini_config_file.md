## C++读取INI文件

`Windows操作系统`专门为此提供了`6个API`函数来对配置设置文件进行读、写：
```cpp
GetPrivateProfileInt() 			// 从私有初始化文件获取整型数值
GetPrivateProfileString() 		// 从私有初始化文件获取字符串型值
GetProfileInt 					// 从win.ini 获取整数值
GetProfileString 				// 从win.ini 获取字符串值
WritePrivateProfileString 		// 写字符串到私有初始化文件
WriteProfileString 				// 写字符串到win.ini
```

我们可以把视图类的`:OnInitialUpdate()`函数作为程序启动时读取配置文件的入口，配置文件的存储格式如下：

```ini
[SECTION 1]
XPos=300
YPos=200

[SECTION 2]
Text=Hello
```

仅有两个节，XPos和YPos标明了待显示信息的坐标，而待显示的信息存储在第二节的Text项中，用读取访问私有配置设置文件的API函数将其分别读入到变量m_nXPos,m_nYPos和m_strText中，并通过Invalidate()调用OnDraw()函数，在其内用TextOut函数将该信息在读取的坐标位置显示出来：

```cpp
m_nXPos = GetPrivateProfileInt("SECTION 1", //节名
							   "XPos", //项名
							   0, //没找到此项时的缺省返回值
							   "C:\test\debug\test.ini"); //配置文件的准确路径

m_nYPos = GetPrivateProfileInt("SECTION 1",
							   "YPos",
							   0,
							   exeFullPath);

char buf[256];

len = GetPrivateProfileString("SECTION 2", //节名
							  "Text", //项名
							  "No Text", //没找到此项时的返回值
							  buf, //目标缓冲区地址
							  256, //目标缓冲区长度
							  "C:\test\debug\test.ini"); //配置文件的准确路径

for(int i=0;i<len;i++)
{
   CString str;
   str.Format("%c",buf[i]);
   m_strText+=str;
}
Invalidate();
```
一般配置文件是和应用程序存放在同一个目录中的，如果用`"C:\test\debug\test.ini"`的绝对路径进行设置就会出现路径改变后找不到配置文件的问题，所以应`动态搜寻配置文件`的存放地址：
```cpp
	Tchar exeFullPath[MAX_PATH]; // MAX_PATH在API中有定义，为128
	int len=GetModuleFileName(NULL,
							  exeFullPath, //应用程序的全路径存放地址
							  MAX_PATH);
							  CString path="\test.ini"; //配置文件名
							  ::strcpy(exeFullPath+len-13,path); //组合出配置文件的全路径
```
值得注意的是这里的13是项目名的大小，但是不同项目可能名字不一样，定义这样的长度过于机械化

```cpp
	char *p = NULL;
	char exeFullPath[128];
	int len=GetModuleFileName(NULL,
	    exeFullPath,                128);
	p=strrchr(exeFullPath, '\\');   
	*p='\0';
```
这样，通过strrchr函数屏蔽掉最后出现的'\'就能够把项目名也屏蔽掉了，根据不同的情况当然也有不同的做法。

写配置文件也基本类似，只是需要把数值类型的变量格式化成字符串再行存储：
```cpp
str.Format("%d",m_nXPos);
WritePrivateProfileString("SECTION 1","XPos",str,exeFullPath);
str.Format("%d",m_nYPos);
WritePrivateProfileString("SECTION 1","YPos",str,exeFullPath);
WritePrivateProfileString("SECTION 2","Text",m_strText,exeFullPath);
```
我们一定遇到过这样的程序：在执行过一遍以后，重启系统会自动加载该程序，其实除了在启动菜单和注册表添加信息外，也可以用 `WriteProfileString()`函数向`win.ini`的"windows"节的"run"项目添加应用程序的全路径来实现，这要比其它两种方法简便的多，而且也比较安全。


二.将信息从INI文件中读入程序中的变量.

1.所用的WINAPI函数原型为:

DWORD GetPrivateProfileString(
LPCTSTR lpAppName,
LPCTSTR lpKeyName,
LPCTSTR lpDefault,
LPTSTR lpReturnedString,
DWORD nSize,
LPCTSTR lpFileName
);

其中各参数的意义:

前二个参数与 WritePrivateProfileString中的意义一样.

lpDefault : 如果INI文件中没有前两个参数指定的字段名或键名,则将此值赋给变量.

lpReturnedString : 接收INI文件中的值的CString对象,即目的缓存器.

nSize : 目的缓存器的大小.

lpFileName : 是完整的INI文件名.

2.具体使用方法:现要将上一步中写入的学生的信息读入程序中.

CString strStudName;
int nStudAge;
GetPrivateProfileString("StudentInfo","Name","默认姓名",strStudName.GetBuffer(MAX_PATH),MAX_PATH,"c:\stud\student.ini");

执行后 strStudName 的值为:"张三",若前两个参数有误,其值为:"默认姓名".

3.读入整型值要用另一个WINAPI函数:

UINT GetPrivateProfileInt(
LPCTSTR lpAppName,
LPCTSTR lpKeyName,
INT nDefault,
LPCTSTR lpFileName
);

这里的参数意义与上相同.使用方法如下:
nStudAge=GetPrivateProfileInt("StudentInfo","Age",10,"c:\stud\student.ini");




贴上自己WIN32测试并通过的一段例子（部分代码，主要功能是如何配置相对路径，后续操作，前面已经有了）

 1    char *p = NULL;
 2    char exeFullPath[128];
 3    int len=GetModuleFileName(NULL,
 4        exeFullPath,                    //应用程序的全路径存放地址
 5        128);
 6    p=strrchr(exeFullPath, '\\');        //屏蔽掉项目名称
 7    *p='\0';
 8    p=strrchr(exeFullPath, '\\');        //屏蔽掉DEBUG（实际开发中这个可能不需要）
 9    *p='\0';
10    len = strlen(exeFullPath);
11    string path="\\system.ini";            //配置文件名
12    ::strcpy(exeFullPath+len,path.c_str()); //组合出配置文件的全路径
13
14
15    char ipstr[20];                        //存储IP地址
16    GetPrivateProfileString("Server","ServerIP",NULL,ipstr,20,exeFullPath);
17    int port;
18    port = GetPrivateProfileInt("Server","Port",0,exeFullPath);


下面这段是公司里工作时候写的，做个记录

 1    //////////////////////////////////////////将内容以','分离
 2    string strFream = szFream;
 3    vector<string> strVec;
 4    char cTrim = ',';
 5    std::string::size_type pos1, pos2;
 6    pos2 = 0;
 7    while (pos2 != std::string::npos)
 8    {
 9        pos1 = strFream.find_first_not_of(cTrim, pos2);
10        if (pos1 == std::string::npos)
11            break;
12        pos2 = strFream.find_first_of(cTrim, pos1 + 1);
13        if (pos2 == std::string::npos)
14        {
15            if (pos1 != strFream.size())
16                strVec.push_back(strFream.substr(pos1)); 
17            break;
18        }
19        strVec.push_back(strFream.substr(pos1, pos2 - pos1));
20    }
21    for(int i = 0; i < strVec.size();++i)
22    {
23        int nTemp = atoi(strVec[i].c_str());
24        if(nTemp < m_nFrameNum)
25            m_vecFrame.push_back(nTemp);
26        else continue;
27    }

**注意**：
strtok
分解字符串为一组字符串。s为要分解的字符，delim为分隔符字符（如果传入字符串，则传入的字符串中每个字符均为分割符）。首次调用时，s指向要分解的字符串，之后再次调用要把s设成NULL。

自动查找文件路径
```cpp
	Tchar exeFullPath[1024]; // MAX_PATH在API中有定义，为128
	int len = GetModuleFileName(NULL,
		exeFullPath, //应用程序的全路径存放地址
		1024);
	CString path = "\config.ini"; //配置文件名
	::strcpy(exeFullPath + len - 13, path); //组合出配置文件的全路径
```
