# qt CheatSheet  

## 代码编辑快捷键  

|    快捷键    |         功能        |
|-------------|--------------------|
|F1           | 查看帮助            | 
|F2           | 跳转到函数定义, 和 Ctrl+鼠标左键 一样的效果 | 
|Shift+F2     | 声明和定义之间切换     |
|F4           | 头文件和源文件之间切换 |
|Ctrl+1       | 欢迎模式            |
|Ctrl+2       | 编辑模式            |
|Ctrl+3       | 调试模式            |
|Ctrl+4       | 项目设置模式         |
|Ctrl+5       | 帮助模式            |
|Ctrl+6       | 输出模式            |
|Alt+0        | 显示或者隐藏侧边条    |
|Esc          | 切换到编辑模式       |
|Alt+1        | 创建细节窗口         |
|Alt+2        | 搜索结果窗口         |
|Alt+3        | 程序输出窗口         |
|Alt+4        | 编译输出窗口         |
|Ctrl+B       | 编译工程            |  
|Ctrl+R       | 运行工程            |  
|Ctrl+I       | 自动对齐            |  
|Ctrl+/       | 注释行, 取消注释行   |
|Ctrl+Shift+< | 折叠代码块          |
|Ctrl+Shift+> | 展开代码块          |
|Ctrl+[       | 跳到代码块的头部     |
|Ctrl+]       | 跳到代码块的尾部     |
|Ctrl+L       | 跳到某一行          |
|Ctrl+F       | 查找替换当前选中的内容|
|Ctrl+Shift+F | 查找内容           |
|F5           | 开始调试           |
|Shift+F5     | 停止调试           |
|F9           | 设置和取消断点      |
|F10          | 单步前进           | 
|F11          | 单步进入函数        |
|Shift + F11  | 单步跳出函数        |
|Ctrl + [num] | 导航栏切换         |

在界面设计模式下, F3/F4 部件编辑和信号编辑操作切换.   

> 可以根据个人喜自己设置    

Ctrl + H  水平布局
Ctrl + L 垂直布局
Alt + Shift + r 预览

设置窗口的固定大小:   

```cpp
this->setMaximumSize(324,118);
this->setMinimumSize(324,118);
```

元对象编译系统 - 元对象编译器   

debug 函数 qDebug()   

## QString 和 数字之间的转换  

1) QString 转 int/float/double/long : 直接调用 toInt() / toFloat()/ toDouble() / toLong() 函数;  
2) int/float/double/long 转 QString, 使用 QString::number();   


## item 项目   

(QComboBox, QListWidget, QTreeWidget, QTableWidget)    

QMessageBox   

```cpp
QMessageBox::warning(this, "error", "divider can't be zero!!!");
QMessageBox::information(this,"result", QString::number(result));
time_POWERON = time_POWERON + 1;
// 判断是否为 8 的倍数, 也就是说 time_POWERON 每增加 8 ,条件成立一次.
if ((time_POWERON & 0x0200) == 0) {
    // 应该是设置 PCDATDIR 寄存器的值,对应的功能要查芯片手册 
    *PCDATDIR = *PCDATDIR | 0x0002;
} else {
    *PCDATDIR = *PCDATDIR | 0xFFFD;
}
```

## uic - ui 文件编译器  

uic: user interface complier.  

```bash
uic pacmanwindow.ui > ui_pacmanwindow.h
```

## QLabel   

```cpp
virtual void changeEvent(QEvent *ev)
virtual void contextMenuEvent(QContextMenuEvent *ev)
virtual bool event(QEvent *e)
virtual void focusInEvent(QFocusEvent *ev)
virtual bool focusNextPrevChild(bool next)
virtual void focusOutEvent(QFocusEvent *ev)
virtual void keyPressEvent(QKeyEvent *ev)
virtual void mouseMoveEvent(QMouseEvent *ev)
virtual void mousePressEvent(QMouseEvent *ev)
virtual void mouseReleaseEvent(QMouseEvent *ev)
virtual void paintEvent(QPaintEvent *)
```


## QLineEdit - 文本输入框   

```cpp
//输入文本后键入回车结束并执行
QObject::connect(ui->lineEdit, SIGNAL(returnPressed()), this, SLOT(executeSlot()));
// 获取 lineEdit 中的内容   
QString text = ui->lineEdit->text();
// 设置文本内容到 lineEdit 上
ui->lineEdit->setText("Just a QString");

```


## QFile - 文本编辑器   

1. 文本框的最大化(将文本框加入垂直布局)     
2. 设置窗口标题(setWindowTitle())   
3. 文件打开/保存   

```cpp
setFileName()
open()
```

## 文件对话框 - QFileDialog  

用户选择打开文件系统中的文件(调用静态方法).   

```cpp
QString QFileDialog::getSaveFileName()[static]     文件保存提示框.   
QString QFileDialog::getOpenFileName()[static]     文件打开提示框.   
QStringList QFileDialog::getOpenFileNames()[static]  多文件打开提示框.  ```

使用 `tr("Image(*.jpg, *.png, *.bmp)")` 过滤文件格式.   


## 目录 - QDir   

QDir::tempPath()[static]    
QDir::homePath()[static]    
QDir::rootPath()[static]     

## QTextStream    
文本编辑器中的文本流.   

5. QDataTime   
6. 设置字体和颜色   
QFontDialog   
QFont   
QColor   
QColorDialog   
QDateTime   


## 动态图标   
1. 资源文件   
add_prefix    
action Editor 添加图标   
将添加完毕的图标直接拖到工具栏   

2. 设置编辑器背景    
changeStyleSheet->border-image    

3. 明文加密   
ui->passwdLineEdit->setEchoMode(QLineEdit::Password);

## qApp
QApplication::instance() 应用程序实例化指针(全局).   
QObject::connect(ui->actionAbout_QT, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  

## QDesktopServices   
QDesktopServices::openUrl() 静态方法    
QDesktopServices::get
QDesktopServices::PicturesLocation, 系统目录->我的图片.  

## QMovie    
QMovie
this->movie = new QMovie("~/images/cool.gif");  
ui->movieLabel->setMovie(this->movie);  
this->movie->start();   

## QSplashScreen   
QT提供了 QSplashScreen 类, 我们可以使用此类给 QT 程序添加启动画面。但是此类只可以加载jpg等静态图像。   
如果我们需要播放 gif 图像, 一般使用 QMoive:    
```cpp
int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  QPixmap pixmap(":/splash.png");
  QSplashScreen splash(pixmap);
  splash.show();
  app.processEvents();
  ...
  QMainWindow window;
  window.show();
  splash.finish(&window);
  return app.exec();
}
``` 

## 打包程序    
ldd   
并没有讲完...   

## 关闭事件    
QCloseEvent   当程序需要关闭时处理的事件.   
所有的事件都是保护成员函数.     


## QPixmap   
存放一张图片.   

// 抓取图片.   
QPixmap::grabWindow(winID)[static]     
// 将 QPixmap 中的文件以fileName为文件名保存.   
QPixmap::save(fileName)   

## QDesktopWidget   
QApplication::desktop()->winID()   
winID(): 获取当前程序所在窗口所在的 ID.   
scaled(): 按比例缩放.  

## 系统剪切板 - QClipboard   
```cpp
QClipboard *clipboard = QGuiApplication::clipboard();
QString originalText = clipboard->text();
...
clipboard->setText(newText);
``` 


## 窗口最小化   



## 截取到的图片另存为    
`void QWidget::contextMenuEvent(QContextMenuEvent *event) [static]`    
This event handler, for event event, can be reimplemented in a subclass to receive widget context menu events.
The handler is called when the widget's contextMenuPolicy is Qt::DefaultContextMenu.
The default implementation ignores the context event. See the QContextMenuEvent documentation for more details.  

`QAction *QMenu::exec(const QPoint &p, QAction *action = Q_NULLPTR)`    
exec(QCursor::pos());    

## 启动一个系统程序    
QProcess    
process->start("notepad.exe")   

## 代码实现 action 和 menu    
```cpp
QMenu *menu = new QMenu();

QAction *actionSaveAs = new QAction(this);
QAction *actionProcess = new QAction(this);

Qobject::connect(actionSaveAs, SINGAL(triggered()), this, SLOT(saveAsSlot()));
Qobject::connect(actionProcess, SINGAL(triggered()), this, SLOT(processSlot()));

actionSaveAs->setText("Save as");
actionProcess->setText("start Notepad");

menu->addAction(actionSaveAs);
menu->addSeparator();
menu->addAction(actionProcess);

menu->exec(QCursor::pos());
```

## QTimer-定时器   
```
QTimer *timer = new QTimer(this);
connect(timer, SIGNAL(timeout()), this, SLOT(close()));
timer->start(1000);  // 以毫秒为单位
```   

## QString 转 char *   
// qt4 中被遗弃的函数   
string.toAscii(). data();   


## 临时文件 - QTemporaryFile     
QDir::tempPath()[static]    
QDir::homePath()[static]    
QDir::rootPath()[static]     


## 文件及文件夹属性操作 - QFileInfo   

常用方法.  

```cpp
QString file_full_path = QFileDialog::getOpenFileName(this,.....);
QFileInfo fileinfo(file_full_path);

fileinfo.fileName()          // 获取 basename 文件名
fileinfo.suffix()            // 文件后缀名
fileinfo.absolutePath()      // 获取绝对路径
fileinfo.isAbsolute()        // 是否是
fileinfo.isDir()             // 是否为目录文件.   
fileinfo.isFile()   
fileinfo.isHidden()   
fileinfo.isSymbLink()   
fileinfo.premission()   
fileinfo.create()      文件创建时间.   
fileinfo.lastRead()     上一次访问时间.   
fileinfo.size()       文件大小. 
```

一个实例.  

```cpp
#include <QFileInfo>
#include <QMessageBox>
#include <QDateTime>

void MainWindow::getFileInfoSlot()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File", QDir::homePath());
    if(fileName.isEmpty()) {
        QMessageBox::infomation(this, "File Empty", "Select a File");
    }

    // 文件大小
    QFileInfo info(fileName);
    ui->fileSizeLineEdit->setText(QString::number(info.size()));
    
    // 创建时间
    QDateTime createTime = info.create();
    ui->createTimeLineEdit->setText(createTime.toString("yyyy-mm-dd hh:mm:ss"));
    
    // 最后访问时间
    QDateTime lastTime = info.lastRead();
    ui->lastAccessLineEdit->setText(lastTime.toString("yyyy-mm-dd hh:mm:ss"));

    // 是否为目录
    if (info.isDir()) { 
        QMessageBox::infomation(this, "info", "File is a Dir");
    }
}
```

## 中文编码 - QTextCodec  

```cpp
#include <QTextCodec>

QTextCodec::setCodecForLocale(QTextCodec::codecForName("gb2312"));
QTextCodec::setCodecForCStrings(QTextCodec::codecForName("gb2312"));
QTextCodec::setCodecForTr(QTextCodec::codecForName("gb2312"));
```
 
## 列表框 - QListWidget / QListWidgetItem  

1. 可以执行插入和删除操作. QListWidgetItem    

```cpp
#include <QListWidget>
#include <QListWidgetItem>

void MainWindow::addCitySlot()
{
    QString cityName = ui->linEdit->text();
    QListWidgetItem item = new QListWidgetItem();
    item->setText(cityName); 

    ui->listWidget->addItem(item);
    ui->lineEdit_>clear();
}

void MainWindow::deleteCitySlot()
{
    int count ui->listWidget.count();
    QListWidgetItem *item = ui->listWidget->takeItem(ui->listWidget->current());
}

void MainWindow::deleteCitySlot()
{
    int count ui->listWidget.count();
    for(int i = 0; i < count; i++) {
        // 从头开始删除. 参数不能为 i, 必须为 0  
        QListWidgetItem *item = ui->listWidget->takeItem(0);
        delete item; 
    }
}
```

2. 可以使用QListWidget 的 setIcon() 函数为每个 item 设置图标.   

右键| Edit items | icon 中选择图标就可.   

3. 显示为图标方式(默认为列表方式):   

ui->listWidget->setViewMode(QListView::IconMode);  

4. QListWidget | signal

```cpp
/**
 * void itemClicked(QListWidgetItem *item)
 * void itemDoubleClicked(QListWidgetItem *item)
 * void itemEntered(QListWidgetItem *item)
*/
```

## QStringList - 字符串列表   

等价于 QList<Qstring> vector;   

```cpp
void MainWindow::showDirSlit()
{
    ui-> listWidget->clear();
    // 获得 fileNames 的话不要使用 QFileDialog::getOpenFileNames().  
    QDir dir = QDir::current();
    QStringList fileNames = dir.entryList();
    
    int size = fileNames.size();
    if(size == 0) {
        cout << "error" << endl;
        return;
    } 

    for(int index = 0; index < size; index++) {
        if(fileNames.at(index) == "." || fileNames.at(index) == "..") {
            continue;
        }

        QListWidgetItem *item = new QListWidgetItem();
        item->setText(fileNames.at(index));
        ui->listWidget->addItem(item);
    }

}
```

## QSharedMemory - 共享内存   


## QImage    

QImage image; 
image.load(fileName);   

## QDataStream    

QDataStream in(&buf);

## QBuffer   

QBuffer buf; 
buf->open(QIODevice::ReadWrite);   


## QWebEngineView   

QT += webenginewidgets
第三方的开源库(Js->Qt->OS).   

1. 打开一个浏览器界面    

```cpp
QWebEngineView *view = new QWebEngineView(parent);
view->load(QUrl("http://www.baidu.com"));
view->show();
```

2. 创建一个浏览器   

可以给 QWebEngineView 设置一个默认的 url.   

```cpp
Public Slots:   
void back()
void forward()
void reload()
void stop()


Signals:
void iconChanged(const QIcon &icon)
void iconUrlChanged(const QUrl &url) 
void loadFinished(bool ok)             // 加载完成   
void loadProgress(int progress)
void loadStarted()
void renderProcessTerminated(QWebEnginePage::RenderProcessTerminationStatus terminationStatus, int exitCode)
void selectionChanged()
void titleChanged(const QString &title)
void urlChanged(const QUrl &url)
```

```cpp
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow
{
    this->setUp(ui);

    // 显示网址 
    QObject::connect(ui->webView, SIGNAL(urlChanged(QUrl)), this, SLOT(setUrlSlot(QUrl)));

    // 浏览网址  
    QObject::connect(ui->lineEdit, SIGNAL(returnPressed), this, SLOT(loadUrlSlot()));

    // 实例化进度条并将进度条加载到状态栏   
    this->progress = new QProgressBar;
    ui->statusBar->addWidget(this->progress);
    QObject::connect(ui->webView, SIGNAL(loadProcess(int)), this->progress, SLOT(setVal(int)));

    // 加载完成后显示状态栏消息
    QObject::connect(ui->webView, SIGNAL(loadFinished(bool), this, SLOT(loadFinishSlot(bool))));

    // 设置浏览器标题  
    QObject::connect(ui->webView, SIGNAL(titleChanged(QString), this, SLOT(setTitleSlot(QString))));
}

// 实时显示浏览器的网址   
void MainWindow::setUrlSlot(QUrl url)
{
    // url 参数是 signal 传递过来的.   
    ui->lineEdit->setText(url.toString());
}

// 网址输入完毕后按回车加载网页  
void MainWindow::loadUrlSlot()
{
    ui->webView->load(QUrl(ui->lineEdit->text()));
}

void MainWindow::loadFinishSlot(bool ok)
{
    if(ok)
        ui->statusBar->showMessage("Load Finished...", 5*1000);
}

void MainWindow::setTitleSlot(QString title)
{
    this->setWindowTitle(title);
}
```

## QProgressBar   

```cpp
Public Slots:
void reset()
void setMaximum(int maximum)
void setMinimum(int minimum)
void setOrientation(Qt::Orientation)
void setRange(int minimum, int maximum)
void setValue(int value)
```

## QML   

UI 与 UX (用户界面和用户体验)   
QML 语言    
QML 元素  
QML 布局   
QML 元素实例   

## 使用配置文件 (.ini - .conf)  

1. 初始化网页主页    

```cpp
// mainPage.ini 文件中只有下面一行    
// mainpage ="http://www.baidu.com"    

void MainWindow::initMainPage()
{
    QString configFile = "mainPage.ini";
    QFile *file = new QFile(configFile);
    bool retVal = file->open(QIODevice::ReadOnly);

    if(retVal) {
      QTextStream in(file);
      QStringList items = in.readLine().split("=");
      QString url = ""; 
      if(items.size() == 2) {
          url = items.at(1)
      } else {
        QMessageBox::information(this,"Error", "Config File error");

          return;
      }

      ui->webView->load(QUrl(url));

      file->close();
      delete file;
      file = NULL;
    } else {
        QMessageBox::information(this,"Error", "Open File Error "+file->errorString());
    }
}
```

2. 查看浏览器网页内容   

```cpp
// page -> frame -> toHtml 
QString context = ui->webView->page()->currentFrame()->toHtml();
ui->textEdit->setPlainText(context);  // 纯文本而不是富文本(setText())
```

3. 设置主页   

```cpp
void MainWindow::setMainPageSlot()
{
    QString configFile = "mainPage.ini";
    QFile *file = new QFile(configFile);
    bool retVal = file->open(QIODevice::WriteOnly);

    if(retVal) {
      QTextStream out(file);
      out << "mainpage=" + ui->lineEdit()->text();

      ui->webView->load(QUrl(ui->lineEdit()->text()));

      file->close();
      delete file;
      file = NULL;
      QMessageBox::information(this,"Info", "Main page has changed");
    } else {
        QMessageBox::information(this,"Error", "Set MainPage Error "+file->errorString());
    }
}
```

```bash
(labelImage:19826): GLib-GObject-WARNING **: cannot register existing type 'GdkWindow'

(labelImage:19826): GLib-CRITICAL **: g_once_init_leave: assertion 'result != 0' failed

(labelImage:19826): GLib-GObject-CRITICAL **: g_type_register_static: assertion 'parent_type > 0' failed

(labelImage:19826): GLib-CRITICAL **: g_once_init_leave: assertion 'result != 0' failed
"/home/klm/data/frame" 
```

```cpp
bool mouseTracking
```

这个属性保存的是窗口部件跟踪鼠标是否生效。如果鼠标跟踪失效 (默认), 当鼠标被移动的时候只有在至少一个鼠标按键被按下时, 这个窗口部件才会接收鼠标移动事件。如果鼠标跟踪生效, 如果没有按键被按下, 这个窗口部件也会接收鼠标移动事件。也可以参考 mouseMoveEvent () 和 QApplication::setGlobalMouseTracking ()。通过 setMouseTracking() 设置属性值并且通过 hasMouseTracking() 来获得属性值。  
 
调用这个函数后, 如想使 mouseMoveEvent 有效, 也就是在鼠标在区域内移动就会触发, 而非鼠标按键按下时才触发, 注意只能是 QWidget, 如果是 QMainwindow, 则无效。  


2. Qt 中 mouseMoveEvent 在 MainWindow 中使用  

最近用 Qt 软件界面, 需要用到 mouseMoveEvent, 在实际使用中发现一些问题, 分享一下。  
        在Qt中要捕捉鼠标移动事件需要重写MouseMoveEvent, 但是MouseMoveEvent为了不太耗资源在默认状态下是要鼠标按下才能捕捉到。要想鼠标不按下时的移动也能捕捉到, 需要setMouseTracking(true)。
bool mouseTracking
这个属性保存的是窗口部件跟踪鼠标是否生效。
如果鼠标跟踪失效 (默认) , 当鼠标被移动的时候只有在至少一个鼠标按键被按下时, 这个窗口部件才会接收鼠标移动事件。
如果鼠标跟踪生效, 如果没有按键被按下, 这个窗口部件也会接收鼠标移动事件。

        QWidget中使用是没有问题的, 但是, 对于QMainWindow即使使用了setMouseTracking(true)依然无法捕捉到鼠标没有按下的移动, 只有在鼠标按下是才能捕捉。

        解决办法：要先把QMainWindow的CentrolWidget使用setMouseTracking(true)开启移动监视。然后在把QMainWindow的setMouseTracking(true)开启监视。之后就一切正常了。

原因：CentrolWIdget是QMainWindow的子类, 你如果在子类上响应鼠标事件, 只会触发子类的mouseMoveEvent, 根据C++继承和重载的原理, 所以子类也要setMouseTracking(true); 所以如果你想响应鼠标事件的控件被某个父控件包含, 则该控件及其父控件或容器也需要setMouseTracking(true);

3. 亲测可用  

今天也是遇到了这个问题, 并非只在QWidget中设置setMouseTracking(true)才好用, 如若在QMainwindow中设置为true还是不能跟踪, 解决办法为在ui中的属性栏主窗口的“mouseTracking”属性勾选上, 就解决了。希望对看回复的朋友有帮助。

```cpp
// 禁止拷贝构造函数的扯淡坑   
virtual void changeEvent(QEvent *ev)
virtual void contextMenuEvent(QContextMenuEvent *ev)
virtual bool event(QEvent *e)
virtual void focusInEvent(QFocusEvent *ev)
virtual bool focusNextPrevChild(bool next)
virtual void focusOutEvent(QFocusEvent *ev)
virtual void keyPressEvent(QKeyEvent *ev)
virtual void mouseMoveEvent(QMouseEvent *ev)
virtual void mousePressEvent(QMouseEvent *ev)
virtual void mouseReleaseEvent(QMouseEvent *ev)
virtual void paintEvent(QPaintEvent *)
```

## Qt弹出对话框"QMessageBox"的按钮显示改为中文  

QMessageBox 是 Qt 框架下专门用于弹出对话框的类, 一般会提供几个按钮 (例如：Ok、Cancel、Yes、No、Ignore等) 供用户点击选择。对话框的按钮在默认情况下为英文显示, 本文提供了一种简单快速的修改为中文的方法。  

```cpp
// 创建一个question弹出对话框, 添加两个按钮：Yes和No
QMessageBox *msgBox = new QMessageBox(QMessageBox::Question, 
                                      QString::fromLocal8Bit("对话框标题"), 
                                      QString::fromLocal8Bit("对话框内容"), 
                                      QMessageBox::Yes | QMessageBox::No);

// 将原本显示“Yes”的按钮改为显示“是”
msgBox->button(QMessageBox::Yes)->setText(QString::fromLocal8Bit("是"));

// 将原本显示“No”的按钮改为显示“否”
msgBox->button(QMessageBox::No)->setText(QString::fromLocal8Bit("否"));

// 隐藏 No button
msgBox->button(QMessageBox::No)->setHidden(true);

// 弹出对话框
msgBox->exec();
```

**实例**  

温馨小提示：上述方法是将默认的英文按钮改为中文按钮, 其实完全可以任意自定义按钮的显示, 例如在制作一个音乐播放器软件时, 当音乐播放停止后, 自动弹出对话框, 用户点击 “Play” 按钮则选择继续播放, 用户点击 “Stop” 按钮则停止播放, 就可以用下面的程序实现。  

```cpp
msgBox->button(QMessageBox::Yes)->setText("Play");

// 将原本显示“No”的按钮改为显示“Stop”
msgBox->button(QMessageBox::No)->setText("Stop");

// 启动对话框, 用res变量记录用户最终点选的按钮
int res = msgBox->exec();
if(QMessageBox::Yes == res)
    Play();     // 继续播放
else if(QMessageBox::No == res)
    Stop();     // 停止播放
else
    ...
```
