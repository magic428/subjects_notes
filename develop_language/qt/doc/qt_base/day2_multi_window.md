# 多窗口设计中涉及到的控件 - QDialog, QmessageBox, QLineEdit  

> QDialog, QMessageBox, QLineEdit, QAction   

## QDialog

QDialog 运行 exec() 函数会显示对话框， 如果运行了 accept() 函数, 那么返回值应该是 Accepted.  

~~~cpp
...
LoginDlg dlg;                        // 建立自己新建的LoginDlg类的实例dlg
if(dlg.exec() == QDialog::Accepted) // 利用Accepted返回值判断按钮是否被按下
{
    w.show();                      // 如果被按下，显示主窗口
    return a.exec();              // 程序一直执行，直到主窗口关闭
}
...
~~~


## Login  


如果你的信号只是简单的触发一个窗体的动作, 那么直接使用信号连接到窗体动作对应的槽函数; 如果你想触发的除了窗体的动作之外, 还有别的动作, 那么应该自己实现槽函数, 在完成了自定义动作后显式调用窗体动作的槽函数.  

## QMessageBox

注意还要添加该类的头文件包含，即：#include <QMessageBox>   

~~~cpp
QMessageBox::warning(   this, 
                        tr("警告！"),
                        tr("用户名或密码错误！"),
                        QMessageBox::Ok );
~~~

Qt 中的 QMessageBox 类提供了多种常用的对话框类型， 比如这里的警告对话框，提示对话框、问题对话框等。  

这里使用了静态函数来设置了一个警告对话框， 这种方式很便捷， 其中的参数依次是：this 表明父窗口是登录对话框、窗口标题、界面显示的内容和最后要显示的按钮， 这里使用了一个 Ok 按钮。   



## QLineEdit  

1) 隐藏输入密码 

对于输入的密码，我们常见的是显示成小黑点的样式。   

点击 logindialog.ui 文件进入设计模式， 然后选中界面上的密码行编辑器，在属性编辑器中将 echoMode 属性选择为 Password。  

当然，除了在属性编辑器中进行更改，也可以在loginDialog类的构造函数中使用setEchoMode(QLineEdit::Password)函数来设置。  

2) 设置行编辑器的文本提示信息   

在行编辑器的属性栏中还可以设置占位符， 就是没有输入信息前的一些提示语句。  

例如将密码行编辑器的 placeholderText 属性更改为“请输入密码”，将用户名行编辑器的更改为“请输入用户名”，运行效果如下图所示。  

3) 去除行编辑器中的空格  

对于行编辑器，还有一个问题就是，比如我们输入用户名，在前面添加了一个空格，这样也可以保证输入是正确的，这个可以使用QString类的trimmed()函数来实现，它可以去除字符串前后的空白字符。  

4) 清空行编辑器的内容  

~~~cpp
ui->usrLineEdit->clear();
ui->usrLineEdit->setFocus();
~~~


## 代码中手动关联信号和槽函数  

下面来看下怎么使用代码自定义槽，然后手动进行关联。   

~~~cpp
connect(sender, signal, this, slot);
~~~


## 添加菜单和菜单图标  

在编辑动作对话框中的“图标”后面有 "选择图标文件"按钮, 黑色箭头下拉框可以选择使用资源还是使用文件来最为图标.   

如果使用文件的话，那么就可以直接在弹出的文件对话框中选择本地磁盘上的一个图标文件。

下面我们来讲述使用资源的方式，如果直接点击这个按钮就是默认的使用资源。    

1) 添加资源文件

我们向项目中添加新文件，模板选择Qt分类中的Qt资源文件（Qt Resource File）。 

添加完文件后会自动打开该资源文件， 需要先添加前缀，点击“添加”按钮，然后选择“添加前缀”，默认的前缀是“/new/prefix1”，这个可以修改为“/myimages”。   

然后再按下添加按钮来添加文件，这里最好将所有要用到的图片放到项目目录中。比如这里在项目目录中新建了一个images文件夹，然后将需要的图标文件粘贴进去。   

当添加完资源后，一定要按下Ctrl + S来保存资源文件，不然在后面可能无法显示已经添加的资源。  

2) 添加菜单项

可以在 UI 设计界面中添加, 也可以使用代码添加.  

下面使用代码再来添加一个菜单项，并为其设置图标。在编辑模式打开 mainwindow.cpp 文件，并在构造函数中添加如下代码：  

~~~cpp
{
    // 创建新的 action
    QAction *openAct = new QAction(tr("&open"), this);
    QIcon openIcon(":/myImages/menu_icon/new.svg");
    openAct->setIcon(openIcon);
    openAct->setShortcut(QKeySequence(tr("Ctrl+O")));
    ui->menu_File->addAction(openAct);
    
    // 创建新的编辑菜单, 同时创建新的 Action  
    QMenu *menu_Edit = ui->menuBar->addMenu(tr("编辑(&E)"));
    QAction *find_Act = new QAction(tr("&查找"), this);
    QIcon findIcon(":/myImages/menu_icon/find.svg");
    find_Act->setIcon(findIcon);
    find_Act->setShortcut(QKeySequence("Ctrl+F"));
    menu_Edit->addAction(find_Act);
}
~~~

菜单栏上可以通过 addMenu() 来添加新的菜单，而菜单中可以使用 addAction() 来创建菜单项。  
