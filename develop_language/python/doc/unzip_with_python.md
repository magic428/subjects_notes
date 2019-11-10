# python 解压压缩包的几种方法

这里讨论使用Python解压例如以下五种压缩文件：

.gz .tar  .tgz .zip .rar

简单介绍
gz： 即gzip。通常仅仅能压缩一个文件。与tar结合起来就能够实现先打包，再压缩。

tar： linux系统下的打包工具。仅仅打包。不压缩

tgz：即tar.gz。先用tar打包，然后再用gz压缩得到的文件

zip： 不同于gzip。尽管使用相似的算法，能够打包压缩多个文件。只是分别压缩文件。压缩率低于tar。

rar：打包压缩文件。最初用于DOS，基于window操作系统。

压缩率比zip高，但速度慢。随机訪问的速度也慢。

关于zip于rar之间的各种比較。可见： 

http://www.comicer.com/stronghorse/water/software/ziprar.htm

gz
因为gz一般仅仅压缩一个文件，全部常与其它打包工具一起工作。比方能够先用tar打包为XXX.tar,然后在压缩为XXX.tar.gz

解压gz，事实上就是读出当中的单一文件，Python方法例如以下：

import gzip
import os
def un_gz(file_name):
    """ungz zip file"""
    f_name = file_name.replace(".gz", "")
    #获取文件的名称，去掉
    g_file = gzip.GzipFile(file_name)
    #创建gzip对象
    open(f_name, "w+").write(g_file.read())
    #gzip对象用read()打开后，写入open()建立的文件里。
    g_file.close()
    #关闭gzip对象

tar
XXX.tar.gz解压后得到XXX.tar，还要进一步解压出来。

*注：tgz与tar.gz是同样的格式，老版本号DOS扩展名最多三个字符，故用tgz表示。

因为这里有多个文件，我们先读取全部文件名称。然后解压。例如以下：

import tarfile
def un_tar(file_name):
       untar zip file"""
    tar = tarfile.open(file_name)
    names = tar.getnames()
    if os.path.isdir(file_name + "_files"):
        pass
    else:
        os.mkdir(file_name + "_files")
    #因为解压后是很多文件，预先建立同名目录
    for name in names:
        tar.extract(name, file_name + "_files/")
    tar.close()

*注：tgz文件与tar文件同样的解压方法。



zip
与tar类似，先读取多个文件名称，然后解压。例如以下：

import zipfile
def un_zip(file_name):
    """unzip zip file"""
    zip_file = zipfile.ZipFile(file_name)
    if os.path.isdir(file_name + "_files"):
        pass
    else:
        os.mkdir(file_name + "_files")
    for names in zip_file.namelist():
        zip_file.extract(names,file_name + "_files/")
    zip_file.close()


rar
由于rar通常为window下使用，须要额外的Python包rarfile。

可用地址： http://sourceforge.net/projects/rarfile.berlios/files/rarfile-2.4.tar.gz/download

解压到Python安装文件夹的/Scripts/文件夹下，在当前窗体打开命令行,

输入Python setup.py install

安装完毕。

import rarfile
import os
def un_rar(file_name):
    """unrar zip file"""
    rar = rarfile.RarFile(file_name)
    if os.path.isdir(file_name + "_files"):
        pass
    else:
        os.mkdir(file_name + "_files")
    os.chdir(file_name + "_files"):
    rar.extractall()
    rar.close()



tar打包
在写打包代码的过程中，使用tar.add()添加文件时，会把文件本身的路径也加进去，加上arcname就能依据自己的命名规则将文件添加tar包
打包代码：
#!/usr/bin/env /usr/local/bin/python  
 # encoding: utf-8  
 import tarfile  
 import os  
 import time  
  
 start = time.time()  
 tar=tarfile.open('/path/to/your.tar,'w')  
 for root,dir,files in os.walk('/path/to/dir/'):  
         for file in files:  
                 fullpath=os.path.join(root,file)  
                 tar.add(fullpath,arcname=file)  
 tar.close()  
 print time.time()-start  


在打包的过程中能够设置压缩规则,如想要以gz压缩的格式打包
tar=tarfile.open('/path/to/your.tar.gz','w:gz')
其它格式例如以下表：
tarfile.open的mode有非常多种：
mode action
'r' or 'r:*'	Open for reading with transparent compression (recommended).
'r:'	Open for reading exclusively without compression.
'r:gz'	Open for reading with gzip compression.
'r:bz2'	Open for reading with bzip2 compression.
'a' or 'a:'	Open for appending with no compression. The file is created if it does not exist.
'w' or 'w:'	Open for uncompressed writing.
'w:gz'	Open for gzip compressed writing.
'w:bz2'	Open for bzip2 compressed writing.
 
tar解包
tar解包也能够依据不同压缩格式来解压。
#!/usr/bin/env /usr/local/bin/python  
 # encoding: utf-8  
 import tarfile  
 import time  
  
 start = time.time()  
 t = tarfile.open("/path/to/your.tar", "r:")  
 t.extractall(path = '/path/to/extractdir/')  
 t.close()  
 print time.time()-start  
 
 
上面的代码是解压全部的，也能够挨个起做不同的处理，但要假设tar包内文件过多，小心内存哦~
tar = tarfile.open(filename, 'r:gz')  
for tar_info in tar:  
    file = tar.extractfile(tar_info)  
    do_something_with(file)  