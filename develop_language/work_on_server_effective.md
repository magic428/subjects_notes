# 如何与深度学习服务器优雅的交互？

-- 转自小夕的专栏  

- ssh一键免密登录
- 内网穿透（跨网段访问服务器）
- 文件传输与实时同步
- 多开发环境管理
- 多任务管理（并行调参）
- 睡觉调参模式（串行调参）
- 关于Jupyer Notebook
- 单任务霸占GPU模式

## 一键免密远程登录   

远程登录最最方便的当然就是 ssh 啦。实现这个非常简单，分两步：   

首先，将你的登录命令写入你本地 pc 端的 bash 启动脚本中。linux系统为 ~/.bashrc。例如你的服务器用户名为 dlnlp, ip 为 102.10.60.23， 那么就把这句登录命令写进去：    

~~~bash
alias sshdlnlp="ssh dlnlp@102.10.60.23"
~~~

当然，登录命令叫 sshdlnlp， 你也可以改成别的。保存后别忘了更新 bash 启动脚本:     

~~~bash
source ~/.bashrc
~~~

然后，经过第一步后，只需要再敲密码就可以进入啦。但是懒癌至深的我们怎么能容忍敲密码这么麻烦的事情呢！但是我们又不能牺牲服务器的安全性，那怎么办呢？    

也很简单，把本地 PC 端的 ssh 公钥写入服务器的 ssh 信任列表里就可以啦。

首先用 `ssh-keygen` 命令生成 rsa 密钥对（生成一只私钥和一只公钥）, 一路 enter, 一路next～   

然后去`~/.ssh/` 文件夹下将公钥发送到服务器上的某文件夹里.然后去服务器上，把你本地 PC 端的公钥丢进 ssh 信任列表：  

~~~bash
cat id_rsa.pub >> ~/.ssh/authorized_keys
~~~

好啦～搞定啦，再回到你的 PC 端登录试试吧，是不是连输入密码都省掉啦。    

## 内网穿透（跨网段访问服务器）   

但是注意哦， 如果你的服务器是在局域网内， 那你的 PC 离开这个局域网的时候当然就找不到你的服务器啦。想要在家里用GPU服务器？ 这就需要内网穿透！   

在内网穿透方面，小夕试了好几种方案后，感觉还是花生壳对新手最友好也最稳定。我们的内网穿透只需要将服务器内网 ip 以及 22 端口号（即 ssh 端口号）映射到外网 ip 的某个端口号。  

这个过程使用花生壳非常简单，在网上有很多教程。之后我们要做的就是将这个外网 ip 和端口号也封装成一条命令，比如花生壳分配给我们的外网ip是 103.44.145.240，端口是 12560，那么只需要把这个写入客户端 shell 启动脚本：    

~~~bash
alias sshdlnlp_remote="ssh -p 12560 dlnlp@103.44.145.240"   
~~~ 

（别忘用 source 刷新启动脚本）  

之后就可以在世界各地用一条命令访问你的 gpu 服务器啦。哦对了，网址 https://hsk.oray.com/download/

**花生壳安装**
 +--------------------------------------------------+
 |             Oray PeanutHull Linux 3.0            |
 +--------------------------------------------------+
 |  SN: ORAY00acc058b07c   Default password: admin  |
 +--------------------------------------------------+
 |    Remote Management Address http://b.oray.com  

赠送一个免费域名: f19f589649.51mypc.cn
帐号：klm200x11
获赠一个免费域名：t1958972r0.iok.la

原价18元，无需备案，支持设置A记录

27.115.36.78

2.安装
32位：dpkg -i phddns_i386.deb
64位：dpkg -i phddns_3.0_x86_64.deb
3.卸载：dpkg -r phddns
4.输入phddns回车后，可以看到扩展的功能：
phddns start（启动）| stop（停止）| status（状态）| restart（重启）|
phddns reset（重置）
phddns version（版本）

## 文件传输与同步   

对于一次性的文件传输，这方面最简单的当然还是直接使用scp命令啦，文件夹和文件都能轻松传输。

但是我们做深度学习的话，在服务器端大面积改代码、重量级调试的话还是不方便，毕竟服务器上没有图形界面，大部分人还是用不惯 vim的，那么能不能在 PC 端用漂亮的编辑器修改代码，将修改结果实时的同步到服务器端呢？ 当然可以！这里推荐文件同步神器 syncthing。  

官网丢过来 https://syncthing.net/, 剩下的就是傻瓜式配置啦。记得要更改文件夹刷新频率哦（默认是60秒，我们可以改的短一点，比如 3 秒）， 这样在客户端我们用漂亮的文本编辑器对代码的改动就能实时的同步到服务器上啦，在服务器端就只需要负责运行就可以咯。 

### 1. stable

~~~bash
# Add the release PGP keys:
curl -s https://syncthing.net/release-key.txt | sudo apt-key add -

# Add the "stable" channel to your APT sources:
echo "deb https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list

# Update and install syncthing:
sudo apt-get update
sudo apt-get install syncthing

The candidate channel is updated with release candidate builds, approximately once every two weeks. These predate the stable builds by about two weeks.

# Add the release PGP keys:
curl -s https://syncthing.net/release-key.txt | sudo apt-key add -

# Add the "candidate" channel to your APT sources:
echo "deb https://apt.syncthing.net/ syncthing candidate" | sudo tee /etc/apt/sources.list.d/syncthing.list

# Update and install syncthing:
sudo apt-get update
sudo apt-get install syncthing
Depending on your distribution, you may see an error similar to the following when running apt-get:

E: The method driver /usr/lib/apt/methods/https could not be found.
N: Is the package apt-transport-https installed?
E: Failed to fetch https://apt.syncthing.net/dists/syncthing/InRelease
If so, please install the apt-transport-https package and try again:

sudo apt-get install apt-transport-https
~~~

## 多开发环境管理   

如果不幸你的 GPU 服务器并不是你一个人用，那么这时多人（尤其是混入小白多话）经常把服务器默认的 python 环境弄的乌烟瘴气，比如有人用 python2，有人用 python3，有人用 tensorflow1.3，有人用 0.12 等...最后导致大家的程序全跑崩了。  

所以在服务器端管理深度学习的开发环境是极其必要的，这里 anaconda 或者 python virtualenv 可以搞定！每个人建立和管理自己的开发环境，包括 python 版本、各种库的版本等，互不干扰。而且在发布 project 时，也方便直接将环境导出为 requirements 文件，免得自己去手写啦。 

## 多任务管理（并行调参）   

如果你的服务器上有多个 GPU，或者你的任务消耗 GPU 资源不多，那么并行的训练模型调参数是极大提高开发效率的！   

以下是几种场景下的常用方案：   

1、比如我们在服务器上除了训练还要接着干别的事情（比如还要捣鼓一下贪吃蛇什么的），或者仅仅不希望 ssh 断开后导致训练任务终止，那么我们就可以直接将训练任务挂后台。具体如下。   

在 linux 中，在命令后面加上&符号可以将命令在后台执行，为了能看到训练日志，我们当时还需要输出重定向（否则会打印到屏幕上干扰正常工作的），所以比如我们调 batchsize 参数时可以这样：   

~~~bash
python train.py --batchsize=16 > log_batch16.txt &
~~~

当然再挂上其他 batchsize 大小，如：

~~~bash
python train.py --batchsize=16 > log_batch16.txt &
python train.py --batchsize=64 > log_batch64.txt &
python train.py --batchsize=128 > log_batch128.txt &
~~~

通过 jobs 命令可以看到后台任务的运行状况（running、stopped等）， 通过 bg [任务号]可以让后台 stopped 的命令继续running， 通过 fg [任务号]可以让后台的任务来前台执行。对于前台已经执行起来的任务，可以 ctrl+z 来丢进后台（丢后台时 stop了的话用 bg 让其 run 起来）。  

2、如果我们特别着急，不仅要并行挂着很多训练任务，而且都要实时的监控它们的训练进展，那么使用 screen 命令吧，这个命令就相当于可以让你同时开很多个窗口（就像桌面上那样，你可以开很多应用程序的很多窗口）， 而且多个窗口之间可以轻松切换，同样这种方法不会因为 ssh 的断开而停止训练任务。   

具体的操作可以直接在 linux 下 man screen 来查看 screen 命令的帮助文档。英文恐惧症的童鞋可以看本文参考文献[1]。   

## 睡觉调参模式（串行调参）   

大部分场合下我们没有那么多充裕的 GPU 可以用，我们一般只能一次挂一个任务，但是我们又有很重的调参任务，那怎么办呢？  

依然很简单啦，首先，装好 python-fire 这个工具，github 链接：

https://github.com/google/python-fire

它可以非常轻松的将你的 python 程序变成命令行程序，并且可以轻松的将你要调的参数封装成命令行参数的形式。   

然后，写一个调参 shell 脚本，把你要调的参数全都写进去！ 比如就像这样：

~~~bash
python train.py main --max_epoch=6 --batchsize=64 --lr=0.0001 --lr2=0.0000
python train.py main --max_epoch=6 --batchsize=128 --lr=0.0001 --lr2=0.0000
python train.py main --max_epoch=6 --batchsize=256 --lr=0.0001 --lr2=0.0000
python train.py main --max_epoch=6 --batchsize=64 --lr=0.001 --lr2=0.0000
python train.py main --max_epoch=6 --batchsize=64 --lr=0.01 --lr2=0.0000
~~~

（当然别忘在代码里将训练的 summary 写到某个文件里）   

然后就可以挂上这个脚本去睡觉啦～睡到天亮发现各个最优参数都找到了。   

## 关于 jupyter notebook   

这也是一个重量级调参神器！或者直接可以说深度学习神器！在服务器端依然犀利的无可替代，只需要如下的 tricks。   

1、服务器端开启 jupyter notebook 后   

然后复制最后那一行的 token=xxx，这个 token 就是远程访问的密码！同时记下最后那行显示的端口号 8888（因为如果服务器上同时开多个的话，端口号就不一定是 8888 了哦），然后去本地 PC 端做一个端口映射！即通过 ssh 隧道来将服务器端的 8888 端口号映射到本地（PC端）的某个端口（如1234）：   

~~~python
ssh -L 1234:localhost:8888 dlnlp@102.10.60.23
# 以下是我的机器上的指令
ssh -L 1234:localhost:8888 lsn@192.168.16.111 -f -N
~~~

这时就可以在PC端的浏览器   

~~~ 
http://localhost:1234
~~~

直接访问服务器上的 jupyter notebook 啦～当然，访问时会让你输入密码，这时就输入之前记下的那个 token 哦。  

2、让 jupyer notebook 跟 anaconda 开发环境融合。

默认的情况下 jupyter notebook 是运行在系统默认环境里的，如果要让它运行在我们自己用 ananconda 创建的环境中，要进入那个环境中，然后安装 nb_conda 这个库：   

~~~bash
conda install nb_conda
~~~

这时再开启 jupyter notebook 就能选择在我们这个环境里运行代码啦。   

## 单任务全霸占模式   

有时我们的训练任务非常重要且急迫，且绝对不允许被别人挤崩，或者我们明知要占用全部 GPU 资源了. 事先说明，非必要时刻请勿频繁使用.   

使用 linux 中的 run-one 命令，这个命令可以保证同一条命令最多同时运行一个。比如 run-one python xxx 就会只允许运行一个python 程序，后来的 python 程序在这个 python 程序执行完毕前是得不到执行的（一执行就会出错返回）。所以我们可以写入 .bashrc：   

~~~bash
alias python='run-one python'
~~~

（别忘 source 激活）   

将第一个 python 挂到后台，后面的 python 完全执行不起来。除非前一个 python 结束。（所以其他小伙伴可能以为自己的程序出问题了，然后陷入了无尽的困惑）   
