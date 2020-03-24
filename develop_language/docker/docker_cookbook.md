# docker 命令使用 cookbook

## 1. 查看 docker 程序是否存在 - `docker info`    

```bash
docker info

Containers: 1
 Running: 1
 Paused: 0
 Stopped: 0
Images: 1
	...
Architecture: x86_64
CPUs: 4
Total Memory: 7.685GiB
Name: klm
...
```

## 2. 查看已经安装的 images - `docker images`    

```bash
docker images
```

## 3. 搜索 Docker 可用容器 - `docker search`      

Docker 可用的容器可以通过搜索命令查找， 社区已经提供了很多可用的容器。如何寻找可用的 Docker 容器，使用以下命令搜索 CentOS 的 Docker 容器。   

```bash
$ docker search centos
```

4. 运行 Docker 容器 - `docker run`  

4.1 在 bash shell 下建立 ubuntu:14.04 容器    

非常简单，只需运行一行命令即可。  

```bash
$ docker run -i -t ubuntu:14.04 /bin/bash
root@696d5fd32bba:/$
```

其中,  
- `-i`选项：让输入输出都在标准控制台进行; 
- `-t`选项：分配一个`tty`; 

在输出提示中， 可以看到使用的标准 ubuntu:14.04 容器。现在可以在 ubuntu:14.04 的 Docker 容器中使用 bash 。如果希望停止/断开连接，可以使用组合键`Ctrl-D`， 然后就会返回到之前的的窗口。   

4.2 创建一个以进程方式运行的容器    

```bash
$ docker run -d ubuntu:14.04 /bin/sh -c "while true; do echo hello world; sleep 1; done"
```

## 6. 删除旧的容器     

```bash
docker ps --filter "status=exited" | grep 'weeks ago' | awk '{print $1}' | xargs --no-run-if-empty docker rm
```

其中, `weeks ago` 可以是 `minutes`、 `hours` 或 `days`。    

## docker cookbook

|docker command|    说明|
|----|------|
|docker info|查看 docker 程序是否存在|    
|docker run|运行一个 docker 容器|    
|docker images|查看已经安装的 docker images |    
|docker search xxx|查找社区已经提供的可用 container |  
|docker logs|查看容器内的标准输出|     
|docker stop|停止容器|      
|docker ps|查看有哪些容器有在运行|     
|docker rmi||     

## 参考资料  

[1. 部署第一个容器](http://guide.daocloud.io/dcs/3-9152643.html)   

curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey |  sudo apt-key add - distribution=$(. /etc/os-release;echo $ID$VERSION_ID) 

curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu18.04/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -distribution=$(. /etc/os-release;echo $ID$VERSION_ID)


## docker 学习笔记  

镜像 (Image) 和容器 (Container) 的关系，就像是面向对象程序设计中的 类 和 实例 一样，镜像是静态的定义，容器是镜像运行时的实体。容器可以被创建、启动、停止、删除、暂停.  

按照 Docker 最佳实践的要求，容器不应该向其存储层内写入任何数据，容器存储层要保持无状态化。所有的文件写入操作，都应该使用 数据卷 (Volume) 、或者绑定宿主目录，在这些位置的读写会跳过容器存储层，直接对宿主(或网络存储)发生读写，其性能和稳定性更高。  

数据卷的生存周期独立于容器，容器消亡，数据卷不会消亡。因此，使用数据卷后，容器可以随意删除、重新 run ，数据却不会丢失。  

Docker 版本分为 CE 和 EE。CE 版本即社区版 (免费，支持周期三个月) ，EE 即企业版，强调安全，付费使用。 


$ sudo systemctl enable docker
$ sudo systemctl start docker

Ubuntu 14.04 请使用以下命令启动：  

$ sudo service docker start


## 建立 docker 用户组

默认情况下， docker 命令会使用 Unix socket 与 Docker 引擎通讯。而只有root 用户和 docker 组的用户才可以访问 Docker 引擎的 Unix socket。出sudo于安全考虑，一般 Linux 系统上不会直接使用 root 用户。因此，更好地做法是将需要使用 docker 的用户加入 docker 用户组。

1\) 建立 docker 组：  

$ groupadd docker  

2\) 将当前用户加入 docker 组：  

```bash
sudo gpasswd -a ${USER} docker
```

3\) 查看 docker 用户组成员  

```bash
cat /etc/group | grep docker
```

4\) 赋予权限

```bash
sudo chmod a+rw /var/run/docker.sock
```

5\) 重新启动 docker 服务  

```bash
sudo systemctl restart docker
```

## docker 加速  

sudo vi /etc/docker/daemon.json  

```json
{
    "registry-mirrors": [
    "https://dockerhub.azk8s.cn",
    "https://reg-mirror.qiniu.com",
    "https://docker.mirrors.ustc.edu.cn"
    ]
}
```

**注意, 一定要保证该文件符合 json 规范，否则 Docker 将不能启动**。

之后重新启动服务。  

```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

查看镜像加速是否成功.  

```bash
sudo docker info|grep "Registry Mirrors" -A 1
```


从 Docker Registry 获取镜像的命令是 docker pull 。其命令格式为： 

docker pull [选项] [Docker Registry地址]<仓库名>:<标签>

具体的选项可以通过 docker pull --help 命令看到，这里我们说一下镜像名称的格式。  

Docker Registry 地址：地址的格式一般是 <域名/IP>[:端口号] 。默认地址是 Docker Hub。  

仓库名：如之前所说，这里的仓库名是两段式名称，既 <用户名>/<软件名> 。对于 Docker Hub，如果不给出用户名，则默认为 library ，也就是官方镜像。

比如：  

```bash
$ docker pull ubuntu:14.04
14.04: Pulling from library/ubuntu
bf5d46315322: Pull complete
9f13e0ac480c: Pull complete
e8988b5b3097: Pull complete
40af181810e7: Pull complete
e6f7c7e5c03e: Pull complete
Digest: sha256:147913621d9cdea08853f6ba9116c2e27a3ceffecf3b3ae97c3d643fbbe
Status: Downloaded newer image for ubuntu:14.
```

运行有了镜像后，我们就可以以这个镜像为基础启动一个容器来运行。以上面的ubuntu:14.04 为例，如果我们打算启动里面的 bash 并且进行交互式操作的话，可以执行下面的命令。

$ docker run -it --rm ubuntu:14.04 bash

docker run 就是运行容器的命令，这里简要的说明一下上面用到的参数。

-it ：这是两个参数，一个是 -i ：交互式操作，一个是 -t 终端。我们这里打算进入 bash 执行一些命令并查看返回结果，因此我们需要交互式终51端。
--rm ：这个参数是说容器退出后随之将其删除。默认情况下，为了排障需求，退出的容器并不会立即删除，除非手动 docker rm 。我们这里只是随便执行个命令，看看结果，不需要排障和保留结果，因此使用 --rm 可以避免浪费空间。

ubuntu:14.04 ：这是指用 ubuntu:14.04 镜像为基础来启动容器。

bash ：放在镜像名后的是命令，这里我们希望有个交互式 Shell，因此用的是 bash 。进入容器后，我们可以在 Shell 下操作，执行任何所需的命令。这里，我们执行了cat /etc/os-release ，这是 Linux 常用的查看当前系统版本的命令，从返回的结果可以看到容器内是 Ubuntu 14.04.5 LTS 系统。最后我们通过 exit 退出了这个容器。获取






