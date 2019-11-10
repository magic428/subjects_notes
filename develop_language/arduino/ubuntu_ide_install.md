# Ubuntu 配置 Arduino 

## 安装 Arduino IDE  

1) 下载安装包   

下载地址:  https://www.arduino.cc/en/Guide/Linux   

2) 解压安装包  

3) 运行 install 脚本安装.   

```bash
sudo ./install
```

4) 查看安装结果和赋予串口访问权限.

```bash
ls -l /dev/ttyACM*
```

输出信息类似这样:  

```bash
crw-rw---- 1 root dialout 188, 0 5 apr 23.01 ttyACM0
```

可以看到, 这个设备属于 dialout 组, 因此为了保证当前用户具有访问权限, 需要将用户加入到 dialout group 中.  

```
sudo usermod -a -G dialout <username>
```

 <username> 指的是 Linux 系统的用户名. 然后你需要注销后重新登录才能生效.    

## S4A  

从官网下载 S4A16.deb  

这个版本是为 32 位系统封装的, 要想在 64-bit Ubuntu/Debian 上安装, 需按照如下操作:   

首先使能 i386 架构, 

```bash
sudo dpkg --add-architecture i386
sudo apt-get update
sudo apt install lib32ncurses5 lib32z1 
```

然后安装 S4A:  

```bash
dpkg -i --force-architecture S4A16.deb
sudo apt-get install squeak-vm:i386 libpulse-dev:i386 
sudo apt-get install libsm6:i386
```

以上步骤亲测可用.  2019-03-16.   

