# `ubuntu 14.04`安装`flightgear`

> 版本声明：MATLAB-2015b + FlightGear 3.0.0     

为了安装这个东东，真的是踩坑无数才搞定。其实`Aerospace Blockset ToolBox`支持的`FlightGear版本有`:

- v2.0   
- v2.4   
- v2.6   
- v2.8   
- v2.10   
- v2.12   
- v3.0   
- v3.2   
- v3.4   
- v2016.1   
- v2016.3   
- v2017.1   

## 具体安装步骤
1. 安装`flightgear`   
```bash
sudo apt-get install flightgear
```
2. 配置`AeroSpace工具箱`和`flightgear`    
其实上面一行命令执行完后`flightgear`已经安装完毕。以下步骤是配置`AeroSpace工具箱`和`flightgear`的连接。配置方式可能根据不同的`flightgear`版本会变，这个是针对`flightgear 3.0.0`的。   
```
cd /usr/games
sudo ln -s fgfs fgfs.sh
mkdir data
sudo cp -R /usr/share/games/flightgear/* ./
sudo  mv -t . Scenery/ WorldScenery/  # WorldScenery包是地域相关的，需要额外下载
```
**注意**：   
`flightgear`安装完毕后只有`Scenery`， `WorldScenery/`要自己去[官网](http://ns334561.ip-5-196-65.eu/~fgscenery/WS2.0/scenery-2.0.1.html)下载，下载完成后解压到`/usr/games/WorldScenery/`目录即可。   
3. 拷贝`H20`机型到`flightgear`的`Aircraft`目录	   
```
sudo cp -R /usr/local/MATLAB/R2015b/toolbox/aero/astdemos/HL20 /usr/games/Aircraft
```

## `asbhl20` 仿真模型    

1. `MATLAB`的`mu toolbox`提供了`PDF`文档,查看一下有个`Robust control`是以`airplane`为例。  
2. 就是启动`MATLAB`后,在有提示符`>>`的`command window`下,输入`asbhl20`就可以启动了,....   
启动编译之后，`MATLAB`会生成一个启动脚本——`runfg.bat`，根据对应的`bash`做少量修改就可以得到下面的启动脚本 `run.sh`：    
```bash
#!/bin/bash 

cd /usr/games

FG_ROOT=/usr/games/data
export FG_ROOT

FG_SCENERY=/usr/games/Scenery:/usr/games/WorldScenery
export FG_SCENERY

fgfs --aircraft=HL20 --fdm=null --native-fdm=socket,in,30,127.0.0.1,5502,udp --native-ctrls=socket,out,30,127.0.0.1,5505,udp --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --enable-freeze --airport=KSFO --runway=10L --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0 --fg-root=/usr/games/data
```
