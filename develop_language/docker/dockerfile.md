# dockerfile 语法详解


```
# Using a compact OS
FROM daocloud.io/nginx:1.11-alpine

MAINTAINER Golfen Guo <golfen.guo@daocloud.io>

# Add 2048 stuff into Nginx server
COPY . /usr/share/nginx/html

EXPOSE 80

# Start Nginx and keep it running background and start php
CMD sed -i "s/ContainerID: /ContainerID: "$(hostname)"/g" /usr/share/nginx/html/index.html && nginx -g "daemon off;"

```


```
#FROM 指令指定基础镜像
#比较常用的基础镜像有ubuntu，centos。这里使用了一个极小的基础镜像alpine
FROM alpine:latest
  
#MAINTAINER指令用于将镜像制作者相关的信息写入到镜像中
#您可以将您的信息填入name以及email
MAINTAINER name <email>
  
#RUN指令可以运行任何被基础image支持的命令，就像在操作系统上直接执行命令一样（如果使用ubuntu为基础镜像，这里应该用apt-get 命令安装）
#安装nginx
RUN apk --update add nginx
  
#配置Nginx，并设置在标准输出流输出日志（这样执行容器时才会看到日志）
RUN sed -i "s#root   html;#root   /usr/share/nginx/html;#g" /etc/nginx/nginx.conf
  
#Nginx日志输出到文件
RUN ln -sf /dev/stdout /var/log/nginx/access.log \
    && ln -sf /dev/stderr /var/log/nginx/error.log
  
#COPY指令复制主机的文件到镜像中 （在这里当前路径就是repo根目录）
#将2048项目的所有文件加入到Nginx静态资源目录里
COPY . /usr/share/nginx/html
  
#EXPOSE：指定容器监听的端口
EXPOSE 80
  
#CMD指令，容器启动时执行的命令
#启动Nginx并使其保持在前台运行
#CMD一般是保持运行的前台命令，命令退出时容器也会退出
CMD ["nginx", "-g", "pid /tmp/nginx.pid; daemon off;"]
```