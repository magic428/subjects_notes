# ROS-从rosbag中提取图像（带时间戳）

## 1. ros之rosbag使用

目录

ros之rosbag
rosbag record
rosbag info
rosbag play
rosbag filter
rosbag compress
rosbag decompress


ros之rosbag
我是大自然的搬运工。。。。

rosbag record
Record all topics.
rosbag record -a
Prepend PREFIX to beginning of bag name before date stamp.
rosbag record -o session1 /chatter
Record to bag with name NAME.bag.
rosbag record -O session2_090210.bag /chatter
Specify the maximum duration of the recorded bag file.
rosbag record --duration=30 /chatter
rosbag record --duration=5m /chatter
rosbag record --duration=2h /chatter
Split the bag when maximum size or duration is reached
rosbag record --split --size=1024 /chatter
rosbag record --split --duration=30 /chatter
rosbag record --split --duration=5m /chatter
rosbag record --split --duration=2h /chatter
其他用法见:
rosbag record -h
回到顶部
rosbag info
Display a summary of the contents of the bag files.
rosbag info session*.bag
Print information in YAML format.
rosbag info -y /path/to/my.bag
其他用法见
rosbag info -h
回到顶部
rosbag play
Play back (publish) the contents of the given bags.
rosbag play recorded1.bag recorded2.bag
注： 如果播放两个及以上bag包，那么他们会第一帧对其，后面根据第一帧时间戳的时间差播放。

Start SEC seconds into the bags.
rosbag play -s 5 recorded1.bag
Loop playback.
rosbag play -l recorded1.bag
Multiply the publish rate by FACTOR.
rosbag play -r 10 recorded1.bag
更多用法见：
rosbag play -h
回到顶部
rosbag filter
filter <in-bag> <out-bag> <expression>
eg.

rosbag filter my.bag only-tf.bag "topic == '/tf'"
filter by topic
rosbag filter my.bag out.bag "topic == '/tf' or topic == '/tf2'"

filter by time - 截取特定时间段内的数据  
rosbag filter my.bag out.bag "t.to_sec() >= 123444.77 and t.to_sec() <= 234545.88"

更多用法
rosbag filter -h
回到顶部
rosbag compress
rosbag compress is a command-line tool for a backup of bag file. Currently, there are two supported formats:BZ2 and LZ4. BZ2 is selected by default. BZ2 generally produces smaller bags than LZ4, However, BZ2 is typically much slower than LZ4.

compress <bag-files>
Compress the given bag files using BZ2.
rosbag compress *.bag
Use LZ4 to compress data.
rosbag compress --lz4 *.bag
更多用法见：
rosbag compress -h
回到顶部
rosbag decompress
decompress <bag-files>
Decompress the given bag files.
rosbag decompress *.bag
回到顶部
参考
rosbag/Commandline - ROS Wiki

（转载请注明作者和出处：http://www.cnblogs.com/ChrisCoder/未经允许请勿用于商业用途）

## 从 rosbag 中提取 topic 信息  

> 原文链接：https://blog.csdn.net/Draonly/article/details/74642747  

关于从rosbag文件提取图像，ros官网上给出了使用建立launch文件的方法。 我们也可以通过编写Python程序按照我们想要的信息及方式来提取。
在本例中，我想把每张图像的时间戳也提取出来，于是就查阅资料，然后动手试一下吧。

带时间戳提取
该程序 实现了读取一个bag文件，将ros msg形式 转换为 cv 图像形式，保存在文件夹中。图像名称 即为时间戳。
另外：改程序提取的图像直接保存在了和 .py 程序，在同一级文件夹下。

```py
#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys


class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('Asus.bag', 'r') as bag:  #要读取的bag文件；
            for topic,msg,t in bag.read_messages():
                if topic == "/camera/rgb/image_color": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".jpg" #图像命名：时间戳.jpg
                        cv2.imwrite(image_name, cv_image)  #保存；

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
```

## rosbag 工具的用法  

https://www.cnblogs.com/ChrisCoder/p/9917649.html#_label4  



## 参考  

1. how to recover the saved images in a bagfile to jpg or png: http://answers.ros.org/question/27713/how-to-recover-the-saved-images-in-a-bagfile-to-jpg-or-png/

2. Convect image time series to ROS bag: http://stackoverflow.com/questions/38700271/python-converting-image-time-series-to-ros-bag
————————————————
