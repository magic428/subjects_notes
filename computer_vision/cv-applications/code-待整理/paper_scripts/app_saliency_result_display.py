#coding=utf-8
import skimage.io as io
import matplotlib.pyplot as plt
import numpy as np
import os
import cv2

from PIL import Image
from matplotlib import pyplot as plt
from pylab import *
# axis()返回坐标轴的默认值(0.0, 1.0, 0.0, 1.0),(xmin, xmax, ymin, ymax)
# plt.axis()
# 设置x轴和y轴的值
# plt.autoscale(),该方法会计算坐标轴的最佳大小以适应数据的显示# plt.axes(),该方法向图形中添加新的坐标轴# rect属性,归一化单位(0, 1)下的left、bottom、width、height四个属性# axisbg参数,指定坐标轴的背景颜色# sharex/sharey参数,接收其他坐标轴的值并让当前坐标轴(x/y)共享相同的值# polar参数,指定是否使用极坐标轴# plt.axhline()/plt.axvline()根据给定的x和y值相应地绘制出相对于坐标轴的水平线/垂直线plt.axhline()plt.axvline()plt.axhline(4)# plt.axhspan()/plt.axvspan()添加一个跨坐标轴的水平带(矩形)# plt.axhspan()/plt.axvspan()必需ymin/xmin和ymax/xmax参数指定水平/垂直带的宽度plt.axhspan(-7, -4)plt.axvspan(0.2, 0.7)# plt.grid()打开网格# which:指定绘制的网格刻度类型(major、minor或者both)# axis:指定绘制哪组网格线(both、x或者y)plt.grid()# 单独一个坐标轴由matplotlib.axis.Axis类表示,matplotlib.axis.XAxis表示x轴# matplotlib.axis.YAxis表示y轴plt.show()

# print(help(PIL))
image_dirs = ['/home/magic/Desktop/saliency_miner/']
result_dir = '/home/magic/Desktop/results/'

# method_names = ["Input", "FT", "GC", "HC", "MSS", "GU", "ARC", "GMR", "GroundTruth"]
method_names = ["ARC"]
labels = ["FT", "GC", "HC", "MSS", "GU", "RC", "Ours"]

file_pre = set()
cols = 8
row_idx = 0
imgs_pair = []
rows = 0

for image_dir in image_dirs:
    img_names = os.listdir(image_dir)
    rows = len(img_names)//cols
    # rows = rows//2
    # rows = 3
    print(rows)

    for col_idx, img_name in enumerate(img_names):
       
        # plt.subplots_adjust(left=0.0, bottom=0.0, right=1.0, top=1.0, hspace=0.01, wspace=0.01)
        base_name = os.path.splitext(img_name)[0]

        # for method_name in (method_names):
        # img_path = image_dir + img_pre + '_' + method_name + ".png"
        img_path = ''
        img_pair = []
        if img_name.endswith('.jpg'):
            img_path = image_dir + img_name
            img_pair.append(img_path)
            
            pos = img_name.rfind('.')
            if -1 != pos:
                img_pre = img_name[:pos]
            img_ARC_path = image_dir + img_pre + "_ARC.png"
            img_pair.append(img_ARC_path)
            imgs_pair.append(img_pair)
        else:
            continue

nfigures = 1;
if rows > 6:
    if rows %6 == 0:
        nfigures = rows//6
    else:
        nfigures =  rows//6+1

for n in range(nfigures):
    # figure(n)
    f, axs = plt.subplots(rows, cols, figsize=(40,40))
    idx = 0
    step = 0
    for col_idx, img_pair in enumerate(imgs_pair[n*4*6:(n+1)*4*6]):
        ## begin plots
        idx += 1
        plt.subplots_adjust(wspace =0.1, hspace =0.05)#调整子图间距
        print(col_idx)

        if idx % 9 == 0:
            idx = 1
            step += 1

        # print(img_pair)
        for i, img in enumerate(img_pair):
            print(6, cols, (col_idx//8+i+step)*(cols)+idx, sep=',')
            # print(6, cols, (idx)//8*cols+idx%8, sep=',')
            # i+=1
            img_data = Image.open(img)
            width, height = img_data.size 
            x_size = max(width, height)
            ratio = height/width
            # xlim(0, x_size)
            # ylim(x_size*ratio, 0)
            img_data = img_data.resize((400, 300), Image.ANTIALIAS)
            # img_data = img_data.resize((x_size, int(x_size*ratio)), Image.ANTIALIAS)
            #ax_values = [0, x_size, x_size*ratio, 0]
            #plt.axis(ax_values)
            plt.subplot(6, cols, (col_idx//8+i+step)*(cols)+idx)
            # plt.tight_layout(pad=0.2, w_pad=0.1, h_pad=0.1) # 调整布局
            plt.axis('off')
            plt.imshow(img_data)
        

    plt.axis('off')
    plt.show()