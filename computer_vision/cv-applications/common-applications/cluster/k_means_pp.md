# 利用 K-Means++算法进行图像分割

3.1、利用K-Means++聚类
在利用K-Means++算法进行图像分割时，将图像中的每一个像素点作为一个样本，对RGB图像来说，每个样本包括三维：(151, 169, 205)，通过归一化，将每个通道的值压缩到[0,1]区间上。数据的导入和处理如下面程序所示：

#coding:UTF-8
import Image as image
import numpy as np
from KMeanspp import run_kmeanspp

def load_data(file_path):
    '''导入数据
    input:  file_path(string):文件的存储位置
    output: data(mat):数据
    '''
    f = open(file_path, "rb")  # 以二进制的方式打开图像文件
    data = []
    im = image.open(f)  # 导入图片
    m, n = im.size  # 得到图片的大小
    print m, n
    for i in xrange(m):
        for j in xrange(n):
            tmp = []
            x, y, z = im.getpixel((i, j))
            tmp.append(x / 256.0)
            tmp.append(y / 256.0)
            tmp.append(z / 256.0)
            data.append(tmp)
    f.close()
    return np.mat(data)
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
最终保存成矩阵的形式，矩阵的行为样本的个数，列为每一个通道的数值(RGB)。在利用K-Means++算法对样本进行聚类。主函数如下述代码所示：

if __name__ == "__main__":
k = 10#聚类中心的个数
# 1、导入数据
print "---------- 1.load data ------------"
data = load_data("001.jpg")
# 2、利用kMeans++聚类
print "---------- 2.run kmeans++ ------------"
run_kmeanspp(data, k)
1
2
3
4
5
6
7
8
9
k表示的是聚类的个数。K-Means++程序的实现如下面程序所示：

# coding:UTF-8
'''
Date:20160923
@author: zhaozhiyong
'''

import numpy as np
from random import random
from KMeans import distance, kmeans, save_result

FLOAT_MAX = 1e100  # 设置一个较大的值作为初始化的最小的距离

def nearest(point, cluster_centers):
    '''计算point和cluster_centers之间的最小距离
    input:  point(mat):当前的样本点
        cluster_centers(mat):当前已经初始化的聚类中心
    output: min_dist(float):点point和当前的聚类中心之间的最短距离
    '''
    min_dist = FLOAT_MAX
    m = np.shape(cluster_centers)[0]  # 当前已经初始化的聚类中心的个数
    for i in xrange(m):
        # 计算point与每个聚类中心之间的距离
        d = distance(point, cluster_centers[i, ])
        # 选择最短距离
        if min_dist > d:
            min_dist = d
    return min_dist

def get_centroids(points, k):
    '''KMeans++的初始化聚类中心的方法
    input:  points(mat):样本
        k(int):聚类中心的个数
    output: cluster_centers(mat):初始化后的聚类中心
    '''
    m, n = np.shape(points)
    cluster_centers = np.mat(np.zeros((k , n)))
    # 1、随机选择一个样本点为第一个聚类中心
    index = np.random.randint(0, m)
    cluster_centers[0, ] = np.copy(points[index, ])
    # 2、初始化一个距离的序列
    d = [0.0 for _ in xrange(m)]

    for i in xrange(1, k):
        sum_all = 0
        for j in xrange(m):
            # 3、对每一个样本找到最近的聚类中心点
            d[j] = nearest(points[j, ], cluster_centers[0:i, ])
            # 4、将所有的最短距离相加
            sum_all += d[j]
        # 5、取得sum_all之间的随机值
        sum_all *= random()
        # 6、获得距离最远的样本点作为聚类中心点
        for j, di in enumerate(d):
            sum_all -= di
            if sum_all > 0:
                continue
            cluster_centers[i] = np.copy(points[j, ])
            break
    return cluster_centers

def run_kmeanspp(data, k):
    # 1、KMeans++的聚类中心初始化方法
    print "\t---------- 1.K-Means++ generate centers ------------"
    centroids = get_centroids(data, k)
    # 2、聚类计算
    print "\t---------- 2.kmeans ------------"
    subCenter = kmeans(data, k, centroids)
    # 3、保存所属的类别文件
    print "\t---------- 3.save subCenter ------------"
    save_result("sub_pp", subCenter)
    # 4、保存聚类中心
    print "\t---------- 4.save centroids ------------"
save_result("center_pp", centroids)
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
在上述代码中主要是初始化k个聚类中心，K-Means算法的核心程序如下所示：

# coding:UTF-8
'''
Date:20160923
@author: zhaozhiyong
'''
import numpy as np

def distance(vecA, vecB):
    '''计算vecA与vecB之间的欧式距离的平方
    input:  vecA(mat)A点坐标
        vecB(mat)B点坐标
    output: dist[0, 0](float)A点与B点距离的平方
    '''
    dist = (vecA - vecB) * (vecA - vecB).T
    return dist[0, 0]

def randCent(data, k):
    '''随机初始化聚类中心
    input:  data(mat):训练数据
        k(int):类别个数
    output: centroids(mat):聚类中心
    '''
    n = np.shape(data)[1]  # 属性的个数
    centroids = np.mat(np.zeros((k, n)))  # 初始化k个聚类中心
    for j in xrange(n):  # 初始化聚类中心每一维的坐标
        minJ = np.min(data[:, j])
        rangeJ = np.max(data[:, j]) - minJ
        # 在最大值和最小值之间随机初始化
        centroids[:, j] = minJ * np.mat(np.ones((k , 1))) + np.random.rand(k, 1) * rangeJ
    return centroids

def kmeans(data, k, centroids):
    '''根据KMeans算法求解聚类中心
    input:  data(mat):训练数据
        k(int):类别个数
        centroids(mat):随机初始化的聚类中心
    output: centroids(mat):训练完成的聚类中心
        subCenter(mat):每一个样本所属的类别
    '''
    m, n = np.shape(data)  # m：样本的个数，n：特征的维度
    subCenter = np.mat(np.zeros((m, 2)))  # 初始化每一个样本所属的类别
    change = True  # 判断是否需要重新计算聚类中心
    while change == True:
        change = False  # 重置
        for i in xrange(m):
            minDist = np.inf  # 设置样本与聚类中心之间的最小的距离，初始值为争取穷
            minIndex = 0  # 所属的类别
            for j in xrange(k):
                # 计算i和每个聚类中心之间的距离
                dist = distance(data[i, ], centroids[j, ])
                if dist < minDist:
                    minDist = dist
                    minIndex = j
            # 判断是否需要改变
            if subCenter[i, 0] <> minIndex:  # 需要改变
                change = True
                subCenter[i, ] = np.mat([minIndex, minDist])
        # 重新计算聚类中心
        for j in xrange(k):
            sum_all = np.mat(np.zeros((1, n)))
            r = 0  # 每个类别中的样本的个数
            for i in xrange(m):
                if subCenter[i, 0] == j:  # 计算第j个类别
                sum_all += data[i, ]
                r += 1
            for z in xrange(n):
                try:
                    centroids[j, z] = sum_all[0, z] / r
                    print r
                except:
                    print " r is zero"   
    return subCenter

def save_result(file_name, source):
    '''保存source中的结果到file_name文件中
    input:  file_name(string):文件名
        source(mat):需要保存的数据
    output: 
    '''
    m, n = np.shape(source)
    f = open(file_name, "w")
    for i in xrange(m):
        tmp = []
        for j in xrange(n):
            tmp.append(str(source[i, j]))
        f.write("\t".join(tmp) + "\n")
    f.close()
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
34
35
36
37
38
39
40
41
42
43
44
45
46
47
48
49
50
51
52
53
54
55
56
57
58
59
60
61
62
63
64
65
66
67
68
69
70
71
72
73
74
75
76
77
78
79
80
81
82
83
84
85
86
87
88
3.2、利用聚类结果生成新的图片
上述的过程中，对每一个像素点进行了聚类，最终利用聚类中心点的RGB值替换原图中每一个像素点的值，便得到了最终的分割后的图片，代码如下所示：

#coding:UTF-8

import Image as image

f_center = open("center_pp")

center = []
for line in f_center.readlines():
    lines = line.strip().split("\t")
    tmp = []
    for x in lines:
        tmp.append(int(float(x) * 256))
    center.append(tuple(tmp))
print center
f_center.close()

fp = open("001.jpg", "rb")
im = image.open(fp)
# 新建一个图片
m, n = im.size
pic_new = image.new("RGB", (m, n)) 

f_sub = open("sub_pp")
i = 0
for line in f_sub.readlines():
    index = float((line.strip().split("\t"))[0])
    index_n = int(index)
    pic_new.putpixel(((i/n),(i % n)),center[index_n])
    i = i + 1
f_sub.close()

pic_new.save("result.jpg", "JPEG")       
1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
27
28
29
30
31
32
33
对于上述的圣托里尼的图片，取不同的k值，得到如下的一些结果：

原图
这里写图片描述

k=3
这里写图片描述

k=5
这里写图片描述

k=7
这里写图片描述

k=10
这里写图片描述

参考文章