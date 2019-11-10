#coding=utf-8


'''
绘制高斯函数曲线
'''
import numpy as np 
import matplotlib.pyplot as plt

sigma = 12
mean = 40
n = 100
coeff = -0.5/(sigma*sigma)

weight = [np.exp(coeff*(x-mean)*(x-mean)) for x in range(n)]

plt.figure()
plt.plot([x for x in range(n)], weight)
plt.show()