
# 随机漫步   
## 1. 创建RandomWalk()类   



```python
from random import choice

class RandomWalk():
    
    def __init__(self, num_points = 5000):
        self.num_points = num_points
        
        # 所有随机漫步都是从(0, 0)点开始   
        self.x_values = [0]        
        self.y_values = [0]
        
    # 决定前进方向以及沿这个方向前景的距离
    def get_step(self):
        direction = choice([-1, 1])         # 正向/反向
        distance = choice([0, 1, 2, 3, 4])  # 运动距离
        return direction * distance
        
    def fill_walk(self):
        # 漫步的长度
        while(len(self.x_values) < self.num_points):
            
            x_step = self.get_step()
            y_step = self.get_step()
            
            # 拒绝原地踏步
            if x_step == 0 and y_step == 0:
                continue
            
            # 给出下一个点的位置
            next_x = self.x_values[-1] + x_step            
            next_y = self.y_values[-1] + y_step
            
            self.x_values.append(next_x)
            self.y_values.append(next_y)     

```

## 2. 开始漫步   

2.1 模拟多次漫步的情况      
使用 while 循环控制.   
2.2 设置随机漫步的样式    
最终结果要使用简单的可视化表示,清楚地指出了每次漫步经过的路径.    
给点着色,使用颜色映射来支出漫步中个点的顺序.具体操作为:将参数 c 设置为一个 list, 指定使用颜色映射 Blues .    
并删除点的轮廓    
2.3 重新绘制起点和终点    
在随机漫步图给出之后,可以重新绘制起点和终点.让起点和终点变得更大,并显示为不同的颜色来突出.   
2.4 隐藏坐标轴    
我们想更多的关注随机漫步轨迹而不是坐标轴.    
2.5 增加漫步的点数    
在创建 RandomWalk() 实例时可以指定漫步点的个数.  
2.6 调整尺寸以适合屏幕   
为了让绘图窗口更适合屏幕大小, plt.figure(dpi = 80, figsize = None), 你可以给形参 figsize 指定一个元组,单位为英寸.  


```python

import matplotlib.pyplot as plt
#from random_walk import RandomWalk

while True:

    rw = RandomWalk(50000)
    rw.fill_walk()
    
    points_num = list(range(rw.num_points))
    
    # 设置窗口大小
    plt.figure(dpi = 120, figsize = (10, 8))
    
    # 设置颜色映射样式
    plt.scatter(rw.x_values, rw.y_values, c = points_num, cmap = plt.cm.Blues, edgecolor = 'none', s = 15)
    #plt.scatter(rw.x_values, rw.y_values, s = 15)
    # 重新设置起点/终点
    plt.scatter(0, 0, c = 'g', edgecolor = 'none', s = 100)
    plt.scatter(rw.x_values[-1], rw.y_values[-1], c = 'r', edgecolor = 'none', s = 100)
    
    # 隐藏坐标轴
    plt.axes().get_xaxis().set_visible(False)
    plt.axes().get_yaxis().set_visible(False)
    
    plt.show()
    
    keep_running = raw_input('Make another walk? (y/n): ')   # python 3.x -> input()
    
    if keep_running == 'n':
        break
```
