# Matplotlib 图像绘制库的使用    

Matplotlib 是 Python 的绘图库。 它可与 NumPy 一起使用，提供了一种有效的 MatLab 开源替代方案。 它也可以和图形工具包一起使用，如 PyQt 和 wxPython。  

## 0. 一个简单的 Matplotlib 使用示例   

```py
import numpy as np 
from matplotlib import pyplot as plt 
 
x = np.arange(1,11) 
y =  2  * x +  5 
plt.title("Matplotlib demo", fontsize=14) 
plt.xlabel("x axis caption", fontsize=14) 
plt.ylabel("y axis caption", fontsize=14) 
# 设置刻度标记的大小-labelsize
plt.tick_params(axis='both', labelsize=14)
# 绘制坐标轴
plt.axis([0, 15, 0, 30])
plt.plot(x, y, 'go', label='line1', linewidth = 5)
plt.show()
```

plt.plot(x,y) 只是将图像绘制到图层上, 需要使用 plt.show() 显示出来.  

## 1. 绘制简单的折线图    

(1) 线型   

作为线性图的替代，可以通过向 plot() 函数的 marker 参数添加格式字符串来显示离散值。 可以使用以下格式化字符。

|character|description|character|description|
|---------|--------|---------|--------
|``'-'``  |实线|``'--'`` |虚线|
|``'-.'`` |点划线|``':'``  |点状虚线|
|``'.'``  |点标记|``','``  |像素点标记(非常小)|
|``'o'``  |圆标记|``'v'``  |倒三角形标记|
|``'^'``  |正三角形标记|``'<'``  |左三角形标记|
|``'>'``  |右三角形标记|``'1'``  |下箭头标记|
|``'2'``  |上箭头标记|``'3'``  |左箭头标记|
|``'4'``  |右箭头标记|``'s'``  |正方形标记|
|``'p'``  |五边形标记|``'*'``  |星形标记|
|``'h'``  |六边形标记1|``'H'``  |六边形标记2|
|``'+'``  |加号标记|``'x'``  |x 标记|
|``'D'``  |菱形标记|``'d'``  |窄菱形标记|
|``'|'``  |竖直线标记|``'_'``  |水平线标记|

(2) 颜色 

|character  | color|character  | color|
|---------|-------|---------|--------
|'b'      |  blue|'g'      |  green|
|'r'      |  red|'c'      |  青色|
|'m'      |  品红色|'y'      |  yellow|
|'k'      |  black|'w'      |  white|


## 2. subplot() 

subplot() 函数允许你在同一图中绘制不同的东西。

以下实例绘制正弦和余弦值:

```py
import numpy as np 
import matplotlib.pyplot as plt 
# 计算正弦和余弦曲线上的点的 x 和 y 坐标 
x = np.arange(0,  3  * np.pi,  0.1) 
y_sin = np.sin(x) 
y_cos = np.cos(x)  
# 建立 subplot 网格，高为 2，宽为 1  
# 激活第一个 subplot
plt.subplot(2,  1,  1)  
# 绘制第一个图像 
plt.plot(x, y_sin) 
plt.title('Sine')  
# 将第二个 subplot 激活，并绘制第二个图像
plt.subplot(2,  1,  2) 
plt.plot(x, y_cos) 
plt.title('Cosine')  
# 展示图像
plt.show()
```

## 柱状图 bar()   

pyplot 子模块提供 bar() 函数来生成条形图。  

以下实例生成两组 x 和 y 数组的条形图。  

```py
from matplotlib import pyplot as plt 
x =  [5,8,10] 
y =  [12,16,6] 
x2 =  [6,9,11] 
y2 =  [6,15,7] 
plt.bar(x, y, align =  'center') 
plt.bar(x2, y2, color =  'g', align =  'center') 
plt.title('Bar graph') 
plt.ylabel('Y axis') 
plt.xlabel('X axis') 
plt.show()
```
## 直方图 numpy.histogram()

numpy.histogram() 函数是数据的频率分布的图形表示。 水平尺寸相等的矩形对应于类间隔，称为 bin，变量 height 对应于频率。

numpy.histogram()函数将输入数组和 bin 作为两个参数。 bin 数组中的连续元素用作每个 bin 的边界。

```py
import numpy as np 
 
a = np.array([22,87,5,43,56,73,55,54,11,20,51,5,79,31,27])
np.histogram(a,bins =  [0,20,40,60,80,100]) 
hist,bins = np.histogram(a,bins =  [0,20,40,60,80,100])  
print (hist) 
print (bins)

plt.hist(a, bins =  [0,20,40,60,80,100]) 
plt.title("histogram") 
plt.show()
```

Matplotlib 可以将直方图的数字表示转换为图形。 pyplot 子模块的 plt() 函数将包含数据和 bin 数组的数组作为参数，并转换为直方图。  


## scatter()绘制散点图   

```python
import matplotlib.pyplot as plt

# 给定x,y 绘制这个离散点
input_values = list(range(1,101))
squares = [x**2 for x in input_values]

# 删除数据点的轮廓: edgecolor = 'none'
# 设置数据点的颜色为red: c = 'r', c = 'red',c = (0, 0, 0.8)-[rgb],
plt.scatter(input_values, squares, c = (0, 0, 0.8), s = 20, edgecolor = 'none')

plt.title("Scatter numbers", fontsize = 14)
plt.xlabel("x values", fontsize = 14)
plt.ylabel("y values", fontsize = 14)

plt.tick_params(axis='both', labelsize=14)

plt.axis([0,110, 0, 11000])

plt.show()
```

## 3. 使用颜色映射   

用较浅的颜色表示较小的值,用较深的颜色表示较大的值.    

```python
import matplotlib.pyplot as plt

input_values = list(range(1,101))
squares = [x**2 for x in input_values]

# 将参数 c 设置为 纵坐标squares的列表   
# 并使用 cmap 告诉 pyplot 使用哪个颜色映射   
plt.scatter(input_values, squares, c = squares, \
            cmap = plt.cm.Reds, s = 20, edgecolor = 'none')

plt.title("Scatter numbers", fontsize = 14)
plt.xlabel("x values", fontsize = 14)
plt.ylabel("y values", fontsize = 14)

plt.tick_params(axis='both', labelsize=14)

plt.axis([0,110, 0, 11000])

plt.show()
```

## 4. 自动保存图表   

`plt.show() ->  plt.savefig()`   
```python
import matplotlib.pyplot as plt

input_values = list(range(1,101))
squares = [x**2 for x in input_values]

# 将参数 c 设置为 纵坐标squares的列表   
# 并使用 cmap 告诉 pyplot 使用哪个颜色映射   
plt.scatter(input_values, squares, c = squares, \
            cmap = plt.cm.Reds, s = 20, edgecolor = 'none')

plt.title("Scatter numbers", fontsize = 14)
plt.xlabel("x values", fontsize = 14)
plt.ylabel("y values", fontsize = 14)

plt.tick_params(axis='both', labelsize=14)

plt.axis([0,110, 0, 11000])

# 第一个参数: 保存的文件名
# 第二个参数: 将图表多余的空白区域裁剪掉 
plt.savefig('squares.png', bbox_inches ='tight')
```


## 图形中文显示   

Matplotlib 默认情况不支持中文，我们可以使用以下简单的方法来解决：

首先下载字体（注意系统）：https://www.fontpalace.com/font-details/SimHei/

SimHei.ttf 文件放在当前执行的代码文件中：

```py
import numpy as np 
from matplotlib import pyplot as plt 
import matplotlib
 
# fname 为 你下载的字体库路径，注意 SimHei.ttf 字体的路径
zhfont1 = matplotlib.font_manager.FontProperties(fname="SimHei.ttf") 
 
x = np.arange(1,11) 
y =  2  * x +  5 
plt.title("Matplotlib - 中文显示测试", fontproperties=zhfont1) 
 
# fontproperties 设置中文显示，fontsize 设置字体大小
plt.xlabel("x 轴", fontproperties=zhfont1)
plt.ylabel("y 轴", fontproperties=zhfont1)
plt.plot(x,y) 
plt.show()
```

此外，我们还可以使用系统的字体：  

```py
from matplotlib import pyplot as plt
import matplotlib
a=sorted([f.name for f in matplotlib.font_manager.fontManager.ttflist])

for i in a:
    print(i)
```

打印出你的 font_manager 的 ttflist 中所有注册的名字，找一个看中文字体例如： STFangsong(仿宋）,然后添加以下代码即可：  

```py
plt.rcParams['font.family']=['STFangsong']
```

## Matplotlib 教程  

1.IPython 以及 pylab 模式  

IPython 是 Python 的一个增强版本。它在下列方面有所增强：命名输入输出、使用系统命令（shell commands）、排错（debug）能力。我们在命令行终端给 IPython 加上参数 -pylab （0.12 以后的版本是 --pylab）之后，就可以像 Matlab 或者 Mathematica 那样以交互的方式绘图。  

### 初级绘制  

这一节中，我们将从简到繁：先尝试用默认配置在同一张图上绘制正弦和余弦函数图像，然后逐步美化它。

第一步，是取得正弦函数和余弦函数的值：  

```py
from matplotlib import pyplot as plt 

X = np.linspace(-np.pi, np.pi, 256,endpoint=True)
C,S = np.cos(X), np.sin(X)

plt.plot(X,C)
plt.plot(X,S)

plt.show()
```

X 是一个 numpy 数组，包含了从 −π 到 +π 等间隔的 256 个值。C 和 S 则分别是这 256 个值对应的余弦和正弦函数值组成的 numpy 数组。  

### 使用默认配置[源码文件] 

Matplotlib 的默认配置都允许用户自定义。 你可以调整大多数的默认配置：图片大小和分辨率（dpi）、线宽、颜色、风格、坐标轴、坐标轴以及网格的属性、文字与字体属性等。 不过，matplotlib 的默认配置在大多数情况下已经做得足够好，你可能只在很少的情况下才会想更改这些默认配置。  

### 默认配置的具体内容 

下面的代码中，我们展现了 matplotlib 的默认配置并辅以注释说明，这部分配置包含了有关绘图样式的所有配置。 代码中的配置与默认配置完全相同，你可以在交互模式中修改其中的值来观察效果。  

```py
from pylab import *

# 创建一个 8 * 6 点（point）的图，并设置分辨率为 80
figure(figsize=(8,6), dpi=80)

# 创建一个新的 1 * 1 的子图，接下来的图样绘制在其中的第 1 块（也是唯一的一块）
subplot(1,1,1)

X = np.linspace(-np.pi, np.pi, 256,endpoint=True)
C,S = np.cos(X), np.sin(X)

# 绘制余弦曲线，使用蓝色的、连续的、宽度为 1 （像素）的线条
plot(X, C, color="blue", linewidth=1.0, linestyle="-")

# 绘制正弦曲线，使用绿色的、连续的、宽度为 1 （像素）的线条
plot(X, S, color="green", linewidth=1.0, linestyle="-")

# 设置横轴的上下限
xlim(-4.0,4.0)

# 设置横轴记号
xticks(np.linspace(-4,4,9,endpoint=True))

# 设置纵轴的上下限
ylim(-1.0,1.0)

# 设置纵轴记号
yticks(np.linspace(-1,1,5,endpoint=True))

# 以分辨率 72 来保存图片
# savefig("exercice_2.png",dpi=72)

# 在屏幕上显示
show()
```

### 改变线条的颜色和粗细   

首先，我们以蓝色和红色分别表示余弦和正弦函数，而后将线条变粗一点。 

```py
plt.figure(figsize=(10,6), dpi=80)
plt.plot(X, C, color="blue", linewidth=2.5, linestyle="-")
plt.plot(X, S, color="red",  linewidth=2.5, linestyle="-")
```

接下来，我们在水平方向拉伸一下整个图。  

### 设置图片边界 - plt.xlim/ylim 

当前的图片边界设置得不好，所以有些地方看得不是很清楚。  

```py 
plt.xlim(X.min()*1.1, X.max()*1.1) 
plt.ylim(C.min()*1.1, C.max()*1.1) 
```

更好的方式是这样：

```py
xmin, xmax = X.min(), X.max()
ymin, ymax = Y.min(), Y.max()

dx = (xmax - xmin) * 0.2
dy = (ymax - ymin) * 0.2

plt.xlim(xmin - dx, xmax + dx)
plt.ylim(ymin - dy, ymax + dy)
```

### 设置坐标记号

```py
plt.xticks( [-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
plt.yticks([-1, 0, +1])
```

### 设置记号的标签 - plt.xticks  

记号现在没问题了， 不过标签却不大符合期望。我们可以把 3.142 当做是 π，但毕竟不够精确.   

我们讨论正弦和余弦函数的时候， 通常希望知道函数在 ±π 和 ±π2 的值。这样看来，当前的设置就不那么理想了。  

当我们设置记号的时候，我们可以同时设置记号的标签。注意这里使用了 LaTeX。   

```py
plt.xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi],
       [r'$-\pi$', r'$-\pi/2$', r'$0$', r'$+\pi/2$', r'$+\pi$'])

plt.yticks([-1, 0, +1],
       [r'$-1$', r'$0$', r'$+1$'])
```

### 移动脊柱 - ax.spines  

坐标轴线和上面的记号连在一起就形成了脊柱（Spines，一条线段上有一系列的凸起，是不是很像脊柱骨啊~）， 它记录了数据区域的范围。 它们可以放在任意位置，不过至今为止，我们都把它放在图的四边。  

实际上每幅图有四条脊柱（上下左右），为了将脊柱放在图的中间，我们必须将其中的两条（上和右）设置为无色，然后调整剩下的两条到合适的位置——数据空间的 0 点。  

```py
ax = plt.gca()
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
```


gca(**kwargs)
    Get the current :class:`~matplotlib.axes.Axes` instance on the
    current figure matching the given keyword args, or create one.
    
    To get the current polar axes on the current figure::
    
        plt.gca(projection='polar')
    
    If the current axes doesn't exist, or isn't a polar one, the appropriate
    axes will be created and then returned.


Help on function axes in module matplotlib.pyplot:

获取当前图层对象的坐标轴.   

axes(arg=None, **kwargs)
    Add an axes to the current figure and make it the current axes.
    
    Call signatures::
    
        plt.axes()
        plt.axes(rect, projection=None, polar=False, **kwargs)
        plt.axes(ax)
    
    Parameters
    ----------
    arg : { None, 4-tuple, Axes }
        The exact behavior of this function depends on the type:
    
        - *None*: A new full window axes is added using
          ``subplot(111, **kwargs)``
        - 4-tuple of floats *rect* = ``[left, bottom, width, height]``.
          A new axes is added with dimensions *rect* in normalized
          (0, 1) units using `~.Figure.add_axes` on the current figure.
        - `~.axes.Axes`: This is equivalent to `.pyplot.sca`.
          It sets the current axes to *arg*. Note: This implicitly
          changes the current figure to the parent of *arg*.
    
          .. note:: The use of an `.axes.Axes` as an argument is deprecated
                    and will be removed in v3.0. Please use `.pyplot.sca`
                    instead.
    
    projection : {None, 'aitoff', 'hammer', 'lambert', 'mollweide', 'polar', 'rectilinear', str}, optional
        The projection type of the `~.axes.Axes`. *str* is the name of
        a costum projection, see `~matplotlib.projections`. The default
        None results in a 'rectilinear' projection.
    
    polar : boolean, optional
        If True, equivalent to projection='polar'.
    
    sharex, sharey : `~.axes.Axes`, optional
        Share the x or y `~matplotlib.axis` with sharex and/or sharey.
        The axis will have the same limits, ticks, and scale as the axis
        of the shared axes.
    
    
    label : str
        A label for the returned axes.
    
    Other Parameters
    ----------------
    **kwargs
        This method also takes the keyword arguments for
        the returned axes class. The keyword arguments for the
        rectilinear axes class `~.axes.Axes` can be found in
        the following table but there might also be other keyword
        arguments if another projection is used, see the actual axes
        class.
          adjustable: {'box', 'datalim'}
      agg_filter: a filter function, which takes a (m, n, 3) float array and a dpi value, and returns a (m, n, 3) array 
      alpha: float
      anchor: 2-tuple of floats or {'C', 'SW', 'S', 'SE', ...}
      animated: bool
      aspect: {'auto', 'equal'} or num
      autoscale_on: bool
      autoscalex_on: bool
      autoscaley_on: bool
      axes_locator: Callable[[Axes, Renderer], Bbox]
      axisbelow: bool or 'line'
      clip_box: `.Bbox`
      clip_on: bool
      clip_path: [(`~matplotlib.path.Path`, `.Transform`) | `.Patch` | None] 
      contains: callable
      facecolor: color
      fc: color
      figure: `.Figure`
      frame_on: bool
      gid: str
      in_layout: bool
      label: object
      navigate: bool
      navigate_mode: unknown
      path_effects: `.AbstractPathEffect`
      picker: None or bool or float or callable
      position: [left, bottom, width, height] or `~matplotlib.transforms.Bbox`
      rasterization_zorder: float or None
      rasterized: bool or None
      sketch_params: (scale: float, length: float, randomness: float) 
      snap: bool or None
      title: str
      transform: `.Transform`
      url: str
      visible: bool
      xbound: (lower: float, upper: float) 
      xlabel: str
      xlim: (left: float, right: float)
      xmargin: float greater than -0.5
      xscale: {"linear", "log", "symlog", "logit", ...}
      xticklabels: List[str]
      xticks: list
      ybound: (lower: float, upper: float) 
      ylabel: str
      ylim: (bottom: float, top: float)
      ymargin: float greater than -0.5
      yscale: {"linear", "log", "symlog", "logit", ...}
      yticklabels: List[str]
      yticks: list
      zorder: float
    
    Returns
    -------
    axes : `~.axes.Axes` (or a subclass of `~.axes.Axes`)
        The returned axes class depends on the projection used. It is
        `~.axes.Axes` if rectilinear projection are used and
        `.projections.polar.PolarAxes` if polar projection
        are used.
    
    Notes
    -----
    If the figure already has a axes with key (*args*,
    *kwargs*) then it will simply make that axes current and
    return it.  This behavior is deprecated. Meanwhile, if you do
    not want this behavior (i.e., you want to force the creation of a
    new axes), you must use a unique set of args and kwargs.  The axes
    *label* attribute has been exposed for this purpose: if you want
    two axes that are otherwise identical to be added to the figure,
    make sure you give them unique labels.
    
    See Also
    --------
    .Figure.add_axes
    .pyplot.subplot
    .Figure.add_subplot
    .Figure.subplots
    .pyplot.subplots
    
    Examples
    --------
    ::
    
        #Creating a new full window axes
        plt.axes()
    
        #Creating a new axes with specified dimensions and some kwargs
        plt.axes((left, bottom, width, height), facecolor='w')

None

### 添加图例 - plt.legend  

我们在图的左上角添加一个图例。为此，我们只需要在 plot 函数里以「键 - 值」的形式增加一个参数。

```py
plt.plot(X, C, color="blue", linewidth=2.5, linestyle="-", label="cosine", zorder=-1)
plt.plot(X, S, color="red",  linewidth=2.5, linestyle="-", label="sine", zorder=-2)

plt.legend(loc='upper left')
```

其中的 zorder 参数用于控制绘图顺序.  

### 给一些特殊点做注释 - plt.annotate

我们希望在 2π/3 的位置给两条函数曲线加上一个注释。首先，我们在对应的函数图像位置上画一个点； 然后，向横轴引一条垂线，以虚线标记； 最后，写上标签。  

```py
t = 2*np.pi/3
plt.plot([t, t], [0, np.cos(t)], color ='blue', linewidth=2.5, linestyle="--")
plt.scatter([t,],[np.cos(t),], 50, color ='blue')

plt.annotate(r'$\sin(\frac{2\pi}{3})=\frac{\sqrt{3}}{2}$',
         xy=(t, np.sin(t)), xycoords='data',
         xytext=(+10, +30), textcoords='offset points', fontsize=16,
         arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

plt.plot([t,t], [0,np.sin(t)], color ='red', linewidth=2.5, linestyle="--")
plt.scatter([t,],[np.sin(t),], 50, color ='red')

plt.annotate(r'$\cos(\frac{2\pi}{3})=-\frac{1}{2}$',
         xy=(t, np.cos(t)), xycoords='data',
         xytext=(-90, -50), textcoords='offset points', fontsize=16,
         arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
```

### 精益求精   

坐标轴上的记号标签被曲线挡住了，作为强迫症患者（雾）这是不能忍的。我们可以把它们放大，然后添加一个白色的半透明底色。这样可以保证标签和曲线同时可见。  

```py
for label in ax.get_xticklabels() + ax.get_yticklabels():
    label.set_fontsize(16)
    label.set_bbox(dict(facecolor='white', edgecolor='None', alpha=0.65 ))
```

## Figure, subplot, axes  

到目前为止，我们都用隐式的方法来绘制图像和坐标轴。 快速绘图中，这是很方便的。 我们也可以显式地控制图像、子图、坐标轴。 

Matplotlib 中的「图像」指的是用户界面看到的整个窗口内容。 在图像里面有所谓「子图」。 子图的位置是由坐标网格确定的， 而「坐标轴」却不受此限制， 可以放在图像的任意位置。 

我们已经隐式地使用过图像和子图：当我们调用 plot 函数的时候，matplotlib 调用 gca() 函数以及 gcf() 函数来获取当前的坐标轴和图像； 如果无法获取图像，则会调用 figure() 函数来创建一个图像。严格地说，是用 subplot(1,1,1) 创建一个只有一个子图的图像。   


### 1.Figure 

所谓「图像」就是 GUI 里以「Figure #」为标题的那些窗口。 图像编号从 1 开始，与 MATLAB 的风格一致，而于 Python 从 0 开始编号的风格不同。 以下参数是图像的属性：

|参数| 默认值| 描述|
|----|----|-----|
|num| 1| 图像的数量|   
|figsize| figure.figsize| 图像的长和宽（英寸）  
|dpi| figure.dpi| 分辨率（点/英寸）  
|facecolor| figure.facecolor| 绘图区域的背景颜色  
|edgecolor| figure.edgecolor| 绘图区域边缘的颜色  
|frameon| True| 是否绘制图像边缘  

这些默认值可以在源文件中指明。 不过除了图像数量这个参数，其余的参数都很少修改。  

你在图形界面中可以按下右上角的 X 来关闭窗口（OS X 系统是左上角）。  Matplotlib 也提供了名为 close 的函数来关闭这个窗口。close 函数的具体行为取决于你提供的参数：  
- 不传递参数：关闭当前窗口；  
- 传递窗口编号或窗口实例（instance）作为参数：关闭指定的窗口；  
- all：关闭所有窗口。  

和其他对象一样，你可以使用 setp 或者是 set_something 这样的方法来设置图像的属性。   

### 2.subplot 子图  

你可以用子图来将图样（plot）放在均匀的坐标网格中。用 subplot 函数的时候，你需要指明网格的行列数量，以及你希望将图样放在哪一个网格区域中。此外，gridspec 的功能更强大，你也可以选择它来实现这个功能。  

```py
from pylab import *

subplot(2,1,1)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,1,1)',ha='center',va='center',size=24,alpha=.5)

subplot(2,1,2)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,1,2)',ha='center',va='center',size=24,alpha=.5)

# plt.savefig('.../snapshots/subplot-horizontal.png', dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/subplot-horizontal.png)

```py
from pylab import *

subplot(1,2,1)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(1,2,1)',ha='center',va='center',size=20,alpha=.5)

subplot(1,2,2)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(1,2,2)',ha='center',va='center',size=20,alpha=.5)

# plt.savefig('../snapshots/subplot-vertical.png', dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/subplot-vertical.png)

```py
from pylab import *

subplot(2,2,1)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,2,1)',ha='center',va='center',size=20,alpha=.5)

subplot(2,2,2)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,2,2)',ha='center',va='center',size=20,alpha=.5)

subplot(2,2,3)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,2,3)',ha='center',va='center',size=20,alpha=.5)

subplot(2,2,4)
xticks([]), yticks([])
text(0.5,0.5, 'subplot(2,2,4)',ha='center',va='center',size=20,alpha=.5)

# savefig('../snapshots/subplot-grid.png', dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/subplot-grid.png)


```py
from pylab import *
import matplotlib.gridspec as gridspec

G = gridspec.GridSpec(3, 3)

axes_1 = subplot(G[0, :])
xticks([]), yticks([])
text(0.5,0.5, 'Axes 1',ha='center',va='center',size=24,alpha=.5)

axes_2 = subplot(G[1,:-1])
xticks([]), yticks([])
text(0.5,0.5, 'Axes 2',ha='center',va='center',size=24,alpha=.5)

axes_3 = subplot(G[1:, -1])
xticks([]), yticks([])
text(0.5,0.5, 'Axes 3',ha='center',va='center',size=24,alpha=.5)

axes_4 = subplot(G[-1,0])
xticks([]), yticks([])
text(0.5,0.5, 'Axes 4',ha='center',va='center',size=24,alpha=.5)

axes_5 = subplot(G[-1,-2])
xticks([]), yticks([])
text(0.5,0.5, 'Axes 5',ha='center',va='center',size=24,alpha=.5)

#plt.savefig('../snapshots/gridspec.png', dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/gridspec.png)

### 3.axes - 坐标轴  

坐标轴和子图功能类似，不过它可以放在图像的任意位置。 因此，如果你希望在一副图中绘制一个小图，就可以用这个功能。  

axes 中的四元组分别是 [left, bottom, width, height].   

```py
from pylab import *

axes([0.1,0.1,.8,.8])
xticks([]), yticks([])
text(0.6,0.6, 'axes([0.1,0.1,.8,.8])',ha='center',va='center',size=20,alpha=.5)

axes([0.2,0.2,.3,.3])
xticks([]), yticks([])
text(0.5,0.5, 'axes([0.2,0.2,.3,.3])',ha='center',va='center',size=12,alpha=.5)

plt.savefig("../snapshots/axes.png",dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/axes.png)

```py
from pylab import *

axes([0.1,0.1,.5,.5])
xticks([]), yticks([])
text(0.1,0.1, 'axes([0.1,0.1,.5,.5])',ha='left',va='center',size=16,alpha=.5)

axes([0.2,0.2,.5,.5])
xticks([]), yticks([])
text(0.1,0.1, 'axes([0.2,0.2,.5,.5])',ha='left',va='center',size=16,alpha=.5)

axes([0.3,0.3,.5,.5])
xticks([]), yticks([])
text(0.1,0.1, 'axes([0.3,0.3,.5,.5])',ha='left',va='center',size=16,alpha=.5)

axes([0.4,0.4,.5,.5])
xticks([]), yticks([])
text(0.1,0.1, 'axes([0.4,0.4,.5,.5])',ha='left',va='center',size=16,alpha=.5)

plt.savefig("../snapshots/axes-2.png",dpi=64)
show()
```

效果图如下:   

![](/dev_tools/python/snapshots/axes-2.png)

## 刻度设置  

良好的记号是图像的重要组成部分。 Matplotlib 里的记号系统里的各个细节都是可以由用户个性化配置的。 你可以用 Tick Locators 来指定在那些位置放置记号，用 Tick Formatters 来调整记号的样式。 主要和次要的记号可以以不同的方式呈现。 默认情况下，每一个次要的记号都是隐藏的，也就是说，默认情况下的次要记号列表是空的 —— NullLocator。  

Tick Locators  

下面有为不同需求设计的一些 Locators。

类型	说明
NullLocator	No ticks.

IndexLocator	Place a tick on every multiple of some base number of points plotted. 

FixedLocator	Tick locations are fixed. 

LinearLocator	Determine the tick locations. 

MultipleLocator	Set a tick on every integer that is multiple of some base. 

AutoLocator	Select no more than n intervals at nice locations. 

LogLocator	Determine the tick locations for log axes. 

这些 Locators 都是 matplotlib.ticker.Locator 的子类，你可以据此定义自己的 Locator。以日期为 ticks 特别复杂，因此 Matplotlib 提供了 matplotlib.dates 来实现这一功能。


[【Matplotlib】 刻度设置(2)](https://www.cnblogs.com/nju2014/p/5633768.html)  
 

## 其他类型的图  

接下来的内容是练习。请运用你学到的知识，从提供的代码开始，实现配图所示的效果。具体的答案可以点击配图下载。   

1. 普通图   

```py
import numpy as np
import matplotlib.pyplot as plt

n = 256
X = np.linspace(-np.pi,np.pi,n,endpoint=True)
Y = np.sin(2*X)

plt.axes([0.025,0.025,0.95,0.95])

plt.plot (X, Y+1, color='blue', alpha=1.00)
plt.fill_between(X, 1, Y+1, color='blue', alpha=.25)

plt.plot (X, Y-1, color='blue', alpha=1.00)
plt.fill_between(X, -1, Y-1, (Y-1) > -1, color='blue', alpha=.25)
plt.fill_between(X, -1, Y-1, (Y-1) < -1, color='red',  alpha=.25)

plt.xlim(-np.pi,np.pi), plt.xticks([])  # 不显示坐标刻度
plt.ylim(-2.5,2.5), plt.yticks([])
savefig('../snapshots/plot_ex.png',dpi=48)
plt.show()
```

fill_between() 用于在曲线之间填充颜色, 曲线由 (x, y1) 和 (x, y2) 定义.  

```py
fill_between(x, y1, y2=0, where=None, interpolate=False, step=None, *, data=None, **kwargs)  
```

where 是一个 bool 数组, x[where] 定义了需要填充区域的坐标.   

效果图如下:   

![](/dev_tools/python/snapshots/plot_ex.png)  

2. 散点图 

```py
scatter(x, y, s=None, c=None, marker=None, cmap=None, norm=None, vmin=None, vmax=None, alpha=None, linewidths=None, verts=None, edgecolors=None, *, data=None, **kwargs)
```

使用不同大小和不同颜色的 marker 绘制散点图.   

x, y : 数组类型, 表示数据的坐标.  
s : optional, 标量或数组类型, 表示 marker 的大小.  
c : optional, 数组类型, 表示 marker 的颜色序列.  
    以下是可以选择的参数类型:   
    - A single color format string.
    - A sequence of color specifications of length n.
    - A sequence of n numbers to be mapped to colors using *cmap* and
        *norm*.
    - 使用只有 1 行的 RGB or RGBA 二维数组 (n,) 
marker : optional, `~matplotlib.markers.MarkerStyle`, 
    默认值: 'o'
    The marker style. *marker* can be either an instance of the class
    or the text shorthand for a particular marker.
    See `~matplotlib.markers` for more information marker styles.
cmap : `~matplotlib.colors.Colormap`, optional,  
norm : `~matplotlib.colors.Normalize`, optional,  
alpha : scalar, optional, 
           
    
```py
import numpy as np
import matplotlib.pyplot as plt

n = 1024
X = np.random.normal(0,1,n)
Y = np.random.normal(0,1,n)
T = np.arctan2(Y,X)

plt.axes([0.025,0.025,0.95,0.95])
plt.scatter(X, Y, s=75, c=T, marker='.', alpha=.5)
plt.pause(0.001)  # pause a bit so that plots are updated

plt.xlim(-1.5,1.5), plt.xticks([])
plt.ylim(-1.5,1.5), plt.yticks([])
savefig('../snapshots/scatter_ex.jpg',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/scatter_ex.jpg)   

3.条形图   

```py
import numpy as np
import matplotlib.pyplot as plt

n = 12
X = np.arange(n)
Y1 = (1-X/float(n)) * np.random.uniform(0.5,1.0,n)
Y2 = (1-X/float(n)) * np.random.uniform(0.5,1.0,n)

plt.axes([0.025,0.025,0.95,0.95])
plt.bar(X, +Y1, facecolor='#9999ff', edgecolor='white')
plt.bar(X, -Y2, facecolor='#ff9999', edgecolor='white')

for x,y in zip(X,Y1):
    plt.text(x, y+0.05, '%.2f' % y, ha='center', va= 'bottom')

for x,y in zip(X,Y2):
    plt.text(x, -y-0.05, '%.2f' % y, ha='center', va= 'top')

plt.xlim(-.5,n), plt.xticks([])
plt.ylim(-1.25,+1.25), plt.yticks([])

plt.savefig('../snapshots/bar_ex.png', dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/bar_ex.png)  

4. 等高线图  

```py
import numpy as np
import matplotlib.pyplot as plt

def f(x,y):
    return (1-x/2+x**5+y**3)*np.exp(-x**2-y**2)

n = 256
x = np.linspace(-3,3,n)
y = np.linspace(-3,3,n)
X,Y = np.meshgrid(x,y)

plt.axes([0.025,0.025,0.95,0.95])

plt.contourf(X, Y, f(X,Y), 8, alpha=.75, cmap=plt.cm.hot)
C = plt.contour(X, Y, f(X,Y), 8, colors='black', linewidth=.5)
plt.clabel(C, inline=1, fontsize=10)

plt.xticks([]), plt.yticks([])
plt.savefig('../snapshots/contour_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/contour_ex.png)  


5. 灰度图（Imshow）  

```py
import numpy as np
import matplotlib.pyplot as plt

def f(x,y):
    return (1-x/2+x**5+y**3)*np.exp(-x**2-y**2)

n = 10
x = np.linspace(-3,3,3.5*n)
y = np.linspace(-3,3,3.0*n)
X,Y = np.meshgrid(x,y)
Z = f(X,Y)

plt.axes([0.025,0.025,0.95,0.95])
plt.imshow(Z,interpolation='bicubic', cmap='bone', origin='lower')
plt.colorbar(shrink=.92)

plt.xticks([]), plt.yticks([])
plt.savefig('../snapshots/imshow_ex.png', dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/imshow_ex.png) 

6. 饼状图  

```py
import numpy as np
import matplotlib.pyplot as plt

n = 20
Z = np.ones(n)
Z[-1] *= 2

plt.axes([0.025, 0.025, 0.95, 0.95])

plt.pie(Z, explode=Z*.05, colors=['%f' % (i/float(n)) for i in range(n)],
        wedgeprops={"linewidth": 1, "edgecolor": "black"})
plt.gca().set_aspect('equal')
plt.xticks([]), plt.yticks([])

plt.savefig('../snapshots/pie_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/pie_ex.png)   

7. 量场图（Quiver Plots）  


```py
import numpy as np
import matplotlib.pyplot as plt

n = 8
X,Y = np.mgrid[0:n,0:n]
T = np.arctan2(Y-n/2.0, X-n/2.0)
R = 10+np.sqrt((Y-n/2.0)**2+(X-n/2.0)**2)
U,V = R*np.cos(T), R*np.sin(T)

plt.axes([0.025,0.025,0.95,0.95])
plt.quiver(X,Y,U,V,R, alpha=.5)
plt.quiver(X,Y,U,V, edgecolor='k', facecolor='None', linewidth=.5)

plt.xlim(-1,n), plt.xticks([])
plt.ylim(-1,n), plt.yticks([])

plt.savefig('../snapshots/quiver_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/quiver_ex.png)   

8. 网格  

```py
import numpy as np
import matplotlib.pyplot as plt

ax = plt.axes([0.025,0.025,0.95,0.95])

ax.set_xlim(0,4)
ax.set_ylim(0,3)
ax.xaxis.set_major_locator(plt.MultipleLocator(1.0))
ax.xaxis.set_minor_locator(plt.MultipleLocator(0.1))
ax.yaxis.set_major_locator(plt.MultipleLocator(1.0))
ax.yaxis.set_minor_locator(plt.MultipleLocator(0.1))
ax.grid(which='major', axis='x', linewidth=0.75, linestyle='-', color='0.75')
ax.grid(which='minor', axis='x', linewidth=0.25, linestyle='-', color='0.75')
ax.grid(which='major', axis='y', linewidth=0.75, linestyle='-', color='0.75')
ax.grid(which='minor', axis='y', linewidth=0.25, linestyle='-', color='0.75')
ax.set_xticklabels([])
ax.set_yticklabels([])

plt.savefig('../snapshots/grid_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/grid_ex.png) 

9. 多重网格  

```py
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
fig.subplots_adjust(bottom=0.025, left=0.025, top = 0.975, right=0.975)

plt.subplot(2,1,1)
plt.xticks([]), plt.yticks([])

plt.subplot(2,3,4)
plt.xticks([]), plt.yticks([])

plt.subplot(2,3,5)
plt.xticks([]), plt.yticks([])

plt.subplot(2,3,6)
plt.xticks([]), plt.yticks([])

plt.savefig('../snapshots/multiplot_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/multiplot_ex.png) 

10. 极轴图  

```py
import numpy as np
import matplotlib.pyplot as plt

ax = plt.axes([0.025,0.025,0.95,0.95], polar=True)

N = 20
theta = np.arange(0.0, 2*np.pi, 2*np.pi/N)
radii = 10*np.random.rand(N)
width = np.pi/4*np.random.rand(N)
bars = plt.bar(theta, radii, width=width, bottom=0.0)

for r,bar in zip(radii, bars):
    bar.set_facecolor( plt.cm.jet(r/10.))
    bar.set_alpha(0.5)

ax.set_xticklabels([])
ax.set_yticklabels([])
plt.savefig('../snapshots/polar_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/polar_ex.png) 

11. 3D 图  

```py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = Axes3D(fig)
X = np.arange(-4, 4, 0.25)
Y = np.arange(-4, 4, 0.25)
X, Y = np.meshgrid(X, Y)
R = np.sqrt(X**2 + Y**2)
Z = np.sin(R)

ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=plt.cm.hot)
ax.contourf(X, Y, Z, zdir='z', offset=-2, cmap=plt.cm.hot)
ax.set_zlim(-2,2)

plt.savefig('../snapshots/plot3d_ex.png',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/plot3d_ex.png) 

12. 手稿图  

```py
import numpy as np
import matplotlib.pyplot as plt

eqs = []
eqs.append((r"$W^{3\beta}_{\delta_1 \rho_1 \sigma_2} = U^{3\beta}_{\delta_1 \rho_1} + \frac{1}{8 \pi 2} \int^{\alpha_2}_{\alpha_2} d \alpha^\prime_2 \left[\frac{ U^{2\beta}_{\delta_1 \rho_1} - \alpha^\prime_2U^{1\beta}_{\rho_1 \sigma_2} }{U^{0\beta}_{\rho_1 \sigma_2}}\right]$"))
eqs.append((r"$\frac{d\rho}{d t} + \rho \vec{v}\cdot\nabla\vec{v} = -\nabla p + \mu\nabla^2 \vec{v} + \rho \vec{g}$"))
eqs.append((r"$\int_{-\infty}^\infty e^{-x^2}dx=\sqrt{\pi}$"))
eqs.append((r"$E = mc^2 = \sqrt{{m_0}^2c^4 + p^2c^2}$"))
eqs.append((r"$F_G = G\frac{m_1m_2}{r^2}$"))


plt.axes([0.025,0.025,0.95,0.95])

for i in range(24):
    index = np.random.randint(0,len(eqs))
    eq = eqs[index]
    size = np.random.uniform(12,32)
    x,y = np.random.uniform(0,1,2)
    alpha = np.random.uniform(0.25,.75)
    plt.text(x, y, eq, ha='center', va='center', color="#11557c", alpha=alpha,
             transform=plt.gca().transAxes, fontsize=size, clip_on=True)

plt.xticks([]), plt.yticks([])
plt.savefig('../snapshots/text_ex.jpg',dpi=48)
plt.show()
```

效果图如下:   

![](/dev_tools/python/snapshots/text_ex.jpg) 
