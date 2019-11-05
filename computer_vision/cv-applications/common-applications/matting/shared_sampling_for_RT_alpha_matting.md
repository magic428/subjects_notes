论文引用: 

> [Shared Sampling for Real-Time Alpha Matting](http://inf.ufrgs.br/~eslgastal/SharedMatting/)
  Eduardo S. L. Gastal and Manuel M. Oliveira
  Computer Graphics Forum. Volume 29 (2010), Number 2.
  Proceedings of Eurographics 2010, pp. 575-584.   

## 1. 序言

这篇论文的入门学习是通过这篇博客: [图像抠图算法学习 - Shared Sampling for Real-Time Alpha Matting](https://www.cnblogs.com/Imageshop/p/3550185.html), 熟悉了其中的大致原理之后才到原文中深入研究, 同时在这篇博客中也引用了文中的一些效果图. 感谢 Imageshop 大神的博客专栏: https://www.cnblogs.com/Imageshop, 在这里学到了很多, 希望跟随大神的脚步不断成长.  

关于这篇论文, 有一个专门的主页: http://www.inf.ufrgs.br/~eslgastal/SharedMatting/, 其中包含了论文 pdf 下载和论文中使用的数据集的下载,  github 中有实现代码: https://github.com/np-csu/AlphaMatting.   

论文的标题简单粗暴, 实时性是最大的亮点. 但是论文中也重点说明其实时性的前提条件是:   

* 必须依赖于 GPU 的并行编程优化;  

另外一个限制是针对算法本身的. 在总结部分会详细说明.  

## 2. 算法细节

接下来根据论文的结构依次描述.  

总的来说, 这篇文章还是属于 Alpha Matting 的范畴. 因此这篇文章要解决的问题依然是 Alpha Matting 的经典方程:  

$$
\tag{1}
C_p = \alpha_p  F_p  + (1 - \alpha_p ) B_p 
$$

式中: $C_p $ 是我们观察到的图像的颜色, 式子右侧的左右变量: $ \alpha_p, F_p, B_p $ 均是未知量, 分别表示 **未知区域** 的透明度, 前景及背景.   

鉴于未知量数目过多, 必须增加一些附加的约束才能求解这个方程. 常用的约束是是: (1) 和源图像大小相同的 TriMap 图; (2) 用户手工画出的草图形式. 如下两图所示:    

![](https://img-blog.csdn.net/20180913133708474?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70) 


TriMap 图像的像素数据类型为 int 型, 如未特别说明, 一般白色部分表示前景, 黑色表示背景, 灰色表示待识别的部分.   

上面的约束条件明确了属于前景 ($\alpha = 1$) 的部分和属于背景 ($\alpha = 0$), 求解方程就只是计算那些未知区域的 $\alpha$ 值. 

**求解方程最终会得到一个 $\alpha$ matte 图.**   

论文中提到, 在 2010 年前后解决 matting 问题的主要方法是基于 sampling, pixel afﬁnities 或者两者的结合 (sampling + pixel afﬁnities), 特别是 pixel afﬁnities 和 两者的结合( sampling + pixel afﬁnities) 是主流的方式. local afﬁnities 被用在求解或精修 matte 图. 但是这两种都需要求解一个大型的线性系统, 系统的大小和未知区域的像素点个数成正比, 因此对于 1MB 左右大小的图, 求解时间在几秒到几分钟不等.  

TODO: 这篇论文提出的算法应该说是基于 sampling 技术的, 他充分利用了相邻像素之间的相似性, 并利用了算法内在的并行性, 结合 GPU 编程, 实现抠图的实时展示. 

TODO: 另外, 论文中反复提到的一个前提: 在一个小的邻域内的像素值的 $(\alpha, F, B)$ 值是相似的.我们会利用这种相似性来大幅度降低计算复杂度.  



总的来说, 论文提出的算法可以分成4个步骤: 

* 第一步: Expansion - 针对用户的输入, 对已知区域进行小规模的缩小, 即扩展对应的背景或前景区域;   

* 第二步: Sample and Gather - 对剩余的未知区域内的每个点按一定的规则取样, 并选择出最佳的一对前景和背景取样点, 并计算 $\alpha_p$ 值;  

* TODO: 第三步: Reﬁnement - 在一定的邻域范围内, 对未知区域内的每个点的最佳配对重新进行组合;  

* 第四步: Local Smoothing - 对得到的前景和背景对以及透明度值进行局部平滑, 以减少噪音.   

**除非特别声明, 否则本文提到的所有邻域均在位置区域内.**

### 2.1  Expansion   

这一步的作用就是减少未知点的个数, 可能在一定程度上减小后期的计算量. 原理也很简单:   

对一个未知点 $p$, 如果在其一定的邻域半径内 (文中推荐值 10 pixel, 并且是圆形半径) 有已知的背景点或前景点 $q$, 计算未知点 $p$ 和 $q$ 的颜色距离 (用 RGB 空间的欧氏距离度量, 表示为: $D{color}$) 满足 $D{color} < k_c$ (文中推荐 $k_c$ 为 5/256), 则把这个未知点 $p$ 归属于 $q$ 所在的区域 (前景或背景).    

TODO: 在 github 提供的参考代码中, 这一部分的编码其实写的还是很有特色的, 他的循环方式不同于我们普通的邻域编码, 他是从像素点逐渐向外部循环开来, 有点类似左图的这种循环方式 (实际上还是有点区别的, 实际是上下两行一起处理, 在左右两列处理, 然后再向外层扩散) , 这种处理方式的明显好处就是, 只要找到某个点颜色距离小于设定的值, 就可以停止循环了, 因为这个点肯定是第一个符合颜色距离条件又同时符合物理距离最小的要求的. 

![TODO](https://img-blog.csdn.net/20180914104100850?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)


我们最终的目的是: 对于未知区域的一个像素点, 找到一个 $(\alpha_p, F_p, B_p)$ 满足方程(1). 论文中反复提到的一个前提: 在一个小的邻域内的像素值的 $(\alpha, F, B)$ 值是相似的, 利用这种相似性可以大幅度降低计算复杂度.  

现在采用以下策略: (1) 将任务划分为不同的邻域 - Sample Gathering, (2) 然后再共享并精修结果 - Sample Refinement.  

### 2.2  Sample Gathering

这一步的作用是: 从每个未知区域的像素点 $p$ 所在邻域的采样点集合对中选择最优的一对采样点, $p$ 所在邻域的若干采样点是通过下面的方式来保证这些采样点对来自不相邻的邻域内.  

这一步是算法的核心部分, 先看下图:  

![](https://img-blog.csdn.net/20180914104124887?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

在这个图中, $p$ 和 $q$ 点都处在未知区域, 我们需要通过一定的原则在已知区域为其取得一定的采样点, 论文中提出的采样方法是:  

设定一个参数 $k_g$, 表示一个未知像素点最多可能取样的前景点和背景点的个数, 因此取样点可以组成的点对数目最多为 $k_g*k_g$ 对, 论文建议  $k_g$ 值取 4 就可以取得很好的效果, 越大则程序越耗时.   

这样对于每个未知点, 从该点出发引出 $k_g$ 条路径, 每个路径之间成 $2\pi / k_g$ 的夹角, 记录下每条路径经过的路线中首次遇到的前景或背景点, 直到超出图像的边缘.   

为了算法的稳定性, 每 3x3 的矩形区域内 (也可以是 4x4 或者 5x5, 当然会更耗时), 起始路径的角度 (和水平线的夹角) 会周期性的改变, 这样相邻像素的 $k_g$ 条路径经过的区域的覆盖面就比较丰富, 最终得到的采样点对集合中的点也就有一定的差异性, 这样的采样点对集合更为有效.   

例如在上图中, $p$ 点对应的 $k_g = 4$ 条路径用红色表示, 其最终得到的采样点为 1 个背景点, 2 个前景点; $q$ 点对应的 $k_g = 4$ 条路径用蓝色表示, 其最终得到的采样点为 1 个背景点, 3 个前景点. 

由上图可以看到, 在不少情况下, 未知点的前景和背景取样数并不能达到 $k_g$ 个, 甚至极端情况下, 找不到任何一个取样点, 这样该点就无法进行 $\alpha_p$ 的计算了, 这就要靠后面的过程了.   

在取样完成后, 需要找出这些取样点集合中最佳的一个点对组合, 既然是最优化问题, 就需要一个目标函数. 在这篇论文中, 目标函数用了四个小函数 (考虑了空间距离, 光学和空间概率信息) 的乘积来计算.   

从采样结合中选择一组前景点和背景点: $\mathbf f_i, \mathbf b_j$, 其 color 分别为: $F^i, B^j$.  

#### 最小化色彩失真   

准则 1: 最优的采样点对应该使色彩失真最小化;   

$$
\tag{2}
M_p(F^i, B^j) = \parallel C_p - (\hat{\alpha_p}F^i + (1- \hat{\alpha_p})B^j) \parallel
$$

其中:   

* $M_p$ 表示色彩失真程度;  
* $C_p$ 是 $p$ 的 color 值; 
* $\hat{\alpha_p}$ 是 $p$ 的不透明度的估计值, 可通过将 $C_p$ 在颜色空间投影到 $F^i, B^j$ 定义的直线上计算出来;    

直观的理解就是: 对于给定的采样点对, 带入公式 (1) 可以计算得到 $\hat{C}_p$, $M_p$ 就是衡量 $p$ 点真实的 color 值 $C_p$ 和 $\hat{C}_p$ 之间的误差;  

需要明白的是: 最优采样点对可以计算得到小的 $M_p$ 值, 但是小的 $M_p$ 值并不是选择最优采样点对的充分条件.  

基于上述原因, 文中提出了一种新的颜色度量方式.

(1) 对于小邻域内的像素在颜色空间内很容易局部聚类, 特别是小邻域内包含有边缘部分时;  
(2) 如果背景和前景的梯度范数 $\parallel \nabla B \parallel$ 和 $\parallel \nabla F \parallel$ 远小于 $\parallel \nabla \alpha \parallel$, 那么图像的梯度 $\parallel \nabla I \parallel$ 就和 $\parallel \nabla \alpha \parallel$ 成正比;  

TODO: 上面的论述是为了什么?   

最优采样点对不仅可以使可以使 $M_p$ 最小, 同时也可以使 $p$ 所在的小邻域内的所有像素的 $M_p$ 最小. 因此有:  

$$
\tag{3}
N_p(\mathbf f_i, \mathbf b_j) = \sum_{q\in\Omega_p}M_q(F^i, B^j)^2
$$

其中:   

* $\Omega_p$ 表示 $p$ 的像素邻域, 大小为 3x3, $p$ 为邻域中心;  
* $M_q$ 即 (公式 2) 的色彩失真;   

公式 (2) 的道理很为明显, 用一对 F/B 算出的 α 值如果很合理的话, 那么用 α 结合 F/B 重新计算出的颜色应该和原始颜色的差距很小. 公式 (3) 在表明在一定的邻域内, 由于像素一般不会有突变, 差值的平均值也应该很小.   

#### 图像空间统计信息     

文中使用图像空间统计信息来估计像素点 $p$ 属于前景的概率, 这个概率是为了纠正之前得到的 $\alpha$ 值.   

图像空间定义为: $D_p(s) = D_{image}(s,p) = \parallel s-p \parallel$.  

定义一个能量函数 $E_p(s)$, 表示从点 $p$ 到一个前景或背景采样点所需要的能量:  

$$
\tag{4}
E_p(s) = \int_L {\parallel \nabla I \cdot dr\parallel}^2 = \int_p^s {\parallel \nabla I \cdot dr\parallel \left(\frac{s-p}{\parallel s-p \parallel}\right)}^2
$$

公式 (4) 的直观理解是, $E_p(s)$ 和 $\nabla I$ 在向量 $(s-p)$ 法线方向上的投影长度成正比. 因此, 如果 $s$ 到 $p$ 形成的路径穿过了图像中 $\parallel \nabla I \parallel$ 值较大的区域 (如边缘区域), 此时就需要更大的能量.   

这时就可以估计像素点 $p$ 属于前景的概率:   

$$
\tag{5}
PF_p = \frac{\text{min}_j(E_p(\mathbf b_j))}{\text{min}_i(E_p(\mathbf f_i)) + \text{min}_j(E_p(\mathbf b_j))} 
$$

公式 (5) 的直观理解是: 如果到达前景采样点所需的最小能量远小于到达背景采样点所需的能量, $PF_p \approx 1$, 也就是说像素 $p$ 属于前景的概率非常大.  

因此, 可以利用公式 (5) 得到的 $PF_p$ 中蕴含的空间统计信息来校正公式 (2) 中计算得到的 $\hat{\alpha}_p$ 值. 因此, 最优的采样点对应该使以下函数最小化:  

$$
\tag{6}
A_p(\mathbf f_i, \mathbf b_j) = PF_p + (1 - 2 PF_p)\hat{\alpha}_p
$$

对上述公式直观的理解就是: 对于给定的采样点对 $\mathbf f_i, \mathbf b_j) $,   

* 当 $PF_p = 0$ 时, $A_p(\mathbf f_i, \mathbf b_j) = \hat{\alpha}_p$, 因此, 最小化 $A_p(\mathbf f_i, \mathbf b_j) $ 的值其实就是最小化 $\hat{\alpha}_p$;  
* 当 $PF_p = 1$ 时, $A_p(\mathbf f_i, \mathbf b_j) = 1 - \hat{\alpha}_p$, 因此, 最小化 $A_p(\mathbf f_i, \mathbf b_j) $ 的值其实就是最大化 $\hat{\alpha}_p$;  
* 当 $PF_p = 0.5$ 时, $A_p(\mathbf f_i, \mathbf b_j) = 0.5$, 此时 $\hat{\alpha}_p$ 对 $A_p(\mathbf f_i, \mathbf b_j) $ 的最小化过程无影响;  

#### 目标函数   

终于到了**主角一号(目标函数)**出场的时刻.   

考虑在未知点到取样的前景和背景点之间的直线路径上, 应该尽量要少有像素的突变. 如果这条路径需要经过图像的边缘区域, 则应该设计一个函数使得该函数的返回值较大, 于是作者使用公式(4), 这样就避开了穿越边界区域的采样点, 比如图 (2) 中的 $q$ 点就具有这样一个点.  
 
考虑未知点和前景点之间的物理距离, 一个好的组合中的前景点应该要尽量靠近未知点;  

考虑未知点和背景点之间的物理距离, 一个好的组合中的背景点也应该要尽量靠近未知点;  

目标函数考虑了像素的空间距离特征, 光学特征和空间统计信息来选择最优的采样点对. 其形式如下:   

$$
\tag{7}
g_p(\mathbf f_i, \mathbf b_j) = N_p(\mathbf f_i, \mathbf b_j)^{e_N} \cdot A_p(\mathbf f_i, \mathbf b_j)^{e_A} \cdot D_p(\mathbf f_i)^{e_f} \cdot  D_p(\mathbf b_i)^{e_b}
$$

其中:  

* $ N_p(\mathbf f_i, \mathbf b_j)$ 在 3x3 邻域内最小化色彩失真;   
* $ A_p(\mathbf f_i, \mathbf b_j)$ 使用前景概率来校正 \hat{\alpha}_p$;   
* $ D_p(\mathbf f_i)$ 和 $ D_p(\mathbf b_i)$ 在空间上保证采样点对中的点尽可能的靠近点 $p$;   
* 常数 ${e_N}, {e_A}, {e_f}, {e_b}$ 是惩罚因子, 目的是放大最终的函数值最小化程度. 文中推荐取值 ${e_N = 3}, {e_A = 2}, {e_f = 1}, {e_b =4}$.   

因此, 最优采样点对通过最小化函数 $g_p(\mathbf f_i, \mathbf b_j)$ 得到:  

$$
\tag{8}
(\hat{\mathbf f}_p, \hat{\mathbf b}_p) = \text{argmin}_{\mathbf f, \mathbf b} g_p(\mathbf f_i, \mathbf b_j)
$$

假设我们已经得到了最优采样点对: $(\hat{\mathbf f}_p, \hat{\mathbf b}_p)$, $(F_p^g, B_p^g)$ 是在 gathering 阶段求出的点 $p$ 的颜色值. 可以计算 $\sigma^2_f$ 和 $\sigma^2_b$:  

$$
\tag{9}
\sigma^2_f = \frac{1}{N}\sum_{q \in \Omega_f} \parallel C_q - F^g_p\parallel\\
\sigma^2_b = \frac{1}{N}\sum_{q \in \Omega_b} \parallel C_q - B^g_p\parallel
$$

其中:   

* $\Omega_f$ 和 $\Omega_b$ 分别是以 $(\hat{\mathbf f}_p, \hat{\mathbf b}_p)$ 为中心的 5x5 邻域; 因此 N = 25. 
* 

假设: $(\hat{\mathbf f}_p, \hat{\mathbf b}_p)$ 点邻域内的颜色分布符合单变量高斯分布, $\sigma^2_f$ 和 $\sigma^2_b$ 测量的是 $(\hat{\mathbf f}_p, \hat{\mathbf b}_p)$ 点所在邻域的局部 color 的方差, 


为方便理解, 我贴出计算α的部分代码: 

```cpp
/** 
 * \brief: 通过当前点、前景点以及背景点的颜色值计算对应的 Alpha 值, 对应论文的公式 (12) . 
 * 
 * \param "BC、GC、RC"> 当前点的 BGR 颜色分量值.  
 *        "BF、GF、RF"> 前景点的 BGR 颜色分量值.  
 *        "BF、GF、RF"> 背景点的 BGR 颜色分量值.  
 * 
 * Alpha 会出现不在 [0,1] 区间的情况, 因此需要限制范围.
 */
double CalcAlpha(int BC, int GC, int RC, int BF, int GF, int RF, int BB, int GB, int RB)
{
    double fen = (double) ((BC - BB) * (BF - BB) + (GC - GB) * (GF - GB) + (RC - RB) * (RF - RB));
    double den = ((BF - BB) * (BF - BB) + (GF - GB) * (GF - GB) + (RF - RB) * (RF - RB) + 0.0000001);
    double Alpha = fen / den;
    
    return min(1, max(0, Alpha));
}
```

### 2.3 Sample Reﬁnement

初步的 gathering 处理后, 正如前文所说, 得到的结果还不够细腻, 并且有些未知点由于采样的过程未收集到有效的前景和背景数据, 造成该点无法进行处理, 因此, 在 Reﬁnement 阶段需要进一步解决这个问题.   

这里就到了**主角二号(shared sampling)**出场的时刻, 也就是论文标题中的 shared sampling, 即共享像素点 $p$ 周围邻域内所有像素的最优采样点对.  

具体做法是: 计算像素点 $p$ 邻域内所有点的最优采样点对, 每个像素点 $p$ 都可以得到一组 $(F, B, \alpha)$ 使得其对应的 $M_p(F^g_q, B^g_q)$ 最小, 然后选择使 $M_p(F^g_q, B^g_q)$ 最小的 $k_r$ 组 $(F, B, \alpha)$, 使用这 $k_r$ 组 (文中推荐值为 5) 数据的平均值来计算 $p$ 的 $\widetilde\tau_p^g = (\widetilde F_p^g, \widetilde B_p^g, \widetilde\sigma_f^2, \widetilde\sigma_f^2 )$. 上述平均值操作可以抑制 matte 图中的噪声.    

然后按照下面这些公式计算新的前景、背景、透明度及可信度, 即 $\tau_p^r = ( F_p^r,  B_p^r, \alpha_p^r, f_p^r )$.   

$$
\tag{10}
F_p^r = 
\begin{cases}
C_p, & \text{if} \quad \parallel C_p - \widetilde F_p^g \parallel^2 \leq \widetilde \sigma_f^2\\
\widetilde F_p^g, &  \text{otherwise}
\end{cases}
$$

$$
\tag{11}
B_p^r = 
\begin{cases}
C_p, & \text{if} \quad \parallel C_p - \widetilde B_p^g \parallel^2 \leq \widetilde \sigma_b^2\\
\widetilde B_p^g, &  \text{otherwise}
\end{cases}
$$

$$
\tag{12}
\alpha^r_p = \frac{(C_p - B_p^r) \cdot (F_p^r - B_p^r)}{\parallel F_p^r - B_p^r \parallel^2}
$$

$$
\tag{13}
f_p^r = 
\begin{cases}
exp\{-\lambda M_p(\widetilde F_p^g, \widetilde B_p^g) \}, & \text{if} \quad F_p^r \neq B_p^r \\
\epsilon, &   \text{if} \quad F_p^r = B_p^r 
\end{cases}
$$

其中:   

* r 上标表示 refinement 阶段;  
* 

关于 $F_p^r$ 的直观理解就是: 如果像素点 $p$ 的颜色值 $C_p$ 和平均颜色值 $\widetilde F_p^g$ 非常相似, 那么 $F_p^r$ 就使用像素点的颜色值, 否则继续使用平均颜色值;   

$B_p^r$ 和 $F_p^r$ 是类似的;   

$\alpha^r_p$ 的物理意义是 $C_p - B_p^r$ 在 $F_p^r - B_p^r$ 向量方向上的投影长度, 即表示像素点 $p$ 的不透明度;   

$f_p^r$ 表示像素点 $p$ 的候选前景值和背景值 $(\widetilde F_p^g, \widetilde B_p^g)$ 准确性的置信度; 如果候选前景值和背景值不能很好的表示 $C_p$, 那么对应的置信度值应该会变小(但是不会急剧变小). 文中 $\lambda$ 推荐取值为 10. 特别地, 当前景值和背景值很相似时, 也就不能准确估计 $\hat{\alpha}_p$ 的值, 因此将其设置为一个很小的值 $\epsilon = 10^{-8}$.  

置信度的计算是为下一步的局部平滑做准备的, 它反应了我们在这一步确定的取样点是否合理程度的一个度量, 经由此步骤, 我们可得到的 matte 图和合成图如下所示:  

![TODO:](https://img-blog.csdn.net/20180914102810836?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

可见在这一步得到的结果对于上图来说已经相当完美了. 

### 2.4 Local Smoothing

这一步说实在的我没有花太多的精力去看, 他的实现过程大概有点类似于高斯模糊, 但里面多了很多其他方面的处理, 一个很好的事情就是在CSDN提供的代码中对这部分每个公式的实现都是正确的, 也是完整的, 因此, 有兴趣的朋友需要自己多看下论文和对应的代码了.  

## 3. 算法的效果

按照论文提供的相关资料集我自己搜集的一些图及配套的 Trimap 测试了该算法的一些结果, 现贴出如下所示: 


![TODO](https://img-blog.csdn.net/20180914103248713?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  


![TODO](https://img-blog.csdn.net/20180914103348895?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)


![TODO](https://img-blog.csdn.net/20180914103434739?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2d6ajIwMTM=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

## 4. 编程实现

在编程实现方面，CSDN提供的那个代码基本的意思已经达到了，并且里面的函数意义也非常清晰，只不过他使用的opencv的库的，相信专心去研究抠图的人，把他改成其他语言也不是个难题，比如里面用到的vector在C#中就可以用list代替，那些Opencv的结构体也可以在C#中重新定义。

不过那个代码占用的内存非常厉害，这主要是由于VECTOR等数据类型决定的，实际上这里完全可以用数组来搞定的。

我贴一部分代码大家看看：

```cpp
/// <summary>
///  对每个未知区域的像素按设定的循环角度搜索有效的前景和背景取样点
/// </summary>
/// <param name="X">未知区域的X坐标。</param>
/// <param name="Y">未知区域的Y坐标。</param>
/// <param name="F">用于保存前景点的内存区域。</param>
/// <param name="B">用于保存背景点的内存区域。</param>
/// <param name="CountF">最终获取的前景点的数量。</param>
/// <param name="CountB">最终获取的背景点的数量。</param>
///    <remarks>对于有些点，是有可能获取不到有效的点的。</remarks>
void Sample(int X, int Y, Point *F, Point *B, int &CountF, int &CountB)
{
    int Z, XX, YY, Alpha;
    bool F1, F2;
    double InitAngle, IncAngle, Angle;
    double Dx, Dy, Step, Value, XD, YD, StepX, StepY;
    IncAngle = 360 / KG;                                                //    每次扫描增加的角度
    InitAngle = (Y % 5 * 5 + X % 25) * IncAngle / 25;                        //  起始角度，范围在[0,IncAngle]之间，按照3*3的方式轮流替换，有利于提供结果的稳定性，如果KG取值较大，也可以使用4*4或者5*5
    CountF = 0; CountB = 0;                                                //    起步时需要记为0，注意参数中的引用（&）
    for (Z = 0; Z < KG; Z++)
    {
        F1 = false; F2 = false;                                            //    开始寻找，暂时未找到任何前景点和背景点
        Angle = (InitAngle + Z * IncAngle) / 180.0f * 3.1415926f;        //    每次搜索的角度
        Dx = cos(Angle);    Dy = sin(Angle);
        Step = min(1.0f / (abs(Dx) + 1e-10), 1.0f / (abs(Dy) + 1e-10));    
        XD = X + 0.5;        YD = Y + 0.5;                                //    +0.5使用于int四舍五入取整，相当于其他语言的round函数
        StepX = Step * Dx ;    StepY = Step * Dy ;                            //    StepX和StepY中必然有一个为1，另外一个小于1，这样保证每次搜索的像素点都会不同，一个好的建议是加大这个循环步长
                                                                        //    这样有两个好处。1：加快查找速度；2：让搜索点可以进入已知点的内部，从而避免了只从已知点的边缘取样。但如果已知点的区域狭长，可能丢失有效取样点。
        for (; ;)                
        {
            XX = int(XD);
            if (XX < 0 || XX >= Width) break;                            //    已经超出了图像的边界了，结束循环
            YY = int(YD);
            if (YY < 0 || YY >= Height) break;
            XD += StepX;    YD += StepY;
            Alpha = Mask[YY * MaskStride + XX];                            //    得到路径中该点的特征值（前景/背景/未知)
            if (F1 == false && Alpha == 0)                                //    如果沿这条路径尚未找到背景点，并且改点具有背景点的特征，则记录下改点到背景点序列中
            {
                B[CountB].X = XX;                                        //    背景点的X坐标
                B[CountB].Y = YY;                                        //    背景点的Y坐标
                CountB++;                                                //    背景点数量增加1
                F1 = true;                                                //    在此路径已经找到了背景点，不用再找背景点了。
            }
            else if (F2 == false && Alpha == 255)                        //    如果沿这条路径尚未找到前景点，并且改点具有前景点的特征，则记录下改点到前景点序列中
            {
                F[CountF].X = XX;                                        //    前景点的X坐标
                F[CountF].Y = YY;                                        //    前景点的X坐标
                CountF++;                                                //    前景点数量增加1
                F2 = true;                                                //    在此路径已经找到了前景点，不用再找前景点了。
            }
            else if (F1 == true && F2 == true)                            //    如果前景点和背景点都已经找到了，则结束循环
            {
                break;
            }
        }
    }
}

```
　　通过以上的Sample代码，我们就可以避免使用Vector之类的数据结构了，速度和内存占用都会得到改进。

     然后在Gather阶段的代码改成如下的方式。


```cpp
void Gathering()
{
    int X, Y, K, L, Index, Speed;
    int CountF, CountB, Fx, Fy, Bx, By;
    double Pfp, Min, Dpf, Gp;
    bool Flag;
    Point *F = (Point *) malloc(KG * sizeof(Point));            //    采用这种方式占用的内存小很多
    Point *B = (Point *) malloc(KG * sizeof(Point));
    for (Y = 0; Y < Height; Y++)
    {
        Index = Y * MaskStride;
        for (X = 0; X < Width; X++)
        {
            tuple[Index].Flag = -1;                                //    先都设置为无效的点
            if (Mask[Index] != 0 && Mask[Index] != 255)            //    只处理未知点
            {
                Sample(X, Y, F, B, CountF, CountB);                //    对当前点进行前景和背景取样        
                Pfp = CalcPFP(X, Y, F, B, CountF, CountB);        //  计算公式（5），因为公式（5）只于前景和背景取样点的整体有关，无需放到下面的循环内部        
                Min = 1e100;
                Flag = false;
                for (K = 0; K < CountF; K++)                    //    对于每一个前景点
                {
                    Dpf = CalcDp(X, Y, F[K].X, F[K].Y, true);    //    计算前景点到中心点的欧式距离
                    for (L = 0; L < CountB; L++)                //    对于每一个背景点
                    {
                        Gp = CalcGp(X, Y, F[K].X, F[K].Y, B[L].X, B[L].Y, Pfp, Dpf);    //    按照公式（7）计算目标函数
                        if (Gp < Min)
                        {
                            Min = Gp;
                            Fx = F[K].X; Fy = F[K].Y;            //    记录下目标函数为最小值处的前景和背景点的坐标
                            Bx = B[L].X; By = B[L].Y;
                            Flag = true;
                        }
                    }
                }
                if (Flag == true)                                //    说明找到了最好的组合了，如果找不到,则原因可能是：（1）Sample过程为找到任何有效的前景和背景点；（2）Sample过程只找到前景点或只找到背景点
                {                                                //    某个点找不到也不用怕，可能在下面的Refine过程中（其算法为领域处理）得以弥补。
                    Speed = Fy * Stride + Fx * 3;                //    记录下最佳前景点的颜色值
                    tuple[Index].BF = ImageData[Speed];
                    tuple[Index].GF = ImageData[Speed + 1];
                    tuple[Index].RF = ImageData[Speed + 2];
                    Speed = By * Stride + Bx * 3;
                    tuple[Index].BB = ImageData[Speed];            //    记录下最佳背景点的颜色值
                    tuple[Index].GB = ImageData[Speed + 1];
                    tuple[Index].RB = ImageData[Speed + 2];
                    tuple[Index].SigmaF = CaclSigma(Fx, Fy);    //    计算前景点周边像素的均方差值
                    tuple[Index].SigmaB = CaclSigma(Bx, By);    //    计算背景点周边像素的均方差值
                    tuple[Index].Flag = 1;                        //    这个像素是个有效的处理点
                }
            }
            Index++;
        }
    }
    free(F);
    free(B);
}

```
由于 github 提供了代码，其他的代码我这里就不提供了。 

## 算法的缺陷

　   以下内容纯属个人意见，请各位阅读的朋友根据自己的知识去判断。

         1、Sample过程存在潜在的问题：论文的图2阐述了对某点进行取样的操作过程，这个过程在第一次遇到前景或背景点时就把该点视为前景或背景的一个取样点。这样的话所有的取样点的数据都只可能取在Trimap或者scripple图的前景和背景区域的边缘处。如果边缘处的像素和内部的有很大的区别，显然会出现部分取样点取到的数据很不合理，从而导致最终的透明度信息错误。一种可行的方案就是在Sample的过程中修改每次取样的步长，比如，下面的方式：

　　　　StepX = Step * Dx * 10 ; StepY = Step * Dy * 10 ;  

　　即每次沿X或者Y方向移动10个像素，这样就会使得取样点会进入已知区域内部最多10个像素左右，同时也会加速取样的速度。

     不过这样做也隐形的带来一个新的问题，即有可能会丢失一些取样点，当已知点的宽度小于10像素时出现这种情况。

      2、还是会存在某些点无法获取有效的取样点的，即使有了refine过程。

　　 3、算法的复杂度还是很高，虽然整体的并行性很好，如果没有GPU参与，纯CPU实现起来一样很吃力。

## 效果验证

       我用C写了个DLL，并提供了三个示例程序，给有兴趣的朋友测试下效果： http://files.cnblogs.com/Imageshop/SharedMatting.rar


## 4. 总结  

另外一个限制是针对算法本身的. 因为文中提出的算法是以一个假设为前提展开的, 也就是假设未知区域的前景色和背景色可以通过分析其邻域像素来显式估计出来. 因此, 对于那些前景完全透明, 或前景色和背景色存在严重重叠的图片就会出现问题.  

## 5. 参考资料  

[1]. [图像抠图算法学习 - Shared Sampling for Real-Time Alpha Matting](https://www.cnblogs.com/Imageshop/p/3550185.html)   