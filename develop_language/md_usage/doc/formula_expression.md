#  MarkDown 中公式编辑语法笔记    

> `$ formula $`会使 `formula` 嵌入到文本行中; `$$ formula $$`会使 `formula` 单独占用一行并居中.    

$A$

## 希腊字母

`\小写希腊字母英文全称` 和 `\首字母大写希腊字母英文全称` 来分别输入小写和大写希腊字母。 

|    显示     |     输入    |   显示    |     输入     |    显示   |      输入     |    显示    |   输入    |
|------------|-------------|----------|-------------|-----------|--------------|---|-------------|
| $\alpha$   |`$\alpha$`   | $A$      | `$A$`       | $\beta$   | `$\beta$`    | $B$        | `$B$`       |
| $\gamma$   |`$\gamma$`   | $\Gamma$ | `$\Gamma$`  | $\delta$  | `$\delta$`   | $\Delta$   | `$\Delta$`  |
| $\epsilon$ |`$\epsilon$` | $E$      | `$E$`       | $\zeta$   | `$\zeta$`    | $Z$        | `$Z$`       |
| $\eta$     |`$\eta$`     | $H$      | `$H$`       | $\theta$  | `$\theta$`   | $\Theta$   | `$\Theta$`  |
| $\iota$    |`$\iota$`    | $I$      | `$I$`       | $\kappa$  | `$\kappa$`   | $K$        | `$K$`       |
| $\lambda$  |`$\lambda$`  | $\Lambda$| `$\Lambda$` | $\nu$     | `$\nu$`      | $N$        | `$N$`       |
| $\mu$      |`$\mu$`      | $M$      | `$M$`       | $\xi$     | `$\xi$`      | $\Xi$      | `$\Xi$`     |
| $o$        |`$o$`        | $O$      | `$O$`       | $\pi$     | `$\pi$`      | $\Pi$      | `$\Pi$`     |
| $\rho$     |`$\rho$`     | $P$      | `$P$`       | $\sigma$  | `$\sigma$`   | $\Sigma$   | `$\Sigma$`  |
| $\tau$     |`$\tau$`     | $T$      | `$T$`       | $\upsilon$| `$\upsilon$` | $\Upsilon$ | `$\Upsilon$`|
| $\phi$     |`$\phi$`     | $\Phi$   | `$\Phi$`    | $\chi$    | `$\chi$`     | $X$        | `$X$`       |
| $\psi$     |`$\psi$`     | $\Psi$   | `$\Psi$`    | $\omega$  | `$\omega$`   | $\Omega$   | `$\Omega$`  |
| $\varsigma$     |`$\varsigma$`     | $\aleph$   | `$\aleph$`    | $\beth$  | `$\beth$`   | $\daleth$   | `$\daleth$`  |
| $\gimel$     |`$\gimel$`     | $\aleph$   | `$\aleph$`    | $\beth$  | `$\beth$`   | $\daleth$   | `$\daleth$`  |


 

## 上标, 下标表示    

## 上标, 下标表示    
- 上标: 用 `^` 后的内容表示上标. 例如: $x^{(i)}$, $y^{(i)}$;   
- 下标, 用 `_` 后的内容表示上标. 例如: $x_{(i)}$, $y_{(i)}$;    
- 上下标混用: 举例： $x_1^2, x^{(n)}_{22}, ^{16}O^{2-}_{32}, x^{y^{z^a}}, x^{y_z}$;  

当角标位置看起来不明显时，可以强制改变角标大小或层次，举例如下：   

$$
y_N, y_{_N} 
$$
第一种输出为正常输出，但输出效果不明显；    
第二种是将一级角标改为二级角标，字体也自动变为二级角标字体；


如果需要使用文字作为角标，首先要把文字放在 `\mbox{}` 文字模式中，另外要加上改变文字大小的命令(如"\tiny", "\scriptsize", "\footnotesize", "\small", "\normalsize", "\large", "\Large", "\LARGE")，例如：  

$$\partial f_{\mbox{\tiny 极大值 }}$$

## 分数形式   

分式格式： `\frac{分子}{分母}`   

`\dfrac{}{}`( 等价于 $\displaystyle\frac{}{} ) 会显示大号的分式, 而 `\tfrac{}{}` 则显示小号的分式.   

$$ \dfrac{1}{2} \tfrac{1}{2} $$     

`\tiny\frac{}{}` 会显示超级小号的公式.   


行内分式 $\frac{x+y}{y+z}$， 
行间分式：$$\frac{x+y}{y+z}$$    

上面的例子表明行内分式字体比行间字体小，因为行内分式使用的是角标字体，可以人工改变行内分式的字体大小，例如：   $\displaystyle\frac{x+y}{y+z}$.   
 
连分式：  
$$
x_0+\frac{1}{x_1+\frac{1}{x_2+\frac{1}{x_3+\frac{1}{x_4}}}}
$$

可以通过强制改变字体大小使得分子分母字体大小一致，例如：   

$$
\newcommand{\FS}[2]{\displaystyle\frac{#1}{#2}}x_0+\FS{1}{x_1+\FS{1}{x_2+\FS{1}{x_3+\FS{1}{x_4}}}}$$

其中第一行命令定义了一个新的分式命令，规定每个调用该命令的分式都按 `\displaystyle` 的格式显示分式；   

分数线长度值是预设为分子分母的最大长度，如果想要使分数线再长一点，可以在分子或分母两端添加一些间隔，例如 $\frac{1}{2}, \frac{\;1\;}{\;2\;}$, 其中第一个显示是正常的显示，第二个显示是分子分母前后都放入一个间隔命令 `\;`.   

## 根式

二次根式命令：`\sqrt{表达式}`.   

如果表达式是个单个字符，则不需要花括号，但需要在字符和 sqrt 之间加入一个空格.  

n次根式命令：`\sqrt[n]{表达式}`.   

被开方表达式字符高度不一致时，根号上面的横线可能不是在同一条直线上；为了使横线在同一条直线上，可以在被开方表达式插入一个只有高度没有宽度的数学支柱 `\mathstrut`, 例如：    
$$
\sqrt{a}+\sqrt{b}+\sqrt{c}, \qquad \sqrt{\mathstrut a}+\sqrt{\mathstrut b}+\sqrt{\mathstrut c}
$$ 

当被开方表达式高时，开方次数的位置显得略低，解决方法为：将开方此时改为上标，并拉近与根式的水平距离，即显示将命令中的 [n] 改为 [^n\!] (其中 ^ 表示是上标， \! 表示缩小间隔)，例如：  
$$
\sqrt{1+\sqrt[p]{1+\sqrt[q]{1+a}}}
$$

$$
\sqrt{1+\sqrt[^p\!]{1+\sqrt[^q\!]{1+a}}}
$$

## 定界符

自适应放大命令：`\left` 和 `\right`， 本命令放在左右定界符前，随着公式内容大小自动调整符号大小.   

例子：

$$
\left(1/xyz\right)
$$

$$
\left(\frac{1}{xyz}\right)
$$

还有另外一种使用方式:   
$$
（）\big(\big) \Big(\Big) \bigg(\bigg) \Bigg(\Bigg)
\big(\Big) \bigg(\Bigg)
$$


## 数学重音符号  

以 a 为例, 如果字母 i 或 j 带有重音，字母 i,j 应该替换为 $\imath$, $\jmath$.  

$$
\hat{a}        \quad   \check{a}  \\ 
\breve{a}      \quad   \tilde{a}  \\
\bar{a}        \quad   \vec{a}    \\
\acute{a}      \quad   \grave{a}  \\
\mathring{a}   \quad   \dot{a}  \quad \ddot{a}
$$

## Norm 符号  

$$
\norm{w}
$$

## 求和与积分, 条件偏导数    

求和命令：`\sum_{k=1}^n 表达式`（求和项紧随其后,下同）.  
积分命令：`\int_a^b 表达式`.   

例如：无穷级数 $\sum_{k=1}^{\infty}\frac{x^n}{n!}$ 可以化为积分 $\int_0^\infty e^x$.   

改变上下限位置的命令：`\limits(强制上下限在上下侧)` 和 `\nolimits(强制上下限在左右侧)`. 例如： $\sum\limits_{k=1}^n$ 和 $\sum\nolimits_{k=1}^n$.   

条件偏导   

$\left.\frac{\partial f(x,y)}{\partial x}\right|_{x=0}$    

## 空白间距 - 占位宽度

`\quad` 代表当前字体下接近字符 `M` 的宽度;  

|符号|Latex|
|---|----| 
|一个 quad 空格| $a \quad b$ | 
|两个 quad 空格| $ a \qquad b$ |
|大空格, 1/3 quad 空格| $a\ b$ |  
|中等空格| $a\:b$, $a\;b$ | 
|小空格| $a\,b$ |  
|没有空格| $ab$ | 
|紧贴, 缩进1/6m宽度| $a\!b$ | 

## 多行公式

无需对齐可使用 `multline`, 需要对齐使用 `split`;   

用 `\\` 和 `&` 来分行和设置对齐的位置.   

$$ \begin{multline}
    x = a+b+c+{} \\
        d+e+f+g
    \end{multline}  $$

对齐:
$$ 
\begin{split}
x = {} & a + b + c +{}\\
	&d + e + f + g
\end{split}
$$

## 公式组

不需要对齐的公式组用 gather，需要对齐使用 align.   

不需要对齐:   
$$ \begin{gather} a = b+c+d\\ x=y+z \end{gather} $$ 
 
对齐:  
$$
\begin{equation}
\begin{aligned}
a &=b+c+d \\
x &=y+z
\end{aligned}
\end{equation}
$$

## 分支公式

分段函数通常用 cases 次环境携程分支公式:   
$$
y=\begin{cases}
-x,\quad x\leq 0\\
x, \quad x>0
\end{cases}$$



## 集合相关的运算命令

|符号|Latex| 
|---|----|  
|集合的大括号| $\{ ...\}$ | 
|属于| $\in $ | 
|不属于| $\not\in $ | 
|A包含于B| $ A\subset B$ | 
|A真包含于B| $A \subsetneqq B $ | 
|A包含B| $A \supset B $ | 
|A真包含B| $A \supsetneqq B$ |  
|A不包含于B|$ A \not \subset B $ | 
|A交B| $A \cap B $ | 
|A并B| $A \cup B $ | 
|A的闭包| $\overline{A}$ | 
|A减去B| $A\setminus B$ | 
|实数集合| $\mathbb{R} $ | 
|空集| $\emptyset $ | 

## 下划线、上划线等

上划线命令： `\overline{公式}`;   
下划线命令： `\underline{公式}`;  

例如：$\overline{\overline{a^2}+\underline{ab}+\bar{a}^3}$.   

上花括弧命令：`\overbrace{公式}{说明}`;  
下花括弧命令：`\underbrace{公式}_{说明}`;  

例如：$\underbrace{a+\overbrace{b+\dots+b}^{m\mbox{\tiny 个}}}_{20\mbox{\scriptsize 个}}$.  

## 省略号

省略号用 `\dots \cdots \vdots \ddots`表示, `\dots` 和 `\cdots` 的纵向位置不同，前者一般用于有下标的序列. 例如:  

$$
x_1, x_2, \dots, x_n \\
1,2,\cdots,n    \\
\vdots          \\
\ddots 
$$

## 堆积符号

\stacrel{上位符号}{基位符号} 基位符号大，上位符号小.   
{上位公式\atop 下位公式} 上下符号一样大.  
{上位公式\choose 下位公式} 上下符号一样大；上下符号被包括在圆弧内.   

例如：   
$$
\vec{x}\stackrel{\mathrm{def}}{=}{x_1,\dots,x_n}\\ 
{n+1 \choose k}={n \choose k}+{n \choose k-1}\\ 
\sum_{k_0,k_1,\ldots>0 \atop k_0+k_1+\cdots=n}A_{k_0}A_{k_1}\cdots
$$ 

## 矩阵  

$$
\begin{array}{ccc}
x_{_{11}} & x_{_{12}} & \dots & x_{_{1n}} \\
x_{_{21}} & x_{_{22}} & \dots & x_{_{2n}} \\
\vdots & \vdots & \ddots  & \vdots  \\
x_{_{m1}} & x_{_{m2}} & \dots & x_{_{mn}} \\
\end{array}
$$

不同包含边界的矩阵:   
$$
\begin{pmatrix} a & b\\ c & d \\ \end{pmatrix} \quad
\begin{bmatrix} a & b \\ c & d \\ \end{bmatrix}\quad
\begin{Bmatrix} a & b \\ c & d\\ \end{Bmatrix}\quad
\begin{vmatrix} a & b \\ c & d \\ \end{vmatrix}\quad
\begin{Vmatrix} a & b\\ c & d \\ \end{Vmatrix}
$$

小型矩阵:   
$$ (\begin{smallmatrix} a & b \\ c & d \end{smallmatrix}) $$

## 给公式加一个方框   

命令: `\boxed{公式}`    
$$ 
E = mc^2 \\
\boxed{E=mc^2} 
$$ 

## 运算符

`+ - * / =` 直接输入，特殊运算则用以下特殊命令:  

$$ 
\pm\; \times\; \div\; \\
\cdot\; \cap\; \cup\; \\
\geq\; \leq\; \neq\; \\
\approx\; \equiv 
$$ 

x 趋向于 0:  
$$x\to0$$ 

和、积、极限、积分等运算符用 `\sum, \prod, \lim, \int`, 这些公式在行内公式被压缩，以适应行高，可以通过`\limits`和`\nolimits`命令显示是否压缩. 例如:   

$ \sum_{i=1}^n i \quad \prod_{i=1}^n \quad  
\lim_{x\to0}x^2 \quad \int_{a}^{b}x^2 dx $

$$ \sum_{i=1}^n i \quad \prod_{i=1}^n 
\quad \lim_{x\to0} x^2 \quad \int_a^b x^2 dx $$

$$ \sum_{i=1}^n i \quad	\prod_{i=1}^n\quad
\lim_{x\to0} x^2 \quad \int_a^b x^2 dx $$

$$\sum\nolimits_{i=1}^n\quad\prod\nolimits_{i=1}^n\quad
\lim\nolimits_{x\to0} x^2 \quad \int\nolimits_a^b x^2 dx $$

多重积分使用如下形式 `\int、\iint、\iiint、\iiiint、\idotsint`，例如

$$
\iint\quad \iiint\quad \iiiint\quad \idotsint
$$

等价于:  

$$ 
\int\int  \\
\int\int\int    \\
\int\int\int\int\\ 
\int\dots\int
$$

## 箭头符号  

$$ 
\leftarrow \\
\rightarrow \\
\leftrightarrow \\
\Leftarrow \\
\Rightarrow  \\
\Leftrightarrow   \\
\longleftarrow \\
\longleftarrow \\
\longleftrightarrow   \\
\Longleftarrow \\
\Longrightarrow  \\
\Longleftrightarrow
$$   

`\xleftarrow` 和 `\xrightarrow` 可根据内容自动调整: 

$$
\xleftarrow{x+y+z} \quad \xrightarrow[x<y]{x+y+z} 
$$



MathJax是一个用来在网页上显示复杂的数学符合和公式的js库。本文总结部分MathJax符号，本页作为一个参考页面，用到的时候随时查询，采用的是LaTex语法（MathJax的LaTex语法，实际上是TeX-AMS，即美国数学学会统一使用的LaTex标准）。

  

逻辑运算符

latex	显示效果
\because	∵
\therefore	∴
\forall	∀
\exists	∃
\not=	≠
\not>	≯
\not\subset	⊄
集合运算符

latex	显示效果
\emptyset	∅
\in	∈
\notin	∉
\subset	⊂
\supset	⊃
\subseteq	⊆
\supseteq	⊇
\bigcap	⋂
\bigcup	⋃
\bigvee	⋁
\bigwedge	⋀
\biguplus	⨄
\bigsqcup	⨆
箭头符号

latex	显示效果
\uparrow	↑
\downarrow	↓
\Uparrow	⇑
\Downarrow	⇓
\updownarrow	↕
\Updownarrow	⇕
\rightarrow	→
\leftarrow	←
\Rightarrow	⇒
\Leftarrow	⇐
\leftrightarrow	↔
\Leftrightarrow	⇔
\longrightarrow	⟶
\longleftarrow	⟵
\Longrightarrow	⟹
\Longleftarrow	⟸
\longleftrightarrow	⟷
\Longleftrightarrow	⟺
更多的箭头符号

latex	显示效果
\mapsto	↦
\longmapsto	⟼
\hookleftarrow	↩
\hookrightarrow	↪
\leftharpoonup	↼
\rightharpoonup	⇀
\leftharpoondown	↽
\rightharpoondown	⇁
\rightleftharpoons	⇌
\leadsto	⇝
\nearrow	↗
\searrow	↘
\swarrow	↙
\nwarrow	↖
\nleftarrow	↚
\nrightarrow	↛
\nLeftarrow	⇍
\nRightarrow	⇏
\nleftrightarrow	↮
\nLeftrightarrow	⇎
\dashrightarrow	⇢
\dashleftarrow	⇠
\leftleftarrows	⇇
\leftrightarrows	⇆
\Lleftarrow	⇚
\twoheadleftarrow	↞
\leftarrowtail	↢
\looparrowleft	↫
\leftrightharpoons	⇋
\curvearrowleft	↶
\circlearrowleft	↺
\Lsh	↰
\upuparrows	⇈
\upharpoonleft	↿
\downharpoonleft	⇃
\multimap	⊸
\leftrightsquigarrow	↭
\rightrightarrows	⇉
\rightleftarrows	⇄
\rightrightarrows	⇉
\rightleftarrows	⇄
\twoheadrightarrow	↠
\rightarrowtail	↣
\looparrowright	↬
\rightleftharpoons	⇌
\curvearrowright	↷
\circlearrowright	↻
\Rsh	↱
\downdownarrows	⇊
\upharpoonright	↾
\downharpoonright	⇂
\rightsquigarrow	⇝
关系运算符

latex	显示效果
\mid	∣
\nmid	∤
\cdot	⋅
\leq	≤
\geq	≥
\neq	≠
\approx	≈
\equiv	≡
\circ	∘
\ast	∗
\bigodot	⨀
\bigotimes	⨂
\bigoplus	⨁
其他运算符

latex	显示效果
\pm	±
\times	×
\div	÷
\sum	∑
\prod	∏
\coprod	∐

（1）任意

$ {\forall}$
（2）存在

$ {\exists}$
（3）



\lbrace1,2,...,n\rbrace