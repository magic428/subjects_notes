# 算法优化 - 让你的软件飞起来 - 算法优化    

朋友曾经给我推荐了一个有关代码优化的 pdf 文档《让你的软件飞起来》, 看完之后感受颇深。下面就是其的详细内容总结.   
 
## 速度取决于算法   
 
同样的事情, 方法不一样, 效果也不一样。比如汽车引擎, 可以让你的速度超越马车, 却无法超越音速；涡轮引擎, 可以轻松超越音障, 却无法飞出地球； 如果有火箭发动机, 就可以到达火星。   
 
代码的运算速度取决于以下几个方面:  
- 1, 算法本身的复杂度, 比如 MPEG 比 JPEG 复杂, JPEG 比 BMP 图片的编码复杂。  
- 2, CPU自身的速度和设计架构  
- 3, CPU的总线带宽   
- 4, 代码的写法   

本文主要介绍如何优化您自己的 code, 实现软件的加速。
 
## 先看看我的需求

我们一个图象模式识别的项目, 需要将 RGB 格式的彩色图像先转换成黑白图像。  

图像转换的公式如下：  
```cpp
Y = 0.299 * R + 0.587 * G + 0.114 * B;
```
图像尺寸 `640*480*24` bit, RGB 图像已经按照 RGBRGB 顺序排列的格式放在内存里面了。  
 
## 第一个优化 - 去掉浮点运算  

我已经悄悄的完成了第一个优化. 以下是输入和输出的定义：   

```cpp
#define XSIZE 640
#define YSIZE 480
#define IMGSIZE XSIZE * YSIZE
typedef struct RGB
{
       unsigned char r;
       unsigned char g;
       unsigned char b;
}RGB;

struct RGB in[IMGSIZE];         // 需要计算的原始数据
unsigned char out[IMGSIZE];     // 计算后的结果
```
优化原则：图像是一个 2D 数组, 我用一个一维数组来存储。编译器处理一维数组的效率要高过二维数组。  
 
先写一个代码：   

```cpp
// Y = 0.299 * R + 0.587 * G + 0.114 * B;   
 
void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i++)
    {
        double r, g, b, y;
        unsigned char yy;

        r = in[i].r;
        g = in[i].g;
        b = in[i].b;
        
        y = 0.299 * r + 0.587 * g + 0.114 * b;
        yy = y;
        out[i] = yy;
    }
}
```
这大概是能想得出来的最简单的写法了, 实在看不出有什么毛病, 好了, 编译一下跑一跑吧。   

**第一次试跑**:   
这个代码分别用 vc6.0 和 gcc 编译, 生成 2 个版本, 分别在 pc 上和我的 embedded system 上面跑。   

速度多少？   
- 在 PC 上, 由于存在硬件浮点处理器, CPU频率也够高, 计算速度为 20 秒。   
- 我的 embedded system, 没有以上 2 个优势, 浮点操作被编译器分解成了整数运算, 运算速度为 120 秒左右。   
 
**去掉浮点运算**:   
上面这个代码还没有跑, 我已经知道会很慢了, 因为这其中有大量的浮点运算。只要能不用浮点运算, 一定能快很多。   
 
这个公式 `Y = 0.299 * R + 0.587 * G + 0.114 * B;` 怎么能用定点的整数运算替代呢？

```cpp
Y = 0.299 * R + 0.587 * G + 0.114 * B;

Y = D + E + F;
D = 0.299 * R;
E = 0.587 * G;
F = 0.114 * B;
```

我们就先简化算式 D 吧！ `0.299 * R` 可以如何化简？RGB 的取值范围都是 0~255, 都是整数, 只是这个系数比较麻烦, 不过这个系数可以表示为：`0.299 = 299 / 1000;`, 因此 `D = ( R * 299) / 1000;`.   

```cpp
Y = (R * 299 + G * 587 + B * 114) / 1000;
```

这一下, 能快多少呢？   
- PC上的速度为 2 秒；  
- Embedded system上的速度为 45 秒；  

## 第二个优化 - 整数除法运算转换为位运算  

```cpp
Y = (R * 299 + G * 587 + B * 114) / 1000;
```

这个式子好像还有点复杂, 可以再砍掉一个除法运算。前面的算式 D 可以这样写：
```cpp
0.299= 299/1000 = 1224/4096
```
所以 D = (R * 1224) / 4096, Y = (R*1224)/4096+(G*2404)/4096+(B*467)/4096;   
再简化为：Y = (R*1224+G*2404+B*467)/4096;   
对于除以 4096, 因为它是 2 的 N 次方, 所以可以用移位操作替代: 右移 12 位就是把某个数除以 4096.    

```cpp
void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i++)
    {
        int r,g,b,y;
        r = 1224 * in[i].r;
        g = 2404 * in[i].g;
        b = 467 * in[i].b;

        y = r + g + b;
        y = y >> 12;   // 这里去掉了除法运算
        out[i] = y;
    }
}
```
这个代码编译后, 又快了 20% 。虽然快了不少, 还是太慢了一些, 在 Embedded system上 20 秒处理一幅图像还是都不能接受。  

## 第三个优化 - 查表  

仔细端详一下这个式子！   
```
Y = 0.299 * R + 0.587 * G + 0.114 * B;
Y=D+E+F;
D=0.299*R;
E=0.587*G;
F=0.114*B;
```
 
RGB 的取值有文章可做, RGB 的取值永远都大于等于 0, 小于等于 255, 我们能不能将 D, E, F 都预先计算好呢？然后用查表算法计算呢？
我们使用 3 个数组分别存放 DEF 的 256 种可能的取值, 然后...   

```cpp
// 查表数组初始化
int D[256],F[256],E[256];

void table_init()
{
    int i;
    for(i=0;i<256;i++)
    {
        D[i]=i*1224; 
        D[i]=D[i]>>12;

        E[i]=i*2404; 
        E[i]=E[i]>>12; 

        F[i]=i*467; 
        F[i]=F[i]>>12;
    }
}

void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i++)
    {
       int r,g,b,y;
        r = D[in[i].r];//查表
        g = E[in[i].g];
        b = F[in[i].b];
        y = r + g + b;
        out[i] = y;
    }
}
```
这一次的成绩把我吓出一身冷汗, 执行时间居然从 30 秒一下提高到了2秒！在PC上测试这段代码, 眼皮还没眨一下, 代码就执行完了。一下提高15 倍, 爽不爽？   

## 第四个优化 - 多个 ALU 并行  

很多 embedded system 的 32bit CPU, 都至少有 2 个ALU, 能不能让 2个ALU 都跑起来？   

```cpp
void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i += 2) //一次并行处理2个数据
    {
        int r,g,b,y,r1,g1,b1,y1;
        r = D[in[i].r];      //查表 //这里给第一个ALU执行
        g = E[in[i].g];
        b = F[in[i].b];
        y = r + g + b;

        out[i] = y;
        r1 = D[in[i + 1].r]; //查表 //这里给第二个ALU执行
        g1 = E[in[i + 1].g];
        b1 = F[in[i + 1].b];
        y = r1 + g1 + b1;
        out[i + 1] = y;
    }
}
```
2 个 ALU 处理的数据不能有数据依赖, 也就是说：某个 ALU 的输入条件不能是别的 ALU 的输出, 这样才可以并行。   
这次成绩是 1 秒。
 
## 第五个优化 - int 到 unsigned short  
查看这个代码:  

```cpp
int D[256],F[256],E[256]; //查表数组
void table_init()
{
    int i;
    for(i=0;i<256;i++)
    {
        D[i]=i*1224; 
        D[i]=D[i]>>12;
        E[i]=i*2404; 
        E[i]=E[i]>>12; 
        F[i]=i*467; 
        F[i]=F[i]>>12;
    }
}
```

到这里, 似乎已经足够快了, 但是我们反复实验, 发现, 还有办法再快！ 可以将 `int D[256],F[256],E[256];` 更改为
`unsigned short D[256],F[256],E[256];`. 这是因为编译器处理 `int` 类型和处理 `unsigned short` 类型的效率不一样。 

再改动:   

```cpp
inline void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i += 2) //一次并行处理2个数据
    {
        int r,g,b,y,r1,g1,b1,y1;
        r = D[in[i].r];//查表 //这里给第一个ALU执行
        g = E[in[i].g];
        b = F[in[i].b];
        y = r + g + b;
        out[i] = y;
        r1 = D[in[i + 1].r];//查表 //这里给第二个ALU执行
        g1 = E[in[i + 1].g];
        b1 = F[in[i + 1].b];
        y = r1 + g1 + b1;
        out[i + 1] = y;
    }
}
```
将函数声明为 inline, 这样编译器就会将其嵌入到母函数中, 可以减少 CPU 调用子函数所产生的开销。  
这次速度：0.5秒。   

其实, 我们还可以飞出地球的！
如果加上以下措施, 应该还可以更快：
- 1、把查表的数据放置在 CPU 的高速数据 CACHE 里面；    
- 2、把函数calc_lum()用汇编语言来写  

## 总结  

其实, CPU 的潜力是很大的.   
- 1、不要抱怨你的CPU, 记住一句话：“只要功率足够, 砖头都能飞！”   
- 2、同样的需求, 写法不一样, 速度可以从120秒变化为0.5秒, 说明CPU的潜能是很大的！看你如何去挖掘。  
- 3、我想：要是Microsoft的工程师都像我这样优化代码, 我大概就可以用 489 跑 windows XP了！  
 
以上就是对《让你的软件飞起来》的摘录, 下面对 RGB 到 YCbCr 的转换算法做以总结。   
 
```cpp
/**
 * Y =   0.299R + 0.587G + 0.114B
 * U = -0.147R - 0.289G + 0.436B
 * V =  0.615R - 0.515G - 0.100B
*/

#deinfe SIZE 256
#define XSIZE 640
#define YSIZE 480
#define IMGSIZE XSIZE * YSIZE
typedef struct RGB
{
       unsigned char r;
       unsigned char g;
       unsigned char b;
}RGB;

struct RGB in[IMGSIZE]; // 需要计算的原始数据
unsigned char out[IMGSIZE * 3]; // 计算后的结果
 
unsigned short Y_R[SIZE],Y_G[SIZE],Y_B[SIZE],U_R[SIZE],U_G[SIZE],U_B[SIZE],V_R[SIZE],V_G[SIZE],V_B[SIZE]; //查表数组
void table_init()
{
    int i;
    for(i = 0; i < SIZE; i++)
    {
        Y_R[i] = (i * 1224) >> 12; // Y 对应的查表数组
        Y_G[i] = (i * 2404) >> 12; 
        Y_B[i] = (i * 467)  >> 12;

        U_R[i] = (i * 602)  >> 12; // U 对应的查表数组
        U_G[i] = (i * 1183) >> 12; 
        U_B[i] = (i * 1785) >> 12;
        
        V_R[i] = (i * 2519) >> 12; // V 对应的查表数组
        V_G[i] = (i * 2109) >> 12; 
        V_B[i] = (i * 409)  >> 12;
    }
}
 
inline void calc_lum()
{
    int i;
    for(i = 0; i < IMGSIZE; i += 2) // 一次并行处理2个数据
    {     
        out[i]               = Y_R[in[i].r] + Y_G[in[i].g] + Y_B[in[i].b]; // Y
        out[i + IMGSIZE]     = U_B[in[i].b] - U_R[in[i].r] - U_G[in[i].g]; // U
        out[i + 2 * IMGSIZE] = V_R[in[i].r] - V_G[in[i].g] - V_B[in[i].b]; // V
 
        out[i + 1]                = Y_R[in[i + 1].r] + Y_G[in[i + 1].g] + Y_B[in[i + 1].b]; // Y
        out[i  + 1 + IMGSIZE]     = U_B[in[i + 1].b] - U_R[in[i + 1].r] - U_G[in[i + 1].g]; // U
        out[i  + 1 + 2 * IMGSIZE] = V_R[in[i + 1].r] - V_G[in[i + 1].g] - V_B[in[i + 1].b]; // V
    }
}
```
 
根据牛人的观点, 这种算法应该是非常快的了, 以后可直接使用了。   