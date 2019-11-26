# 3D LineDetection

Fast 3D Line Segment Detection From Unorganized Point Cloud. 论文地址: https://arxiv.org/abs/1901.02532.  

github: https://github.com/xiaohulugo/3DLineDetection.  

> 应用: 通过线检测和分割, 将点云中的大致结构勾勒出来.  

## 论文 Pipeline
 
- Point Cloud Segmentation: 使用区域增长和区域合并方法 (region growing and region
merging) 将输入的点云分割成多个 3D 平面;  
- Plane Based 3D Line Detection: 将每个 3D 平面上的所属点投影到其上形成 2D 平面, 然后使用 2D 轮廓提取和最小二乘拟合法 (Least Square Fitting) 来得到每个 2D 平面的线分割. 然后将这些 2D 线重投影到 3D 平面上得到 3D 线分割;  
- Post-processing: 通过探索场景的结构化信息移除前面得到的 3D 平面和 3D 线分割的 outliers, 之后合并第二步中分割得到的相邻的 3D 线.  

## 算法具体实现  

### A 点云分割  

点云分割算法通常有: edge/border based, region growing based and hybrid (refer to [15] for a review).   

A much robuster way is extracting small compact regions first and then merge those regions to get the final result.  

normal calculation, region growing and region merging.

**法向计算 - normal calculation**:  

KNN 获取邻域点, 然后给予 PCA 在邻域点构成的曲面上计算法向.  

- 首先建立 k-d 树;  
- 然后计算每个点的协方差矩阵(其实就是方差);  

协方差（Covariance）在概率论和统计学中用于衡量两个变量的总体误差。  而方差是协方差的一种特殊情况，即当两个变量是相同的情况。  

Solving the following standard eigenvalue equation via Singular Value Decomposition (SVD), we can finally get the normal of the point, which is the third eigenvector in the matrix of eigenvectors V.

矩阵 V 的第三个向量就是特征向量. lambda 的第三个值 $ lambda_3 $ 表示点的曲率, 因此 $ lambda_3 $ 越小, 该点所在的曲率越小.    

根据 KNN 结果来简单估计该点在邻域中的分布的 scale, 其定义为该点和其邻域中第三个最近点之间的距离.  

We denote the normal, the curvature, the scale and the set of KNNs of point Pi as nPi , lambda Pi , sPi and IPi , respectively.  

**区域增长: Region Growing**  

- (1) 根据曲率对所有点进行升序排序;  
- (2) 从曲率最小的点 P_s 开始, 创建一个 list T 来保存所有和 P_s 共面的点, 自然地 P_s 是第一个点; 然后对 T 中未被处理的点 P_i, 依次遍历其邻域中未被处理的点 P_j, 如果满足以下条件, 则将 P_j 添加到 T 中:  

区域增长规则限制条件. (参数 \theta 为角度阈值, th_o 为正交距离阈值, th_p 为并行距离阈值).  

Both tho and thp are self-adapted by the scale of Ps.  

第一个条件保证 P_s 和 P_j 共面;  
第 3 个条件保证区域是紧凑地聚集在 P_s 周围;  

Once all the points in IPi are traversed, we mark Pi as processed. The region is growing point by point until all the points in list T are processed, the result of which is a planar region with all its points stored in list T .  

planar region: 平面区域.  

- (3) The second step is iteratively performed until all the points are processed. The set of all the planar regions obtained is denoted as R.   

**Region Merge**  

Firstly, for each region Ri,




 
## 调参  

在两个平面不是很平整的情况下, 如何界定两个 3D 平面的交线?   



这个算法对于包含平整的平面和线条的场景效果良好, 但是对于井下场景的坑坑哇哇并不适用.   

算法中使用的区域增长思路和 PCA 思路.  


