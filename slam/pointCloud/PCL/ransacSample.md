# RANSAC - PCL 采样一致性算法  

在计算机视觉领域中排除错误的样本时, 广泛地使用各种采样一致性参数估计算法, 样本不同对应的应用不同, 例如剔除错误的配准点对, 分割出处在模型上的点集,  PCL 中以随机采样一致性算法 ( RANSAC ) 为核心, 同时实现了五种类似与随机采样一致形算法的随机参数估计算法, 例如随机采样一致性算法 ( RANSAC ) 最大似然一致性算法 (MLESAC) , 最小中值方差一致性算法 (LMEDS) 等, 所有估计参数算法都符合一致性原则. 在 PCL 中设计的采样一致性算法的应用主要就是对点云进行分割, 根据设定的不同的几个模型, 估计对应的几何参数模型的参数, 在一定容许的范围内分割出在模型上的点云.   

## 1 RANSAC 随机采样一致性 

RANSAC 是 "RANdom SAmple Consensus (随机抽样一致)" 的缩写. 它可以从一组包含 "局外点" 的观测数据集中, 通过迭代方式估计数学模型的参数. 它是一种不确定的算法——它有一定的概率得出一个合理的结果；为了提高概率必须提高迭代次数.   

数据分两种: 有效数据 (inliers) 和无效数据 (outliers). 偏差不大的数据称为有效数据, 偏差大的数据是无效数据. 如果有效数据占大多数, 无效数据只是少量时, 我们可以通过最小二乘法或类似的方法来确定模型的参数和误差；如果无效数据很多 (比如超过了 50% 的数据都是无效数据), 最小二乘法就失效了, 我们需要新的算法.  

一个简单的例子是从一组观测数据中找出合适的 2 维直线. 假设观测数据中包含局内点和局外点, 其中局内点近似的被直线所通过, 而局外点远离于直线. 简单的最小二乘法不能找到适应于局内点的直线, 原因是最小二乘法尽量去适应包括局外点在内的所有点. 相反, RANSAC 能得出一个仅仅用局内点计算出模型, 并且概率还足够高. 但是, RANSAC 并不能保证结果一定正确, 为了保证算法有足够高的合理概率, 我们必须小心的选择算法的参数.  

RANSAC 算法 Pipeline:  

RANSAC 算法的输入是一组观测数据, 一个可以解释或者适应于观测数据的参数化模型, 一些可信的参数.  
RANSAC 通过反复选择数据中的一组随机子集来达成目标. 被选取的子集被假设为局内点, 并用下述方法进行验证:  

1. 有一个模型适应于假设的局内点, 即所有的未知参数都能从假设的局内点计算得出.  
2. 用 1 中得到的模型去测试所有的其它数据, 如果某个点适用于估计的模型, 认为它也是局内点.  
3. 如果有足够多的点被归类为假设的局内点, 那么估计的模型就足够合理.   
4. 用所有假设的局内点去重新估计模型, 因为它仅仅被初始的假设局内点估计过.  
5. 最后, 通过估计局内点与模型的错误率来评估模型.  

伪码形式的算法如下所示:   

```py
'''
@param: 
    data: 一组观测数据
    model: 适应于数据的模型
    n: 适用于模型的最少数据个数
    k: 算法的迭代次数
    t: 用于决定数据是否适应于模型的阀值
    d: 判定模型是否适用于数据集的数据数目

@return:  
    best_model: 跟数据最匹配的模型参数 (如果没有找到好的模型, 返回null) 
    best_consensus_set: 估计出模型的数据点
    best_error: 跟数据相关的估计出的模型错误
'''

iterations = 0
best_model = null
best_consensus_set = null
best_error = 无穷大

while ( iterations < k )
    maybe_inliers = 从数据集中随机选择 n 个点
    maybe_model = 适合于 maybe_inliers 的模型参数
    consensus_set = maybe_inliers

    for ( 每个数据集中不属于 maybe_inliers 的点 ) 
        if ( 如果点适合于 maybe_model, 且错误小于 t ) 
            将点添加到 consensus_set
    if  ( consensus_set 中的元素数目大于 d ) 
        已经找到了好的模型, 现在测试该模型到底有多好
        better_model = 适合于 consensus_set 中所有点的模型参数
        this_error = computer_error(better_model, consensus_set) # 误差度量函数
        if ( this_error < best_error )
            我们发现了比以前好的模型, 保存该模型直到更好的模型出现
            best_model =  better_model
            best_consensus_set = consensus_set
            best_error =  this_error
    k += 1
return best_model, best_consensus_set, best_error
```

## 2 最小中值法 (LMedS) 

LMedS 的做法很简单, 就是从样本中随机抽出 N 个样本子集, 使用最大似然 (通常是最小二乘) 对每个子集计算模型参数和该模型的偏差, 记录该模型参数及子集中所有样本中偏差居中的那个样本的偏差 (即 Med 偏差) , 最后选取 N 个样本子集中 Med 偏差最小的所对应的模型参数作为我们要估计的模型参数.   

## 3. PCL 中 sample_consensus (分割)模块支持的几何模型  

(1) SACMODEL_PLANE  

平面模型. 模型对应的 4 个参数是其 Hessian Normal form: [normal_x normal_y normal_z d].  

(2) SACMODEL_LINE  

直线模型. 模型对应的 6 个参数分别是线上一点和直线的方向向量: [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z].  

(3) SACMODEL_CIRCLE2D  

平面内部的一个 2D 圆. 模型对应的 3 个参数分别是圆心坐标和半径: [center.x center.y radius].  

(4) SACMODEL_CIRCLE3D  

平面内的一个 3D 圆. 模型对应的 7 个参数分别为圆心坐标, 半径和法向量: [center.x, center.y, center.z, radius, normal.x, normal.y, normal.z].  

(5) SACMODEL_SPHERE   

球面模型. 对应的 4 个参数分别为球心坐标和半径: [center.x center.y center.z radius].  

(6) SACMODEL_CYLINDER  

圆柱面模型. 对应的 7 个参数分别为圆柱中心轴上的一点, 轴向和半径: [point_on_axis.x point_on_axis.y point_on_axis.z axis_direction.x axis_direction.y axis_direction.z radius].  

(7) SACMODEL_CONE  

圆锥体模型. 对应的 7 个参数分别为顶点坐标,圆锥中心轴的方向向量和开角: [apex.x, apex.y, apex.z, axis_direction.x, axis_direction.y, axis_direction.z, opening_angle].  

(8) SACMODEL_PARALLEL_LINE  

与指定轴平行的线, 需要指定一个最大偏离角. 对应的参数和 SACMODEL_LINE 的参数相同.  

(9) SACMODEL_PERPENDICULAR_PLANE  

在给定偏角范围内找到与指定轴垂直的平面. 对应的参数和 SACMODEL_PLANE 相同.  

(10) SACMODEL_NORMAL_PLANE  

使用表面法向量确定平面. 在给定的最大偏角范围内, 每个 inlier 点的表面法向量和输出平面的法向量相同. 该模型的参数和 SACMODEL_PLANE 相同.   

(11) SACMODEL_PARALLEL_PLANE  

在给定偏角范围内找到与指定轴平行的平面. 对应的参数和 SACMODEL_PLANE 相同.  

(12) SACMODEL_NORMAL_PARALLEL_PLANE  

平面和用户指定的轴平行, 且满足表面法向量约束. 等价于 SACMODEL_NORMAL_PLANE + SACMODEL_PARALLEL_PLANE. 对应的参数和 SACMODEL_PLANE 相同.   

(13) SACMODEL_TORUS, SACMODEL_PARALLEL_LINES  

预留接口, 未实现.  

## 4. PCL 中 sample_consensus (分割)模块及类的介绍  

PCL 中 Sample_consensus 库实现了随机采样一致性及其泛化估计算法, 例如平面, 柱面, 等各种常见的几何模型, 用不同的估计算法和不同的几何模型自由的结合估算点云中隐含的具体几何模型的系数, 实现对点云中所处的几何模型的分割.  

线, 平面, 柱面和球面都可以在 PCL 库中实现, 平面模型经常被用到常见的室内平面的分割提取中, 比如墙, 地板, 桌面, 其他模型常应用到根据几何结构检测识别和分割物体中, 一共可以分为两类: 一类是针对采样一致性及其泛化函数的实现, 一类是几个不同模型的具体实现, 例如: 平面, 直线, 圆球等.  

pcl::SampleConsensusModel< PointT > 是随机采样一致性估计算法中不同模型实现的基类, 所有的采样一致性估计模型都继承与此类, 定义了采样一致性模型的相关的一般接口, 具体实现由子类完成. 例如 `pcl::SampleConsensusModelCircle2D< PointT >` 实现采样一致性 计算二位平面圆周模型; `pcl::SampleConsensusModelCone< PointT, PointNT >` 实现采样一致性计算的三维椎体模型.  
  
pcl::RandomizedMEstimatorSampleConsensus< PointT > 实现了 RANSAC 算法, RANSAC 算法适用与处理数据点中局内点比例比较大的情况, 可快速地进行局外点的剔除.  

## 5. 代码实例 random_sample_consensus.cpp

```cpp
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
    viewer->addCoordinateSystem (1.0, "global");

    viewer->initCameraParameters();
    return (viewer);
}

/***************************************************************************** 
 对点云进行初始化, 并对其中一个点云填充点云数据作为处理前的的原始点云, 
 其中大部分点云数据是基于设定的圆球和平面模型计算, 而得到的坐标值作为局内点, 
 有1/5的点云数据是被随机放置的组为局外点. 
*****************************************************************************/
int main(int argc, char** argv)
{
    // 初始化点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //存储源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);   //存储提取的局内点

    // 填充点云数据
    cloud->width    = 500;                 //填充点云数目
    cloud->height   = 1;                     //无序点云
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    const int rand_cursor = RAND_MAX / 2;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        if (pcl::console::find_argument (argc, argv, "-s") >= 0 
            || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        {

            //根据命令行参数用 x^2 + y^2 + z^2 = 1 设置一部分点云数据, 此时点云组成 1/4 个球体作为内点
            cloud->points[i].x = (rand() - rand_cursor) / (rand_cursor + 1.0);  // 生成 ±[0~1] 之间的浮点数
            cloud->points[i].y = (rand() - rand_cursor) / (rand_cursor + 1.0);
            
            if (i % 5 == 0)
                cloud->points[i].z = (rand() - rand_cursor) / (rand_cursor + 1.0);   // 此处对应的点为局外点
            else if(i % 2 == 0) 
                cloud->points[i].z = sqrt( 1 - (cloud->points[i].x * cloud->points[i].x) - (cloud->points[i].y * cloud->points[i].y));
            else
                cloud->points[i].z = - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x) - (cloud->points[i].y * cloud->points[i].y));
            
            std::cout << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << std::endl;
        } else {

            // 用 x+y+z=1 设置一部分点云数据, 此时点云组成的菱形平面作为内点
            cloud->points[i].x = (rand() - rand_cursor) / (rand_cursor + 1.0);  // 生成 ±[0~1] 之间的浮点数
            cloud->points[i].y = (rand() - rand_cursor) / (rand_cursor + 1.0);

            if( i % 2 == 0)
                cloud->points[i].z = (rand() - rand_cursor) / (rand_cursor + 1.0);
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }

    std::vector<int> inliers;  //存储局内点集合的点的索引的向量

    //创建随机采样一致性对象
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));    //针对球模型的对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));   //针对平面模型的对象
    if(pcl::console::find_argument (argc, argv, "-f") >= 0)
    {  
        //根据命令行参数, 来随机估算对应平面模型, 并存储估计的局内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.01);    //与平面距离小于0.01 的点称为局内点考虑
        ransac.computeModel();                   //执行随机参数估计
        ransac.getInliers(inliers);                 //存储估计所得的局内点
    }
    else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
    { 
        //根据命令行参数  来随机估算对应的圆球模型, 存储估计的内点
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (.01);    // 0.01
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // 复制估算模型的所有的局内点到final中
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    // 创建可视化对象并加入原始点云或者所有的局内点
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //viewerOneOff(*viewer);
    if (pcl::console::find_argument (argc, argv, "-f") >= 0 
        || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        viewer = simpleVis(final);
    else
        viewer = simpleVis(cloud);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    return 0;
}
```

运行结果: 

在没有任何参数的情况下, 三维窗口显示创建的原始点云 (含有局内点和局外点) , 如图所示, 很明显这是一个带有噪声的菱形平面, 噪声点是立方体, 自己要是我们在产生点云是生成的是随机数生在 (0, 1) 范围内. 

```bash
# 生成平面的内外点数据
./random_sample_consensus

# 平面检测
./random_sample_consensus -f

# 生成球面的内外点数据
./random_sample_consensus -s

# 球面检测
./random_sample_consensus -sf
```
