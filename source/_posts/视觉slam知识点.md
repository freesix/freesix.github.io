---
title: slam知识点
data: 2023-12-24 20:09:07
tags: 知识点随笔
categories: 知识点随笔
mathjax: true
---

slam中涉及到的一些知识点，做个总结，尽量结合实际项目或代码，持续更新增加。。。

## 光流法跟踪特征点
以vins中的光流前端为例子。在slam前端用光流法的优点是计算效率高，适用于实时性要求较高的场景(
相较于特征点法而言)，当然现在有些特征点法速率也不错，但其精度和速率总是难以两全，虽然SIFT计算
复杂，但其精度、稳定性等仍然在层出不穷的传统特征点提取算法中保持优越。

光流法有较强的假设性：

- 亮度恒定，投影在图像中的同一点随着时间不会变化。
- 小运动，就是随着时间的变化不会引起位置的剧烈变化，这样灰度才能对位置求偏导。
- 空间一致，一个场景上邻近点投影到图像上也是邻近点，且邻近点速度一致，这是对于Lucas-Kanade
光流法特有的假设，因为光流法基本方程约束只有一个，而要求$x$，$y$方向上的速度有两个未知量，假
设了邻域运动一致，就可以联立多个方程。

虽然现在有些光流法通过金字塔或者亮度质心等来去除或者松弛这些约束，但这样一来和特征点法有何差异
呢，也是个计算效率和准确率的取舍问题。

**VINS中采用的是LK金字塔光流法**

### LK光流法
相机的不同帧图像是随时间变化的，那么在$t$时刻，位于$(x,y)$处的像素，它的灰度为$\mathbf{I}(x,y,t)$。
根据上面假设条件一有：
$$
    \mathbf{I}(x+dx,y+dy,t+dt) = \mathbf{I}(x,y,t)
$$
对左边进行泰勒展开，保留一阶项：
$$
    \mathbf{I}(x+dx,y+dy,t+dt)\approx\boldsymbol{I}\left(x,y,t\right)+
    \frac{\partial\boldsymbol{I}}{\partial x}\mathrm{d}x+
    \frac{\partial\boldsymbol{I}}{\partial y}\mathrm{d}y+
    \frac{\partial\boldsymbol{I}}{\partial t}\mathrm{d}t
$$
因为**假设了灰度不变**，于是下一时刻的灰度等于上一时刻的灰度，从而有：
$$
    \frac{\partial\boldsymbol{I}}{\partial x}\mathrm{d}x+
    \frac{\partial\boldsymbol{I}}{\partial y}\mathrm{d}y+
    \frac{\partial\boldsymbol{I}}{\partial t}\mathrm{d}t = 0
$$
$$
     \frac{\partial\boldsymbol{I}}{\partial x}\frac{\mathrm{d}x}{\mathrm{d}t}+
    \frac{\partial\boldsymbol{I}}{\partial y}\frac{\mathrm{d}y}{\mathrm{d}t}+
    = -\frac{\partial\boldsymbol{I}}{\partial t}
$$
其中$\frac{\partial\boldsymbol{I}}{\partial x}$为像素在x方向的梯度，而
$\frac{\partial\boldsymbol{I}}{\partial y}$为在y方向的梯度，记为$\mathbf{I}_{x}$，
$\mathbf{I}_y$。同时$\frac{\mathrm{d}x}{\mathrm{d}t}$
是像素在x轴上的运动速度，而$\frac{\mathrm{d}y}{\mathrm{d}t}$是在y轴上的速度，记为$u$，$v$。
写成矩阵形式有：
$$
\left.\left[\begin{array}{cc}I_x&I_y\end{array}\right.\right]\left[\begin{array}{c}u\\\\v\end{array}\right]=-\boldsymbol{I}_t
$$

这个方程中有两个未知数，因此至少需要两个方程才能解，根据前面的假设，认为在**邻域内像素和该像素
具有相同的运动状态**，可以构建$n^{2}$个方程：
$$
\left.\left[\begin{array}{cc}\boldsymbol{I}_x&\boldsymbol{I}_y\end{array}\right.\right]_k\left[\begin{array}{c}u\\\\v\end{array}\right]=-\boldsymbol{I}_{t k},\quad k=1,\ldots,n^2
$$

## 特征点法跟踪(以ORB为例)

### 什么是ORB特征点

特征点一般由关键点和描述子两部分组成。ORB特征点是在FAST特征点的基础上加入了方向信息，对应的在
描述子中加入了方向的描述。(Oriented FAST关键点和Streered BRIEF描述子)两部分。

ORB特征选取策略：
- 在图像中选取某个像素$p$，其灰度值为$I_{p}$。
- 设定一个阈值$T$，以$p$为圆心，半径为3个像素的16个像素点上比大小。
- 如果16个像素点上有连续$N$个灰度大于或小于$I_{p}+T$，则确定为关键点，在ORB-slam中$N=3$
- 实际中为了加速，通常选取第1、5、9、13个像素点的灰度值来比较，有大于等于三个像素点满足条件
则认为中心像素点为一个关键点。

为了保证特征点的尺度不变性和计算方向，引入了图像金字塔和灰度质心法。简要记一下灰度质心法，计算
关键点为圆心指定半径中园的灰度质心，圆心到质心方向即为关键点方向。

图像的矩定义为：
$$
m_{pq}=\sum_{x,y}x^py^qI(x,y),\quad p,q=\{0,1\}
$$
就是一定方向或者一定范围内灰度值和坐标的乘积。

那么分别在坐标轴$x$，$y$方向上的图像矩分别为：
$$
m_{10}=\sum_{x=-R}^R\sum_{y=-R}^RxI(x,y)\\
m_{01}=\sum_{x=-R}^R\sum_{y=-R}^RyI(x,y)
$$

对应圆形区域内所有像素的灰度值总和为：
$$
m_{00}=\sum_{x=-R}^R\sum_{y=-R}^RI(x,y)
$$

那么图像的质心为：
$$
C=(c_x,c_y)=\left(\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}}\right)
$$

同时，可以计算出关键点的旋转角度为：
$$
\theta=\arctan2\left(c_y,c_x\right)=\arctan2\left(m_{01},m_{10}\right)
$$

然后就可以将图像按照$\theta$旋转：
![旋转灰度质心园和主方向坐标轴对齐](./slam/image.png)

### 描述子 Steered BRIEF
BRIEF是一种二进制编码的描述子，在ORB-SLAM2中它是一个256bit的向量。以下是计算方法：

- 为减少噪声干扰，先对图像进行高斯滤波。
- 以关键点为中心，取一定大小的图像窗口$p$，在窗口内随机选取一对点，比较二者像素的大小，进行
如下二进制赋值。
$$
\tau(p;x,y):=\begin{cases}1&:p(x)<p(y)\\0&:p(x)\geqslant p(y)\end{cases}
$$
- 在窗口中随机选取$N$，(在ORB-slam2中是256)对随机点，重复上面的步骤，得到一个256维的二进制
描述子。

对于选点，在ORB-slam2中采用固定的选点模板，是一个256$\times$4个值组成的数组，每行的4个值表
示一对点的坐标。

而增加了旋转方向的Steered BRIEF描述子则是先将此区域旋转(或者通过下面的矩阵找到旋转前的点在
图像中的位置)。
$$
\begin{bmatrix}x'\\y'\end{bmatrix}=\begin{bmatrix}\cos\theta&-\sin\theta\\[0.3em]\sin\theta&\cos\theta\end{bmatrix}\begin{bmatrix}x\\[0.3em]y\end{bmatrix}
$$

### 特征点均匀化
- 根据总的图像金字塔层级和待提取的特征点总数，计算图像金字塔中每个层级需要提取的特征点数量。
- 划分格子(在ORB-slam2中固定尺寸为30像素$\times$30)
- 对每个格子提取角点，如果初始的FAST角点阈值没有检测到角点，则降低阈值，再提取一次，若还是没有
角点，则不在这个格子提取。
- 用四叉树均匀地选取角点。

### 特征点匹配
在特征点匹配中(单指帧与帧之间的匹配)，这里不同于光流法的直接跟踪，特征点的匹配需要去对应帧全图
寻找匹配点，很耗费计算资源，因此需要采取一定的策略加速匹配。在ORB-slam2中是在匹配点对应的匹配
帧的对应位置一定范围内进行匹配，并且为了加速，还将匹配区域划分为一个一个的网格，遍历网格寻找匹
配点。

为了减少误匹配，ORB-slam2中还采用了方向一致性检验，简而言之就是将源匹配点和目标匹配点的主方向
做差，然后统计这个差的直方图分布，选取排在前三的直方图格子，其余在外的就认为是误匹配点对。

### 词袋模型匹配
这个一般直接调用库，后面再细究。

