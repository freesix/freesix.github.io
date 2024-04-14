---
title: steger算法笔记   
data: 2023-12-01 19:00:07
tags: 学习笔记
categories: 学习笔记
mathjax: true
---

# steger算法

Steger算法的基本思路就是首先通过Hessian矩阵获得光条的法线方向，然后在其法线方向对像素灰度应用泰勒多项式展开从而得到灰度的分布函数，进而计算出光条中心的亚像素位置。由于光条特征的灰度在其法线方向上近似为高斯分布，因而越靠近光条中心点的像素其灰度值越大。因此对该法线方向上的灰度分布函数的二阶泰勒多项式求取极值，即可得到光条在该法线方向上的中心点坐标。

在工程优化问题过程中，目标函数通常会以非线性多元函数的形式出现，这种函数形式非常复杂，不便计算，经常会将目标函数在某点的邻域内展开成泰勒多项式，以此来近似或逼近原目标函数。

**Hessian**矩阵：对于一个实值多元函数$f(x_1,x_2,\dots,x_n)$，如果函数$f$的阶偏导数存在，则定义函数$f$的Hessian矩阵$\mathbf{H}(f)$为：
$$
\boldsymbol{H}(f)=\begin{bmatrix}\frac{\partial^2f}{\partial x_1^2}&\frac{\partial^2f}{\partial x_1\partial x_2}&\cdots&\frac{\partial^2f}{\partial x_1\partial x_n}\\\frac{\partial^2f}{\partial x_2\partial x_1}&\frac{\partial^2f}{\partial x_2^2}&\cdots&\frac{\partial^2f}{\partial x_2\partial x_n}\\\vdots&\vdots&&\vdots\\\frac{\partial^2f}{\partial x_n\partial x_1}&\frac{\partial^2f}{\partial x_n\partial x_2}&\cdots&\frac{\partial^2f}{\partial x_n^2}\end{bmatrix}
$$
对于二维离散图像$I(u,v)$来说，其Hessian矩阵可以表示为：
$$
\mathbf{H}(u,v)=\left[\begin{array}{cc}I_{uu} & I_{uv} \\ I_{uv} & I_{vv}\end{array}\right]
$$
其中$I(u,v)$表示像素$(u,v)$的灰度，可将$I(u,v)$视作该图像的灰度分布函数。$I_{uu}、I_{uv}、I_{vv}$分别表示二维高斯函数$G(u,v)$的二阶偏导数与图像$I(u,v)$进行卷积运算的结果。（为什么？）

对一个多元函数泰勒展开的矩阵形式可以表示为：
$$
f(\mathbf{x})=f(\mathbf{x}_k)+[\nabla f(\mathbf{x}_k)]^T(\mathbf{x}-\mathbf{x}_k)+\frac{1}{2!}[\mathbf{x}-\mathbf{x}_k]^TH(\mathbf{x}_k)[\mathbf{x}-\mathbf{x}_k]+o^n
$$
其中$\mathbf{H}(x_k)$就是上面提到的Hessian矩阵。

那么二维图像中的泰勒展开和Hessian矩阵对于图像中光条上的任意像素$(u_0,v_0)$，设其法线方向的单位方向向量为
$$
\mathrm{\overrightarrow{e}=[e_u,e_v]}
$$
则在该像素处将光条法线方向上的灰度分布函数$I(u,v)$沿方向$\mathrm{\overrightarrow{e}}$展开成泰勒多项式的形式，并忽略二阶以上的展开项。因此光条法线方向上的像素$(\mathrm{u_0~+~t\cdotp e_u~,v_0~+~t\cdotp e_v~})$的灰度$I((\mathrm{u_0~+~t\cdotp e_u~,v_0~+~t\cdotp e_v~}))$可以由该点的灰度和二阶泰勒多项式表示为：
$$
I\left(u_0+t\cdot e_u,v_0+t\cdot e_v\right)=I(u_0,v_0)+t\cdot e\cdot[I_u,I_v]^T+\frac{t^2}{2!}\cdot e\cdot H(u,\nu)\cdot e^T
$$
上面这个式子就是光条在像素$(u_0,v_0)$处横截面上的灰度分布函数，而使其一阶导数为零的点就是待提取的光条中心点。因此，求上面式子求一阶导
$$
\mathrm{\frac{\partial I}{\partial t}~=e\cdot[I_u,I_v]+t\cdot e\cdot H(u,v)\cdot e^T=0}
$$
再将这个式子以向量形式表示：
$$
[\mathrm e_\mathrm{u},\mathrm e_\mathrm{v}]\cdot\begin{bmatrix}\mathrm I_\mathrm{u}\\\mathrm I_\mathrm{v}\end{bmatrix}+\mathrm t\cdot[\mathrm e_\mathrm{u},\mathrm e_\mathrm{v} ]\cdot\begin{bmatrix}\mathrm I_\mathrm{uu}&\mathrm I_\mathrm{uv}\\\mathrm I_\mathrm{uv}&\mathrm I_\mathrm{vv}\end{bmatrix}\cdot\begin{bmatrix}\mathrm e_\mathrm{u}\\\mathrm e_\mathrm{v}\end{bmatrix}=0
$$
简化可得：
$$
t=-\frac{e_{u}\cdot I_{u}+e_{\nu}\cdot I_{\nu}}{e_{u}^{2}\cdot I_{uu}+2\cdot e_{u}\cdot e_{\nu}\cdot I_{u\nu}+e_{\nu}^{2}\cdot I_{\nu\nu}}
$$
如果$(\mathrm{t}\cdot\mathrm{e}_\mathrm{u},\mathrm{t}\cdot\mathrm{e}_\mathrm{v})\in[-0.5,0.5]\times[-0.5,0.5]$，即一阶导数为零的点位于当前像素内，且$(\mathrm{e}_\mathrm{u},\mathrm{e}_\mathrm{v})$方向的二阶导数大于指定的阈值，则该点$(\mathrm{u_0,v_0})$为光条的中心点，$\mathrm{(e_x,e_y)=(u_0+t\cdot e_u,v_0+t\cdot e_v)}$则为所求的亚像素坐标。

**根据有关文献可知，在求Hessian矩阵之前对图像进行高斯滤波时，设置高斯方差$\sigma<\frac{\omega}{\sqrt{3}}$，其中$\omega$为光条宽度。**

**steger算法的一些缺点**：多次卷积带来计算复杂度高，速度慢，如果整幅图像仅使用相同的高斯卷积核大小，则在光条宽度分布不均匀且曲率变化大时难以准确提取中心线，甚至导致光条不连续。

