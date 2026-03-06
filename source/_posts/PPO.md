---
title: PPO阅读笔记
date: 2025-11-25 14:46:00
tags: 论文阅读笔记
categories: 论文阅读笔记
mathjax: true
---
Proximal Policy Optimization Algorithms论文阅读笔记

这是一种新的策略梯度算法，标准策略梯度算法是对每个样本数据进行一次梯度更新，而本文的近端策略梯度法可对小批量样本重复梯度更新。

主要贡献是提出了截断的代理目标函数和自适应KL散度惩罚来控制策略更新的幅度，提高了训练的稳定性和数据利用率。

## 策略梯度方法回顾
* 策略梯度方法

策略梯度方法直接是对策略进行参数化，通过最大化预期回报来优化策略。常用的策略梯度估计器为：
$$
g=\mathbb{\hat{E}}_t[\nabla_\theta\log\pi_\theta(a_t|s_t)\mathbf{\hat{A}}]
$$

* 信赖域梯度优化（TRPO）

为了解决策略更新不稳定的问题，TRPO被提出，思想是在策略更新时加入约束，限制新旧策略之间的差异。
$$
\begin{aligned}
\max_{\theta}\quad&\mathbb{\hat{E}}_t\left[\frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}\mathbf{\hat{A}}_t\right] \\
s.t\quad&\mathbb{\hat{E}}\left[KL[\pi_{\theta_{old}}(\cdot|s_t),\pi_\theta(\cdot|s_t)]\right]\leq\delta
\end{aligned}
$$

* 近端策略优化（PPO）

PPO引入了用于截断的代理目标函数，首先定义概率比率：
$$
r_t(\theta)=\frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}
$$
有此，在TRPO中，优化的目标函数就变为了：
$$
L^{CPI}(\theta)=\mathbb{\hat{E}}\left[r_t(\theta)\mathbf{\hat{A}}_t\right]
$$
为了避免策略的过度更新，PPO引入了截断函数，定义新的目标函数为：
$$
L^{CLIP}(\theta)=\mathbb{\hat{E}}\left[\min\left(r_t(\theta)\mathbf{\hat{A}}_t,clip(r_t(\theta),1-\epsilon,1+\epsilon)\right)\mathbf{\hat{A}}_t\right]
$$
另外一种控制策略更新的方式就是加入KL散度惩罚项，并自适应调整惩罚系数：
$$
L^{KLPEN}(\theta)=\mathbb{\hat{E}}\left[r_t(\theta)\mathbf{\hat{A}}_t-\beta KL[\pi_{\theta_{old}}(\cdot|s_t),\pi_\theta(\cdot|s_t)]\right]
$$
自适应调整$\beta$，当实际KL散度$d$小于目标值$d_{targ}$的1.5倍时，减小$\beta$，允许更大的策略更新。当$d$大于$d_{targ}$的1.5倍时，增大$\beta$，限制策略更新的幅度。