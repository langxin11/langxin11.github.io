---
title: CMU Optimal control and reinforcement learning 16-745 2025学习简要记录
published: 2025-07-28
---



# CMU Optimal control and reinforcement learning 16-745 2025学习简要记录

主要是配合Homework来的（在课程官网https://optimalcontrol.ri.cmu.edu/homeworks/有完整的代码）

更完整的课程记录可以参考

## lecture 1: Introduction and Dynamic review

$\dot{x}=f(x,u)$例子-单摆 $m g l sin \theta+m \ddot {\theta} l^2=u$

1. 状态空间方程

   $\dot X=A X+B U$

   取状态变量$X=(\theta,\dot \theta)^T$
   $$
   \begin{bmatrix}
   \dot{x}_1 \\
   \dot{x}_2
   \end{bmatrix}
   =
   \begin{bmatrix}
   \dot{\theta} \\
   \ddot{\theta}
   \end{bmatrix}
   =
   \begin{bmatrix}
   x_2 \\
   -\frac{g}{l} \sin\theta + \frac{u}{m l^2}
   \end{bmatrix}
   $$

2. 仿射系统方程

3. 机械臂系统方程

   $M(q) v + C(q,u)=B u$

   $M$质量矩阵，$C$科氏力

4. 线性系统

   $\dot X=f(X,U)->\dot{X} =\frac{df} {dX}X+\frac{df} {dU}U$

5. 平衡点

   $\dot{X}=0$代入动力学方程所处的平衡状态，改变控制输入可以改变平衡点

6. 平衡点稳定性

   $R e [\text{eig}\frac{df}{dx}]<0$

##  lecture 2: Dynamics Discretization

1. 显式欧拉法
   $x_(k+1)=x_k+T f(x_k,u_k)$

2. RK4

   $x_{k+1} = x_k + 1/6 T (k_1 +k_2 +k_3 +k_3)$

3. 隐式中点法
   $x_(k+1)=x_k+T f(x_(k+1),u_k)$

## Lecture3 optimization part 1

### 基础概念（梯度，雅可比矩阵，Hessian矩阵）

1. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的梯度向量(Gradient)

$$
\nabla f(\mathbf{x} )=\left [  \frac{\partial f}{\partial x_1}, \frac{\partial f}{\partial x_2},..,\frac{\partial f}{\partial x_n}\right ]^\top
$$

    示例$f(x,y)=x^2+y^3$的梯度为$\nabla{f}=\left[2x,3y^2\right]^\top$

2. 向量值函数$\mathbf{F}(\mathbf{x}):\mathbb{R} ^n→\mathbb{R}^m$的雅可比矩阵(Jacobain)

$$
\mathbf{J}_{\mathbf{F}} = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix}
$$

    $\mathbf{F}(x, y) = \begin{bmatrix} x^2 y \\ \sin y \end{bmatrix}$的雅可比矩阵为$\mathbf{J} = \begin{bmatrix} 2xy & x^2 \\ 0 & \cos y \end{bmatrix}$

3. 标量函数$ f:\mathbb{R} ^n→\mathbb{R}$的Hessian矩阵：：$n\times n$对称矩阵

   $$
   \mathbf{H}_f = \begin{bmatrix}
   \frac{\partial^2 f}{\partial x_1^2} & \cdots & \frac{\partial^2 f}{\partial x_1 \partial x_n} \\
   \vdots & \ddots & \vdots \\
   \frac{\partial^2 f}{\partial x_n \partial x_1} & \cdots & \frac{\partial^2 f}{\partial x_n^2}
   \end{bmatrix}
   $$

   - 标量函数梯度的雅可比矩阵即是Hessian矩阵
   - $m=1$时，雅可比矩阵退化为梯度的转置
   - 行主导和列主导向量函数的复合求导会导致链式法则的式子不一样，由于矩阵的维度不一致)

$$
\frac{dF(g(\mathbf{x}))}{d\mathbf{x}}=\frac{dF}{d\mathbf{x}}|_{g(\mathbf{x})}\frac{dG(\mathbf{x})}{d\mathbf{x}}|_\mathbf{x}
$$

### 求解一个非线性方程$f(x)=0$的方法（求根）

- 不动点迭代法

  离散系统动力学求平衡点

  $$
  x_{k+1}=f(x_k,u_k)\\
  f^*=x-f
  $$
- Newton方法

  $$
  f(x+dx)=f(x)+\frac{df}{dx}\Delta x=0\\
  \Delta x= -\frac{df}{dx}^{-1}f\\
  x \leftarrow x+\Delta x\\
  \text{Loop until convergence}
  $$

在无约束优化问题中，可以用Newton法求解$\Delta g=0$（一阶必要条件）

## HW1_S25_Q3 QP求解器（Log-Domain Interior Point Method）

$$
\begin{align}
\min_x \quad & \frac{1}{2}x^TQx + q^Tx \\ 
\text{s.t.}\quad &  Ax -b = 0 \\ 
&  Gx - h \geq 0 
\end{align}
$$

引入拉格朗日乘子

- ``μ ∈ ℝᵖ`  对应等式约束
- `λ ∈ ℝᵐ`对应不等式约束（要求  $\lambda≥ 0$）

拉格朗日函数为：

$$
\mathcal{L}(x, \mu,\lambda) = \frac{1}{2}x^\top Q x + q^\top x 
+ \mu^\top (Ax - b)
- \lambda^\top (Gx - h)
$$

KKT条件：梯度条件、原始可行性、对偶可行性、互补松弛条件

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \lambda&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h &\geq 0 \quad \quad \text{(primal feasibility)} \\
\lambda &\geq 0 \quad \quad \text{(dual feasibility)} \\
\lambda \circ(Gx - h) &= 0 \quad \quad \text{(complementarity)} 
\end{align}
$$

映入非负松弛变量$s>0$使得$Gx-h=s$

新的拉格朗日函数

$$
\mathcal{L}(x, \lambda, \mu) = \frac{1}{2}x^\top Q x + q^\top x 
+ \mu^\top (Ax - b)
+ \lambda^\top [s-(Gx - h)]
$$

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \lambda&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h-s &=0 \quad \quad \text{(primal feasibility)} \\
\lambda &\geq 0 \quad \quad \text{(dual feasibility)} \\
s &\geq 0\\
\lambda^\top \circ s &= \mathbf{1}^\top \rho \quad \quad \text{(complementarity)} 
\end{align}
$$

令$\lambda=\sqrt{\rho}e^{-\sigma},s=\sqrt{\rho}e^{\sigma}$，得到转换后的ip_KKT条件方程

$$
\begin{align}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&= 0 \quad \quad \text{(stationarity)} \\
Ax - b&= 0 \quad \quad \text{(primal feasibility)} \\
 Gx - h-\sqrt{\rho}e^{\sigma} &=0 \quad \quad \text{(primal feasibility)} \\
\end{align}
$$

新的拉格朗日函数$\mathcal{L}(x, \lambda, \sigma)$对应的Jacobian和Hessian矩阵如下

$$
\mathbf{J}=\begin{bmatrix}
Qx+q+A^\top\mu - G^\top \sqrt{\rho}e^{-\sigma}&\\
Ax - b\\
Gx - h-\sqrt{\rho}e^{\sigma}\\
\end{bmatrix}
$$

$$
\mathbf{H}=\begin{bmatrix}
Q& A^\top& G^\top \text{diag}(\sqrt{\rho}\odot e^{-\sigma})\\
A&\mathbf{0}&\mathbf{0}\\
G&\mathbf{0} &\text{diag}( \sqrt{\rho}\odot e^{-\sigma})  \\
\end{bmatrix}
$$

这样我们求解QP问题就转换为求解用Newton法求解关于$z=[x; \mu;\sigma]$的KKT方程（求根）

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/20250729103953454.png)



这里的$P_0$和$P_1$代表原始KKT和IP_KKT的残差，也就是

$$
\begin{align}
P_0=\begin{bmatrix}
  Qx+q+A^\top\mu - G^\top \lambda&\\
  Ax - b&\\
  min.(Gx - h,\mathbf{0})\\
  min.(\lambda,\mathbf{0})\\ 
  \lambda \circ(Gx - h) & 
\end{bmatrix}
\end{align}
$$

$$
\begin{align}
P_1=\begin{bmatrix}
  Qx+q+A^\top\mu - G^\top \lambda&\\
  Ax - b&\\
 Gx - h-s\\
\end{bmatrix}
\end{align}
$$

### 砖块掉落仿真

不考虑砖块的旋转，一个掉落的砖块的动力学方程可以写成

$$
M \dot{v}  + M g = J^T \mu \\ \text{ where } M = mI_{2\times 2}, \; g = \begin{bmatrix} 0 \\ 9.81 \end{bmatrix},\; J = \begin{bmatrix} 0 & 1 \end{bmatrix}
$$

其中，$v=[v_x;v_z]$是砖块的水平速度和竖直速度，$q=[q_x;q_z]$是砖块的水平和竖直位移，$\lambda$是地面给砖块的法向接触力，通过backward Euler离散

$$
\begin{bmatrix} v_{k+1} \\ q_{k+1} \end{bmatrix} = \begin{bmatrix} v_k \\ q_k \end{bmatrix}+ \Delta t \cdot \begin{bmatrix} \frac{1}{m} J^T \mu_{k+1} - g \\ v_{k+1} \end{bmatrix}
$$

约束

$$
\begin{align}
J q_{k+1} &\geq 0 &&\text{(不会穿透地面)} \\
\mu_{k+1} &\geq 0 &&\text{(接触力方向只向上)} \\
\mu_{k+1} J q_{k+1} &= 0 &&\text{(没接触无接触力)}
\end{align}
$$

等价转换成如下的QP问题(可以通过KKT条件证明)

$$
\begin{align}
    &\text{minimize}_{v_{k+1}} && \frac{1}{2} v_{k+1}^T M v_{k+1} + [M (\Delta t \cdot g - v_k)]^Tv_{k+1} \\
    &\text{subject to} && J(q_k + \Delta t \cdot v_{k+1}) \geq 0 \\
\end{align}
$$

引入拉格朗日乘子$\mu \geq 0$

$$
\mathcal{L}=\frac{1}{2} v_{k+1}^T M v_{k+1} + [M (\Delta t \cdot g - v_k)]^Tv_{k+1} -\mu \cdot J(q_k + \Delta t \cdot v_{k+1})
$$

KKT 条件

$$
\begin{align}
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)^\mathrm{T}-J^\mathrm{T}\mu\Delta t&=0\qquad\text{梯度条件}\\
J(q_k + \Delta t \cdot v_{k+1}) &\geq 0\qquad\text{原始可行性}\\
\mu &\geq 0\qquad\text{对偶可行性}\\
\mu \cdot J(q_k + \Delta t \cdot v_{k+1})&=0\qquad\text{互补松弛 }
\end{align}
$$

引入松弛能量$s=\sqrt{\rho}e^{-\sigma}$，转换

$$
\begin{align}
Mv_{k+1} + M(\Delta{t}\cdot g-v_k)^\mathrm{T}-J^\mathrm{T}\mu\Delta t&=0\\
J(q_k + \Delta t \cdot v_{k+1}) -\sqrt{\rho}e^{-\sigma}&= 0\\

\end{align}
$$

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/20250729104519288.gif)

## Lecture 9 Linear Quadratic Regulator

 线性二次型最优控制问题

$$
\begin{align}
\min_{x_{1:N},{u}_{1:N-1}}& \quad \sum_{k=1}^{N-1}(\frac{1}{2}{x_k}^TQx_k + \frac{1}{2}{u_k}^TRu_k)+\frac{1}{2}{x_N}^TQ_Nx_N \\
\text{s.t.}&\quad x_{k+1}=A_kx_k+B_ku_k
\end{align}
$$

其中，$Q\succeq 0,R\succ 0$。

- 根据$A,B,Q,R$是否随时间变化可以分为时不变和时变LQR，time-invariant LQRs 经常用于在稳定平衡点，TVLQR 用于轨迹跟踪
- 可以通过线性化求解非线性优化问题，在实际问题中得到广泛运用

  ### 将LQR转换为一个标准的QP问题求解

  优化变量$z$


$$
  z=\begin{bmatrix}
  u_1 \\
  x_2 \\
  u_2 \\
  . \\
  . \\
  x_N
  \end{bmatrix}
$$

  $J=\frac{1}{2}z^THz$

$$
  H=
  \begin{bmatrix}
  R_1 & 0 & ... & 0 \\
  0 & Q_2 & ... & 0 \\
   & & . \\
  0 & 0 & ... & Q_N
  \end{bmatrix}
$$

  $Cz=d$

$$
  \begin{aligned}
   & C=
  \begin{bmatrix}
  B_1 & (-I) & ... & ... & ... & 0 \\
  0 & A & B & (-I) & ... & 0 \\
   & & . \\
  0 & 0 & ... & A_{N-1} & B_{N-1} & (-I)
  \end{bmatrix} \\
   & d=
  \begin{bmatrix}
  -A_1x_1 \\
  0 \\
  . \\
  0
  \end{bmatrix}
  \end{aligned}
$$

  这样就形式上转换成了一个标准的QP问题

$$
  \begin{aligned}
   & \min_z\frac{1}{2}z^THz \\
   & s.t.\quad Cz=d
  \end{aligned}
$$

  通过引入拉格然日乘子给出拉格朗日函数，得到KKT方程，就可以用Newton方法迭代求解

$$
  \begin{bmatrix}
  H & C^T \\
  C & 0
  \end{bmatrix}
  \begin{bmatrix}
  z \\
  \lambda
  \end{bmatrix}=
  \begin{bmatrix}
  0 \\
  d
  \end{bmatrix}
$$

  ### Riccati 方程求解（利用KKT中的稀疏性）

$$
  \begin{bmatrix}
  R & & & & & & . & B^T &  \\
   & Q & & & & & . & -I & A^T  \\
   & & R & & & & . & & B^T  \\
   & & & Q & & & . & & -I & A^T \\
   & & & & R & & . & & & B^T \\
   & & & & & Q_N & . & & & -I \\
  . & . & . & . & . & . & . & . & . & . \\
  B & -I & & & & & . & 0 & 0 & 0 \\
   & A & B & -I & & & . & 0 & 0 & 0 \\
   & & & A & B & -I & . & 0 & 0 & 0
  \end{bmatrix}
  \begin{bmatrix}
  u_1 \\
  x_2 \\
  u_2 \\
  x_3 \\
  u_3 \\
  x_4 \\
  \lambda_2 \\
  \lambda_3 \\
  \lambda_4
  \end{bmatrix}=
  \begin{bmatrix}
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  0 \\
  -Ax_1 \\
  0 \\
  0
  \end{bmatrix}
$$

  从末态$x_4$开始

$$
  Q_Nx_4-\lambda_4 \Longrightarrow\lambda_4  =Q_Nx_4
$$

  考虑$u_3$(分别代入$\lambda_4  =Q_Nx_4$和$x_4=Ax_3+Bu_3$)

$$
  \begin{aligned}
  &Ru_3+B^T\lambda_4=Ru_3+B^TQ_Nx_4=Ru_3+B^TQ_N(Ax_3+Bu_3)=0\\
  &\Longrightarrow u_3  =-(R+B^TQ_NB)^{-1}B^TQ_NAx_3
  \end{aligned}
$$

  记成$u_3=-K_3x_3$

  到$x_3$

$$
  \begin{align}
  &Qx_3-\lambda_3+A^T\lambda_4=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_Nx_4=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_N(Ax_3+Bu_3)=0\\
  \Longrightarrow& Qx_3-\lambda_3+A^TQ_N(A-BK_3)x_3=0\\
  \Longrightarrow& \lambda_3= [Q+A^TQ_N(A-BK_3)]x_3\\
  \end{align}
$$

  记为$\lambda_3=P_3x_3$

  这样依次递推出$K_n$和$P_n$，便可求出控制序列$u_{1:N-1}$

$$
  \begin{align}
  P_N& = Q_N\\
  K_n& = (R+B^TP_{n+1}B)^{-1}B^TP_{n+1}A\\
  P_n& = Q+A^TP_{n+1}(A-BK_n)
  \end{align}
$$

### 例子-HW2 Q1

#### Part A 离散化动力学模型

考虑一个二阶积分系统，状态和控制变量如下

$$
\begin{align} x &= [p_1, p_2, v_1, v_2] \\ u &= [a_1, a_2] \end{align}
$$

状态空间方程$\dot{x}=Ax+Bu$

$$
\begin{align} \dot{x} =  \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix} x + \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1 \end{bmatrix} u\end{align} 
$$

离散状态空间方程$x_{k+1}=A_dx_k+B_du_k$

其中$A_d=e^{Adt},B_d=Idt+A\frac{dt^2}{2}(\text{注意这里的}A^2=0)$

#### Part B: Finite Horizon LQR via Convex Optimization

定义性能指标和约束方程，使用 `Convex.jl`求解得到 `Xcvx,Ucvx = convex_trajopt(A,B,Q,R,Qf,N,x_ic)`

$$
\begin{align} \min_{x_{1:N},u_{1:N-1}} \quad & \sum_{i=1}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st} \quad & x_1 = x_{\text{IC}} \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = 1,2,\ldots,N-1 
 \end{align}
$$

初态$x_{ic} = [5,7,2,-1.4]$

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_2.svg)

**验证LQR问题的Bellman's Principle of Optimality**，即在1：N时间的中间任何时刻$L(1<L<N)$到达平衡点都是最优的，也即是原始LQR问题和新的LQR问题得到的轨迹是重合的。
$$
\begin{align} \min_{x_{L:N},u_{L:N-1}} \quad & \sum_{i=L}^{N-1} \bigg[ \frac{1}{2} x_i^TQx_i + \frac{1}{2} u_i^TRu_i \bigg] + \frac{1}{2}x_N^TQ_fx_N\\ 
 \text{st     } \quad & x_L = x^*_L \\ 
 & x_{i+1} = A x_i + Bu_i \quad \text{for } i = L,L + 1,\ldots,N-1 
 \end{align}
$$

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_3.svg)

#### Part C：Finite-Horizon LQR via Riccati

使用Riccati 方程递推求解离散LQR问题的解析解并与convex.jl的求解结果进行对比，结果是一致的

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_4.svg)

多次随机初始状态结果也是一致的

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_5.svg)

#### Part D: Why LQR is so great LQR的优异性

考虑求出控制序列后，给实际的动力学系统加入噪声

$$
x_{k+1} = Ax_k + Bu_k + \text{noise}
$$

```julia
noise = [.005*randn(2);.1*randn(2)]
```

可以看出与求解关于优化变量$z=[x1,x2,..,x_N,u_1,...,u_{N-1}]^\mathrm{T}$的QP问题，由Ricatti递推求解得到的控制律$u=-Kx$鲁棒性更强。

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_6.svg)

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_7.svg)

设定一个不在平衡点的目标xgoal = [-3.5,-3.5,0,0]，LQR求的控制律$u=-K(x-x_{goal})$依旧能实现

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_8.svg)

#### Part E: Infinite -horizon LQR 无限时间二次型调节问题

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_9.svg)

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q1_10.svg)

矩阵$P K$各元素的变化如图所示，可以看出对于一个有限时间的LQR系统，矩阵$P(n_x\times n_x)$和$K(n_u\times n_x)$在Ricatti反向迭代的过程中，经过一段时间就很快收敛。

那么对于一个无限时间LQR系统，求出的$K$会是一个常数矩阵。

### 例子-HW Q2 LQR for nonlinear systems

#### Part 0 预备知识

**非线性系统线性化**

给定参考状态轨迹$\bar{x}_{1:N}$和参考控制轨迹$\bar{u}_{1:N-1}$，定义增量坐标

$$
x_k=\bar{x}_k+\Delta x_k,u_k=\bar{u}_k+\Delta u_k.
$$

对离散非线性动力学系统$x_{k+1}=f(x_k,u_k)$进行线性化（一阶泰勒展开）

$$
x_{k+1}\approx f(\bar{x}_k,\bar{u}_k)+\underbrace{\frac{\partial f}{\partial x}|_{\bar{x}_k,\bar{u}_k}}_{A_k}\Delta x_k+\underbrace{\frac{\partial f}{\partial u}|_{\bar{x}_k,\bar{u}_k}}_{B_k}\Delta u_k
$$

其中，$A_k$是状态雅可比矩阵$(n_x\times n_x)$，$B_k$是控制雅可比矩阵$(n_x\times n_u)$

如果参考轨迹是动态可行的（即满足$\bar{x}_{k+1}=f(\bar{x}_k,\bar{u}_k)$），则泰勒展开式可简化为

$$
\bar{x}_{k+1}+\Delta x_{k+1}\approx f(\bar{x}_k,\bar{u}_k)+A_k \Delta x_k+B_k \Delta u_k
$$

得到增量动力学方程

$$
\Delta x_{k+1}\approx A_k \Delta x_k+B_k \Delta u_k
$$

**线性时不变系统离散化方法**

$$
\dot{x}(t)=Ax(t)+Bu(t)
$$

连续系统的解为

$$
x(t)=e^{A(t-t_0)}x(t_0)+\int_{t_0}^{t}e^{A(t-\tau)}Bu(\tau)d\tau
$$

使用零阶保持器(ZOH)控制$u(t)=u_k$在$t\in[t_k,t_{k+1}]$，则

$$
x_{k+1}=e^{A\Delta t}x_k+(\int_{0}^{\Delta t}e^{A\tau}d\tau)Bu_k
$$

构造增广系统

$$
\frac{d}{dt}
\begin{bmatrix}
x(t) \\
u(t)
\end{bmatrix}=
\begin{bmatrix}
A & B \\
0 & 0
\end{bmatrix}
\begin{bmatrix}
x(t) \\
u(t)
\end{bmatrix}
$$

在ZOH假设下$\dot{u}(t)=0$，计算该系统的状态转移矩阵

$$
\exp\left(
\begin{bmatrix}
A & B \\
0 & 0
\end{bmatrix}\Delta t\right)=
\begin{bmatrix}
e^{A\Delta t} & \int_{0}^{\Delta t}e^{A\tau}d\tau B \\
0 & I
\end{bmatrix}
$$

通过计算增广矩阵的指数

$$
\exp\left(\begin{bmatrix}A&B\\0&0\end{bmatrix}\Delta t\right)=\begin{bmatrix}A_k&B_k\\0&I\end{bmatrix}
$$

- $A_k$为左上$n\times n$块
- $B_k$为右上$n\times m$块

#### Part A: Infinite Horizon LQR about an equilibrium

小车倒立摆（CartPole）的动力学方程

$$
H(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=Bu
$$

- 广义坐标$q=[p,\theta]$
- 质量惯性矩阵$H=
  \begin{bmatrix}
  m_c+m_p & m_pl\cos\theta \\
  m_pl\cos\theta & m_pl^2
  \end{bmatrix}$
- 科里奥利矩阵$C=
  \begin{bmatrix}
  0 & -m_pl\dot{\theta}\sin\theta \\
  0 & 0
  \end{bmatrix}$
- 重力向量$G=\begin{bmatrix}0\\m_pglsin\theta\end{bmatrix}$
- 输入映射矩阵$B=\begin{bmatrix}1\\0\end{bmatrix}$



<img src="https://gitee.com/xiao_2003/figurebed/raw/master/blog/cartpole.png" style="zoom: 33%;" />

使用Infinite Horizon LQR将倒立摆小车稳定到平衡位置

```Julia
xgoal = [0, pi, 0, 0]
x0 = [0, pi, 0, 0] + [1.5, deg2rad(-20), .3, 0]
```

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_1.svg)

<img src="https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_1.gif" style="zoom: 67%;" />

#### Part B : Basin of Attarction吸引域分析

LQR控制器是基于系统在$(x_{goal},u_{goal})$处的线性近似设计的，当系统状态远离线性化点时，真实非线性动力学与线性模型差异变大，导致控制器性能下降甚至失效。

这里测试LQR控制器在不同初始条件下的稳定性，绘制吸引域，即能成功稳定的初始状态范围。

```julia
# create a span of initial configurations 
M=20
ps = LinRange(-7, 7, M)
thetas = LinRange(deg2rad(180-60), deg2rad(180+60), M)
```

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_2.svg)

#### Part C : 无限时域LQR调参

通过调整$Q$和$R$中的权重参数，来实现5秒仿真结束时，系统状态与目标状态的2范数误差小于0.1$||x_{end}-x_{goal}||<0.1$，并且满足执行器限制$-3\le u\le 3$（限幅）。

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_3.svg)

#### Part D: TVLQR for  trajectory tracking

在参考轨迹的每个点上线性化系统，设计时变反馈增益$K(t)$，使系统能稳定跟踪时变目标。

```julia
function TVlqr(A_list::Vector{Matrix{Float64}}, B_list::Vector{Matrix{Float64}}, Q::Matrix, R::Matrix,
        Qf::Matrix,N::Int64)::Tuple{Vector{Matrix{Float64}}, Vector{Matrix{Float64}}}
      
        nx, nu = size(B_list[1])
      
        P = [zeros(nx,nx) for i = 1:N]
        K = [zeros(nu,nx) for i = 1:N-1]
        P[N] = deepcopy(Qf)
        for i = N-1:-1:1
            A,B = A_list[i],B_list[i]
            K[i] = (R + B'*P[i+1]*B) \ (B'*P[i+1]*A)
            P[i] = Q + A'*P[i+1]*(A - B*K[i])  
        end
        return P,K
    end
```

![](https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_4.svg)

<img src="https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q2_2.gif" style="zoom: 50%;" />

## Lecture10 Convex Model-Predictive Control

### HW2_Q3 Optimal Rendezvous and Docking

<img src="https://gitee.com/xiao_2003/figurebed/raw/master/blog/cmu_optimal/HW2_Q3_MPC.gif" style="zoom:50%;" />

### 无人机悬停案例

1.平面无人机动力学

$$
\begin{align}

\ddot{x}&=\frac{1}{m}(u_1+u_2)sin\theta\\
\ddot{y}&=\frac{1}{m}(u_1+u_2)cos\theta-g\\
\ddot{\theta}&=\frac{1}{J}l^2(u_2-u_1)\\
\end{align}
$$

    在平衡点线性化$(u_1=u_2 =\frac{1}{2}mg,\theta=0)$写成矩阵形式

$$
\Longrightarrow
\begin{align}
\Delta\ddot{x}&=g\theta\\
\Delta\ddot{y}&=\frac{1}{m}(\Delta u_1+\Delta u_2)\\
\Delta\ddot{\theta}&=\frac{1}{J}l^2(\Delta u_2-\Delta u_1)\\
\end{align}
$$

$$
\begin{align}
\begin{bmatrix}\Delta\dot{x}\\ \Delta\dot{y} \\ \Delta\dot{\theta} \\\Delta\ddot{x}\\ \Delta\ddot{y} \\ \Delta\ddot{\theta}\end{bmatrix}=
\begin{bmatrix}0&0 &0 & 1&0 &0  \\ 0& 0&0 &0 &1 &0  \\0&0 &0 &0 &0 &1\\
0&0&g&0&0&0\\0&0&0&0&0&0\\0&0&0&0&0&0\end{bmatrix}

\begin{bmatrix}{\Delta x}\\ {\Delta y} \\ {\Delta \theta}\\\Delta \dot{x}\\ \Delta \dot{y} \\ \Delta\dot{\theta} \end{bmatrix}
+
\begin{bmatrix}0&0\\0&0 \\0&0 \\0&0 \\ \frac{1}{m}&\frac{1}{m}\\-\frac{l^2}{J}&\frac{l^2}{J}\end{bmatrix}
\begin{bmatrix}\Delta u_1\\ \Delta u_2\end{bmatrix}
\end{align}
$$

```julia
using LinearAlgebra
using ForwardDiff
using OSQP
#Model parameters
g = 9.81 #m/s^2
m = 1.0 #kg 
ℓ = 0.3 #meters
J = 0.2*m*ℓ*ℓ

h = 0.05 #time step (20 Hz)

#Planar Quadrotor Dynamics
function quad_dynamics(x,u)
    θ = x[3]
    ẍ = (1/m)*(u[1] + u[2])*sin(θ)
    ÿ = (1/m)*(u[1] + u[2])*cos(θ) - g
    θ̈ = (1/J)*(ℓ/2)*(u[2] - u[1])
    return [x[4:6]; ẍ; ÿ; θ̈]
end
function quad_dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = quad_dynamics(x, u)
    f2 = quad_dynamics(x + 0.5*h*f1, u)
    f3 = quad_dynamics(x + 0.5*h*f2, u)
    f4 = quad_dynamics(x + h*f3, u)
    return x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

#Linearized dynamics for hovering
x_hover = zeros(6)
u_hover = [0.5*m*g; 0.5*m*g]
A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,u_hover),x_hover);
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x_hover,u),u_hover);
quad_dynamics_rk4(x_hover, u_hover)
```

### MPC

离散时不变系统的状态空间方程

$$
{x}_{k+1}=Ax_k+Bu_k
$$

定义未来$p$个周期内预测的系统状态$(n_xn_h\times1)$

$$
X_k=[x_{k+1|k} ^\top,x_{k+2|k} ^\top,...,x_{k+p|k} ^\top]^\top
$$

定义未来到达$p$个周期内的系统输入$(n_un_h\times1)$

$$
U_k=[u_{k|k} ^\top,u_{k+1|k} ^\top,...,u_{k+p-1|k} ^\top]^\top
$$

由系统离散动力学方程可以得到$x_{k|k}\rightarrow x_{k+h|k}$的状态转移方程

$$
\begin{align}
x_{k+1|k}&=Ax_{k|k}+Bu_{k|k}\\
x_{k+2|k}&=A^2x_{k|k}+ABu_{k|k}+Bu_{k+1|k}\\
...&\\
x_{k+p|k}&=A^px_{k|k}+A^{p-1}Bu_{k|k}+...+ABu_{k+p-1|k}+Bu_{k+p|k}
\end{align}
$$

写成矩阵形式

$$
X_k=\Phi x_{k|k}+\Gamma U_k
$$

其中

$$
\begin{align}
\Phi = \begin{bmatrix}A\\A^2\\\vdots \\ A^p\end{bmatrix}\quad
\Gamma = \begin{bmatrix}B& \mathbf{0}&\dots&\mathbf{0}\\AB&B& \dots&\mathbf{0}\\\vdots &\vdots&&\vdots\\A^{p-1}B&  B&\dots&AB\end{bmatrix}
\end{align}
$$

二次型性能指标

$$
\begin{align}
J&=\sum_{i=1}^{p-1}(\frac{1}{2}{x_{k+i|k}^\top}Qx_{k+i|k} + \frac{1}{2}{u_{k+i|k}^\top}Ru_{k+i|k})+\frac{1}{2}{x_{k+p|k}^\top}Q_Nx_{k+p|k}\\
&=\frac{1}{2}{x_{k|k}^\top}Qx_{k|k}+\frac{1}{2}X_{k}^\top\Omega X_{k} + \frac{1}{2}U_{k}^\top\Psi U_{k}
\end{align}
$$

其中

$$
\begin{align}
\Omega = \begin{bmatrix}Q&\dots&\mathbf{0}\\\vdots&Q&\vdots \\ \mathbf{0}&\dots&Q_N\end{bmatrix}\quad
\Psi = \begin{bmatrix}R&\dots&\mathbf{0}\\\vdots&\ddots&\vdots \\ \mathbf{0}&\dots&R\end{bmatrix}\quad\end{align}
$$

在通过变换用$U_k$表示$X_k$（忽略$x_{k|k}$决定项）

$$
\begin{align}
J&=\frac{1}{2}U_k^\top \mathbf{H}U_k+U_k^\top \mathbf{F}x_{k|k}
\end{align}
$$

其中

$$
\begin{align}
\mathbf{H}=\Gamma ^\top\Omega\Gamma+\Psi, \quad \mathbf{F}=\Gamma ^\top\Omega\Phi
\end{align}
$$

### osqp求解器的使用

QSQP求解器是一个用于求解凸二次规划（形式如下）的数值优化软件包

$$
\begin{split}\begin{array}{ll}
  \mbox{minimize} & \frac{1}{2} x^T P x + q^T x \\
  \mbox{subject to} & l \leq A x \leq u
\end{array}\end{split}
$$

其中$x$是优化变量，$P\in \mathbf{S}_+^n$s是一个半正定矩阵。

考虑一个线性时不变动力学系统到某个参考状$x_r\in \mathcal{R}^{n_x}$的问题

$$
\begin{split}\begin{array}{ll}
  \mbox{minimize}   & (x_N-x_r)^T Q_N (x_N-x_r) + \sum_{k=0}^{N-1} (x_k-x_r)^T Q (x_k-x_r) + u_k^T R u_k \\
  \mbox{subject to} & x_{k+1} = A x_k + B u_k \\
                    & x_{\rm min} \le x_k  \le x_{\rm max} \\
                    & u_{\rm min} \le u_k  \le u_{\rm max} \\
                    & x_0 = \bar{x}
\end{array}\end{split}
$$

1.给出系统的$Q,R,Q_N,A,B$

```julia
Nx = 6     # number of state
Nu = 2     # number of controls
Tfinal = 10.0 # final time
Nt = Int(Tfinal/h)+1    # number of time steps
thist = Array(range(0,h*(Nt-1), step=h));

# Cost weights
Q = Array(1.0*I(Nx));
R = Array(.01*I(Nu));
Qn = Array(1.0*I(Nx));

#Thrust limits
umin = [0.2*m*g; 0.2*m*g]
umax = [0.6*m*g; 0.6*m*g]
```

2.转换成标准QP问题

优化变量

$$
z=[x_1^T,x_2^T,\dots,x_p^T,u_0^T,u_1^T,\dots,u_{p-1}^T]^\top
$$

$$
J=z^\top \text{diag}(Q,\dots,Q,Q_N,R,...,R)z+2[Qx_{r_1},\dots,Qx_{r_k},\mathbf{0},\dots,\mathbf{0}]z
$$

```julia
U = kron(Diagonal(I,Nh), [I zeros(Nu,Nx)]) #Matrix that picks out all u
Θ = kron(Diagonal(I,Nh), [0 0 0 0 1 0 0 0]) #Matrix that picks out all x3 (θ)
H = sparse([kron(Diagonal(I,Nh-1),[R zeros(Nu,Nx); zeros(Nx,Nu) Q]) zeros((Nx+Nu)*(Nh-1), Nx+Nu); zeros(Nx+Nu,(Nx+Nu)*(Nh-1)) [R zeros(Nu,Nx); zeros(Nx,Nu) P]])
b = zeros(Nh*(Nx+Nu))
C = sparse([[B -I zeros(Nx,(Nh-1)*(Nu+Nx))]; zeros(Nx*(Nh-1),Nu) [kron(Diagonal(I,Nh-1), [A B]) zeros((Nh-1)*Nx,Nx)] + [zeros((Nh-1)*Nx,Nx) kron(Diagonal(I,Nh-1),[zeros(Nx,Nu) Diagonal(-I,Nx)])]])

#Dynamics + Thrust limit constraints
D = [C; U]
lb = [zeros(Nx*Nh); kron(ones(Nh),umin-u_hover)]
ub = [zeros(Nx*Nh); kron(ones(Nh),umax-u_hover)]

rob = OSQP.Model()
OSQP.setup!(prob; P=H, q=b, A=D, l=lb, u=ub, verbose=false, eps_abs=1e-8, eps_rel=1e-8, polish=1);
```

## Julia 简介

1. 下载：windows 通过winget下载juliaup下载指定julia版本
2. julia虚拟环境创建

```julia
#创建虚拟环境
using Pkg
Pkg.activate(@__DIR__)
#初始化虚拟环境
Pkg.instantiate()
#安装包
Pkg.add("Plots")
#检查虚拟环境状态
Pkg.status()
```

3. pyplot的使用

   ```julia
   # 在 Julia 中执行（替换为你的 Python 路径）
   ENV["PYTHON"] = raw"C:\Python39\python.exe"  # Windows 示例
   # ENV["PYTHON"] = "/usr/bin/python3"        # Linux/macOS 示例
   
   # 重新构建 PyCall
   using Pkg
   Pkg.build("PyCall")
   ```

## HW1

### Q1

Julia 求导————————ForwardDiff.jl

1. 标量函数$f(x)$对标量$x$的导数

```julia
import ForwardDiff as FD
function f(x)
    return x^2
end
x=randn()
dx=FD.derivative(f,x)
```

1. 标量函数$f(x)$对向量$X=[x_1,x_2,...,x_n]$的导数--雅可比矩阵/梯度
2. 向量函数$f(X)=[f_1(X),f_2(X),...,f_m(X),]$对向量$X=[x_1,x_2,...,x_n]$的Jacobian矩阵

## 模型预测控制器MPC

$X_(k+1)=A x_(k) + B u_k$

# 额外知识

# 卡尔曼滤波

1.递归算法

实际测量有随机误差，多次测量取平均



未完待续

## 参考笔记

[Optimal_control_16-745/CMU16_745_Optimial control Lecture_Notes_zhihai Bi.pdf at main · Zhihaibi/Optimal_control_16-745](https://github.com/Zhihaibi/Optimal_control_16-745/blob/main/CMU16_745_Optimial%20control%20Lecture_Notes_zhihai%20Bi.pdf)

期刊：ICAR IROS RSS TRO IJRR
