### 卡尔曼滤波 ###

**卡尔曼滤波器的操作包含两个阶段：预测和更新。**

Kalman滤波包括两个阶段：预测和更新。在预测阶段，滤波器利用上一状态的估计做出对当前状态的估计；在更新阶段，滤波器利用在当前状态的观测值优化在预测阶段获得的预测值，以获的一个更精确的当前状态的估计。

Note：
> 用$n-1$帧的状态预测$n$帧的状态，再由第$n$帧的**输出**更新第$n$帧的状态。

想要用kalman滤波，要知道前一时刻的状态估计值$x$，当前的观测值$y$，还得建立状态方程和量测方程，有了这些就可以运用kalman滤波了。

卡尔曼有三种用途：回归、滤波和预测。

1\. 回归问题

给定多个自变量、一个因变量以及代表它们之间关系的一些训练样本，如何来确定它们的关系的问题为回归问题。

2\. 滤波问题

3\. 预测问题

### 运动模型 ###

1\. 匀速模型

匀速模型假设车辆以较为恒定的速度行驶，则匀速模型为：

\begin{align*}
  \dot{X}(t)=AX(t)+w(t)
\end{align*}

其中，

\begin{align*}
A=
\begin{pmatrix}
0 & 1 \cr
0 & 0 \cr
\end{pmatrix}  \hspace{8mm}
X(t)=
\begin{pmatrix}
x(t) \cr
\dot{x}(t) \cr
\end{pmatrix} \cr
w(t)=\begin{pmatrix}
0\cr
a(t)\cr
\end{pmatrix}  \hspace{8mm}
R(t)=\begin{pmatrix} \cr
0 & 0 \cr
0 & \sigma^2_a \cr
\end{pmatrix}
\end{align*}

其中，$\sigma^2_a$为加速度的方差。

离散系统的采样间隔$T_k=t_{k+1}-t_k$，其状态转移矩阵和状态噪声的协方差阵分别为：

\begin{align*}
X(k+1) &=\Phi(k)X(k)+w(k) \\
\Phi(k)&=(e^{{A}{T}_k}) \\
    &=(I+{A}{T_k}+A^2\dfrac{{{T_k}^2}}{2!}+A^3\dfrac{{T_k}^3}{3!}+\dots) \\
    &=\begin{pmatrix} 1 & T_k \cr 0 & 1 \cr \end{pmatrix} \\
Q(k)&={\sigma}^2_a
       \begin{pmatrix}
         \frac{1}{3}T^3_k & \frac{1}{2}T^2_k \\
         \frac{1}{2}T^2_k & T_k \\
       \end{pmatrix} \\
\end{align*}

加速过程的噪声$\sigma^2_a$只有在对车辆实际行驶过程进行了测量才能确定。对于完全可预测的系统，位置测量信息一定是可估计的。与上式相对应的离散后的测量方程为：

\begin{align*} 
Y(k)=H(k)X(k)+v(k)
\end{align*}

其中，对于仅有位置可观测的系统，有：

\begin{align*}
H(k)&=\begin{pmatrix}
1 & 0 \cr
\end{pmatrix}  \hspace{8mm}
Y(k)&=(y(k))  \hspace{8mm}
R(k)&=({\sigma}^2_a)  \hspace{8mm}
\end{align*}

对于位置可速度都可观测的系统，则有：

\begin{align*}
H(k)&=\begin{pmatrix}
1 & 1 \cr
\end{pmatrix}  \hspace{8mm}
Y(k)&=\begin{pmatrix}
(y(k)) \cr
\ddot{y}(k) \cr
\end{pmatrix}  \hspace{8mm}
R(k)&=\begin{pmatrix}
{\sigma}_x^2 & {\sigma}_{\dot{x}x} \cr
{\sigma}_{\dot{x}x} & {\sigma}^2_{\dot{x}} \cr
\end{pmatrix}
\end{align*}

这里的$\sigma^2_x$、$\sigma^2_\dot{x}$、$\sigma_{x\dot{x}}$分别为位置方差、速度方差以及位置和速度的协方差。这些参数由定位系统的特性决定，并且可能随着时间、位置的不同而变化。因为卡尔曼滤波器是递归形式的，所以需要给定初始状态和初始状态的误差。
如果观测值只有位置，则需要$Y(0)$和$Y(1)$两个测量值来初始化一阶系统，其初始状态为：

\begin{align*}
\hat{X}(1|1)=
\begin{pmatrix}
Y_1(1) \cr
\frac{1}{T_0}(Y_1(1)-Y_1(0)) \cr
\end{pmatrix}
\end{align*}

假设在两次测量中测量噪声是不变的，即$R(0)=R(1)$，状态误差矩阵为：

\begin{align*}
P(1|1)=
\begin{pmatrix}
\sigma^2_x & \frac{1}{T_0}{\sigma}^2_x \cr
\frac{1}{T_0}{\sigma}^2_x & \frac{2}{T^2_0}{\sigma}^2_x+\frac{T_0}{3}{\sigma^2_a} \cr
\end{pmatrix}
\end{align*}

其中，$\sigma^2_x=R_{11}(0)$为定位系统测量误差的方差。

如果车辆从静止开始起步，那么滤波器只需要一个初始位置进行初始化。初始状态估计和相关的状态量方差如下：

\begin{align*}
\hat{X}(0|0)&=
\begin{pmatrix}
Y_1(0) \cr
0 \cr
\end{pmatrix}  \hspace{16mm}
P(0|0)&=
\begin{pmatrix}
{\sigma}^2_x & 0 \cr
0 & {\sigma^2_{\dot{x}}} \cr
\end{pmatrix}
\end{align*}

其中，$\sigma^2_{\dot{x}}$为车辆的速度方差。

如果车辆的速度和位置都可观测，初始状态为：

\begin{align*}
\hat{X}(0|0)&=
\begin{pmatrix}
Y_1(0) \cr
Y_2(0) \cr
\end{pmatrix} \cr
&=
\begin{pmatrix}
Y(0) \cr
\dot{Y}(0) \cr
\end{pmatrix} \cr
P(0|0)&=R(0) \cr
\end{align*}

2\. 匀加速模型

匀加速模型的滤波方程与匀速模型类似，其连续系统的参数如下所示：

\begin{align*}
    A(t)&=
    \begin{pmatrix}
    0 & 1 & 0 \cr
    0 & 0 & 1 \cr
    0 & 0 & 0 \cr
    \end{pmatrix}  \hspace{16mm}
    X(t)=
    \begin{pmatrix}
    x(t) \cr
    \dot{x}(t) \cr
    \ddot{x}(t) \cr
    \end{pmatrix} \cr
    w(t)&=
    \begin{pmatrix}
    0 \cr
    0 \cr
    \dot{a}(t) \cr
    \end{pmatrix} \hspace{16mm}
    Q(t)=
    \begin{pmatrix}
    0 & 0 & 0 \cr
    0 & 0 & 0 \cr
    0 & 0 & {\sigma}^2_{\dot{a}}
    \end{pmatrix}
\end{align*}

其中，${\sigma}^2_{\dot{a}}$为加速度变化率的方差。

离散模型的状态转移矩阵为：

\begin{align*}
\Phi(k)=
\begin{pmatrix}
1 & T_k & \frac{1}{2}T^2_k \cr
0 & 1 & T_k \cr
0 & 0 & 1 \cr
\end{pmatrix} \cr
\end{align*}

状态噪声的协方差阵为：

\begin{align*}
Q(k)=\sigma^2_\dot{a}
\begin{pmatrix}
\frac{1}{20}T^5_k & \frac{1}{8}T^4_k & \frac{1}{6}T^3_k \cr
\frac{1}{8}T^4_k & \frac{1}{3}T^3_k & \frac{1}{2}T^2_k \cr
\frac{1}{6}T^3_k & \frac{1}{2}T^2_k & T_k \cr
\end{pmatrix} \cr
\end{align*}

假定定位系统不能直接测量到加速度，因此，观测方程的参数在不同条件下有如下形式：

若仅有位置测量值，则：

\begin{align*}
H=
\begin{pmatrix}
1 & 0 & 0 \cr
\end{pmatrix} \hspace{16mm}
Y(k)=(y(k))  \hspace{16mm}
R(k)=(\sigma^2_x)
\end{align*}

若位置和速度测量值都存在，则：

\begin{align*}
H=
\begin{pmatrix}
1 & 0 & 0 \cr
\end{pmatrix} \hspace{16mm}
Y(k)=
\begin{pmatrix}
y(k)  \cr
\dot{y}(k) \cr
\end{pmatrix} \hspace{16mm}
R(k)=
\begin{pmatrix}
\sigma^2_x & \sigma_{x\dot{x}} \cr
\sigma_{x\dot{x}} & \sigma^2_\dot{x} \cr
\end{pmatrix}
\end{align*}

当位置是唯一的状态观测量时，初始化滤波器需要三个等间隔（$T_0=T_1=T_2$）的测量值$Y(0)$、$Y(1)$和$Y(2)$，这时系统的初始状态和相应的状态误差矩阵为：

\begin{align*}
\hat{X}(2|2)&=
\begin{pmatrix}
Y_1(2) \cr
\frac{1}{T}(Y_1(2)-Y_1(1)) \cr
\frac{1}{T^2}(Y_1(2)-2Y_1(1)+Y_1(0)) \cr
\end{pmatrix} \cr
P(2|2)&=
\begin{pmatrix}
\sigma^2_x & \frac{1}{T}\sigma^2_x & \frac{1}{T^2}\sigma^2_x \cr
\frac{1}{T}\sigma^2_x & \frac{2}{T^2}\sigma^2_x+\frac{T^2}{4}\sigma^2_a+\frac{T^3}{5}\sigma^2_\dot{x} & \frac{3}{T^3}\sigma^2_x+\frac{7}{40}T^2\sigma^2_\dot{a} \cr
\frac{1}{T^2}\sigma^2_x & \frac{3}{T^3}\sigma^2_x+\frac{7}{40}T^2\sigma^2_\dot{a} & \frac{6}{T^4}\sigma^2_x+\frac{23}{30}T\sigma^2_\dot{a} \cr
\end{pmatrix}
\end{align*}

其中，$\sigma^2_a$是加速度的方差。

通常采用的方法是假设除了位置以外的所有状态变量在初始时为零，此时初始状态和方差针为：


\begin{align*}
\hat{X}(1|1)&=
\begin{pmatrix}
Y_1(1) \cr
0 \cr
0 \cr
\end{pmatrix} \cr
P(1|1)&=
\begin{pmatrix}
\sigma^2_x & 0 & 0 \cr
0 & \sigma^2_\dot{x} & 0 \cr
0 & 0 & \sigma^2_a \cr
\end{pmatrix}
\end{align*}

其中，$\sigma^2_\dot{x}$和$\sigma^2_a$分别是车辆速度和加速度的方差。如果位置和速度都可得到，则初始状态和方差为：

\begin{align*}
\hat{X}(1|1)&=
\begin{pmatrix}
Y_1(1) \cr
Y_2(1) \cr
\frac{Y_1(1)-Y_1(0)}{T} \cr
\end{pmatrix} \cr
P(1|1)&=
\begin{pmatrix}
\sigma^2_x & \sigma_{x\dot{x}} & \frac{1}{T}\sigma_{x\dot{x}} \cr
\sigma_{x\dot{x}} & \sigma^2_\dot{x} & \frac{1}{T}\sigma^2_{\dot{x}} \cr
\frac{1}{T}\sigma_{x\dot{x}} & \frac{1}{T}\sigma^2_{\dot{x}} & \frac{2}{T^2}\sigma^2_\dot{x}+\frac{T}{3}\sigma^2_\dot{a} \cr
\end{pmatrix}
\end{align*}


---
状态转移矩阵：

\begin{align*}
F(t)=
\begin{pmatrix}
1 & 0 & t & 0 & \frac{1}{2}t^2 & 0 \cr
0 & 1 & 0 & t & 0 & \frac{1}{2}t^2 \cr
0 & 0 & 1 & 0 & t & 0 \cr
0 & 0 & 0 & 1 & 0 & t \cr
0 & 0 & 0 & 0 & 1 & 0 \cr
0 & 0 & 0 & 0 & 0 & 1 \cr
\end{pmatrix} \cr
\end{align*}

观测模型：

\begin{align*}
H(t)=
\begin{pmatrix}
1 & 0 & 0 & 0 & 0 & 0 \cr
0 & 1 & 0 & 0 & 0 & 0 \cr
\end{pmatrix} \cr
\end{align*}


---

#### 分析 ####

针对GPS位置滤波（经度和纬度），状态数为4，包括$(x,y,\dot{x}, \dot{y})$，即为状态量，其中$x$、$y$为位置量，即能看到的坐标值；$\dot{x}$、$\dot{y}$为速度（即每次移动的距离）。

假定预测量为$\hat{x}$、$\hat{y}$，那么：

\begin{align*}
\begin{pmatrix}
\hat{x} \cr
\hat{y} \cr
dx \cr
dy \cr
\end{pmatrix}
&=
\begin{pmatrix}
1 & 0 & 1 & 0 \cr
0 & 1 & 0 & 1 \cr
0 & 0 & 1 & 0 \cr
0 & 0 & 0 & 1 \cr
\end{pmatrix}
\begin{pmatrix}
x \cr
y \cr
dx \cr
dy \cr
\end{pmatrix}
\end{align*}

由于物体做匀速运动，则$dx=\dot{x},dy=\dot{y}$。

即，

\begin{align*}
\left\{
\begin{aligned}
\hat{x}&=x+dx=x+\dot{x} \cr
\hat{y}&=y+dy=y+\dot{y} \cr
\end{aligned}
\right.
\end{align*}


---


#### 说明 ####
1. 摘自：[GPS动态滤波的理论、方法及其应用][1]
2. C语言实现的Kalman库：[iKalman][2]


---
参考链接：

1\. [Sensor Fusion using the Kalman Filter][3]；

2\. [Example_application.2C_technical][4]；

3\. [Implementing a Kalman filter for position, velocity, acceleration][5]；

4\. [More on: Kalman filter for position and velocity][6]。


[1]: http://books.google.com.hk/books?id=Jy6YAAAACAAJ
[2]: https://github.com/lacker/ikalman
[3]: http://campar.in.tum.de/Chair/KalmanFilter
[4]: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
[5]: http://dsp.stackexchange.com/questions/8840/implementing-a-kalman-filter-for-position-velocity-acceleration
[6]: http://dsp.stackexchange.com/questions/8860/more-on-kalman-filter-for-position-and-velocity
