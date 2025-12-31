### 第一部分：拉格朗日力学基础 (建模工具)

相比于牛顿力学（分析受力，$F=ma$），拉格朗日力学更关注**能量**。对于复杂的连杆、机械臂或倒立摆，去分析每一个关节的“反作用力”会让人头大，而算能量则简单得多。

#### 1. 广义坐标 ($q$)

**定义：** 描述系统位置所需的最少独立变量。在牛顿力学里我们习惯用直角坐标 $(x, y, z)$。但在拉格朗日力学里，我们可以选任何方便的量。

**在本例（单摆）中：** $q$ 就是角度 **$\theta$**。只要知道 $\theta$，我就知道摆球在哪。

- **例子：** 单摆不需要 $(x, y)$ 两个坐标，因为绳长 $l$ 是固定的。只需要一个角度 $\theta$ 就能确定球的位置。
- **在这里：** $q = \theta$（角度），$\dot{q} = \dot{\theta}$（角速度）。

#### 2. 动能 ($T$) 与 势能 ($V$)

- **动能 ($T$)：** 一般是 $\frac{1}{2}mv^2$ 或 $\frac{1}{2}I\omega^2$。
  - 单摆的速度 $v = l\dot{\theta}$。
  - 所以 $T = \frac{1}{2}m(l\dot{\theta})^2 = \frac{1}{2}ml^2\dot{\theta}^2$。
- **势能 ($V$)：** 一般是 $mgh$（重力）或 $\frac{1}{2}kx^2$（弹簧）。
  - 取悬挂点为零势能面，摆球下降了 $l\cos\theta$ 的距离。
  - $V = -mgl\cos\theta$ (或者取最低点为0，则是 $mgl(1-\cos\theta)$，结果是一样的，因为常数求导为0)。

#### 3. 拉格朗日量 ($L$) 与 方程

- **定义：** $L = T - V$。

  - 本例中：$L = \frac{1}{2}ml^2\dot{\theta}^2 - (-mgl\cos\theta) = \frac{1}{2}ml^2\dot{\theta}^2 + mgl\cos\theta$。

- 欧拉-拉格朗日方程（核心公式）：
  $$
  \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}}\right) - \frac{\partial L}{\partial q} = \tau
  $$
  欧拉-拉格朗日方程其实就是**能量版本的“F=ma”**。让我们拆解那两个偏导数，看看它们变成了什么。

  - $\tau$ 是广义力（如电机输入的扭矩 $u$）。如果没有外力，右边为0。

**实战推导（高数偏导）：**

1. 对 $\dot{q}$ ($\dot{\theta}$) 求偏导： 把 $\theta$ 当常数。

   

   $$\frac{\partial L}{\partial \dot{\theta}} = \frac{1}{2}ml^2 \cdot (2\dot{\theta}) = ml^2\dot{\theta}$$

2. 对时间 $t$ 求全导： 注意 $\dot{\theta}$ 随时间变，导数是 $\ddot{\theta}$。

   

   $$\frac{d}{dt}(ml^2\dot{\theta}) = ml^2\ddot{\theta}$$

3. 对 $q$ ($\theta$) 求偏导： 把 $\dot{\theta}$ 当常数。

   

   $$\frac{\partial L}{\partial \theta} = mgl(-\sin\theta) = -mgl\sin\theta$$

4. 代入方程：

   

   $$ml^2\ddot{\theta} - (-mgl\sin\theta) = u$$

   $$ml^2\ddot{\theta} + mgl\sin\theta = u$$

这就是系统的**非线性运动方程**。