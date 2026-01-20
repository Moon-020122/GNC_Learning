

# 序言

## 1. 欧拉角流派

- **公式**：
  $$
  \begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix} = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\ 0 & \cos\phi & -\sin\phi\\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \end{bmatrix} \begin{bmatrix}p\\q\\r\end{bmatrix}
  $$
  

## 2. 四元数流派

- **公式**：
  $$
  \dot{q} = \frac{1}{2} q \otimes \omega
  $$

# **公式：** $\dot V = g^n + \frac{1}{m} R_b^n f^b$ 

这是牛顿第二定律 $F=ma$ 的变体。

- **$\dot V$ (Acceleration)**：速度的变化率（加速度）。我们算它的目的是为了积分得到速度 $V$。
- **$g^n$ (Gravity)**：重力加速度向量。
  - 注意那个上标 **$n$**：代表 **Navigation Frame (导航坐标系)**，也就是“大地坐标系”。
  - 在大地看来，重力永远指向地心。如果你的 $Z$ 轴向下（NED坐标系），那么 $g^n = [0, 0, 9.8]^T$。
- **$m$ (Mass)**：飞机的质量。
- **$f^b$ (Force in Body)**：飞机自己产生的力。
  - 注意上标 **$b$**：代表 **Body Frame (机体坐标系)**。
  - 对于四旋翼无人机，螺旋桨推力总是垂直于机身向上的。在机体坐标系里，这个力很简单，只有 Z 轴有值，所以是 $[0, 0, -T]^T$（这里 $-T$ 是因为推力朝上，而通常机体系 Z 轴朝下）。
- **$R_b^n$ (Rotation Matrix)**：**最关键的“翻译官”**。
  - 你的推力 $f^b$ 是跟着机身歪来歪去的（机身歪了，推力就斜了）。
  - 但是我们要算位置变化，必须在“大地”上算。
  - $R_b^n$ 的作用就是：**把“机身方向”的力，翻译成“大地方向”的力。**
  - *例子*：飞机向左倾斜 30 度。推力本来是纯向上的，经过 $R_b^n$ 转换后，在大地坐标系里就变成了一个“向上 + 向左”的力。

# 姿态运动学：飞机是怎么“转头”的？

## **概念一：角速度 $\omega$**

- **它是谁？** 它是**陀螺仪**测量出来的东西。
- **body 还是 nav？** 这一点必须死记：**传感器测量的角速度 $\omega$ 永远是在 Body Frame (机体系) 下的。**
  - 想象你坐在飞机驾驶舱里，你感觉到飞机绕着**你自己的**左手右手（X轴）、头顶脚底（Z轴）旋转的快慢。这就叫 $\omega$。

## **概念二：欧拉角率 $\dot\phi, \dot\theta, \dot\psi$**

- **它是谁？** 它是滚转角(Roll)、俯仰角(Pitch)、偏航角(Yaw) 的变化速度。
- **区别**：
  - $\omega$ 是**物理上**转得有多快。
  - $\dot\phi, \dot\theta, \dot\psi$ 是我们在**数学表达上**看着那个角度数字跳得有多快。

## **关键关系：映射矩阵**

二者需要一个映射矩阵（ZYX yaw-pitch-roll 约定下）：

1）从欧拉角率到 body 角速度（永远不奇异）
$$
\begin{bmatrix}p\\q\\r\end{bmatrix} = \underbrace{ \begin{bmatrix} 1 & 0 & -\sin\theta\\ 0 & \cos\phi & \sin\phi\cos\theta\\ 0 & -\sin\phi & \cos\phi\cos\theta \end{bmatrix}}_{E(\phi,\theta)} \begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}
$$
2）从 body 角速度到欧拉角率（会在$\theta=\pm 90^\circ $奇异）
$$
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix} = \underbrace{ \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\ 0 & \cos\phi & -\sin\phi\\ 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \end{bmatrix}}_{W(\phi,\theta)} \begin{bmatrix}p\\q\\r\end{bmatrix}
$$
奇异的原因就一个：里面有$ \frac{1}{\cos\theta}$。当 $\theta \to \pm 90^\circ$，$\cos\theta\to 0$，就炸了。

> 工程结论（你要记住）：
>  ✅ 控制/估计里可以用欧拉角做“人能看懂的输出”
>  ✅ **积分姿态（状态传播）用四元数**，避免奇异

这就好比“地球仪”的原理。

- 当飞机平飞时，绕机头转动（$\omega_x$）就等于滚转角的变化（$\dot\phi$）。
- **但是！** 当飞机机头垂直朝天（$\theta = 90^\circ$）时，这套数学系统就**崩溃**了。
  - 这就是所谓的**奇异点 (Singularity)**。
  - 在 90 度时，你没法区分什么是滚转，什么是偏航（万向节死锁）。数学上表现为那个映射矩阵的分母变成了 0。

### 映射矩阵推导

我们要找出一个矩阵 $E$，把**欧拉角的变化率**（$\dot{\phi}, \dot{\theta}, \dot{\psi}$）变成**机体轴的角速度**（$p, q, r$）。
$$
\begin{bmatrix} p \\ q \\ r \end{bmatrix}_b = \text{某个矩阵} \times \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
$$
核心难点：这三个变化率，不是在同一个坐标系里发生的！

- $\dot{\psi}$ (偏航率)：绕着**地面**的 Z 轴转。
- $\dot{\theta}$ (俯仰率)：绕着**中间过程**的 Y 轴转。
- $\dot{\phi}$ (滚转率)：绕着**最终机身**的 X 轴转。

我们的任务：**把这三个不同世界的向量，全部投影（翻译）到最终的机身坐标系 ($X_b, Y_b, Z_b$) 里，然后加起来。**

- **地面 Z 轴（Yaw）**：就像**坦克的底座**。底座转的时候，上面的炮塔架子也跟着转。
- **中间 Y 轴（Pitch）**：就像**炮管的转轴**。炮管抬高/压低，是绕着炮塔上的轴转的。如果底座转了，这个轴的方向也就变了。
- **机头 X 轴（Roll）**：就像**炮管本身**。炮弹在炮管里旋转（来福线），那就是绕着炮管自己的轴转。

因为我们要把 “底座的转速” + “炮管抬起的转速” + “炮弹自旋的转速”，全部换算成 “炮弹视角下的三个分量”。

![image-20260112113136273](E:\DockerDesktopWSL\GNC_Learning\02_UAV\note\image\image-20260112113136273.png)

#### 定义坐标系和旋转顺序 (Z-Y-X)

我们采用航空航天标准的 **NED (北东地)** 到 **Body (机体)** 的旋转顺序：**Yaw $\to$ Pitch $\to$ Roll**。

我们需要定义 **4 个坐标系**，像剥洋葱一样：

1. **$N$ 系 (导航/地面系)**：不动。
   - 转动：$\dot{\psi}$ (Yaw) 绕 $Z_n$ 轴。
   - 得到：**$1$ 系 (中间系 1)**。
2. **$1$ 系**：
   - 转动：$\dot{\theta}$ (Pitch) 绕 $Y_1$ 轴 (注意：是 $1$ 系的新 Y 轴)。
   - 得到：**$2$ 系 (中间系 2)**。
3. **$2$ 系**：
   - 转动：$\dot{\phi}$ (Roll) 绕 $X_2$ 轴 (注意：是 $2$ 系的新 X 轴)。
   - 得到：**$B$ 系 (最终机体系)**。

​	GNC 的标准旋转顺序是 **Z-Y-X** (Yaw $\to$ Pitch $\to$ Roll)。这意味着任何向量要进入“机体坐标系（终点）”，都必须严格按照这个顺序过关卡。

想象我们有三个房间，中间由三扇门连接：

- **房间 N (起点)**：大地。
- **门 1 (Yaw/Z轴)**：穿过这扇门，你就进了一号走廊。
- **门 2 (Pitch/Y轴)**：穿过这扇门，你就进了二号走廊。 
- **门 3 (Roll/X轴)**：穿过这扇门，你就进了 **机体驾驶舱 (终点/B系)**。

​	一定要记住的单次旋转矩阵（基础积木）：把向量从“旧系”投影到“新系”的旋转矩阵（右手定则，矩阵乘法右边先作用，注意之前说的ZYX实际上作用过程是XYZ）：(**物理含义**：**点不动，坐标系动。**导航系 (Nav) 投影到 机体系 (Body))[[可视化文件]](./visual/RorationMatrix.html)

![image-20260112123505197](E:\DockerDesktopWSL\GNC_Learning\02_UAV\note\image\image-20260112123505197.png)

- $R_x(\phi)$ (绕 X 转)：
  $$
  \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi \end{bmatrix}
  $$

- $R_y(\theta)$ (绕 Y 转)：
  $$
  \begin{bmatrix} \cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta \end{bmatrix}
  $$
  

- $R_z(\psi)$ (绕 Z 转)：
  $$
  \begin{bmatrix} \cos\psi & \sin\psi & 0 \\ -\sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
  $$

#### 投影求和

1. 处理滚转率 $\dot{\phi}$

- **它在哪？** 它发生在最后一步，绕着 $X_2$ 轴转。

- **关键点**：$X_2$ 轴其实就是最终的机体 $X_b$ 轴（因为最后一步 Roll 是绕 X 转的，X 轴本身不动）。

- 投影结果：它已经在机体系里了，不需要旋转矩阵！
  $$
  \vec{\omega}_{\phi}^b = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix}
  $$

2. 处理俯仰率 $\dot{\theta}$

- **它在哪？** 它发生在中间步骤，绕着 $Y_1$ 轴转（也就是 $Y_2$ 轴）。

- **怎么去机体系？** 它在 $2$ 系里，要变成 $B$ 系，只需要经历**最后一步旋转（Roll）**。

- 数学操作：用 $R_x(\phi)$ 把它乘一下。
  $$
  \vec{\omega}_{\theta}^b = R_x(\phi) \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix}= \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi \end{bmatrix} \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} \\
  \vec{\omega}_{\theta}^b = \begin{bmatrix} 0 \\ \dot{\theta} \cos\phi \\ -\dot{\theta} \sin\phi \end{bmatrix}
  $$
  

3. 处理偏航率 $\dot{\psi}$ 

- **它在哪？** 它在最开始，绕着地面的 $Z_n$ 轴转（也就是 $Z_1$ 轴）。

- **怎么去机体系？** 它离机体系最远，隔了两层。

  1. 先经过 Pitch 旋转 ($R_y(\theta)$)，从 $1$ 系变到 $2$ 系。
  2. 再经过 Roll 旋转 ($R_x(\phi)$)，从 $2$ 系变到 $B$ 系。

- **数学操作**：$\vec{\omega}_{\psi}^b = R_x(\phi) \cdot R_y(\theta) \cdot \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix}$

  第一步：先乘 $R_y(\theta)$
  $$
  \text{中间向量} = \begin{bmatrix} \cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} -\dot{\psi} \sin\theta \\ 0 \\ \dot{\psi} \cos\theta \end{bmatrix}
  $$
  第二步：再乘 $R_x(\phi)$
  $$
  \vec{\omega}_{\psi}^b = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi \end{bmatrix} \begin{bmatrix} -\dot{\psi} \sin\theta \\ 0 \\ \dot{\psi} \cos\theta \end{bmatrix} \\
  \vec{\omega}_{\psi}^b = \begin{bmatrix} -\dot{\psi} \sin\theta \\ \dot{\psi} \sin\phi \cos\theta \\ \dot{\psi} \cos\phi \cos\theta \end{bmatrix}
  $$
  现在我们有了三个分量，全部都在机体系 B 下了。机体总角速度 $\omega_b = [p, q, r]^T$ 就是这三个向量相加：
  $$
  \begin{bmatrix} p \\ q \\ r \end{bmatrix} = \vec{\omega}_{\phi}^b + \vec{\omega}_{\theta}^b + \vec{\omega}_{\psi}^b 
  = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix} + \begin{bmatrix} 0 \\ \dot{\theta} \cos\phi \\ -\dot{\theta} \sin\phi \end{bmatrix} + \begin{bmatrix} -\dot{\psi} \sin\theta \\ \dot{\psi} \sin\phi \cos\theta \\ \dot{\psi} \cos\phi \cos\theta \end{bmatrix} 
  $$

  $$
  \begin{bmatrix} p \\ q \\ r \end{bmatrix} = \underbrace{ \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \sin\phi \cos\theta \\ 0 & -\sin\phi & \cos\phi \cos\theta \end{bmatrix} }_{E(\phi, \theta)} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
  $$

  

  