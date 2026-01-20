# 转动动力学（欧拉方程）

$$
\tau = I\,\dot{\omega} + \omega \times (I\omega)
$$

这里全部默认在 **Body 坐标系表达**（这是工程最常用的形式）：

- $\omega \in \mathbb{R}^3$：**机体系角速度**（也就是陀螺给的 $p,q,r$）
- $\dot{\omega}$：角加速度
- $I$：**刚体转动惯量矩阵**（在 body 坐标下是常数矩阵）
- $\tau$：外部力矩（电机、舵面、气动、重力梯度等都算，单位 N·m）
- $\omega \times (I\omega)$：**陀螺耦合项/非线性项**

## 1.旋转坐标系里“求导”会多一项

核心关系是：
$$
\left(\frac{d\mathbf{L}}{dt}\right)_{inertial} = \left(\frac{d\mathbf{L}}{dt}\right)_{body} +\boldsymbol{\omega}\times\mathbf{L}
$$

### 用“基向量在转”来推

在机体系里，用机体系的三个单位基向量表示任何向量：
$$
\mathbf{v} = v_1\mathbf{e}_1+v_2\mathbf{e}_2+v_3\mathbf{e}_3
$$
注意：**在机体系里，$\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3$ 本身会随时间转动**。

对时间求导（这是纯粹的乘法求导法则）：
$$
\frac{d\mathbf{v}}{dt} = \dot v_1\mathbf{e}_1+\dot v_2\mathbf{e}_2+\dot v_3\mathbf{e}_3 +v_1\dot{\mathbf{e}}_1+v_2\dot{\mathbf{e}}_2+v_3\dot{\mathbf{e}}_3
$$
前一组 $\dot{v}_i\mathbf{e}_i$ 是“**坐标数值变了**”产生的变化；后一组$v_i\dot{\mathbf{e}}_i$ 是“**坐标轴在转**”产生的变化。

旋转的坐标轴满足：
$$
\dot{\mathbf{e}}_i = \boldsymbol{\omega}\times \mathbf{e}_i
$$
这句话的意思是：轴在以角速度$\boldsymbol{\omega}$转动，所以每个基向量的变化方向垂直于它本身，有点线速度的感觉，大小与$\omega$成比例，形式就是叉乘。

把它代回去：
$$
\frac{d\mathbf{v}}{dt} = \underbrace{\left(\dot v_1\mathbf{e}_1+\dot v_2\mathbf{e}_2+\dot v_3\mathbf{e}_3\right)}_{\text{机体系下“只看坐标变化”的导数}} + \boldsymbol{\omega}\times \underbrace{\left(v_1\mathbf{e}_1+v_2\mathbf{e}_2+v_3\mathbf{e}_3\right)}_{\mathbf{v}}
$$
于是得到：
$$
\left(\frac{d\mathbf{v}}{dt}\right)_{inertial} = \left(\frac{d\mathbf{v}}{dt}\right)_{body} +\boldsymbol{\omega}\times\mathbf{v}
$$

> 这就是“为什么会多出来$\omega\times(\cdot)$”：
>  因为**你在一个会转的坐标系里写向量**，基向量本身在变。

## 2.欧拉方程逐字逐句解释

你给的欧拉方程（在 body 表达）：
$$
\boldsymbol{\tau} = I\,\dot{\boldsymbol{\omega}} +\boldsymbol{\omega}\times(I\boldsymbol{\omega})
$$
我现在把每个符号都讲清楚它“在干什么”。

**1）$\boldsymbol{\omega}\in\mathbb{R}^3$角速度**

这是“机体相对惯性系”的角速度向量，用机体系坐标写出来就是：[角速度方向可视化](.\visual\AngleVelVector.html)
$$
\boldsymbol{\omega} = \begin{bmatrix} p\\q\\r \end{bmatrix}
$$
它告诉你：此刻绕机体 $x,y,z$轴各自转得多快。

**2）$\dot{\boldsymbol{\omega}}$：角加速度**

就是角速度对时间的变化率：
$$
\dot{\boldsymbol{\omega}}= \begin{bmatrix} \dot p\\\dot q\\\dot r \end{bmatrix}
$$
**3）$I$：转动惯量矩阵（惯量张量）**

这是 $3\times 3$ 矩阵，描述“物体质量怎么分布”。它把角速度映射成角动量：
$$
\mathbf{L}=I\boldsymbol{\omega}
$$

- 如果你沿某轴的质量分布更“远离轴”，绕那轴转就更难，惯量就更大。
- $I $一般是**对称矩阵**。在选到“主惯量轴”（principal axes）时它是对角阵：

$$
I= \begin{bmatrix} I_{xx}&0&0\\ 0&I_{yy}&0\\ 0&0&I_{zz} \end{bmatrix}
$$

> **你不需要**会从连续介质积分推 $I$ 的通式（那是更偏理论力学/连续介质）。
> **你需要**理解它的物理意义，并能对简单形状查表或算出$I_{xx},I_{yy},I_{zz}$​。

**4）$\boldsymbol{\tau}$：外力矩**

就是“拧它的力”，来自：

- 反作用飞轮/控制力矩陀螺
- 推力器
- 舵面气动
- 重力梯度、磁力矩、气动力矩等

**5）第一项 $I\dot{\boldsymbol{\omega}}$：惯性项**

如果没有耦合项，你会以为：
$$
\boldsymbol{\tau} = I\dot{\boldsymbol{\omega}}
$$
这就像直线运动的 $F=ma$：给力矩 → 产生角加速度。

**6）第二项 $\boldsymbol{\omega}\times(I\boldsymbol{\omega})$：陀螺耦合项（非线性项）**

它的来源是我们刚证明的“旋转系求导多一项”。

### 把欧拉方程从“角动量定律”推出来

**第一步：定义角动量**
$$
\mathbf{L}=I\boldsymbol{\omega}
$$
（在刚体、body轴固定于刚体的前提下，$I$在 body 坐标里是常数矩阵, 角动量的方向，代表了物体**“目前正在死守的旋转面”**。）

- $L$：角动量（老大，守恒的，指向北极星）。
- $\omega$：旋转轴（物体实际绕着转的轴）。
- **$I$（惯量）：** 它**不是一个数字**，它是一个**矩阵（3x3 的表格）**！而且这个矩阵还跟物体的形状有关。

因为 $I$ 是个矩阵，它起到了一个**“扭曲镜”**的作用。

- 即使 $L$ 指向正北，$\omega$ 可能指向“北偏东 30度”。
- 只要 $I \cdot \omega_{\text{歪的}}$ 算出来的结果等于 $L_{\text{正北}}$，这个方程就成立！
- **结论：** 物理定律允许 $\omega$ 和 $L$ 不重合。只要它们满足那个矩阵关系，大自然就允许这种“由于形状不对称导致的歪着转”存在。

**第二步：惯性系里的基本定律（物理核心）**
$$
\boldsymbol{\tau}= \left(\frac{d\mathbf{L}}{dt}\right)_{inertial}
$$
**第三步：把“惯性系导数”换成“机体系导数 + 叉乘项”**
$$
\left(\frac{d\mathbf{L}}{dt}\right)_{inertial} = \left(\frac{d\mathbf{L}}{dt}\right)_{body} +\boldsymbol{\omega}\times\mathbf{L}
$$
**第四步：代入$ \mathbf{L}=I\boldsymbol{\omega}$**
$$
\boldsymbol{\tau}= \left(\frac{d(I\boldsymbol{\omega})}{dt}\right)_{body} +\boldsymbol{\omega}\times(I\boldsymbol{\omega})
$$
**第五步：刚体 + body惯量常数 ⇒ $\frac{d}{dt}(I\omega)=I\dot\omega$**
$$
\boldsymbol{\tau}=I\dot{\boldsymbol{\omega}} +\boldsymbol{\omega}\times(I\boldsymbol{\omega})
$$

> ✅ 你必须理解：**耦合项不是“额外的物理力”，而是“你在旋转坐标系里描述同一个物理定律所必须出现的项”**。
>
> $ω×(Iω)$ 不是“额外出现的神秘力矩”，它描述的是：因为你站在一个正在旋转的坐标系里观察，角动量向量会“看起来在转”，为了让角动量按物理规律变化，需要补上这部分等效变化。
> 这就是“陀螺效应/耦合”的根源。

当机体系刚好对准**主惯量轴**，那么惯量矩阵是对角阵：
$$
I= \begin{bmatrix} I_{xx}&0&0\\ 0&I_{yy}&0\\ 0&0&I_{zz} \end{bmatrix}
$$
角速度：
$$
\omega= \begin{bmatrix} p\\q\\r \end{bmatrix}
$$
**第一步：算 $I\omega$**
$$
I\omega= \begin{bmatrix} I_{xx}p\\ I_{yy}q\\ I_{zz}r \end{bmatrix}
$$
**第二步：算叉乘 $\omega\times(I\omega)$**

叉乘的分量公式（这是标准定义）：
 若 $a=(a_1,a_2,a_3), b=(b_1,b_2,b_3)$，则
$$
a\times b= \begin{bmatrix} a_2b_3-a_3b_2\\ a_3b_1-a_1b_3\\ a_1b_2-a_2b_1 \end{bmatrix}
$$
所以：
$$
\omega\times(I\omega)= \begin{bmatrix} (I_{zz}-I_{yy})qr\\ (I_{xx}-I_{zz})rp\\ (I_{yy}-I_{xx})pq \end{bmatrix}
$$
**第三步：把欧拉方程按分量写出来**
$$
\tau = I\dot{\omega} + \omega\times(I\omega)
$$
其中
$$
I\dot{\omega}= \begin{bmatrix} I_{xx}\dot p\\ I_{yy}\dot q\\ I_{zz}\dot r \end{bmatrix}
$$
所以分量相加得到：
$$
\begin{aligned} \tau_x &= I_{xx}\dot p + (I_{zz}-I_{yy})qr \\ \tau_y &= I_{yy}\dot q + (I_{xx}-I_{zz})rp \\ \tau_z &= I_{zz}\dot r + (I_{yy}-I_{xx})pq \end{aligned}
$$

### 主惯量轴

一般情况下，惯量矩阵不是对角的，而是：
$$
I= \begin{bmatrix} I_{xx}&-I_{xy}&-I_{xz}\\ -I_{xy}&I_{yy}&-I_{yz}\\ -I_{xz}&-I_{yz}&I_{zz} \end{bmatrix}
$$
其中$ I_{xy}, I_{xz}, I_{yz} $叫“惯量积/积惯量”，它们不为 0 就意味着：你选的坐标轴不是“最舒服的轴”。

**主惯量轴**就是这样一组互相垂直的轴，使得在这组轴下惯量矩阵变成对角阵（惯量积全为 0）：
$$
I= \begin{bmatrix} I_1&0&0\\ 0&I_2&0\\ 0&0&I_3 \end{bmatrix}
$$
这三个$ I_1,I_2,I_3$叫**主惯量（principal moments of inertia）**。

惯量矩阵 $I $是对称矩阵 ⇒ 一定可以找一组正交特征向量：

- 特征向量方向 = **主惯量轴方向**
- 特征值 = **主惯量大小**

这一步推导属于线性代数的标准定理，你不需要从头证明，但你需要会用：

> **“对称矩阵能正交对角化”**

### 耦合项的物理意义

#### 最直觉类比

**推门（近似一维）**

门的质量分布相对铰链轴很简单，你施加力矩后：

- 角速度方向 = 铰链轴方向
- 角动量方向也基本沿铰链轴
   所以不会出现“别的轴也动”的感觉。

**不对称哑铃/书本（典型三维刚体）**

如果你拿一本书在空中转（尤其绕“中间那条轴”），你会看到它会**开始“翻滚/摆动”**，不是干净地绕一个轴一直转。这就是“带起来”的直观现象（在物理上叫网球拍定理/中间轴不稳定）。角动量的方向，代表了物体**“目前正在死守的旋转面”**。

背后的原因不是“凭空多出力”，而是：

> 质量分布不一样，导致“转起来想保持的东西（角动量）”方向和“你此刻转的方向（角速度）”不一致。

#### 为什么“会把别的轴带起来”？

因为$ L $想保持不变，但机体系在转，一个关键定律：在没有外力矩（$\tau=0$）时，
$$
\left(\frac{dL}{dt}\right)_{\text{惯性系}}=0
$$
意思是：**在空间里，角动量向量 $L $的方向保持不变**（像一根指向固定星空方向的箭头）。

但你用机体系（跟着物体转的坐标系）去描述它时，机体系本身在转，所以你“看到”的 $L$ 坐标会变。

这在数学上就是：
$$
\left(\frac{dL}{dt}\right)_{\text{惯性系}} = \left(\frac{dL}{dt}\right)_{\text{机体系}} +\omega\times L
$$
所以：
$$
\left(\frac{dL}{dt}\right)_{\text{机体系}} = -\,\omega\times L
$$
**这句话的直觉翻译：**

> 因为机体系在绕 $\omega$ 转，相对的，所以在机体系里，L 会“绕着 $\omega$ 转起来”。
> 而如果 $L$ 又不和 $\omega$ 同方向，那它的变化就会有 y、z 分量，于是你会感觉“别的轴也被带起来了”。

牛顿告诉我们：动量的变化率 = 力（力矩）。
$$
\frac{dL}{dt} = \tau
$$
**螺旋力矩：**

- 虽然在上帝看来 $L$ 没动。
- 但在你（机体）看来，Y 轴上的 $L_y$ 分量突然变大了（树转到右手边了）。
- 你会惊呼：“卧槽！谁推了我的 Y 轴？哪里来的力矩？”
- **这个让你感觉 Y 轴被推了一把的“幽灵力矩”，就是 $\omega \times (I\omega)$。**

## 3.陀螺力矩

### 为什么“转子会额外产生力矩”？

一个高速自转的转子（电机+桨）就像一个小陀螺。它有一个很大的“自旋角动量” $H$。

- 你让机体姿态变化（机体角速度 $\omega$不为 0），等价于你在“转动转子自旋轴的方向”
- 但高速自旋的东西**不喜欢**你去改变它的自旋轴方向（这不是意志，是力学）
- 为了让它的角动量方向跟着你转，就必须施加力矩；反过来它会对机体施加**反作用力矩**

这就是“陀螺力矩”。

------

### 这些符号分别在干什么？

我们把每个转子 $i$ 的量写清楚（都用 body 坐标系表达）：

**机体角速度**
$$
\omega \in \mathbb{R}^3
$$
就是你 IMU 的 $p,q,r$。

**转子自旋轴方向（单位向量）**
$$
\mathbf{n}_i \in \mathbb{R}^3,\quad \|\mathbf{n}_i\|=1
$$
对常见多旋翼，$\mathbf{n}_i$基本就是机体系的 $+z$ 或 $−z$。

**转子自旋角速度（标量）**
$$
\Omega_i \in \mathbb{R}
$$
单位 rad/s。注意它是**标量**（带正负号表示顺/逆时针）。

**转子绕自旋轴的转动惯量（标量）**
$$
J_r \in \mathbb{R}
$$
单位 kg·m²。它是“转子绕自己转轴转起来有多难”。

**转子自旋角动量（向量）**
$$
\mathbf{H}_i = J_r \Omega_i \mathbf{n}_i
$$
单位 N·m·s。方向沿转子轴 $\mathbf{n}_i$，大小跟 $\Omega_i$ 成正比。

------

### 陀螺力矩的核心来源（掌握）

刚体里最基本的关系是：
$$
\boldsymbol{\tau} = \left(\frac{d\mathbf{H}}{dt}\right)_{\text{惯性系}}
$$
但我们常在 body 系里写 $\mathbf{H}_i$。旋转坐标系求导有公式：
$$
\left(\frac{d\mathbf{H}}{dt}\right)_{\text{惯性系}} = \left(\frac{d\mathbf{H}}{dt}\right)_{\text{body}} + \omega \times \mathbf{H}
$$
对“工程简化”的陀螺项，我们通常先忽略 $\Omega_i$的快速变化（也就是忽略 $\dot\Omega_i$ 对这一项的贡献），把 $\mathbf{H}_i$ 视为在 body 系里近似常值方向（沿 $\mathbf{n}_i$）：
$$
\left(\frac{d\mathbf{H}_i}{dt}\right)_{\text{body}} \approx 0
$$
于是：
$$
\left(\frac{d\mathbf{H}_i}{dt}\right)_{\text{惯性系}} \approx \omega\times \mathbf{H}_i
$$
这表示：要让转子角动量方向跟着机体角速度 $\omega$ 转过去，你必须提供一个力矩大小约为 $\omega\times \mathbf{H}_i$。

上面推出来的是“**作用在转子角动量上所需的外力矩**”。但你在机体动力学里要加的是“**转子对机体的反作用力矩**”，方向相反：

- 需要施加在转子上的力矩：
  $$
  \tau_{\text{on rotor}} \approx \omega\times \mathbf{H}_i
  $$

- 转子反作用到机体上的力矩：

$$
\tau_{\text{on body}} \approx -\,\omega\times \mathbf{H}_i
$$

利用叉乘反对称性 $\omega\times H = H\times \omega$，所以你也可以写成：
$$
\tau_{\text{gyro, body}} \approx \mathbf{H}_i \times \omega
$$

### 工程实现

更清晰、工程可实现的写法是：
$$
\boxed{ \tau_{\text{gyro, body}} \approx \sum_{i=1}^{N} \big(J_r \Omega_i \mathbf{n}_i\big)\times \omega }
$$
实现步骤（每个时间步）：

1.计算每个转子的角动量向量
$$
\mathbf{H}_i = J_r \Omega_i \mathbf{n}_i
$$
2.计算每个转子的陀螺反作用力矩（加到机体上）
$$
\tau_{\text{gyro},i} = \mathbf{H}_i \times \omega
$$
3.求和
$$
\tau_{\text{gyro}} = \sum_i \tau_{\text{gyro},i}
$$
总力矩进入欧拉方程
$$
\tau_{\text{total}} = \tau_{\text{cmd}} + \tau_{\text{gyro}} \quad (\text{再加别的：气动、重力梯度等})
$$
你已经有了欧拉方程的左边（惯性项）和右边（力矩项）。经典欧拉方程：
$$
I \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (I \boldsymbol{\omega}) = \boldsymbol{\tau}_{\text{total}}
$$
把你自己推导的项代进去，就是这个**终极方程**：
$$
I \dot{\boldsymbol{\omega}} + \underbrace{\boldsymbol{\omega} \times (I \boldsymbol{\omega})}_{\text{机体自身陀螺项}} = \underbrace{\boldsymbol{\tau}_{\text{cmd}}}_{\text{舵面/喷气}} + \underbrace{\sum (\mathbf{H}_i \times \boldsymbol{\omega})}_{\text{转子陀螺反作用力}} + \boldsymbol{\tau}_{\text{其他}}
$$

- **左边**：是你这块铁疙瘩（卫星/飞机）的运动状态变化。
- **右边**：是所有揍它的力（你自己给的控制力 + 转子给的捣乱力 + 气动重力等）。

------

我们现在的目标是求出 **$\dot{\boldsymbol{\omega}}$（角加速度）**。因为知道了加速度，积分就能得到速度，再积分就是位置。

我们把方程变换一下，把 $\dot{\boldsymbol{\omega}}$ 孤立出来：

1. 把 $\boldsymbol{\omega} \times (I \boldsymbol{\omega})$ 移到等号右边：
   $$
   I \dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{\text{cmd}} + \sum (\mathbf{H}_i \times \boldsymbol{\omega}) - \boldsymbol{\omega} \times (I \boldsymbol{\omega}) + \boldsymbol{\tau}_{\text{其他}}
   $$

2. 两边同乘惯量矩阵的逆矩阵 $I^{-1}$：
   $$
   \dot{\boldsymbol{\omega}} = I^{-1} \left[ \boldsymbol{\tau}_{\text{cmd}} + \underbrace{\sum (\mathbf{H}_i \times \boldsymbol{\omega})}_{\text{转子捣乱}} - \underbrace{\boldsymbol{\omega} \times (I \boldsymbol{\omega})}_{\text{自身捣乱}} + \boldsymbol{\tau}_{\text{其他}} \right]
   $$

**这就是计算机每一毫秒都在疯狂计算的公式！**

------

算出了此刻的角加速度 $\dot{\boldsymbol{\omega}}$ 之后，计算机就开始“算命”了（推演下一刻的状态）。

假设时间步长 $dt = 0.01s$（10毫秒）：

1. 更新角速度 ($\omega$)：
   $$
   \boldsymbol{\omega}_{\text{new}} = \boldsymbol{\omega}_{\text{old}} + \dot{\boldsymbol{\omega}} \times dt
   $$
   (现在的转速 = 刚才的转速 + 加速度 $\times$ 时间)

2. 更新姿态 (角度/四元数)：
   $$
   \text{Angle}_{\text{new}} = \text{Angle}_{\text{old}} + \boldsymbol{\omega}_{\text{new}} \times dt
   $$
   (现在的角度 = 刚才的角度 + 速度 $\times$ 时间)

------

在这个最终的 $\dot{\boldsymbol{\omega}}$ 公式里，你能看到两个“陀螺项”在打架：
$$
\dot{\boldsymbol{\omega}} = I^{-1} [ \dots + \mathbf{H}_i \times \boldsymbol{\omega} - \boldsymbol{\omega} \times I \boldsymbol{\omega} \dots ]
$$

1. **$\mathbf{H}_i \times \boldsymbol{\omega}$**：这是**转子**给你的反作用力（我们刚才讨论半天的那个）。
2. **$-\boldsymbol{\omega} \times (I \boldsymbol{\omega})$**：这是**机体本身**因为形状不对称（$I$ 不是标量）产生的欧拉耦合力。