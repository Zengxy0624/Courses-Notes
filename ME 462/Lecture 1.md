# Continuous-Time (CT) Systems: Nonlinear to Linear Approximation  

## 1. System Model  
\[
\dot{x} = f(x,u), \quad y = g(x,u)
\]  

- \(x\): state vector  
- \(u\): input  
- \(y\): output  

---

## 2. Augmented Vector  
\[
r = \begin{bmatrix} x \\ u \end{bmatrix}, \quad 
r_s = \begin{bmatrix} x_s \\ u_s \end{bmatrix}, \quad 
\Delta r = r - r_s
\]

---

## 3. Taylor Expansion (Linearization)  
\[
\dot{x} \approx f(x_s,u_s) + 
\left.\frac{\partial f}{\partial r}\right|_{r_s}(r-r_s) + H.O.T
\]  

\[
y \approx g(x_s,u_s) + 
\left.\frac{\partial g}{\partial r}\right|_{r_s}(r-r_s)+H.O.T
\]

- Higher-order terms (H.O.T.) are negligible near equilibrium.  
- Jacobian Expansion in Linearization of State Equation Jacobian:
\[
\left.\frac{\partial f}{\partial r}\right|_{r_s}
=
\left[
\;\left.\frac{\partial f}{\partial x}\right|_{(x_s,u_s)}
\;\;\;
\left.\frac{\partial f}{\partial u}\right|_{(x_s,u_s)}
\right]
=
\big[\,A\;\;B\,\big]
\]
---

## 4. Equilibrium Condition  
\[
f(x_s, u_s) = 0, \quad \dot{x}_s = 0
\]

---

## 5. Linearized State-Space Form  
Define deviations:  
\[
\Delta x = x - x_s,\quad \Delta u = u - u_s,\quad \Delta y = y - y_s
\]

Resulting linear model:  
\[
\dot{\Delta x} = A \Delta x + B \Delta u
\]  
\[
\Delta y = C \Delta x + D \Delta u
\]

where  
\[
A = \frac{\partial f}{\partial x}\Big|_{(x_s)}, \quad 
B = \frac{\partial f}{\partial u}\Big|_{(u_s)}, \quad 
C = \frac{\partial g}{\partial x}\Big|_{(x_s)}, \quad 
D = \frac{\partial g}{\partial u}\Big|_{(u_s)}
\]

---



Now neglect \(H.O.T.\) (higher-order terms) and rewrite in terms of deviations from equilibrium \((x_s,u_s)\):  



## 6. State Equation  
\[
\dot{x} = f(x,u) \approx f(x_s,u_s) + 
\left.\frac{\partial f}{\partial r}\right|_{r_s} \Delta r
\]

Since \(f(x_s,u_s)=0\):  
\[
\dot{x} - f(x_s,u_s) \approx 
\left.\frac{\partial f}{\partial r}\right|_{r_s}\Delta r
\]
Since assume small error

\[
\dot{x} - \dot{x}_s = \Delta \dot{x} =
\left.\frac{\partial f}{\partial r}\right|_{r_s}\Delta r
\]

---

## 7. Output Equation  
\[
y = g(x,u) \approx g(x_s,u_s) + 
\left.\frac{\partial g}{\partial r}\right|_{r_s} \Delta r
\]

Since \(y_s=g(x_s,u_s)\):  
\[
y - g(x_s,u_s) \approx 
\left.\frac{\partial g}{\partial r}\right|_{r_s}\Delta r
\]
Since assume small error
\[
y - y_s = \Delta y =
\left.\frac{\partial g}{\partial r}\right|_{r_s}\Delta r
\]

---

## 8. Result increments with respect to the equilibrium point

- Further decomposing this equation in terms of **x** and **u** components of *r*, yields:

\[
\Delta \dot{x} = \left.\frac{\partial f}{\partial x}\right|_{x = x_s} (x - x_s) + \left.\frac{\partial f}{\partial u}\right|_{u = u_s} (u - u_s)
\]


\[
\Delta y = \left.\frac{\partial g}{\partial x}\right|_{x = x_s} \cdot (x - x_s) + \left.\frac{\partial g}{\partial u}\right|_{u = u_s} \cdot (u - u_s)
\]

其中：
- \(\Delta x = (x - x_s)\)
- \(\Delta u = (u - u_s)\)

---

## 图示说明
- **Actual function**: 原始非线性函数 \( f(r) \)
- **Linear approximation**: 在平衡点 \( r_s \) 附近的线性化近似  

# 9. state-space notation

### 1) 变量与维度（在平衡点 \((x_s,u_s)\) 处取值）
$$\left.\frac{\partial f}{\partial x}\right|_{(x_s,u_s)}=A\in\mathbb{R}^{n\times n}$$
$$\left.\frac{\partial f}{\partial u}\right|_{(x_s,u_s)}=B\in\mathbb{R}^{n\times m}$$
$$\left.\frac{\partial g}{\partial x}\right|_{(x_s,u_s)}=C\in\mathbb{R}^{p\times n}$$
$$\left.\frac{\partial g}{\partial u}\right|_{(x_s,u_s)}=D\in\mathbb{R}^{p\times m}$$

> 记 \(x\in\mathbb{R}^n,u\in\mathbb{R}^m,y\in\mathbb{R}^p\)。上面四个矩阵是雅可比在 \((x_s,u_s)\) 的数值。
>-\ $\mathbb{R}^{n \times n}$：所有 $n \times n$ 的实矩阵的集合。
 $\mathbb{R}^{n}$：所有长度为 $n$ 的实列向量（等价于 $n \times 1$ 矩阵）。在矩阵乘法里默认是列向量。
---

## 2) 非线性系统与在 \((x_s,u_s)\) 的线性近似
$$\dot{x}=f(x,u)$$
$$y=g(x,u)$$

> 令 \(\Delta x=x-x_s,\Delta u=u-u_s,\Delta y=y-g(x_s,u_s)\)。

$$\Delta\dot{x}=A\,\Delta x+B\,\Delta u$$
$$\Delta y=C\,\Delta x+D\,\Delta u$$

---

## 3) 连续时间 LTI 状态空间（坐标平移后去掉 Δ）

$$
\Sigma_{\text{CTLTI}}=\left\{\begin{array}{l}
\dot{x}=A\cdot x+B\cdot u,\quad x(t_0)=x_0\\[2pt]
y=C\cdot x+D\cdot u
\end{array}\right.
$$
>CTLTI 是 Continuous Time Linear Time-Invariant
---

## 4) 针对本页的补充与思考

- **为什么是这四个矩阵**：\(A,B,C,D\) 分别度量在工作点处 \(f,g\) 对状态/输入的一阶敏感度，决定了局部线性动力学与测量关系。
- **维度检查（形状规则）**：若 \(A\in\mathbb{R}^{a\times b},X\in\mathbb{R}^{b\times c}\)，则 \(AX\in\mathbb{R}^{a\times c}\)。因此 \(A(n\times n)\cdot\Delta x(n\times1)\to\mathbb{R}^{n}\)，\(B(n\times m)\cdot\Delta u(m\times1)\to\mathbb{R}^{n}\)，\(C(p\times n)\cdot\Delta x(n\times1)\to\mathbb{R}^{p}\)，\(D(p\times m)\cdot\Delta u(m\times1)\to\mathbb{R}^{p}\)。
- **为何要线性化**：把非线性系统在工作点附近化为 LTI，便于用极点配置、LQR、可控/可观、卡尔曼滤波等标准工具；且 \(A,B,C,D\) 为常数，分析与实现简单。
- **Δ的作用与去除**：先用 \(\Delta\) 表示“相对平衡点的增量”，再把工作点作为新原点（变量重命名）即可得到标准 LTI 形式。
- **适用范围**：忽略高阶项仅在“小邻域、小扰动”内有效；偏离工作点较大时需重新线性化或改用非线性/增益调度方法。
- **关于 \(D\)**：如果输出不直接包含输入的瞬时通道（无直通项），在很多系统中 \(D=0\)。

# 10. space state to transfer function
**单边拉普拉斯变换记号：**  
$$x(t)\xleftrightarrow{\mathcal{L}}X(s),\quad y(t)\xleftrightarrow{\mathcal{L}}Y(s),\quad u(t)\xleftrightarrow{\mathcal{L}}U(s),\quad \mathcal{L}\{\dot{x}\}=sX(s)-x(0)$$

**对状态空间取拉普拉斯（允许 \(x(0)\neq0\)）：**  
$$sX(s)-x(0)=A\cdot X(s)+B\cdot U(s)$$
$$Y(s)=C\cdot X(s)+D\cdot U(s)$$

**把标量 \(s\) 升阶为同型矩阵 \(sI\)，并解出 \(X(s)\)：**  
$$(sI-A)\cdot X(s)=x(0)+B\cdot U(s)$$
$$X(s)=(sI-A)^{-1}\cdot x(0)+(sI-A)^{-1}\cdot B\cdot U(s)$$

**代回输出得到总响应与传递函数矩阵：**  
$$Y(s)=C\cdot(sI-A)^{-1}\cdot x(0)+[C\cdot(sI-A)^{-1}\cdot B+D]\cdot U(s)$$
$$\boxed{G(s)=C\cdot(sI-A)^{-1}\cdot B+D}$$
![Summary](<Figure/Figure 1.1.png>)
**零输入/零状态分解：**  
- 零输入响应（初值引起）：$$Y_{\text{ZI}}(s)=C\cdot(sI-A)^{-1}\cdot x(0)$$
- 零状态响应（输入引起）：$$Y_{\text{ZS}}(s)=G(s)\cdot U(s)$$
- 总响应：$$Y(s)=Y_{\text{ZI}}(s)+Y_{\text{ZS}}(s)$$

**思考：为什么是 \(sI-A\) 与可逆性？**  
- 标量 \(s\) 需与 \(A\) 同型，故写 \(sI\)。  
- 可逆性：存在 \((sI-A)^{-1}\) 等价于 \(\det(sI-A)\neq0\)；当 \(\det(sI-A)=0\)（\(s\) 为 \(A\) 的特征值）时不可逆，这些 \(s\) 就是系统的**极点**。

**思考：为什么要做拉普拉斯变换？**  
- 把微分方程变为代数方程（便于求解）。  
- 把时域卷积变乘法（\((sI-A)^{-1}B\cdot U(s)\)）。  
- 单边变换自然带入初始条件 \(x(0)\)。  
- 直接得到传递函数矩阵 \(G(s)\) 用于频域分析与互联系统设计。

---
# 11. CT linear systems: general solution（整理 + 分析）

## 1) 卷积视角（因果 LTI，零初始条件）
**频域乘法 → 时域卷积：**
$$X_{\text{out}}(s)=H(s)X_{\text{in}}(s)$$
$$\Longrightarrow\ x_{\text{out}}(t)=\mathcal{L}^{-1}\{H(s)X_{\text{in}}(s)\}$$
$$=\int_{0}^{t}h(\tau)x_{\text{in}}(t-\tau)\,d\tau=\int_{0}^{t}x_{\text{in}}(\tau)h(t-\tau)\,d\tau$$

> **分析**  
> - \(h(t)=\mathcal{L}^{-1}\{H(s)\}\) 为**冲激响应**（零初始，输入 \(\delta(t)\) 时的输出）。  
> - 上式是单边卷积（下限 0），体现**因果性**与**零初始条件**。

---

## 2) 从状态空间的 \(Y(s)\) 逆变换到时域
**已知（上一页）**：
$$Y(s)=C(sI-A)^{-1}x(0)+\big[C(sI-A)^{-1}B\big]U(s)$$

**利用矩阵逆拉普拉斯**：
$$\mathcal{L}^{-1}\{(sI-A)^{-1}\}=e^{At}$$

**得到一般输出解**（不写直通项 \(D\) 的版本）：
$$y(t)=C e^{At}x(0)+\int_{0}^{t}C e^{A(t-\tau)}B\,u(\tau)\,d\tau$$

> **分析**  
> - 第一项 \(C e^{At}x(0)\) 是 **Free response（零输入/自由响应）**，只由初始状态引起。  
> - 第二项 \(\int_{0}^{t}C e^{A(t-\tau)}B\,u(\tau)\,d\tau\) 是 **Forced response（零状态/强迫响应）**，由输入卷积核 \(C e^{A(t-\tau)}B\) 加权叠加而成。  
> - 若存在直通项 \(D\)，则**完整形式**为：  
>   $$y(t)=C e^{At}x(0)+\int_{0}^{t}C e^{A(t-\tau)}B\,u(\tau)\,d\tau+D\,u(t)$$
>   此时冲激响应为 \(h(t)=C e^{At}B+D\,\delta(t)\)；若 \(D=0\) 则 \(h(t)=C e^{At}B\)。

---

## 3) 关键对应关系（一眼记住）
- **传递函数 ↔ 冲激响应**：\(H(s)=\mathcal{L}\{h(t)\}\)，\(h(t)=\mathcal{L}^{-1}\{H(s)\}\)。  
- **强迫响应（零状态）**：\(y_{\text{ZS}}(t)=\int_{0}^{t}h(t-\tau)u(\tau)\,d\tau\)。  
- **自由响应（零输入）**：\(y_{\text{ZI}}(t)=C e^{At}x(0)\)。  
- **总响应**：\(y(t)=y_{\text{ZI}}(t)+y_{\text{ZS}}(t)\)。

---
# 12. Embedding initial conditions into the input as an impulsive disturbance  
**CT linear systems — system response to an impulsive input disturbance**

---

## 1) 目标与直觉
- 把**初始条件的影响**并入**输入端**，用在 \(t=0\) 的**冲激**来等效，这样总响应可统一写成**卷积**：核 \(h(t)\) 作用于“真实输入 + 冲激输入”。

---

## 2) 例子与设定（讲义一阶系统）
- 系统：$$\dot{x}=a\,x+u,\quad y=x,\quad x(0)\ \text{给定}$$
- 其冲激响应（因 \(y=x,\ B=C=1,\ D=0\)）：$$h(t)=e^{a t}$$

---

## 3) 从一般解出发（零输入 + 零状态）
- 总响应：$$y(t)=e^{a t}\,x(0)+\int_{0}^{t} e^{a(t-\tau)}\,u(\tau)\,d\tau$$
  - 第一项：自由响应（Zero-Input）  
  - 第二项：强迫响应（Zero-State）= 卷积 \(h*u\)

---

## 4) 用 δ 把“初值项”改写成卷积并合并
- 取样性质 \(\int f(\tau)\delta(\tau)\,d\tau=f(0)\) 给出：$$e^{a t}x(0)=\int_{0}^{t} e^{a(t-\tau)}\,x(0)\,\delta(\tau)\,d\tau$$
- 合并成单个卷积：$$y(t)=\int_{0}^{t} e^{a(t-\tau)}\,[\,x(0)\delta(\tau)+u(\tau)\,]\,d\tau$$
- 定义等效输入：$$u_1(\tau)=x(0)\delta(\tau)+u(\tau)$$
- 于是：$$y(t)=\int_{0}^{t} e^{a(t-\tau)}\,u_1(\tau)\,d\tau=\int_{0}^{t} e^{a(t-\tau)}\,u(\tau)\,d\tau+\int_{0}^{t} e^{a(t-\tau)}\,x(0)\delta(\tau)\,d\tau$$

> **注释（对应图中的蓝色标注）**  
> - \(x(0)\delta(\tau)\)：**Impulsive input disturbance**（把初值视作输入端在 \(t=0\) 的冲激）  
> - \(\int_{0}^{t} e^{a(t-\tau)}u(\tau)d\tau\)：**System response to input**  
> - \(\int_{0}^{t} e^{a(t-\tau)}x(0)\delta(\tau)d\tau\)：**System output disturbance due to the impulsive input**

---

## 5) 为什么成立？（关键点）
- **状态跳变推导**：在 \(t=0\) 的无穷小区间积分 \(\dot{x}=a x+u_1\)，有 \(x(0^+)-x(0^-)=\int u_1\,dt\approx\int x(0)\delta(t)\,dt=x(0)\)。因此“零初始 + 冲激输入”与“有初始 + 无冲激”在 \(t>0\) 的演化等价。  
- **因果/单边**：积分下限 0 反映因果系统与单边拉普拉斯的设定。  
- **与卷积/传递函数一致**：零状态响应恒为 \(h*u\)；把 \(x(0)\delta(t)\)并入输入后，自然产生自由响应项 \(h(t)\,x(0)\)。

---

## 6) 推广与备注
- 一般 CTLTI：\(\dot{x}=A x+B u,\ y=C x+D u\) 时 $$y(t)=C e^{A t}x(0)+\int_{0}^{t} C e^{A(t-\tau)}B\,u(\tau)\,d\tau + D\,u(t)$$
- 若要把初值并入输入端，严格写法需通过 \(B\) 注入相应方向；当 \(D\neq0\) 时，冲激响应含 \(D\delta(t)\)。  
- 工程直觉：初值像在 \(t=0\) 对系统“瞬间一推”，之后按内在动力学 \(e^{a t}\)（或 \(e^{A t}\)）传播。

---

# 13. Regulation as rejection of the output disturbance induced by the impulsive input (state) disturbance

## 1) 概念与任务
- **Regulation（调节）**：在设定值附近工作时，**抑制外部扰动对系统输出的影响**，把系统拉回平衡点。
- 若系统在 \(t=t_1\) 被扰动离开稳态 \(x_s\) 而到达 \(x(t_1)\)，可把 \(x(t_1)\) 视为**新的初始条件**；调节器需要把系统从该新初值拉回平衡。

---

## 2) 初始条件等效为“冲激型输入（状态）扰动”
- 因果 CT LTI 一阶示例：\(\dot x=ax+u,\ y=x\)，其冲激响应为 \(h(t)=e^{a t}\)。
- 把在 \(t_1\) 的状态突变等效为输入端的**冲激** \(x(t_1)\delta(\tau-t_1)\)：
  $$y_{\text{out dist}}(t)=\int_{t_1}^{t} e^{a(t-\tau)}\,x(t_1)\,\delta(\tau-t_1)\,d\tau = x(t_1)\,e^{a(t-t_1)}$$
- 这里用到 **δ 的筛选性质（sifting property）**：\(\int f(\tau)\delta(\tau-t_1)\,d\tau = f(t_1)\)。

> 结论：**输出扰动 = 冲激输入经系统冲激响应传播后的结果**；对一阶例子就是 \(x(t_1)e^{a(t-t_1)}\)。

---

## 3) 一般 CTLTI 形式（假设 \(D=0\)）
- 对 \(\dot x=Ax+Bu,\ y=Cx\)：
  $$y_{\text{dist}}(t)=\int_{t_1}^{t} C e^{A(t-\tau)}\,x(t_1)\,\delta(\tau-t_1)\,d\tau = C\,e^{A(t-t_1)}\,x(t_1)$$

---

## 4) 控制设计含义（为什么这页重要）
- **调节目标**：让由“状态冲击”引起的输出扰动 \(y_{\text{dist}}(t)\) **尽快衰减到 0**（小超调、短整定时间）。
- 反馈 \(u=-Kx\) 后闭环矩阵 \(A_{\text{cl}}=A-BK\)；选择闭环极点使 \(e^{A_{\text{cl}}(t-t_1)}\) 衰减更快（连续时间：极点更靠左；离散时间：模更小）。

---

## 5) 一眼带走的关键式（单行）
- \(y_{\text{out dist}}(t)=\int_{t_1}^{t} e^{a(t-\tau)}\,x(t_1)\,\delta(\tau-t_1)\,d\tau = x(t_1)e^{a(t-t_1)}\)
- \(y_{\text{dist}}(t)=C\,e^{A(t-t_1)}\,x(t_1)\)（一般 CTLTI，\(D=0\)）

> 这页本质在说：**非零初始条件 ≙ 在某时刻的冲激扰动**；而**调节**就是**拒绝**这种由冲激引起的输出响应。
# 14. Controller design as a realization of the **Internal Model Principle**

## 1) 观察“输出扰动”的传播
- 由上一页得到（以一阶为例）：  
  **输出扰动**：$$y_{\text{out dist}}(t)=e^{a(t-t_1)}\,x(t_1)$$  
  **一般 CTLTI**：$$y_{\text{dist}}(t)=C\,e^{A(t-t_1)}\,x(t_1)$$
- 含义：初始条件（或在 \(t_1\) 处的**冲激型输入/状态扰动**）会沿系统自身的动力学传播。一阶时由常数 \(a\) 决定；一般情形由矩阵 \(A\) 决定。

---

## 2) Internal Model Principle（内部模型原理，IMP）
> **定理要点**：**要实现对某一类外部信号的完美跟踪/拒斥，控制器必须内含该类信号的模型。**

- 追踪/拒斥“常值” → 控制器内含**积分器**（\(s=0\) 的模型）。  
- 追踪/拒斥“正弦” → 控制器内含**谐振/振荡器**（\(s=\pm j\omega\) 的模型）。  
- **本页结论**：当扰动是“**冲激引起的状态偏移**”时，**扰动的模型就是系统自身的动力学**（由 \(A\) 或 \(a\) 决定）。

---

## 3) 为什么控制器需要“内含系统动力学”？
- 扰动效应的形状是 \(e^{A(t-t_1)}\)；若控制器不含这一“内在模式”，就无法生成**幅度与相位都匹配**的反作用来抵消它。  
- 因此，为把系统从设定偏差拉回，调节器（regulator）应包含**系统模型**（或其复制/估计）以针对性抵消由冲激（或新初值）引起的输出扰动。

> 这正是**基于模型控制（model-based control）**的基础思想：在控制器中显式或隐式携带被控对象的模型（典型做法是**全阶观测器 + 状态反馈**，或 LQR/LQG、极点配置等）。

---

## 4) 设计含义（实践指南）
- **闭环极点**：通过反馈把闭环极点放得更“靠左”（连续时间）或模更小（离散时间），使 \(e^{A_{\text{cl}}(t-t_1)}\) 衰减更快。  
- **内部模型的具体形态**：  
  - 常值扰动 → 在控制器里加一阶积分器。  
  - 正弦扰动 → 在控制器里嵌入二阶振荡器 \(s^2+\omega^2\)。  
  - 冲激型状态扰动 → 在控制器/观测器中复制被控对象的 **A** 动力学，以便生成匹配的补偿并实现快速衰减。  
- **估计问题**：扰动未知时，用观测器 \( \dot{\hat x}=A\hat x+Bu+L(y-C\hat x) \) 复制 \(A\) 的内部模式，配合反馈 \(u=-K\hat x\) 达到抑制效果。

---

## 5) 小结（一眼记住）
- 扰动响应形状：\(y_{\text{dist}}(t)=C\,e^{A(t-t_1)}\,x(t_1)\)。  
- **IMP**：想把某类信号“压没”，控制器里必须**包含该类信号的模型**。  
- 对“冲激引起的状态偏移”，这类信号的模型就是**被控对象的动力学本身** → 需要**基于模型**的调节器（含对象模型/观测器）。
# 15. Controller design as a realization of the **Internal Model Principle** (IMP)

## 1) 输出扰动如何传播（由系统自身动力学决定）
- 一阶示例的输出扰动：$$y_{\text{out dist}}(t)=e^{a(t-t_1)}\,x(t_1)$$
- 一般 CTLTI：$$y_{\text{dist}}(t)=C\,e^{A(t-t_1)}\,x(t_1)$$
> 含义：在时刻 \(t_1\) 的状态/输入“冲击”会沿**被控对象的动力学**传播（由 \(a\) 或 \(A\) 决定）。

---

## 2) 内部模型原理（IMP）— 核心表述
> **要对某类外部信号实现完美跟踪/拒斥，控制器必须内含该类信号的模型。**
- 常值扰动 → 控制器需要**积分器**（在 \(s=0\) 含极点）。
- 正弦扰动 → 控制器需要**谐振对**（在 \(s=\pm j\omega\) 含极点）。
- 本页情形：扰动的“模型”就是对象的**系统动力学** \(A\)。

---

## 3) 设计含义（为什么控制器要“带着模型”）
- 扰动响应的模式是 \(e^{A(t-t_1)}\)；若控制器不含此模式，就难以生成**相位与增益**都匹配的补偿来抵消它。
- 因此要把系统从偏差拉回，调节器应在内部**包含对象模型**（或其复制/估计）以实现针对性的拒斥。

---

## 4) 典型实现路径
- **观测器 + 状态反馈（在控制器里复制 \(A\)）**  
  观测器：\(\dot{\hat x}=A\hat x+Bu+L(y-C\hat x)\)，反馈：\(u=-K\hat x\)，闭环：\(\dot x=(A-BK)x\)。  
  扰动衰减：$$y_{\text{dist}}(t)=C\,e^{(A-BK)(t-t_1)}\,x(t_1)$$
  通过选极点使 \(A_{\text{cl}}=A-BK\) 的特征值足够靠左（或离散时模足够小），令扰动快速衰减。
- **其他内部模型例子**  
  - 抑制常值：在控制器中加入积分器。  
  - 抑制正弦：在控制器中加入频率为 \(\omega\) 的谐振器。  

---

## 5) 极简一阶算例（看见“极点位移”）
- 对象：\(\dot x=a x+u,\ y=x\)；选反馈 \(u=-k x\)。  
- 开环扰动：$$y_{\text{out dist}}(t)=e^{a(t-t_1)}\,x(t_1)$$
- 闭环扰动：$$y_{\text{out dist}}(t)=e^{(a-k)(t-t_1)}\,x(t_1)$$
> 选择 \(k>a\) 令 \(a-k\ll0\)，扰动衰减更快；若 \(x\) 不可测，用观测器在控制器内**复制 \(A=a\)**。

---

## 6) 一眼带走
- 扰动形状：\(y_{\text{dist}}(t)=C\,e^{A(t-t_1)}\,x(t_1)\)（由对象动力学决定）。  
- **IMP**：想完美拒斥哪类信号，就把该类信号的**模型**放进控制器。  
- 这正是**基于模型控制（model-based control）**的依据：控制器内含对象模型（或其副本），才能有效抵消由该模型激发的扰动。
