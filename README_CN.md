# ARCS: 通用集群任务韧性控制仿真平台
### (Active Resilience Control for Swarms)

**定义对抗环境下集群生存与任务协同的通用研究范式**

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-blue)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-MIT-green)](https://opensource.org/licenses/MIT)
[![Paper](https://img.shields.io/badge/Status-Under--Review-orange)]()

> **[English Document]** | **[中文文档]**
>
> **官方实现**: 论文 "Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics" (已投稿至 *Reliability Engineering & System Safety*).

---

<div align="center">
  <img src="./assets/header_demo.gif" width="800px" alt="ARCS Simulation Demo">
  <br>
  <em>图示：ARCS 构建了一个开放的"干扰感知—任务能力度量—韧性控制"研究平台，支持从"被动效能优化"（中）转向"主动韧性控制"（右）的仿真实验。</em>
</div>

---

## 📖 1. ARCS 平台：主动韧性控制的通用理论与仿真架构

ARCS 旨在打破单一算法的局限，建立一套通用的**"感知—度量—控制"**科学范式。该范式**源于我们对集群在复杂干扰环境下任务协同机理的基础研究 [1]**，旨在为该领域提供标准化的研究底座。

### 1.1 科学范式：定义"感知—度量—控制"三元组 (The Scientific Paradigm)

本平台将集群韧性研究抽象为三个核心维度的耦合问题，支持研究者在此框架下进行模块化探索：

1.  **感知维度：干扰感知 (Interference Perception)**
    * **科学挑战**：复杂环境的本质是存在时空分布未知的**干扰场**。单体智能受限于观测范围，缺乏集群级的分布式协同优势。
    * **平台定义**：利用集群的时空分布特性，将其视为"分布式传感器阵列"。ARCS 支持接入各类协同估计算法（如 STCL-NN），实现对干扰模型、参数及**置信度 (Confidence)** 的动态重构。

2.  **度量维度：任务能力度量 (Task Capability Metrics)**
    * **科学挑战**：物理层的干扰（如信噪比下降）难以直接指导决策层的规划。我们需要建立从"环境干扰"到"任务后果"的定量映射。
    * **平台定义**：ARCS 引入**"等效任务能力"**概念和**干扰损伤动力学方程**。它将多源异构的干扰统一量化为能力的物理损耗，实现任务韧性指标的实时度量。

3.  **控制维度：主动韧性控制 (Active Resilience Control)**
    * **科学挑战**：系统往往陷入"任务效能"与"任务韧性"的零和博弈。传统方法常处于被动规避（高耗时）或盲目穿越（高风险）的极端。
    * **平台定义**：ARCS 确立了**"韧性即状态 (Resilience as a State)"**的控制理念。平台支持设计主动韧性控制律，在保证最低能力底线（韧性约束）的前提下，动态寻找效能最优的帕累托前沿。

### 1.2 核心架构：主动韧性控制闭环原理框架 (The Core Architecture)

为了将上述科学范式转化为可执行的仿真系统，ARCS 建立了一个标准化的**三层闭环架构**（如图 1 所示）。

<div align="center">
  <img src="./assets/theory_framework_CN.jpg" width="800px" alt="Theoretical Framework">
  <br>
  <em>图 1：标准化的 ARCS 三层闭环框架原理图</em>
</div>

* **输入端**：接收来自"复杂干扰环境"的物理场信息和"集群系统"的分布式传感数据。
* **核心处理层**：
    * **协同干扰感知层**：将杂乱的原始传感数据转化为结构化的环境知识（估计值 + 置信度）。
    * **任务能力的度量与映射层**：通过干扰损伤动力学模型，将环境信息映射为**"任务韧性状态 ($\sigma$)"**。
    * **主动韧性控制层**：基于韧性状态，在效能目标与安全底线之间进行动态权衡（帕累托优化）。
* **执行与反馈端**：控制指令更新智能体状态，进而改变其观测视角，形成动态耦合闭环。

---

**💡 学术起源与典范实现 (Foundational Work)**
上述通用框架的**首次完整理论构建与数学实现**详见我们的基础论文 **[1]**：
* **感知层**：提出了 **STCL-NN** (时空协同定位神经网络) 算法。
* **度量层**：建立了基于**损伤动力学方程**的韧性度量模型。
* **控制层**：设计了基于 **PMP (庞特里亚金极小值原理)** 的最优控制器。

> **[1]** *Zeng Y, Zhuang X, Li J, et al. Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics. Reliability Engineering & System Safety, 2025 (Under Review).*

---

## 🏗️ 2. 典范实例构建：基础场景与框架实现

本章详细阐述基于基础论文的**首次典范实例构建 (Premier Instantiation)**。

### 2.1 基础场景定义 (The Baseline Scenario)

ARCS 内置了论文中经典的"集群穿越与搜索"任务场景：
* **任务目标**：5 架无人机编队穿越 $50 \times 50$ km 的未知干扰区。
* **动力学约束**：无人机依据**损伤动力学方程**遭受不可逆的任务能力衰减。

**决策困境与核心挑战 (The Dilemma & Core Challenge):**

<div align="center">
  <img src="assets/dilemma_diagram_CN.jpg" width="600px" alt="Decision Dilemma">
  <br>
  <em>图 2.1：未知干扰环境下的典型决策困境</em>
</div>

<br>

> **❓ 核心科学挑战 (Core Scientific Challenge)**
>
> 在不确定性的复杂干扰条件下，是否存在一种控制范式，能够在**持续保障系统任务能力底线（韧性）**的同时，实现**任务执行的高效性（效能）**？

### 2.2 框架实现与代码组织 (Implementation)

ARCS 采用模块化设计，将论文算法精准映射到通用框架中（如图 2）。

<div align="center">
     <img src="./assets/algImplement.png" width="800px" alt="Implementation Architecture">
     <br>
     <em>图 2：从 STCL-NN 干扰感知到 PMP 韧性控制的典范实例原理图</em> 
</div>

**代码文件组织结构 (File Organization):**

```text
/ARCS_Root
├── Main.m                  % [主程序] 仿真入口：负责场景配置与调度
├── ResilienceSim.m         % [引擎]   仿真内核：负责时间步进与数据分发
├── Modules/                % [组件库] 核心算法实现
│   ├── STCL_NN.m           % [感知层] 协同定位算法 (STCL-NN 实现)
│   ├── calcResilience.m    % [度量层] 损伤动力学计算 (Resilience Metrics 实现)
│   ├── flyController.m     % [控制层] PMP 最优控制器 (Control Law 实现)
│   └── ...                 % [绘图工具] 各类结果可视化脚本
└── Results/                % [数据仓] 存放仿真生成的 .mat 文件
└── Figs/                   % [图表仓] 存放生成的 .fig 图片

```

---

## 🧩 3. 核心算法详解与源码映射 (Core Algorithms & Implementation)

本节解析 ARCS 核心模块的工程实现，代码结构严格遵循 **"Perception-Metric-Control"** 闭环理论框架。

### 3.1 仿真引擎架构：闭环调度机制

**核心模块**：`ResilenceSim.m` (Simulation Engine)

`ResilenceSim.m` 是整个仿真系统的核心调度器，协调各子模块的数据交换。

**系统运行流程图 (System Dataflow)**:

<div align="center">
<img src="assets/system_flowchart.png" width="80%" alt="System Dataflow">





<em>Fig 3.1: ARCS 仿真引擎闭环数据流图 (Simulation Engine Architecture)</em>
</div>

### 3.2 协同感知层：STCL-NN 模块实现

**模块对应**：`Modules/STCL_NN_Real.m`

为了解决强噪声环境下的干扰源定位问题，本项目实现了论文提出的 **STCL-NN** 模块。该模块采用**"物理引导（Physics-Guided）"的混合架构**。

如图 3 (Fig. 3 in Paper) 所示，代码严格遵循三阶段处理流水线：

#### 3.2.1 Stage 1: 分布式特征提取与时空压缩

代码构建了基于**滑动窗口卷积算子**的特征提取器。

| 论文概念 (Paper Term) | 源码函数 (Code Function) | 实现机制解析 (Mechanism) |
| --- | --- | --- |
| **Gated Acquisition** | `STCL_NN_Real` | **门控采集机制**：<br>

<br>引入 `MIN_ATTENUATION` 阈值，仅当信号超过底噪时激活记录。 |
| **CNN Feature Compression** | `optimizedSlidingWindow` | **时空卷积 (Convolution)**：<br>

<br>利用滑动窗口提取累积能量与趋势斜率，映射为"时空协同压缩轨迹"。 |

#### 3.2.2 Stage 2: 基于注意力的协同样本筛选

代码实现了一套基于 **注意力机制** 的软筛选策略。

| 论文概念 (Paper Term) | 源码函数 (Code Function) | 实现机制解析 (Mechanism) |
| --- | --- | --- |
| **Feature Embedding** | `calculateWindowConfidence` | **特征嵌入**：<br>

<br>提取 **一致性特征** (衰减规律符合度) 和 **空间特征** (几何分布多样性)。 |
| **Attention Selection** | `selectDiverse...Points` | **注意力筛选**：<br>

<br>计算"信息置信度权重"，结合 Top-M 策略构建高可信数据集。 |

#### 3.2.3 Stage 3: 非线性辨识与置信度评估

回归物理模型，完成精确反演。

| 论文概念 (Paper Term) | 源码函数 (Code Function) | 实现机制解析 (Mechanism) |
| --- | --- | --- |
| **MLE Solution** | `nonlinearLeastSquaresFit` | **MLE 求解器**：<br>

<br>使用 **LM 算法** 迭代求解非线性最小二乘问题，得到参数 。 |
| **Confidence** | `calculateFinalConfidence` | **多维置信度量化**：<br>

<br>综合拟合残差和空间构型，生成全局置信度 。 |

---

### 3.3 动态度量层：损伤动力学与韧性计算

**模块对应**：`Modules/calcResilience.m`

#### 3.3.1 损伤动力学方程 (Damage Dynamics)

代码实现了论文 Eq. (14) 描述的微分方程：
$$ \frac{\mathrm{d}\eta}{\mathrm{d}t} = -\beta \cdot [C(t) \cdot \hat{s}(t)] \cdot \eta(t) - \gamma \cdot \eta(t) $$

* **实现逻辑**: 采用离散化 Euler 方法。其中  对应损伤系数， 对应感知置信度。

#### 3.3.2 动态性能因子 (Dynamic Performance Factor)

* **实现逻辑**: 计算论文定义的实时性能因子 。代码融合了历史观测与未来预测，评估当前时刻的能力余量。

---

### 3.4 韧性控制层：多模式飞行控制器

**模块对应**：`Modules/flyController.m`

`flyController.m` 集成了三种对比策略，通过 `simParams.CtrlMode` 切换。

#### 3.4.1 基准算法实现 (Benchmark Implementation)

* **C0: 无控制基准 (Open-loop Baseline)**
* **参数**: `CtrlMode = 0`
* **逻辑**: 仅保留编队保持，不规避干扰，测试原始杀伤力。


* **C1: 人工势场法 (APF)**
* **参数**: `CtrlMode = 1`
* **逻辑**: 传统的反应式避障，产生虚拟排斥力。



#### 3.4.2 提议算法：PMP 主动韧性控制 (Proposed PMP Control)

* **参数**: `CtrlMode = 3`
* **逻辑**: 实现了基于 ** 状态机** 的最优控制律。

```matlab
% PMP Optimization Logic (Simplified)
if sigma_t >= sigma0       % Regime A: Efficiency-First
    target_efficiency = 0.95; 
elseif sigma_t >= sigma_l  % Regime B: Resilience-Critical
    delta_sigma = (sigma0 - sigma_t)/(sigma0 - sigma_l);
    target_efficiency = max(0.55, 0.85 - 0.3*delta_sigma); 
end

```

---

## 📊 4. ARCS Benchmark (基准测试)

ARCS 提供了标准化的实验集，代码集成了论文第 4 章的全流程对比实验。

### 4.1 实验一：协同感知性能评估 (Collaborative Perception)

**运行指令**: `Main('exp01fig01', true)`

评估 STCL-NN 算法在强噪声下的定位精度。

* **结论**: 验证了 **"协同增益"** 机制。随着接入节点增加，STCL-NN 利用空间多样性有效抑制噪声，最终误差收敛至  km。

<div align="center">
<img src="assets/exp01fig01.png" width="600px" alt="Perception Result">





<em>Fig 4.1: 干扰源定位结果对比</em>
</div>

### 4.2 实验二：动态韧性度量验证 (Dynamic Resilience Measurement)

**运行指令**: `Main('exp02', true)`

验证损伤动力学模型的保真度及预测能力。

* **结论**: 预测指标  能够提前预警退化风险，并敏锐反映航向调整带来的生存潜力提升，具备**"前瞻性"**评估能力。

### 4.3 实验三：韧性控制策略对比 (Control Strategy Comparison)

**运行指令**: `Main('exp03fig01' ~ 'exp03fig04', true)`

| 实验 ID | 策略名称 | 典型表现 | 科学结论 |
| --- | --- | --- | --- |
| `'exp03fig01'` | **无控制** | ⏱️ 最快 / 📉 **任务失败** | 证明了"效率优先"会导致系统能力崩溃。 |
| `'exp03fig02'` | **APF** | 🛡️ 载荷完好 / ⚡ **剧烈抖动** | 证明了局部避障过于保守，严重牺牲效率。 |
| `'exp03fig03'` | ** 控制** | 📉 **控制振荡** | 证明直接反馈全局指标会导致梯度耦合问题。 |
| `'exp03fig04'` | **PMP- (本文)** | ✅ **最优平衡** | 实现了效能与韧性的 **帕累托最优**。 |

<div align="center">
<img src="assets/benchmark_comparison.png" width="800px" alt="Control Strategy Comparison">





<em>Fig 4.3: 不同控制策略下的轨迹与载荷曲线对比</em>
</div>

---

## 📝 5. 引用与致谢

ARCS 是一个开放的科研项目，如果您在研究中使用了本代码库，请引用我们的基础工作：

```bibtex
@article{zeng2025active,
  title={Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics},
  author={Zeng, Yifan and Zhuang, Xuebin and Li, Jinning and Wu, Meng},
  journal={Reliability Engineering \& System Safety},
  year={2025}
}

```

## 📧 6. 作者与联系方式 (Contact)

本项目由 曾逸凡 (Yifan Zeng) 开发与维护。 如果您对代码实现有疑问、发现 Bug 或有进一步的学术合作意向，欢迎通过以下方式联系：

  个人邮箱 (Permanent): zyfkd@qq.com (推荐，长期有效)      学术邮箱 (Academic): zengyf29@mail2.sysu.edu.cn


* **环境要求**：MATLAB R2020b 或更高版本（纯 .m 实现，无工具箱依赖）。
* **版权**：© 2025 中山大学 系统科学与工程学院 (SYSU). 遵循 MIT 协议。

```

```