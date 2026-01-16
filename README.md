# ARCS: General Simulation Platform for Task Resilience Control of Swarms

### (Active Resilience Control for Swarms)

**Defining a General Research Paradigm for Swarm Survivability and Mission Collaboration in Hostile Environments**

> **[English Document](./README.md)** | **[Chinese Document](./README_CN.md)**
> **Official Implementation**: The paper "Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics" (Submitted to *Reliability Engineering & System Safety*).

---

<div align="center">
<img src="./assets/header_demo.gif" width="800px" alt="ARCS Simulation Demo">





<em>Figure: ARCS builds an open research platform for "Interference Perception — Capability Metric — Resilience Control", supporting simulation experiments shifting from "Passive Efficiency Optimization" (Center) to "Active Resilience Control" (Right).</em>
</div>

---

## 📖 1. ARCS Platform: General Theory and Simulation Architecture

ARCS aims to break the limitations of single algorithms and establish a general scientific paradigm of **"Perception-Metric-Control"**. This paradigm **derives from our fundamental research on swarm mission collaboration mechanisms in complex interference environments [1]**, aiming to provide a standardized research foundation for the field.

### 1.1 The Scientific Paradigm: Defining the "Perception-Metric-Control" Triad

The platform abstracts swarm resilience research into a coupled problem of three core dimensions, supporting modular exploration:

1. **Perception Dimension: Interference Perception**
* **Scientific Challenge**: The essence of complex environments is the presence of **interference fields** with unknown spatiotemporal distributions. Individual agents are limited by observation range and lack distributed collaborative advantages.
* **Platform Definition**: Utilizing the spatiotemporal distribution characteristics of the swarm, treating it as a "Distributed Sensor Array". ARCS supports various collaborative estimation algorithms (e.g., STCL-NN) to achieve dynamic reconstruction of interference models, parameters, and **Confidence**.


2. **Metric Dimension: Task Capability Metrics**
* **Scientific Challenge**: Physical layer interference (e.g., SNR drop) is difficult to directly guide decision-layer planning. A quantitative mapping mechanism from "Environmental Interference" to "Task Consequences" is needed.
* **Platform Definition**: ARCS introduces the concept of **"Equivalent Task Capability"** and **Interference Damage Dynamics Equations**. It quantifies multi-source heterogeneous interference as the physical decay of the swarm's ability to complete functional tasks, thereby achieving real-time measurement of task resilience metrics.


3. **Control Dimension: Active Resilience Control**
* **Scientific Challenge**: Systems often fall into a zero-sum game between "Task Efficacy" and "Task Resilience". Traditional methods often oscillate between passive avoidance (high time cost) or blind traversal (high risk).
* **Platform Definition**: ARCS establishes the control philosophy of **"Resilience as a State"**. The platform supports designing active resilience control laws that dynamically seek the Pareto Frontier of optimal efficacy while ensuring a minimum capability baseline (resilience constraint).



### 1.2 The Core Architecture: Closed-Loop Framework

To translate the above scientific paradigm into an executable simulation system, ARCS establishes a standardized **Three-Layer Closed-Loop Architecture** (as shown in Fig. 1).

<div align="center">
<img src="./assets/theory_framework_EN.jpg" width="800px" alt="Theoretical Framework">
<em>Fig 1: Schematic of the Standardized ARCS Three-Layer Closed-Loop Framework</em>
</div>

* **Input End**: Receives physical field information from the "Complex Interference Environment" and distributed sensor data from the "Swarm System".
* **Core Processing Layers**:
* **Collaborative Perception Layer**: Acts as the information gateway, transforming raw sensor data into structured environmental knowledge (Estimates + Confidence).
* **Metric & Mapping Layer**: Acts as the **"Bridge"** connecting the physical world and the control world. It maps environmental information to the system's current **"Task Resilience State ()"** via damage dynamics models.
* **Active Resilience Control Layer**: Acts as the decision brain, performing dynamic trade-offs (Pareto optimization) between efficacy targets and safety baselines based on current resilience states.


* **Execution & Feedback End**: Control commands update agent states, which in turn alter their observation perspectives, forming a dynamic coupled closed-loop.

---

**💡 Foundational Work & Premier Instantiation**

The **first complete theoretical construction and mathematical implementation** of the above general framework is detailed in our foundational paper **[1]**:

* **Perception Layer Implementation**: Proposed the **STCL-NN** (Spatiotemporal Collaborative Localization Neural Network) algorithm.
* **Metric Layer Implementation**: Established a resilience metric model based on **Damage Dynamics Equations**.
* **Control Layer Implementation**: Designed an optimal controller based on **PMP (Pontryagin's Minimum Principle)**.

> **[1]** *Zeng Y, Zhuang X, Li J, et al. Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics. Reliability Engineering & System Safety, 2025 (Under Review).*

---

## 🏗️ 2. Premier Instantiation: Baseline Scenario & Implementation

This chapter details the **Premier Instantiation** based on our foundational paper.

### 2.1 The Baseline Scenario

ARCS includes the classic "Swarm Traversal and Search" task scenario from the paper:

* **Task Goal**: A formation of 5 UAVs must traverse a  km unknown area to reach a destination, preserving enough "Effective Functional Payload" to complete subsequent missions.
* **Dynamics Constraints**: For every second exposed to the interference field, UAVs suffer irreversible physical decay of task capability according to specific **Damage Dynamics Equations**.

**The Dilemma & Core Challenge:**

<div align="center">
<img src="assets/dilemma_diagram_EN.jpg" width="600px" alt="Decision Dilemma">
<em>Fig 2.1: Typical Decision Dilemma in Unknown Interference Environments</em>
</div>

> **❓ Core Scientific Challenge**
> Under conditions of uncertain and complex interference, is there a control paradigm that can **continuously guarantee the system's task capability baseline (Resilience)** while achieving **high task execution efficiency (Efficacy)**?

### 2.2 Implementation & File Organization

ARCS employs a modular design, precisely mapping the paper's algorithms to the general framework (as shown in Fig. 2).

<div align="center">
<img src="./assets/algImplement.png" width="800px" alt="Implementation Architecture">
<em>Fig 2: Schematic of the Premier Instantiation from STCL-NN Perception to PMP Resilience Control</em>
</div>

**File Organization:**

```text
/ARCS_Root
├── Main.m                  % [Entry] Simulation Entry: Scenario config & Benchmark scheduling
├── ResilienceSim.m         % [Engine] Simulation Core: Discrete time stepping & Data dispatch
├── Modules/                % [Components] Core Algorithm Implementation
│   ├── STCL_NN.m           % [Perception] Collaborative Localization (STCL-NN Implementation)
│   ├── calcResilience.m    % [Metric] Damage Dynamics Calculation (Resilience Metrics Implementation)
│   ├── flyController.m     % [Control] PMP Optimal Controller (Control Law Implementation)
│   └── ...                 % [Tools] Visualization scripts
└── Results/                % [Data] Storage for generated .mat files
└── Figs/                   % [Charts] Storage for generated .fig files

```

---

## 🧩 3. Core Algorithms & Implementation

This section provides a detailed analysis of the engineering implementation of the ARCS core modules. The code structure strictly follows the **"Perception-Metric-Control"** closed-loop theoretical framework.

### 3.1 Simulation Engine Architecture: Closed-Loop Scheduling Mechanism
**Core Module**: `ResilenceSim.m` (Simulation Engine)

`ResilenceSim.m` is the core scheduler of the entire simulation system, coordinating data exchange between various sub-modules.

**System Dataflow**:

<div align="center">
    <img src="assets/systemFlowchart_CN.png" width="80%" alt="systemDataflow">
    <br>
    <em>Figure 4: ARCS Simulation Engine System Dataflow (ARCS Simulation Engine Architecture)</em>
</div>

### 3.2 Collaborative Perception Layer: STCL-NN Module Implementation
**Module Correspondence**: `Modules/STCL_NN_Real.m`

To solve the interference source localization problem in strong noise environments, this project implements the **STCL-NN (Spatiotemporal Collaborative Learning Neural Network)** module proposed in the paper.

STCL-NN is a novel **feedforward neural network based on mechanism weighting**.

Unlike deep learning models that rely on backpropagation (BP) for parameter iteration, STCL-NN returns to the essence of neural network **"layered information processing"**, achieving training-free perception through **structural isomorphism** and **mechanism embedding**:

* **Topology**: Completely retains the multi-layer feedforward structure of deep neural networks: **"Convolutional Feature Extraction $\to$ Attention Screening $\to$ Nonlinear Regression"**.
* **Weighting**: Abandons high-cost data training and directly maps physical field equations (spatiotemporal integration, attenuation consistency) to the network layer's **fixed weights and activation operators**.

This design possesses both the powerful **nonlinear mapping capability** of neural networks and the **full interpretability** of physical models, making it a typical practice of the "AI for Science" concept in the field of swarm perception.

As shown in Figure 3 (Fig. 3 in Paper), the code strictly follows the three-stage processing pipeline:

#### 3.2.1 Stage 1: Distributed Feature Extraction and Spatiotemporal Compression
The paper proposes using a lightweight CNN for "spatiotemporal collaborative compression trajectory" mapping. At the code implementation level, a feature extractor based on **sliding window convolution operators** is constructed to capture gradient changes in local signals.

| Paper Term | Code Function | Mechanism Analysis |
| :--- | :--- | :--- |
| **Gated Acquisition** | `STCL_NN_Real`<br>(Main Loop) | **Gated Acquisition Mechanism**: <br>Introduces `MIN_ATTENUATION` threshold judgment, activating recording only when the monitored signal strength exceeds the noise floor, corresponding to the event-triggering mechanism in the paper. |
| **CNN Feature Compression** | `optimizedSlidingWindow` | **Spatiotemporal Convolution**: <br>Uses a sliding window to execute convolution operations along the time dimension. This process extracts cumulative energy and trend slopes within the window, mapping raw physical data to the "Spatiotemporal Collaborative Compression Trajectory" described in the paper. |

#### 3.2.2 Stage 2: Attention-based Collaborative Sample Screening
To eliminate NLOS noise and outliers, the code implements a soft screening strategy based on **Attention Mechanism**, achieving data purification by calculating confidence weights of multi-dimensional features.

| Paper Term | Code Function | Mechanism Analysis |
| :--- | :--- | :--- |
| **Feature Embedding** | `calculateWindowConfidence` | **Feature Embedding**: <br>The code extracts two types of high-dimensional features to calculate attention scores: <br>1. **Consistency Features**: The degree to which signal attenuation conforms to physical laws; <br>2. **Spatial Features**: The geometric distribution diversity of sampling points. |
| **Attention Selection** | `selectDiverseHighConfidencePoints` | **Attention Screening**: <br>Calculates the "Information Confidence Weight" for each sampling point. Based on the Top-M strategy described in the paper, the code dynamically screens the top $M$ frames of data with the highest confidence to construct a High-confidence Dataset. |

#### 3.2.3 Stage 3: Nonlinear Identification and Confidence Evaluation
This stage reverts to the physical model to complete the precise inversion from feature space to physical parameters.

| Paper Term | Code Function | Mechanism Analysis |
| :--- | :--- | :--- |
| **MLE Solution** | `nonlinearLeastSquaresFit` | **MLE Solver**: <br>Based on the screened dataset, uses the **LM (Levenberg-Marquardt)** algorithm to iteratively solve the nonlinear least squares problem, rapidly converging to obtain the optimal interference parameters $\hat{\xi}$. |
| **Confidence Quantification** | `calculateFinalConfidence` | **Multi-dimensional Confidence Quantification**: <br>Synthesizes MLE fitting residuals (model conformity) and spatial geometric configuration (observation completeness) to generate a global confidence score $C \in [0,1]$, directly driving subsequent resilience control decisions. |

---

### 3.3 Dynamic Metric Layer: Damage Dynamics & Resilience Calculation
**Module Correspondence**: `Modules/calcResilience.m` (Core Algorithm) & `ResilenceSim.m` (Scheduling)
**Paper Section**: Section 2.2 (Metric Layer)

This module is responsible for establishing the dynamic mapping between "physical interference" and "task capability". To address the lag problem of traditional static indicators, the code implements a **predictive metric framework based on "Ghost Simulation"**.

#### 3.3.1 Damage Dynamics Equation
The code implements the differential equation described in Eq. (14) of the paper:
$$\frac{\mathrm{d}\eta}{\mathrm{d}t} = -\beta \cdot [C(t) \cdot \hat{s}(t)] \cdot \eta(t) - \gamma \cdot \eta(t)$$

**Core Logic**: Mapping physical field intensity to irreversible capability loss.

| Paper Term | Code Function | Mechanism Analysis |
| :--- | :--- | :--- |
| **Damage Evolution**<br>(Eq. 14) | `updateDamageFactors` | **Discretized Euler Integration**: <br>The code implements the numerical solution of the differential equation via `newDamage = currentDamage + (damageRate - recoveryRate) * dt`. Here, `damageRate` is deeply coupled with **perception confidence** (`Ck`) and **physical field intensity** (`interferenceStrength`), achieving a robust design where "updates are moderate if perception is inaccurate". |
| **Self-Recovery** | `initializeDamageParameters` | **Elastic Recovery Mechanism**: <br>Introduces the `gamma_recovery` parameter to simulate the system's self-repair capability. The code logic ensures that after flying away from the interference zone, system capability recovers exponentially, consistent with physical reality. |

#### 3.3.2 Model Prediction-based Performance Factor $\sigma(t)$
**Core Logic**: Fusing "historical observation" and "future prediction" to construct a forward-looking state variable.

The code does not use simple linear extrapolation but constructs an **accelerated predictor** (`predictFuturePayloads`). This predictor utilizes the currently perceived interference model parameters $\hat{\xi}$ to deduce the swarm's future survival state in a virtual space-time.

| Paper Term | Code Function | Mechanism Analysis |
| :--- | :--- | :--- |
| **Active Prediction**<br>(Model Predictive) | `predictFuturePayloads` | **"Simulation-based" Finite Horizon State Extrapolation**: <br>1. **Predictive Simulation Step**: Uses `predDt = 5 * dt` for coarse-grained rapid deduction; <br>2. **Virtual Mapping**: Reconstructs a virtual interference field using the **estimated parameters** ($\hat{\alpha}, \hat{\beta}, \hat{d_0}$) output by the perception layer; <br>3. **Dynamic Termination**: Automatically stops when the virtual swarm flies out of the interference zone or reaches the maximum prediction horizon (`maxPredictionTime`). |
| **Dynamic Weighted Fusion**<br>(Eq. 18) | `calculateResilienceMetrics` | **Confidence Gated Fusion**: <br>The calculation formula for $\sigma(t)$ is as follows: <br>`sigma = ((1-conf)*Hist + conf*Pred) / Target`<br>**Mechanism Highlight**: Uses confidence `conf` as a weighting factor. When perception is unreliable, it degrades to relying on historical data; when perception is precise, it favors future prediction, thereby achieving an optimal balance between robustness and foresight. |

### 3.4 Resilience Control Layer: Multi-Mode Flight Controller
**Module Correspondence**: `Modules/flyController.m`

`flyController.m` integrates three comparison strategies, switched via `simParams.CtrlMode`.

#### 3.4.1 Benchmark Algorithm Implementation
* **C0: Open-loop Baseline**
    * **Parameter**: `CtrlMode = 0`
    * **Logic**: Retains formation keeping only, does not evade interference, tests raw lethality.
* **C1: Artificial Potential Field (APF)**
    * **Parameter**: `CtrlMode = 1`
    * **Logic**: Traditional reactive obstacle avoidance, generating virtual repulsive forces.

#### 3.4.2 Proposed Algorithm: PMP Active Resilience Control
* **Mode Parameter**: `simParams.CtrlMode = 3`
* **Core Mechanism**: **State machine switching** based on $\sigma(t)$ and **differential trend compensation**.

This module implements the optimal control law derived from Eq. (25) in the paper. Distinct from passive threshold triggering, the code introduces the $\dot{\sigma}(t)$ term, achieving **active anticipation and suppression** of performance decay trends.

**1). Control Law Implementation Mechanism Analysis**

| Theoretical Term | Code Logic | Physical Meaning |
| :--- | :--- | :--- |
| **PMP Costates ($\lambda_p, \lambda_s$)** | `efficiencyWeight` | **Costate Weight Allocation**: <br>The code discretizes abstract costate variables into piecewise functions of $\sigma(t)$, dynamically adjusting the weights of task and survival. |
| **Active Trend Prediction** | `sigma_dot > 0` | **Differential Feedforward (D-Term)**: <br>When a performance recovery trend is detected, efficiency weight is actively released. This corresponds to the anticipatory characteristic of costate equations evolving backward in time. |
| **Hamiltonian Min ($u^*$)** | `desiredDir` | **Optimal Heading Synthesis**: <br>Synthesizes the optimal control variable $u^*$ at the current moment through vector weighting, i.e., finding the optimal tangent point on the Pareto frontier. |

**2). Core Implementation Snippet**

```matlab
% --- A. PMP State Machine Weight Solution (PMP State Machine) ---
% Calculate deviation and rate of change relative to baseline sigma0
sigma_deviation = sigma_t - sigma0;
sigma_dot = (sigma_t - prev_sigma) / dt;

% Regime 1: Sufficient Performance (Consume redundancy, accelerate task)
if sigma_t >= sigma0
    target_efficiency = 0.95; 

% Regime 2: Critical Resilience (Dynamic balance, PMP linear mapping)
elseif sigma_t >= sigma_l 
    delta_sigma = (sigma0 - sigma_t) / (sigma0 - sigma_l);
    target_efficiency = max(0.55, 0.85 - 0.3 * delta_sigma); 

% Regime 3: Emergency Protection (Bottom-line priority)
else 
    target_efficiency = 0.45; 
end

% --- B. Active Trend Compensation ---
% Key Innovation: If performance is recovering (sigma_dot > 0), release efficiency weight early
% This simulates the dynamic prediction characteristic of PMP costate variables, preventing "excessive avoidance"
if sigma_dot > 0 && sigma_t < sigma0
    alpha = 0.6; % Pre-adjustment coefficient
    target_efficiency = target_efficiency + alpha * sigma_dot * (1.0 - target_efficiency);
    target_efficiency = min(0.95, target_efficiency); 
end
```
---

## 📊 4. ARCS Benchmark

ARCS provides a standardized set of experiments, integrating the full-process comparative experiments from Chapter 4 of the paper.

### 4.1 Experiment 1: Collaborative Perception Performance

**Command**: `Main('exp01fig01', true)`

Evaluates STCL-NN localization accuracy under high noise.

* **Conclusion**: Validates the **"Cooperative Gain"** mechanism. As nodes increase, STCL-NN leverages spatial diversity to suppress noise, converging error to  km.

<div align="center"> <img src="assets/exp01Fig01.png" width="600px" alt="Perception Result">


<em>Figure 5: Comparison of Interference Source Localization Results</em> </div>


### 4.2 Experiment 2: Dynamic Resilience Measurement Verification

**Command**: `Main('exp02', true)`

Validates the fidelity and predictive capability of the Damage Dynamics model.

<div align="center"> <img src="assets/exp01Fig02A.png" width="600px" alt="Perception Result">


<em>Figure 6: Correlation between various confidence indicators and localization error</em> </div> <div align="center"> <img src="assets/exp02Fig02B.png" width="600px" alt="Perception Result">


<em>Figure 7: Real-time predictive measurement of mission resilience</em> </div>

* **Conclusion**: The predictive indicator  provides early warning of degradation risks and sensitively reflects survival potential improvements from heading adjustments, demonstrating **"Forward-looking"** assessment capabilities.

### 4.3 Experiment 3: Control Strategy Comparison

**Command**: `Main('exp03fig01' ~ 'exp03fig04', true)`

| Exp ID | Strategy | Typical Behavior | Scientific Conclusion |
| --- | --- | --- | --- |
| `'exp03fig01'` | **Uncontrolled** | ⏱️ Fastest / 📉 **Mission Failure** | Proves "Efficiency First" leads to system collapse. |
| `'exp03fig02'` | **APF** | 🛡️ Payload Intact / ⚡ **Severe Jitter** | Proves local avoidance is overly conservative and inefficient. |
| `'exp03fig03'` | ** Control** | 📉 **Oscillation** | Proves global metric feedback causes gradient coupling. |
| `'exp03fig04'` | **PMP- (Ours)** | ✅ **Optimal Balance** | Achieves **Pareto Optimality** between efficacy and resilience. |

The flight trajectories and mission resilience performance indicators of various control strategies are shown in Figures 8~11. 
For details, please refer to the foundational paper [1].
<div align="center"><img src="assets/exp02Fig02B.png" width="600px" alt="Perception Result">
    <em>Figure 8: No control experiment results (high efficiency but severe payload loss). 
Vertical and horizontal red dashed lines indicate swarm entry time and mission requirement baseline respectively.</em>
</div>

<div align="center"><img src="assets/exp03Fig02AB.png" width="600px" alt="Perception Result">
    <em>Figure 9: APF autonomous interference avoidance results (high safety cost and excessive maneuvering). 
Vertical and horizontal red dashed lines indicate swarm entry time and mission requirement baseline respectively.</em>
</div>

<div align="center"><img src="assets/exp03Fig03AB.png" width="600px" alt="Perception Result">
    <em>Figure 10: Active elastic optimal control results based on $R(t)$ (control oscillation and insufficient payload protection). 
Vertical and horizontal red dashed lines indicate swarm entry time and mission requirement baseline respectively.</em>
</div>

<div align="center"><img src="assets/exp03Fig04AB.png" width="600px" alt="Perception Result">
    <em>Figure 11: Active elastic optimal control results based on $\sigma(t)$ (optimal balance of smoothness, payload and efficiency). 
Vertical and horizontal red dashed lines indicate swarm entry time and mission requirement baseline respectively.</em>
</div>

The table below compares and analyzes the four control strategies from multiple dimensions: mission completion time, payload variation, 
control strategy trigger times, and mission resilience indicator satisfaction.
The experimental results show that the proposed strategy achieves the highest effective payload lower limit (21.77) and the lowest number of control triggers (867).
Crucially, it maintains a relatively short mission completion time (1294.7 s), only 1.25% longer than the theoretical lower bound (no control).
This verifies the core logic of our framework:By accurately regulating the dynamic performance factor $\sigma(t)$ as a real-time controlled variable, 
the system implicitly ensures that the comprehensive elasticity $R(t)$ remains above the safety baseline $R^\*$ throughout the mission.
This confirms that the control anchored on $\sigma(t)$ successfully achieves **the trade-off optimization between "resilience demand" and "control efficiency"**, 
realizing high mission resilience at marginal efficiency cost.

Table: Comparison of Key Performance Indicators of Four Control Strategies

| Control Strategy | Mission Completion Time (s) | Effective Payload Range | Control Trigger Times | $R(t) \ge R^\*$ Satisfaction Rate |
| :--- | :---: | :---: | :---: | :---: |
| No Control | 1278.7 | [14.73, 30.00] | 0 | 0% |
| APF Control | 1321.9 | [27.88, 30.00] | 2416 | 100% |
| $R(t)$ Control | 1293.2 | [18.28, 30.00] | 2279 | 40% |
| $\sigma(t)$ Control (Proposed) | 1294.7 | [21.77, 30.00] | 867 | 100% |

---

## 📝 5. Citation

ARCS is an open research project. If you use this code or extend new scenarios based on this framework, please cite our foundational work:

```bibtex
@article{zeng2025active,
  title={Active Resilience Control for UAV Swarms: A Closed-Loop Framework Integrating Collaborative Perception and Dynamic Metrics},
  author={Zeng, Yifan and Zhuang, Xuebin and Li, Jinning and Wu, Meng},
  journal={Reliability Engineering \& System Safety},
  year={2025}
}

```

## 📧 6. Contact

This project is developed and maintained by **Yifan Zeng**.
If you have questions about the code, find bugs, or are interested in academic collaboration, please contact:

* **Personal Email (Permanent)**: [zyfkd@qq.com](mailto:zyfkd@qq.com) (Recommended)
* **Academic Email**: [zengyf29@mail2.sysu.edu.cn](mailto:zengyf29@mail2.sysu.edu.cn)
* **Requirements**: MATLAB R2020b or higher (Pure .m implementation, no toolboxes required).
* **Copyright**: © 2025 School of Systems Science and Engineering, Sun Yat-sen University (SYSU). MIT License.