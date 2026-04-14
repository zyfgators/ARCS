function [newDirection, controlAction] = flyController(uavStates, resilienceMetrics, estimatedPos, confidence, simParams, formation, stclState)
% 飞行控制器主函数（模块化设计，集成四种控制模式）- 新增CtrlMode=3（σₜ性能因子控制）
% 输入参数：
%   uavStates       - 无人机状态结构体数组
%   resilienceMetrics - 韧性指标结构体（含sigma字段）
%   estimatedPos    - 干扰源估计位置（2维）
%   confidence      - 干扰估计置信度（0~1）
%   simParams       - 仿真参数（含CtrlMode控制模式及σₜ相关参数）
%   formation       - 编队信息（用于航向参考）
%   stclState       - STCL状态（判断是否在干扰区）
% 输出参数：
%   newDirection    - 新航向向量（1×2行向量）
%   controlAction   - 控制动作详情

    %% 1. 初始化输出结构与基础参数
    controlAction = struct(...
        'adjustHeading', false, ...
        'reason', '', ...
        'CtrlMode', simParams.CtrlMode, ...
        'confidence', confidence, ...
        'weight', NaN, ...
        'avoidanceWeight', NaN, ...
        'resilienceStatus', 'normal', ...
        'controlVar', ''); % 新增：标记控制变量（R(t)或σₜ）
    
    % 获取当前航向
    if isfield(formation, 'directionVec') && ~isempty(formation.directionVec)
        currentDirection = formation.directionVec;
    else
        currentVelocity = uavStates(1).velocity(1:2);
        currentDirection = currentVelocity / norm(currentVelocity + 1e-6);
    end
    newDirection = currentDirection;
    
    %% 2. 韧性指标历史缓存（用于判定恒定状态）
    persistent resilienceHistory; % 持久化存储历史R值
    if isempty(resilienceHistory) % 仿真初始化
        resilienceHistory = zeros(1, 5); % 缓存最近5步R值
    end

    % 提取当前R值（确保有效性）
    currentR = 1.0;
    if isfield(resilienceMetrics, 'R') && ~isnan(resilienceMetrics.R)
        currentR = resilienceMetrics.R;
    else
        warning('韧性指标R未初始化，默认设为1.0');
    end

    % % alpha=0.3（平滑与响应的平衡点，可调整）
    currentR = lowpassFilter(currentR, 0.3);
    % 更新历史缓存（左移并加入当前值）
    resilienceHistory = [resilienceHistory(2:end), currentR];
    validHistory = resilienceHistory(resilienceHistory ~= 0); % 过滤无效值

    %% 2. 根据控制模式执行对应策略
    switch simParams.CtrlMode
        case 0  % 无控制模式
            [newDirection, controlAction] = noControlStrategy(currentDirection, controlAction);
            
        case 1  % 人工势场法（APF）控制模式
            [newDirection, controlAction] = APFControlStrategy(...
                currentDirection, uavStates, estimatedPos, confidence, simParams, controlAction, formation, stclState);
            
        case 2  % 韧性控制模式（R(t)为变量）
            [newDirection, controlAction, resilienceHistory] = resilienceControlStrategy(...
                currentDirection, uavStates, resilienceMetrics, estimatedPos, ...
                confidence, simParams, controlAction, formation, stclState, ...
                currentR, validHistory, resilienceHistory);
        case 3  % σₜ性能因子控制模式
            [newDirection, controlAction] = sigmaControlStrategy(...
                currentDirection, uavStates, resilienceMetrics, estimatedPos, ...
                confidence, simParams, controlAction, formation, stclState);
        otherwise
            warning('无效控制模式（CtrlMode=%d），默认保持航向', simParams.CtrlMode);
            controlAction.reason = sprintf('无效模式（%d）：默认保持航向', simParams.CtrlMode);
    end
end

%% -------------------------- 子函数：无控制策略 --------------------------
function [newDirection, controlAction] = noControlStrategy(currentDirection, controlAction)
    % 无控制策略：保持当前航向（确保返回1×2行向量）
    newDirection = currentDirection; % 直接传递原有航向，非空
    controlAction.adjustHeading = false;
    controlAction.reason = '无控制模式：保持当前航向';
end

%% -------------------------- 子函数：人工势场法（APF）控制策略 --------------------------
function [newDirection, controlAction] = APFControlStrategy(...
        currentDirection, uavStates, estimatedPos, confidence, simParams, controlAction, formation, stclState)
    % APF控制策略：基于引力场（目标）和斥力场（干扰源）的合力导航
    % 适配条件：
    %   - 初始阶段无干扰时，仅受引力场作用
    %   - 进入干扰区且有可靠定位时，叠加斥力场作用
    
    %% 1. 基础参数与状态提取
    currentDirection = currentDirection / norm(currentDirection + 1e-6); % 单位化
    clusterCenter = mean([uavStates(:).position], 1); % 集群中心（x,y,z）
    clusterCenter_2d = clusterCenter(1:2); % 2维坐标（排除z轴）
    
    % 修复：从simParams获取目标位置（替代formation.targetPosition）
    % 确保simParams中定义了targetPosition（如 [100, 100, 0]）
    targetPos_2d = simParams.targetPosition(1:2);
    
    % APF参数（从simParams获取，支持灵活调整）
    if ~isfield(simParams, 'apfParams')
        % 默认参数（确保兼容性）
        simParams.apfParams = struct(...
            'k_att', 0.4, ...         % 引力系数
            'k_rep', 1.2, ...         % 斥力系数   由0.8更改为2.0
            'target_radius', 5.0, ... % 目标点引力饱和半径
            'intf_radius', 4.5);     % 干扰源斥力作用半径
    end
    ap = simParams.apfParams;
    
    %% 2. 计算引力场（始终有效，指向目标点）
    % 引力向量（集群中心→目标点）
    attractive_vec = targetPos_2d - clusterCenter_2d;
    distance_to_target = norm(attractive_vec);
    
    % 引力场模型（避免目标点附近引力无穷大）
    if distance_to_target < ap.target_radius
        % 目标附近：引力线性衰减（防止震荡）
        attractive_force = attractive_vec / ap.target_radius;
    else
        % 远处：引力与距离成正比（加速靠近）
        attractive_force = attractive_vec * ap.k_att;
    end
    attractive_dir = attractive_force / norm(attractive_force + 1e-6); % 引力方向
    
    %% 3. 计算斥力场（增强干扰区边缘效果）
    repulsive_dir = [0, 0]; % 初始化为零向量
    is_repulsive_active = false;
    
    % 触发条件：在干扰区内 + 定位置信度足够 + 定位有效
    if stclState.inInterferenceZone && (confidence > 0.5) && (norm(estimatedPos) > 1e-3)
        % 干扰源位置（复用STCL_NN估计结果）
        intf_pos_2d = estimatedPos(1:2);
        
        % 斥力向量（集群中心→远离干扰源）
        repulsive_vec = clusterCenter_2d - intf_pos_2d;
        distance_to_intf = norm(repulsive_vec);
        
        % 仅在干扰影响范围内生效
        if distance_to_intf < ap.intf_radius && distance_to_intf > 1e-3
            % 核心优化1：增大基础斥力系数（从0.8→1.2，可根据效果调整）
            % base_k_rep = 1.2; 
            base_k_rep = ap.k_rep;   % 更改为由simParams.apfParams的参数获取
            
            % 核心优化2：距离自适应增强（干扰区边缘斥力额外提升50%）
            % 当距离 > 0.7×干扰半径时（边缘区域），额外增加斥力
            if distance_to_intf > 0.7 * ap.intf_radius
                adaptive_k_rep = base_k_rep * 1.5; % 边缘区域斥力增强50%
            else
                adaptive_k_rep = base_k_rep; % 核心区域保持基础斥力
            end
            
            % 斥力场模型（应用自适应系数）
            repulsive_force = (1/distance_to_intf - 1/ap.intf_radius) ...
                            * (1/distance_to_intf^2) ...  % 距离越近，斥力非线性递增
                            * (repulsive_vec / distance_to_intf) ...
                            * adaptive_k_rep; % 使用自适应系数
            repulsive_dir = repulsive_force / norm(repulsive_force + 1e-6);
            is_repulsive_active = true;
            
            % 日志补充：记录是否启用边缘增强
            edge_flag = (distance_to_intf > 0.7 * ap.intf_radius);
            controlAction.repulsiveEnhanced = edge_flag; % 新增：标记是否边缘增强
        end
    end   
    
    %% 4. 合力计算与航向决策
    % 固定权重（引力:斥力 = 0.6:0.4，与韧性控制的动态权重形成对比）
    % controlAction.attractiveWeight = 0.6;
    % controlAction.repulsiveWeight = 0.4;
    controlAction.attractiveWeight = 0.1;
    controlAction.repulsiveWeight = 0.9;
    
    % 计算合力方向
    total_force = controlAction.attractiveWeight * attractive_dir ...
                + controlAction.repulsiveWeight * repulsive_dir;
    
    % 特殊情况处理（合力为零时保持当前航向）
    if norm(total_force) < 1e-6
        desired_dir = currentDirection;
    else
        desired_dir = total_force / norm(total_force);
    end
    
    %% 5. 应用航向约束（与韧性控制共享同一约束机制）
    [newDirection, angleInfo] = applyHeadingConstraint(currentDirection, desired_dir, simParams.maxHeadingChange);
    
    %% 6. 控制动作日志（区分不同阶段）
    if is_repulsive_active
        controlAction.adjustHeading = true;
        controlAction.reason = sprintf('APF（引力+斥力）：目标距离=%.1f，干扰距离=%.1f，转向=%.1f°', ...
            distance_to_target, distance_to_intf, rad2deg(angleInfo.actualAngle));
    else
        % 无有效斥力时，仅受引力控制
        controlAction.adjustHeading = (norm(desired_dir - currentDirection) > 1e-3);
        controlAction.reason = sprintf('APF（仅引力）：目标距离=%.1f，无有效斥力，转向=%.1f°', ...
            distance_to_target, rad2deg(angleInfo.actualAngle));
    end
end

%% -------------------------- 子函数：基于sigma(t)的韧性控制策略 --------------------------
function [newDirection, controlAction] = sigmaControlStrategy(...
        currentDirection, uavStates, resilienceMetrics, estimatedPos, ...
        confidence, simParams, controlAction, formation, stclState)
    % 核心逻辑：以sigma0为任务性能基线（目标），sigma_l为基线的百分比阈值
    % 确保最终到达目的地时sigma(t)收敛至sigma0附近
    
    %% 1. 初始化与参数提取（完全复用源程序逻辑）
    controlAction.controlVar = 'σₜ_optimal（基线对齐版）';
    sigma_l = simParams.sigma_threshold;  % 基线的百分比阈值（如sigma0的80%）
    sigma0 = sum(simParams.initialPayloads)/simParams.targetLoad;  % 任务性能基线（核心目标）
    max_heading_change = simParams.maxHeadingChange;
    
    % 复用源程序的比例计算（确保与同类程序一致）
    smooth_ratio = max(0.2, min(0.5, (sigma0 - sigma_l)/sigma0));
    heading_scale = 0.5 + 0.5*(sigma0 / (sigma0 + sigma_l));
    
    % 理论参数（针对基线收敛优化）
    k_omega = 0.4;      % 平滑系数（基于基线比例）
    alpha = 0.5;        % 预调整系数
    converge_threshold = 0.05;  % 收敛至基线附近的允许偏差
    
    %% 2. 获取σ(t)值（严格沿用源程序的边界处理）
    if isfield(resilienceMetrics, 'sigma') && ~isnan(resilienceMetrics.sigma)
        sigma_t = resilienceMetrics.sigma;
        % 源程序的边界限制，确保在基线合理范围内
        sigma_t = max(min(sigma_t, sigma0 + 0.3*(sigma0 - sigma_l)), ...
                     sigma_l - 0.2*(sigma0 - sigma_l));
    else
        sigma_t = sigma0;
        warning('[sigmaControlOptimal] 使用初始sigma值（基线）');
    end
    
    % 计算关键指标（基于基线的偏差和变化率）
    persistent prev_sigma;
    if isempty(prev_sigma), prev_sigma = sigma_t; end
    sigma_dot = (sigma_t - prev_sigma) / simParams.dt;  % 性能变化率
    sigma_deviation = sigma_t - sigma0;  % 与基线的偏差（核心控制指标）
    prev_sigma = sigma_t;
    
    %% 3. 方向向量计算（完全复用源程序代码）
    clusterCenter_2d = mean([uavStates(:).position], 1);
    clusterCenter_2d = clusterCenter_2d(1:2);
    
    % 3.1 任务方向（与源程序一致）
    targetVec = simParams.targetPosition(1:2) - clusterCenter_2d;
    targetDir = targetVec / norm(targetVec + 1e-6);
    
    % 3.2 规避方向（复用源程序的干扰处理）
    avoidanceDir = [0, 0];
    isAvoidActive = false;
    interferenceVec = [0, 0];  % 源程序的初始化解
    if stclState.inInterferenceZone && (confidence > 0.5) && (norm(estimatedPos) > 1e-3)
        interferenceVec = clusterCenter_2d - estimatedPos(1:2);
        if norm(interferenceVec) > 1e-3
            avoidanceDir = interferenceVec / norm(interferenceVec);
            isAvoidActive = true;
        end
    end
    
    %% 4. 动态加权系数（核心优化：收敛至基线）
    persistent prev_efficiencyWeight;
    if isempty(prev_efficiencyWeight)
        prev_efficiencyWeight = 1.0;  % 源程序的初始权重
    end
    
    % 4.1 接近目的地时的强制收敛（确保最终接近基线）
    target_dist = norm(targetVec);
    is_near_target = (target_dist < 5.0);  % 距目标较近时（可根据场景调整）
    if is_near_target && abs(sigma_deviation) > converge_threshold
        % 强制向基线收敛：偏差越大，效率权重越高
        target_efficiency = 0.9 - 0.3 * min(1.0, abs(sigma_deviation)/0.2);
        resilienceStatus = sprintf('接近目标（收敛至基线，偏差=%.2f）', sigma_deviation);
    else
        % 4.2 正常场景：基于基线和阈值的动态调整
        if ~isAvoidActive
            target_efficiency = 1.0;  % 无干扰时全力向基线收敛
            resilienceStatus = '无干扰（效率优先，收敛基线）';
        else
            if sigma_t >= sigma0  % 高于基线（有冗余）
                target_efficiency = 0.9;  % 消耗冗余，加速向基线靠近
                resilienceStatus = '高于基线（消耗冗余）';
            elseif sigma_t >= sigma_l  % 基线与阈值之间（临界）
                delta_sigma = (sigma0 - sigma_t)/(sigma0 - sigma_l);  % 源程序的比例计算
                target_efficiency = max(0.5, 0.8 - 0.3*delta_sigma);  % 比源程序更偏向效率
                resilienceStatus = '临界区间（动态平衡）';
            else  % 低于阈值（需保护）
                target_efficiency = 0.4;  % 仅维持必要韧性
                resilienceStatus = '低于阈值（韧性保护）';
            end
            
            % 4.3 回升趋势预调整（加速收敛至基线）
            if sigma_dot > 0 && sigma_t < sigma0
                target_efficiency = target_efficiency + alpha * sigma_dot * (1.0 - target_efficiency);
                target_efficiency = min(0.9, target_efficiency);  % 不超过上限
                resilienceStatus = sprintf('回升调整（σ̇=%.3f，向基线收敛）', sigma_dot);
            end
        end
    end
    
    % 复用源程序的平滑更新（确保兼容性）
    efficiencyWeight = smooth_ratio * target_efficiency + (1 - smooth_ratio) * prev_efficiencyWeight;
    prev_efficiencyWeight = efficiencyWeight;
    avoidanceWeight = 1 - efficiencyWeight;
    
    %% 5. 合成航向与约束（复用源程序逻辑）
    desiredDir = efficiencyWeight * targetDir + avoidanceWeight * avoidanceDir;
    desiredDir = desiredDir / norm(desiredDir + 1e-6);
    dynamic_max_heading = max_heading_change * heading_scale;
    [newDirection, angleInfo] = applyHeadingConstraint(currentDirection, desiredDir, dynamic_max_heading);
    
    %% 6. 日志记录（与源程序格式一致）
    interferenceDist = norm(interferenceVec);
    controlAction.adjustHeading = (angleInfo.actualAngle > deg2rad(0.5));
    controlAction.weight = efficiencyWeight;
    controlAction.avoidanceWeight = avoidanceWeight;
    controlAction.sigmaValue = sigma_t;
    controlAction.sigma_deviation = sigma_deviation;  % 新增：与基线的偏差
    controlAction.isAvoidActive = isAvoidActive;
    
    controlAction.reason = sprintf('%s：σ=%.2f（基线=%.2f），权重=%.2f，转向=%.1f°', ...
        resilienceStatus, sigma_t, sigma0, efficiencyWeight, rad2deg(angleInfo.actualAngle));
end

function [newDirection, controlAction] = sigmaControlOptimal(...
        currentDirection, uavStates, resilienceMetrics, estimatedPos, ...
        confidence, simParams, controlAction, formation, stclState)
    % 核心调整：进一步降低性能因子，增强向基线收敛的力度
    % 关键优化：提高效率权重、收紧冗余容忍度、提前启动收敛逻辑
    
    %% 1. 初始化与参数提取（微调收敛参数）
    controlAction.controlVar = 'σₜ_optimal（低冗余版）';
    sigma_l = simParams.sigma_threshold;
    sigma0 = sum(simParams.initialPayloads)/simParams.targetLoad;  % 性能基线
    max_heading_change = simParams.maxHeadingChange;
    
    % 复用源程序比例计算，微调平滑系数增强响应
    smooth_ratio = max(0.15, min(0.45, (sigma0 - sigma_l)/sigma0));  % 降低平滑度（更灵敏）
    heading_scale = 0.6 + 0.4*(sigma0 / (sigma0 + sigma_l));  % 放宽转向限制
    
    % 关键参数调整（降低冗余）
    k_omega = 0.5;      % 提高权重调整速率
    alpha = 0.6;        % 增强预调整力度
    converge_threshold = 0.03;  % 收紧收敛偏差（从0.05→0.03）
    near_target_dist = 7.0;     % 提前启动收敛（从5.0→7.0）
    
    %% 2. 获取σ(t)值（强化上限控制）
    if isfield(resilienceMetrics, 'sigma') && ~isnan(resilienceMetrics.sigma)
        sigma_t = resilienceMetrics.sigma;
        % 进一步压低上限（从0.3→0.2冗余）
        sigma_t = max(min(sigma_t, sigma0 + 0.2*(sigma0 - sigma_l)), ...
                     sigma_l - 0.2*(sigma0 - sigma_l));
    else
        sigma_t = sigma0;
        warning('[sigmaControlOptimal] 使用初始sigma值');
    end
    
    % 性能指标计算（更灵敏的偏差反馈）
    persistent prev_sigma;
    if isempty(prev_sigma), prev_sigma = sigma_t; end
    sigma_dot = (sigma_t - prev_sigma) / simParams.dt;
    sigma_deviation = sigma_t - sigma0;  % 与基线的偏差（核心监控）
    prev_sigma = sigma_t;
    
    %% 3. 方向向量计算（复用源程序逻辑）
    clusterCenter_2d = mean([uavStates(:).position], 1);
    clusterCenter_2d = clusterCenter_2d(1:2);
    
    targetVec = simParams.targetPosition(1:2) - clusterCenter_2d;
    targetDir = targetVec / norm(targetVec + 1e-6);
    
    avoidanceDir = [0, 0];
    isAvoidActive = false;
    interferenceVec = [0, 0];
    if stclState.inInterferenceZone && (confidence > 0.5) && (norm(estimatedPos) > 1e-3)
        interferenceVec = clusterCenter_2d - estimatedPos(1:2);
        if norm(interferenceVec) > 1e-3
            avoidanceDir = interferenceVec / norm(interferenceVec);
            isAvoidActive = true;
        end
    end
    
    %% 4. 动态加权系数（核心降冗余逻辑）
    persistent prev_efficiencyWeight;
    if isempty(prev_efficiencyWeight)
        prev_efficiencyWeight = 1.0;  % 初始即效率优先
    end
    
    % 4.1 更早启动目标收敛（距离更远时开始）
    target_dist = norm(targetVec);
    is_near_target = (target_dist < near_target_dist);
    if is_near_target
        % 偏差越大，效率权重越高（强化降冗余）
        over_deviation = max(0, sigma_deviation);  % 只关注正偏差（冗余）
        target_efficiency = 0.95 - 0.4 * min(1.0, over_deviation / 0.15);
        resilienceStatus = sprintf('提前收敛（偏差=%.2f，压减冗余）', sigma_deviation);
    else
        % 4.2 正常场景：更激进的效率导向
        if ~isAvoidActive
            target_efficiency = 1.0;  % 无干扰时全力降冗余
            resilienceStatus = '无干扰（效率优先，降低冗余）';
        else
            if sigma_t >= sigma0  % 高于基线（冗余状态）
                % 比之前更激进（0.9→0.95），加速消耗冗余
                target_efficiency = 0.95;
                resilienceStatus = '高于基线（强压冗余）';
            elseif sigma_t >= sigma_l  % 临界区间
                delta_sigma = (sigma0 - sigma_t)/(sigma0 - sigma_l);
                % 效率权重比之前提高5-10%
                target_efficiency = max(0.55, 0.85 - 0.3*delta_sigma);
                resilienceStatus = '临界区间（效率倾斜）';
            else  % 低于阈值
                target_efficiency = 0.45;  % 略微提高效率权重
                resilienceStatus = '低于阈值（适度保护）';
            end
            
            % 4.3 回升时更强的预调整（加速降冗余）
            if sigma_dot > 0 && sigma_t < sigma0
                target_efficiency = target_efficiency + alpha * sigma_dot * (1.0 - target_efficiency);
                target_efficiency = min(0.95, target_efficiency);
                resilienceStatus = sprintf('强回升调整（σ̇=%.3f）', sigma_dot);
            end
        end
    end
    
    % 更快的权重更新（降低平滑滞后）
    efficiencyWeight = smooth_ratio * target_efficiency + (1 - smooth_ratio) * prev_efficiencyWeight;
    prev_efficiencyWeight = efficiencyWeight;
    avoidanceWeight = 1 - efficiencyWeight;
    
    %% 5. 航向与输出（保持兼容性）
    desiredDir = efficiencyWeight * targetDir + avoidanceWeight * avoidanceDir;
    desiredDir = desiredDir / norm(desiredDir + 1e-6);
    dynamic_max_heading = max_heading_change * heading_scale;
    [newDirection, angleInfo] = applyHeadingConstraint(currentDirection, desiredDir, dynamic_max_heading);
    
    controlAction.adjustHeading = (angleInfo.actualAngle > deg2rad(0.5));
    controlAction.weight = efficiencyWeight;
    controlAction.avoidanceWeight = avoidanceWeight;
    controlAction.sigmaValue = sigma_t;
    controlAction.sigma_deviation = sigma_deviation;
    controlAction.isAvoidActive = isAvoidActive;
    
    controlAction.reason = sprintf('%s：σ=%.2f（基线=%.2f），权重=%.2f，转向=%.1f°', ...
        resilienceStatus, sigma_t, sigma0, efficiencyWeight, rad2deg(angleInfo.actualAngle));
end

%% -------------------------- 公共辅助函数（全适配版） --------------------------
function [targetDir, avoidanceDir] = calculateBaseDirections(uavStates, estimatedPos, simParams, formation)
% 计算任务效率方向与规避方向（均返回1×2行向量）
    % 1. 集群中心（2维）
    clusterCenter_2d = mean([uavStates(:).position], 1);
    clusterCenter_2d = clusterCenter_2d(1:2);
    
    % 2. 任务效率方向（朝向目标点，1×2）
    targetVec = simParams.targetPosition(1:2) - clusterCenter_2d; % targetPosition（1×3）适配原有程序
    if norm(targetVec) < 1e-6
        targetDir = formation.directionVec; % 已达目标，用当前编队方向
    else
        targetDir = targetVec / norm(targetVec);
    end
    
    % 3. 规避方向（远离干扰源，1×2）
    avoidanceVec = clusterCenter_2d - estimatedPos(1:2);
    if norm(avoidanceVec) < 1e-6
        avoidanceDir = [-formation.directionVec(2), formation.directionVec(1)]; % 垂直当前方向
    else
        avoidanceDir = avoidanceVec / norm(avoidanceVec);
    end
end

function [adjustedDir, angleInfo] = applyHeadingConstraint(currentDir, desiredDir, maxAngle)
% 应用最大转向角约束（输入输出均为1×2行向量，核心适配点4）
    % 确保输入为列向量用于计算
    currentDir_col = currentDir(:);
    desiredDir_col = desiredDir(:);
    
    % 限制为2维向量
    if length(currentDir_col) > 2
        currentDir_col = currentDir_col(1:2);
    end
    if length(desiredDir_col) > 2
        desiredDir_col = desiredDir_col(1:2);
    end
    
    % 计算夹角
    angleBetween = acos(dot(currentDir_col, desiredDir_col) / (norm(currentDir_col)*norm(desiredDir_col) + 1e-6));
    angleInfo.originalAngle = angleBetween;
    angleInfo.actualAngle = angleBetween;
    
    % 应用约束
    if angleBetween > maxAngle
        theta = maxAngle;
        % 判断旋转方向
        crossVal = currentDir_col(1)*desiredDir_col(2) - currentDir_col(2)*desiredDir_col(1);
        if crossVal < 0
            rotMat = [cos(theta) sin(theta); -sin(theta) cos(theta)]; % 顺时针
        else
            rotMat = [cos(theta) -sin(theta); sin(theta) cos(theta)]; % 逆时针
        end
        adjustedDir_col = rotMat * currentDir_col;
        angleInfo.actualAngle = maxAngle;
    else
        adjustedDir_col = desiredDir_col;
    end
    
    % 核心适配：转回1×2行向量，匹配原有程序的formation.directionVec格式
    adjustedDir = adjustedDir_col';
    adjustedDir = adjustedDir / norm(adjustedDir + 1e-6);
end

function deg = rad2deg(rad)
% 弧度转角度（避免依赖工具箱）
    deg = rad * 180 / pi;
end

% 1. 新增独立低通滤波函数（放在文件末尾，不影响原有结构）
function filteredValue = lowpassFilter(originalValue, alpha, resetFlag)
    % 新增resetFlag参数，支持仿真初始化时重置缓存
    persistent lastFilteredValue; 
    if nargin >=3 && resetFlag % 外部传入重置指令（如仿真开始时）
        lastFilteredValue = originalValue;
    elseif isempty(lastFilteredValue)
        lastFilteredValue = originalValue;
    end
    filteredValue = alpha * originalValue + (1 - alpha) * lastFilteredValue;
    lastFilteredValue = filteredValue;
end

%% -------------------------- 子函数：韧性控制策略（核心适配版） --------------------------
function [newDirection, controlAction, resilienceHistory] = resilienceControlStrategy(...
        currentDirection, uavStates, resilienceMetrics, estimatedPos, ...
        confidence, simParams, controlAction, formation, stclState, ...
        currentR, validHistory, resilienceHistory)
    % 核心优化：
    % 1. 识别韧性值恒定状态（自恢复系数=0场景）
    % 2. 区分"干扰区内+R变化"、"干扰区内+R恒定"、"干扰区外"三种场景
    
    % 新增：迟滞阈值（避免小幅波动导致的权重反复切换）
    HYSTERESIS_THRESH = 0.05;  % 权重变化超过5%才触发切换

    % 1. 基础状态判定
    threshold = simParams.minResilienceThreshold;
    isInInterference = any(stclState.inInterferenceZone);  % 当前是否在干扰区
    wasInInterference = any(stclState.prevUavInZoneArray);  % 上一步是否在干扰区
    
    % 2. 韧性值恒定判定（关键逻辑）
    isResilienceConstant = false;
    if length(validHistory) >= 3  % 至少3个有效历史点
        rVariance = var(validHistory); % 计算方差
        isResilienceConstant = (rVariance < 1e-6); % 方差过小视为恒定
    end
    
    % 3. 场景分类与权重计算（新增迟滞逻辑）
    persistent lastEfficiencyWeight; % 记录上一步的任务权重
    if isempty(lastEfficiencyWeight)
        lastEfficiencyWeight = 1.0; % 初始为纯任务权重
    end
    
    if isInInterference
        % 场景A：在干扰区内
        if isResilienceConstant && currentR < threshold
            efficiencyWeight = 0.9;  
            avoidanceWeight = 0.1;   
        else
            efficiencyWeight = calculateDynamicWeight(currentR, threshold);
            avoidanceWeight = 1 - efficiencyWeight;
        end
        
        % 迟滞逻辑：从干扰区外→内时，权重变化需超过阈值才切换
        if ~wasInInterference && abs(efficiencyWeight-lastEfficiencyWeight) < HYSTERESIS_THRESH
            efficiencyWeight = lastEfficiencyWeight; % 保持上一步权重，避免小幅波动
            avoidanceWeight = 1 - efficiencyWeight;
        end
    else
        % 场景B：已离开干扰区
        if wasInInterference || isResilienceConstant
            efficiencyWeight = 1.0;  
            avoidanceWeight = 0.0;   
        else
            efficiencyWeight = 1.0;
            avoidanceWeight = 0.0;
        end
        
        % 迟滞逻辑：从干扰区内→外时，权重变化需超过阈值才切换
        if wasInInterference && abs(efficiencyWeight-lastEfficiencyWeight) < HYSTERESIS_THRESH
            efficiencyWeight = lastEfficiencyWeight; % 保持上一步权重
            avoidanceWeight = 1 - efficiencyWeight;
        end
    end
    
    % 更新历史权重（用于迟滞判断）
    lastEfficiencyWeight = efficiencyWeight;
    
    % 4. 计算基础方向向量
    [targetDir, avoidanceDir] = calculateBaseDirections(uavStates, estimatedPos, simParams, formation);
    
    % 5. 合成期望方向
    desiredDir = efficiencyWeight * targetDir + avoidanceWeight * avoidanceDir;
    desiredDir = desiredDir / norm(desiredDir + 1e-6);  % 归一化
    
    % 6. 应用航向约束
    [newDirection, angleInfo] = applyHeadingConstraint(currentDirection, desiredDir, simParams.maxHeadingChange);
    
    % 7. 更新控制动作信息
    controlAction.adjustHeading = (avoidanceWeight > 0.01);
    controlAction.weight = efficiencyWeight;
    controlAction.avoidanceWeight = avoidanceWeight;
    
    % 8. 生成状态描述
    if isInInterference
        if isResilienceConstant
            controlAction.reason = sprintf('干扰区内（R恒定）：任务权重=%.2f，规避权重=%.2f', ...
                efficiencyWeight, avoidanceWeight);
        else
            controlAction.reason = sprintf('干扰区内（R变化）：R=%.3f，任务权重=%.2f，规避权重=%.2f', ...
                currentR, efficiencyWeight, avoidanceWeight);
        end
    else
        controlAction.reason = sprintf('干扰区外：任务权重=%.2f，规避权重=%.2f', ...
            efficiencyWeight, avoidanceWeight);
    end
end

% 配套修改动态权重计算函数（保持干扰区内的韧性响应）
function weight = calculateDynamicWeight(R, threshold)
    % 干扰区内：韧性越低，规避权重越高（任务权重越低）
    if R <= 0
        weight = 0.3;  % 最低任务权重（保留基础航向）
    elseif R >= threshold
        weight = 0.8;  % 韧性达标时仍保留保留20%规避权重，避免突然切换
    else
        % S型曲线平滑过渡（中点在threshold*0.6，让权重更敏感）
        midPoint = threshold * 0.5;
        weight = 1 / (1 + exp(-0.8 * (R - midPoint)));  % k=0.8 灵敏度系数，该值越小weight越小，轻效率，重韧性
        weight = max(weight, 0.3);  % 下限保护
    end
end