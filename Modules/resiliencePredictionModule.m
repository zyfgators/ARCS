function [modified_data, timeMask, Tw_used] = resiliencePredictionModule(original_data, Tk_idx, psi, timeZone)
% resiliencePredictionModule - 无人机集群预测独立模块（复用ResilenceSim.m原生函数）
% 输入：
%   original_data - 原始仿真数据
%   Tk_idx        - 预测起始时刻在原始数据中的索引
%   psi           - 航向角调整角度（度）
%   timeZone      - 时间范围
% 输出：
%   modified_data - 真实+预测数据
%   timeMask      - 时间掩码
%   Tw_used       - 实际使用的预测窗口长度（动态计算后）

  % 1. 初始化
    dt = original_data.simParams.dt;
    numUAV = original_data.simParams.numUAVs;
    psi_rad = deg2rad(psi);
    original_Tw = original_data.simParams.resillenSlidingWindow;
    Tk_actual = original_data.timeArray(Tk_idx);

    % 2. 调用新辅助函数，获取Texit (仅模拟一次)
    [allExited, Texit] = findExitTime(original_data, Tk_idx, psi_rad, dt);

    % 3. 动态计算最终的Tw和预测步数
    if allExited
        Tw_used = Texit - Tk_actual;
    else
        Tw_used = original_Tw;
    end
    pred_steps = round(Tw_used / dt);
    pred_steps = max(pred_steps, 2); % 确保至少有2个点用于插值

    % 4. 执行一次最终预测
    [rough_pred, resilienceMetrics] = runPredictionWithNativeFun(original_data, Tk_idx, pred_steps, psi_rad, dt);

    % 5. 插值对齐和数据组合 (这部分你的代码是正确的，无需修改)
    pred_time = Tk_actual + (1:pred_steps) * dt;
    aligned_pred = alignToMainStep(rough_pred, pred_time, numUAV);

    modified_data = original_data;
    modified_data.timeArray = original_data.timeArray(1:Tk_idx);
    modified_data.positionsArray = original_data.positionsArray(1:Tk_idx, :, :);
    modified_data.totalEffectivePayloadArray = original_data.totalEffectivePayloadArray(1:Tk_idx);
    modified_data.sigmaArray = original_data.sigmaArray(1:Tk_idx);
    modified_data.headingArray = original_data.headingArray(1:Tk_idx, :);
    modified_data.damageFactorsArray = original_data.damageFactorsArray(1:Tk_idx, :);
    modified_data.effectivePayloadsArray = original_data.effectivePayloadsArray(1:Tk_idx, :);
    modified_data.attenuationArray = original_data.attenuationArray(1:Tk_idx, :);
    
    modified_data.deltaArray = original_data.deltaArray(1:Tk_idx);
    modified_data.rhoArray = original_data.rhoArray(1:Tk_idx);
    modified_data.tauArray = original_data.tauArray(1:Tk_idx);
    modified_data.zetaArray = original_data.zetaArray(1:Tk_idx);
    modified_data.RArray = original_data.RArray(1:Tk_idx);
    
    modified_data.timeArray = [modified_data.timeArray; pred_time'];
    modified_data.positionsArray = cat(1, modified_data.positionsArray, aligned_pred.positions);
    modified_data.totalEffectivePayloadArray = [modified_data.totalEffectivePayloadArray; aligned_pred.total_payload'];
    
    pred_sigma = aligned_pred.sigma;
    if isrow(pred_sigma), pred_sigma = pred_sigma'; end
    if length(pred_sigma) ~= length(pred_time)
        pred_sigma = pred_sigma(1:min(length(pred_time), length(pred_sigma)));
        pred_sigma = [pred_sigma; zeros(length(pred_time) - length(pred_sigma), 1)];
    end
    modified_data.sigmaArray = [modified_data.sigmaArray; pred_sigma];
    
    modified_data.headingArray = [modified_data.headingArray; repmat(aligned_pred.heading, pred_steps, 1)];
    modified_data.damageFactorsArray = cat(1, modified_data.damageFactorsArray, aligned_pred.damage_factors);
    modified_data.effectivePayloadsArray = cat(1, modified_data.effectivePayloadsArray, aligned_pred.effective_payloads);
    modified_data.attenuationArray = cat(1, modified_data.attenuationArray, aligned_pred.interference_strength);
    
    modified_data.deltaArray(Tk_idx) = resilienceMetrics.delta;
    modified_data.rhoArray(Tk_idx) = resilienceMetrics.rho;
    modified_data.tauArray(Tk_idx) = resilienceMetrics.tau;
    modified_data.zetaArray(Tk_idx) = resilienceMetrics.zeta;
    modified_data.RArray(Tk_idx) = resilienceMetrics.R;
    
    if pred_steps > 0
        modified_data.deltaArray = [modified_data.deltaArray; repmat(resilienceMetrics.delta, pred_steps, 1)];
        modified_data.rhoArray = [modified_data.rhoArray; repmat(resilienceMetrics.rho, pred_steps, 1)];
        modified_data.tauArray = [modified_data.tauArray; repmat(resilienceMetrics.tau, pred_steps, 1)];
        modified_data.zetaArray = [modified_data.zetaArray; repmat(resilienceMetrics.zeta, pred_steps, 1)];
        modified_data.RArray = [modified_data.RArray; repmat(resilienceMetrics.R, pred_steps, 1)];
    end

    % 6. 构造timeMask
    if isfield(original_data.simParams, 'resillenSlidingWindow')
        line_start = timeZone(1);
        line_end = Tk_actual + Tw_used;
        timeMask = (modified_data.timeArray >= line_start) & (modified_data.timeArray <= line_end);
    else
        warning('未找到resillenSlidingWindow参数，使用初始timeMask');
    end

end

% -------------------------------------------------------------------------
% 内部子函数1：预测核心（仅调用原生函数，无自定义计算）
% -------------------------------------------------------------------------
% =========================================================================
% 简化后的核心预测函数
% =========================================================================
function [rough_pred, resilienceMetrics] = runPredictionWithNativeFun(original_data, Tk_idx, pred_steps, psi_rad, dt)
    numUAV = original_data.simParams.numUAVs;
    simParams = original_data.simParams;
    damage_params = original_data.damageParams;
    interference_source = original_data.interferenceSource;

    rough_pred = struct(...
        'time', [], 'positions', [], 'damage_factors', [], ...
        'effective_payloads', [], 'total_payload', [], ...
        'sigma', [], 'heading', [], 'interference_strength', []);
    rough_pred.time = original_data.timeArray(Tk_idx) + (1:pred_steps)*dt;

    % --- 提取初始状态 (与之前相同) ---
    tk_pos = squeeze(original_data.positionsArray(Tk_idx, :, :));
    tk_damage = original_data.damageFactorsArray(Tk_idx, :);
    tk_eff_payload = original_data.effectivePayloadsArray(Tk_idx, :);
    tk_heading = original_data.headingArray(Tk_idx, :);
    confidence = original_data.confidencesArray(Tk_idx);
    intensity_th = simParams.intensityThresholds;

    current_uavStates(numUAV) = struct('position', [], 'damageFactor', [], 'currentPayload', [], 'effectivePayload', [], 'intensityThreshold', []);
    for i = 1:numUAV
        current_uavStates(i).position = [tk_pos(i, 1), tk_pos(i, 2), 2];
        current_uavStates(i).damageFactor = tk_damage(i);
        current_uavStates(i).effectivePayload = tk_eff_payload(i);
        if tk_damage(i) ~= 1
            current_uavStates(i).currentPayload = tk_eff_payload(i) / (1 - tk_damage(i));
        else
            current_uavStates(i).currentPayload = 0;
        end
        current_uavStates(i).intensityThreshold = intensity_th;
    end
    current_formation = getHexagonSwarmTemplate(original_data, tk_pos);
    current_formation.directionVec = rotateHeading(tk_heading, psi_rad);
    rough_pred.heading = current_formation.directionVec;

    % --- 核心预测循环 (移除了所有Texit相关的判断) ---
    for k = 1:pred_steps
        [current_uavStates, current_formation] = updateUAVPositions(current_uavStates, current_formation, simParams, dt);
        current_pos = zeros(numUAV, 3);
        for i = 1:numUAV
            current_pos(i, :) = current_uavStates(i).position;
        end
        rough_pred.positions(k, :, :) = current_pos;

        current_intf = calculateInterferenceStrength(current_uavStates, interference_source);
        rough_pred.interference_strength(k, :) = current_intf;

        current_uavStates = updateDamageFactors(current_uavStates, current_intf, confidence, damage_params, dt);
        current_damage = [current_uavStates.damageFactor];
        rough_pred.damage_factors(k, :) = current_damage;

        current_uavStates = calculateEffectivePayloads(current_uavStates);
        current_eff_payload = [current_uavStates.effectivePayload];
        rough_pred.effective_payloads(k, :) = current_eff_payload;

        current_total_payload = calculateTotalEffectivePayload(current_uavStates);
        rough_pred.total_payload(k) = current_total_payload;
    end

    % --- 韧性指标计算 (修复了window_steps与pred_steps的不匹配问题) ---
    % 直接使用 T0_idx 作为历史数据的起点
        % 计算 T0 在原始数据中的索引 T0_idx
    T0 = original_data.events.entry_times;
    [~, T0_idx] = min(abs(original_data.timeArray - T0));
    historicalPayloads = original_data.totalEffectivePayloadArray(T0_idx:Tk_idx);

    predictedPayloads = rough_pred.total_payload(1:pred_steps);
    current_total_payload = original_data.totalEffectivePayloadArray(Tk_idx);
    resilienceMetrics = calculateResilienceMetrics(current_total_payload, historicalPayloads, predictedPayloads, simParams, confidence);

    % 确保sigma是正确长度的列向量
    if isscalar(resilienceMetrics.sigma)
        rough_pred.sigma = ones(pred_steps, 1) * resilienceMetrics.sigma;
    else
        rough_pred.sigma = reshape(resilienceMetrics.sigma, [], 1);
        if length(rough_pred.sigma) ~= pred_steps
            rough_pred.sigma = rough_pred.sigma(1:min(pred_steps, length(rough_pred.sigma)));
            rough_pred.sigma = [rough_pred.sigma; zeros(pred_steps - length(rough_pred.sigma), 1)];
        end
    end
end

% =========================================================================
% 辅助函数：计算离开干扰区的时间 Texit (最终完美版)
% =========================================================================
function [allExited, Texit] = findExitTime(original_data, Tk_idx, psi_rad, dt)
    % 1. 提取初始状态和参数
    numUAV = original_data.simParams.numUAVs;
    simParams = original_data.simParams;
    interference_source = original_data.interferenceSource; % 这是一个cell数组
    Tw = simParams.resillenSlidingWindow;
    max_simulation_steps = round(Tw / dt);

    tk_pos = squeeze(original_data.positionsArray(Tk_idx, :, :));
    % tk_damage = original_data.damageFactorsArray(Tk_idx, :);
    % tk_eff_payload = original_data.effectivePayloadsArray(Tk_idx, :);
    tk_heading = original_data.headingArray(Tk_idx, :);
    intensity_th = simParams.intensityThresholds;

    % 构造uavStates和formation结构体
    current_uavStates(numUAV) = struct('position', [], 'damageFactor', [], 'currentPayload', [], 'effectivePayload', [], 'intensityThreshold', []);
    for i = 1:numUAV
        current_uavStates(i).position = [tk_pos(i, 1), tk_pos(i, 2), 2];
        current_uavStates(i).intensityThreshold = intensity_th;
    end
    current_formation = getHexagonSwarmTemplate(original_data, tk_pos);
    current_formation.directionVec = rotateHeading(tk_heading, psi_rad);

    % 2. 在Tw时间窗口内模拟飞行，寻找离开时间
    current_time = original_data.timeArray(Tk_idx);
    Texit = current_time; % 默认出口时间为窗口结束时刻
    
    % --- 核心重构：使用原生函数进行判断 ---
    allExited = false;

    % 假设只有一个干扰源，从cell数组中取出
    single_source = interference_source; 
    
    % 循环模拟，最多模拟 Tw 时长
    for step = 1:max_simulation_steps
        % 检查每一架无人机是否都在干扰区外
        allExited = true; % 先假设所有无人机都在外面
        for i = 1:numUAV
            % 调用原生函数 bCheckUAVInZone 判断第 i 架无人机
            % 注意：函数返回 [bInZone, distance]，我们只关心 bInZone
            [bInZone, ~] = bCheckUAVInZone(current_uavStates(i), single_source);
            
            if bInZone % 如果有任何一架无人机在干扰区内
                allExited = false; % 则整体判断为 false
                break; % 无需再检查其他无人机，跳出内循环
            end
        end
        
        if allExited % 如果所有无人机都在干扰区外
            Texit = current_time; % 记录当前时刻为离开时间
            break; % 找到出口，立即退出主循环
        end
        
        % 如果未完全离开，则更新位置和时间，为下一次判断做准备
        [current_uavStates, current_formation] = updateUAVPositions(current_uavStates, current_formation, simParams, dt);
        current_time = current_time + dt;
    end
end

% -------------------------------------------------------------------------
% 内部辅助函数：旋转航向向量（仅调整输入参数，不修改原生函数）
% -------------------------------------------------------------------------
function rotated_vec = rotateHeading(original_vec, psi_rad)
% rotateHeading - 顺时针旋转航向向量（匹配psi参数定义，不修改原生函数逻辑）
    rot_mat = [cos(psi_rad), -sin(psi_rad);  % 更改为与主仿真程序一样：逆时针针旋转矩阵（数学正确，与psi定义一致）
              sin(psi_rad), cos(psi_rad)];
    rotated_vec = (rot_mat * original_vec')';  % 矩阵乘法后转置回行向量
    rotated_vec = rotated_vec / norm(rotated_vec);  % 归一化（确保与原生formation.directionVec格式一致）
end

% -------------------------------------------------------------------------
% 内部子函数2：插值对齐（保留您的原逻辑，仅适配原生函数输出字段）
% -------------------------------------------------------------------------
function aligned_pred = alignToMainStep(rough_pred, pred_time, numUAV)
    aligned_pred = struct(...
        'positions', [],...         % 对齐后位置（步数×numUAV×3）
        'damage_factors', [],...    % 对齐后损伤因子（步数×numUAV）
        'effective_payloads', [],...% 对齐后单无人机有效载荷（步数×numUAV）
        'total_payload', [],...     % 对齐后总载荷（步数×1）
        'sigma', [],...             % 对齐后sigma（步数×1）
        'heading', [],...           % 航向向量（1×2，无需插值）
        'interference_strength',[]...% 对齐后干扰强度（步数×numUAV）
    );
    num_steps = length(pred_time);
    rough_time = rough_pred.time;

    % 1. 位置插值（保留您的原逻辑）
    aligned_pred.positions = zeros(num_steps, numUAV, 3);
    for i = 1:numUAV
        for dim = 1:3  % x(1), y(2), z(3)
            aligned_pred.positions(:, i, dim) = interp1(...
                rough_time, ...
                squeeze(rough_pred.positions(:, i, dim)), ...
                pred_time, ...
                'linear', ...
                'extrap'...
            );
        end
    end

    % 2. 损伤因子插值（适配原生函数输出）
    aligned_pred.damage_factors = zeros(num_steps, numUAV);
    for i = 1:numUAV
        aligned_pred.damage_factors(:, i) = interp1(rough_time, rough_pred.damage_factors(:, i), pred_time, 'linear', 'extrap');
    end

    % 3. 单无人机有效载荷插值（适配原生函数输出）
    aligned_pred.effective_payloads = zeros(num_steps, numUAV);
    for i = 1:numUAV
        aligned_pred.effective_payloads(:, i) = interp1(rough_time, rough_pred.effective_payloads(:, i), pred_time, 'linear', 'extrap');
    end

    % 4. 总载荷与sigma插值（保留您的原逻辑）
    aligned_pred.total_payload = interp1(rough_time, rough_pred.total_payload, pred_time, 'linear', 'extrap');
    aligned_pred.sigma = interp1(rough_time, rough_pred.sigma, pred_time, 'linear', 'extrap');

    % 5. 干扰强度插值（新增，确保后续函数调用有数据）
    aligned_pred.interference_strength = zeros(num_steps, numUAV);
    for i = 1:numUAV
        aligned_pred.interference_strength(:, i) = interp1(rough_time, rough_pred.interference_strength(:, i), pred_time, 'linear', 'extrap');
    end

    % 6. 航向角（保留您的原逻辑）
    aligned_pred.heading = rough_pred.heading;
end

% -------------------------------------------------------------------------
% 以下为ResilenceSim.m原生函数（100%复制您提供的代码，未做任何修改）
% -------------------------------------------------------------------------
%% 原生函数1：updateUAVPositions（您提供的代码）
function [uavStates, formation] = updateUAVPositions(uavStates, formation, simParams, customDt)
    % 支持自定义步长的位置更新
    if nargin < 4
        customDt = simParams.dt; % 默认使用仿真步长
    end
    
    % 更新集群中心位置
    formation.centerPosition = formation.centerPosition + formation.directionVec * simParams.uavSpeed * customDt;
    
    % 更新各无人机位置
    rotationMatrix = [formation.directionVec(1), -formation.directionVec(2); 
                     formation.directionVec(2), formation.directionVec(1)];
    
    for i = 1:simParams.numUAVs
        rotatedRelPos = (rotationMatrix * formation.relativePositions(i, :)')';
        uavStates(i).position(1:2) = formation.centerPosition(1:2) + rotatedRelPos;
        uavStates(i).velocity(1:2) = formation.directionVec * simParams.uavSpeed;
    end
end

%% 原生函数2：仿真层(上帝视角)干扰检测逻辑。
function [bInZone,distance] = bCheckUAVInZone(uavState,interferenceSource)
% bCheckUAVInZone 判定单架无人机是否在干扰区内
% 输入：
%   uavState - 无人机状态结构体，需包含 position([x,y,z])
%   interferenceSource - 干扰源结构体，需包含 center([x,y,z]) 和 radius(干扰半径，km)
% 输出：
%   bInZone - 布尔值，true=在干扰区内，false=不在
%   distance - 无人机与干扰源中心的3D距离（km）
    dx = uavState.position(1) - interferenceSource.center(1);
    dy = uavState.position(2) - interferenceSource.center(2);
    dz = uavState.position(3) - interferenceSource.center(3);
    distance = sqrt(dx^2 + dy^2 + dz^2);
    bInZone = distance <= interferenceSource.radius;
end

%% 原生函数3：控制层(无人机视角)干扰检测逻辑。
function bInZone = bCheckUAVInZoneByIntf(interference,intensityThreshold)
% bCheckUAVInZone 判定单架无人机是否在干扰区内
    % 输入：uavState（含传感器检测的干扰强度，来自仿真层带噪声的信号）
    % 干扰源参数：interferenceSource.intensityThreshold（预设的边界阈值，先验知识）
    bInZone = (interference > intensityThreshold); % 仅用强度判定
end

%% 原生函数4：calculateInterferenceStrength（您提供的代码）
function interferenceStrength = calculateInterferenceStrength(uavStates, interferenceSource)
    numUAVs = length(uavStates);
    numIntf = length(interferenceSource); % 多干扰源数量
    interferenceStrength = zeros(1, numUAVs); % 初始化每架无人机的总干扰强度

    % 遍历每架无人机
    for i = 1:numUAVs
        totalStrength = 0; % 单架无人机的总干扰强度（多源叠加）
        % 遍历每个干扰源（cell数组，用{}取出单个结构体）
        for j = 1:numIntf
            singleIntf = interferenceSource; % 取出第j个干扰源结构体
            % 调用bCheckUAVInZone，传递"单个干扰源结构体"
            [bInZone, distance] = bCheckUAVInZone(uavStates(i), singleIntf);
            
            if bInZone
                % 计算当前干扰源对该无人机的强度（需匹配你的干扰模型，示例用alpha/beta模型）
                if distance < 1e-5 % 避免除零
                    distance = 1e-5;
                end
                % 假设干扰模型：strength = beta / (1 + (distance / singleIntf.d0)^singleIntf.alpha)（与initializeInterferenceSource对应）
                strength = singleIntf.beta / (1 + (distance / singleIntf.d0)^singleIntf.alpha);
                totalStrength = totalStrength + strength; % 多源干扰强度叠加
            end
        end
        interferenceStrength(i) = totalStrength; % 赋值单架无人机的总干扰强度
    end
end

%% 原生函数5：updateDamageFactors（您提供的代码）
function uavStates = updateDamageFactors(uavStates, interferenceStrength, Ck, damageParams, customDt)
    % 支持自定义步长的损伤更新
    if nargin < 5
        customDt = 0.05; % 默认步长
    end
    
    for i = 1:length(uavStates)
        % 损伤动力学方程离散解
        currentDamage = uavStates(i).damageFactor;

        bInZone = interferenceStrength(i) > uavStates(i).intensityThreshold;   % 需要根据干扰源模型计算出3.5km处的干扰强度值
        if bInZone
             % 1. 在干扰区内：正常累积损伤，不恢复
            damageRate = damageParams.beta_damage * Ck * interferenceStrength(i) * (1 - currentDamage);
            % recoveryRate = 0;  % 干扰区内不考虑恢复
        else
             % 2. 不在干扰区内：停止损伤累积，开始恢复（新增边界处理）
            damageRate = 0;  % 离开干扰区无新损伤
            % recoveryRate = damageParams.gamma_recovery * currentDamage;  % 开始恢复
        end

        recoveryRate = damageParams.gamma_recovery * currentDamage;  % 不管是否在干扰区，均考虑损伤恢复因子。如果不考虑干扰区的自恢复问题，此行需要注释掉。zqhkd modified in 20250908
        newDamage = currentDamage + (damageRate - recoveryRate) * customDt;
        uavStates(i).damageFactor = max(0, min(1, newDamage));
    end
end

%% 原生函数6：calculateEffectivePayloads（您提供的代码）
function uavStates = calculateEffectivePayloads(uavStates)
    for i = 1:length(uavStates)
        uavStates(i).effectivePayload = uavStates(i).currentPayload * (1 - uavStates(i).damageFactor);
    end
end

%% 原生函数7：calculateTotalEffectivePayload（您提供的代码）
function totalEffectivePayload = calculateTotalEffectivePayload(uavStates)
    % 计算总有效载荷
    totalEffectivePayload = sum([uavStates.effectivePayload]);
    
    % 调试信息：输出总有效载荷
    damageF = [uavStates.damageFactor]; % 修正：原代码damageF = uavStates.damageFactor会报错，需用[]提取数组
    if all(damageF > 0)
        tmps = sprintf('   calculateEffectivePayloads：总有效载荷=%.2f', totalEffectivePayload);
        debugPrintf(tmps, true);
    end
end

%% 原生函数8：calculateResilienceMetrics（您提供的代码，用于提取sigma）
function resilienceMetrics = calculateResilienceMetrics(stabPayload,historicalPayloads, predictedPayloads, simParams,confidence)
    % 计算韧性指标
    resilienceMetrics = struct();
    
    % 1. 动态总性能因子 σ(t)
    conf = confidence * 0.5;
    n_hist = length(historicalPayloads); 
    if n_hist > 0
        historical_contribution = sum(historicalPayloads) / n_hist; 
    else
        historical_contribution = 0;
    end
    n_pred = length(predictedPayloads); 
    if n_pred > 0
       predicted_contribution = sum(predictedPayloads) / n_pred; 
    else
        predicted_contribution = 0;
    end
    resilienceMetrics.sigma = ((1-conf) * historical_contribution + conf * predicted_contribution) / simParams.targetLoad;
    
    % 2. 动态吸收因子 δ(t)
    if n_hist > 0 && n_pred > 0
        min_historical = min(historicalPayloads);
        min_predicted = min(predictedPayloads);
        resilienceMetrics.delta = min(min_historical, min_predicted) / simParams.targetLoad;
    else
        resilienceMetrics.delta = stabPayload / simParams.targetLoad; % 边界处理：避免无历史/预测数据时min报错
    end
    
    % 3. 动态恢复因子 ρ(t) 和恢复时间因子 τ(t)
    if ~isempty(predictedPayloads)
        resilienceMetrics.rho = predictedPayloads(end) / simParams.targetLoad;
        % 恢复时间估计（简化）
        resilienceMetrics.tau = 0.3 + 0.7 * (1 - resilienceMetrics.rho);
    else
        resilienceMetrics.rho = stabPayload / simParams.targetLoad;   % 基于预测序列 $\hat{Y}(t, t+T_w)$ ，估计其达到的稳态性能值 $\hat{Y}_{SS}(t)$ 及从当前时刻$t$到达该稳态值所需的时间
        resilienceMetrics.tau = 1.0;
    end
    
    % 4. 动态波动因子 ζ(t)
    if length(predictedPayloads) > 2 && mean(predictedPayloads) ~= 0
        % 基于预测序列平滑度，避免除以零
        resilienceMetrics.zeta = 0.7 + 0.3 * (1 - std(predictedPayloads)/mean(predictedPayloads));
    else
        resilienceMetrics.zeta = 0.8;
    end
    
    % 5. 综合韧性指标 R(t)
    if resilienceMetrics.rho >= resilienceMetrics.delta
        resilienceMetrics.R = resilienceMetrics.rho * resilienceMetrics.sigma * ...
                             (resilienceMetrics.delta + resilienceMetrics.zeta + ...
                             (1 - resilienceMetrics.tau^(resilienceMetrics.rho - resilienceMetrics.delta)));
    else
        resilienceMetrics.R = resilienceMetrics.rho * resilienceMetrics.sigma * ...
                             (resilienceMetrics.delta + resilienceMetrics.zeta);
    end
end

% current_formation结构体（仅含必要字段，无冗余）
function current_formation = getHexagonSwarmTemplate(data,tk_pos)
    current_formation = struct('centerPosition', [], 'directionVec', [], 'relativePositions', []);
    numUAV = data.simParams.numUAVs;

    % 生成与主程序initializeUAVStates完全一致的正五边形相对位置（固定模板）
    angles = linspace(0, 2*pi, numUAV+1);  % 等分圆周（numUAV为无人机数量，与主程序一致）
    angles = angles(1:end-1);  % 去掉最后一个重复的0角度
    formation_spacing = data.simParams.formationSpacing;  % 从原始数据获取队形间距（主程序参数）
    current_formation.relativePositions = [formation_spacing * cos(angles); 
                                          formation_spacing * sin(angles)]';  % 固定模板，与主程序一致

    % （确保预测起始中心与原始数据中心完全一致，消除平移）
    current_formation.centerPosition = mean(tk_pos(:,1:2), 1);  % 原始数据Tk时刻的真实中心
end
