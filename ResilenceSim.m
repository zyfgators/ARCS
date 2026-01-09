function ResilenceSim(curMode, bConfAnaFlg, gammaRecovery)
%% 无人机集群韧性控制仿真 - 多模式对比版（支持C0/C1/C2）
% 功能：生成区分控制模式的科研数据，支持无控制(C0)、经典规避(C1)、韧性控制(C2)

% 新增控制模式参数CtrlMode（0 = 无控制，1 = 经典干扰规避，2 = 韧性控制）；
% 完善resilienceController调用逻辑，将控制指令接入编队航向更新；
% 数据存盘文件名标注控制模式（如ResilenceCtrlSimData_C2.mat）；
% 确保所有模式下数据记录维度一致，便于后续对比分析。

    %% 获取数据
    % clear; close all; clc;
    rng(42); % 设置随机种子以保证结果可重现
    
    % 全局缓存：存储[时间戳, 估计x, 估计y, 置信度]，最多保留10个最新高置信度结果
    global high_conf_pos_cache;
    if ~exist('high_conf_pos_cache', 'var')
        high_conf_pos_cache = [];
    end
    
    %% 1. 初始化参数
    simParams = initializeSimulationParameters();
    
    simParams.CtrlMode   = curMode; % 控制模式：0=无控制，1= 人工势场法APF，2=R韧性控制, 3=性能因子
    simParams.ConfAnaFlg = bConfAnaFlg;   % 置信度分析标志、韧性分析标志
    
    if simParams.ConfAnaFlg
       simParams.ResilenceSimStep = 1;  % 当进行置信度分析实验时，提高和韧性相关参数的输出周期
    end
    
    %% 2. 初始化干扰源
    % simParams.numInterference = 2; % 干扰源数量（默认为1个干扰源）
    
    interferenceSource = cell(1, simParams.numInterference); % 改为cell数组存储多个干扰源
    for i = 1:simParams.numInterference
        interferenceSource{i} = initializeInterferenceSource(i); % 传入序号，便于区分参数（如位置、强度）
    end
    globalIntensityThreshold = min([interferenceSource{:}.intensityThreshold]);
    simParams.intensityThresholds = globalIntensityThreshold;   % 保存最小感染阈值，以方便事后分析
    
    %% 3. 初始化损伤动力学参数
    damageParams = initializeDamageParameters(0.03, gammaRecovery);   % 自恢复速率系数
    
    %% 4. 初始化无人机状态
    [uavStates, formation] = initializeUAVStates(simParams,globalIntensityThreshold);
    
    %% 5. 初始化韧性评估参数
    [resilienceMetrics, resilienceParams] = initializeResilienceParameters(simParams);
    
    %% 6. 使用工作区变量代替数据记录结构体
    % 6.1 基础状态数组
    timeArray = zeros(simParams.totalSteps, 1);
    positionsArray = zeros(simParams.totalSteps, simParams.numUAVs, 3); % [步长, 无人机数, xyz]
    velocitiesArray = zeros(simParams.totalSteps, simParams.numUAVs, 3);
    attenuationArray = zeros(simParams.totalSteps, simParams.numUAVs); % 干扰强度
    damageFactorsArray = zeros(simParams.totalSteps, simParams.numUAVs); % 损伤因子
    effectivePayloadsArray = zeros(simParams.totalSteps, simParams.numUAVs); % 单机有效载荷
    totalEffectivePayloadArray = zeros(simParams.totalSteps, 1); % 总有效载荷
    
    % 6.2 韧性指标数组
    sigmaArray = zeros(simParams.totalSteps, 1);
    deltaArray = zeros(simParams.totalSteps, 1);
    rhoArray = zeros(simParams.totalSteps, 1);
    tauArray = zeros(simParams.totalSteps, 1);
    zetaArray = zeros(simParams.totalSteps, 1);
    RArray = zeros(simParams.totalSteps, 1);
    
    % 6.3 干扰估计与控制指令数组（关键对比数据）
    estimatedPositionsArray = zeros(simParams.totalSteps, 2); % 干扰源估计位置
    confidencesArray = zeros(simParams.totalSteps, 1); % 估计置信度
    headingArray = zeros(simParams.totalSteps, 2); % 编队航向向量（x/y）
    controlTriggerArray = zeros(simParams.totalSteps, 1); % 控制触发标记（1=触发，0=未触发）
    % 新增：提前初始化干扰源匹配数组（避免循环内重复创建）
    matchedIntfIndexArray = zeros(simParams.totalSteps, simParams.numUAVs); % 步长×无人机数，值=干扰源序号（0=无）
    
    %% 7. 初始化STCL状态管理
    % 7.1 关键事件状态
    stclState.inInterferenceZone = false;
    stclState.interferenceStartTime = 0;
    stclState.lastInterferenceTime = 0;
    stclState.active = false;
    % 上一时刻单机在干扰区状态（与无人机数量一致）
    stclState.prevUavInZoneArray = false(1, simParams.numUAVs);
    % 上一时刻STCL激活状态（用于避免重复记录激活事件）
    stclState.prevActive = false;
    
    totalTimeSteps = simParams.totalTime / simParams.dt; % 计算总时间步数
    stclEstimatedPara = repmat(struct( ...
        'time', 0, ...                  % 时间（秒）
        'alpha', 1.2, ...              % 其他参数1（根据实际参数定义）
        'beta', 1.0, ...              % 其他参数2
        'd0', 1.0), ...            % 标记该时间步是否有有效估计
        totalTimeSteps, 1);             % 预分配总时间步大小的数组
    
    % 7.2 用于事后数据分析和绘图的初始化关键事件记录（仅记录时间戳）
    global researchData;
    researchData.events = struct();
    researchData.events.entry_times = [];      % 编队进入干扰区时间
    researchData.events.exit_times = [];       % 编队离开干扰区时间
    researchData.events.stcl_activation_times = []; % stcl激活时间
    researchData.events.control_trigger_times = []; % 控制触发时间
    % 1. 单机干扰区时间记录（用于计算次数和飞行时间）
    for i = 1:simParams.numUAVs
        researchData.events.uav{i} = struct(...
            'entry_time', [], ...  % 第i架进入时间
            'exit_time', [], ...   % 第i架离开时间
            'in_zone_duration', 0); % 第i架在干扰区总飞行时间
    end
    % 2. 到达目标时的关键状态（新增）
    researchData.final_state = struct(...
        'total_time', 0,...                
        'uav_effective_payloads', [], ...  
        'total_effective_payload', 0, ...   
        'interference_zone_total_time', 0, ... % 后续会删除，暂保留
        'obj_interference_zone_total_time', 0, ... % 新增：客观时长初始化
        'sub_interference_zone_total_time', 0); % 新增：主观时长初始化
    
    %% 主仿真循环（核心：接入控制指令更新编队航向）
    fprintf('开始仿真... 总步长：%d，时间步长：%.2fs\n', simParams.totalSteps, simParams.dt);
    
    confidence = 0;          
    estimatedPos = [40, 35];  % 初始干扰源估计（默认值）
    estimatedInterference = struct('position', estimatedPos, 'confidence', confidence, ...
        'all_intf_params', interferenceSource, ...
        'alpha',1.2,'beta',1,'d0',1); % 传入所有干扰源的完整参数（cell数组）
    % 始终初始化confidenceData，避免字段不存在
    estimatedInterference.confidenceData = struct('Q1', 0, 'Q2', 0, 'DQ', 0, 'RE', 0, 'SI', 0);
    
    % 在主仿真循环前初始化controlAction
    controlAction = struct('adjustHeading', false,'reason', '初始状态','CtrlMode', simParams.CtrlMode,'confidence', 0);
    
    for t = 1:simParams.totalSteps
        currentTime = t * simParams.dt;
        timeArray(t) = currentTime;
        
        %% -------------------------- 1. 无人机运动更新（含控制指令）--------------------------
        % 1.1 调用resilienceController获取新航向（根据CtrlMode输出不同指令）
        [newDirection, controlAction] = flyController(...
            uavStates, resilienceMetrics, estimatedPos, confidence, simParams, formation, stclState);
        
        % 1.2 更新编队航向（核心：将控制指令接入运动学）
        if ~isempty(newDirection)
            formation.directionVec = newDirection;
        end
        headingArray(t, :) = formation.directionVec; % 记录航向，用于后续分析
        controlTriggerArray(t) = controlAction.adjustHeading; % 记录控制触发状态
        if controlAction.adjustHeading && ~ismember(currentTime, researchData.events.control_trigger_times)
            researchData.events.control_trigger_times = [researchData.events.control_trigger_times, currentTime];
        end
    
        % 1.3 更新无人机位置与速度
        [uavStates, formation] = updateUAVPositions(uavStates, formation, simParams);
    
        % 1.4 存储位置和速度
        for i = 1:simParams.numUAVs
            positionsArray(t, i, :) = uavStates(i).position;
            velocitiesArray(t, i, :) = uavStates(i).velocity;
        end
        
        %% -------------------------- 2. 干扰强度与损伤更新 --------------------------
        % 2.1 干扰源干扰特性模拟
        interferenceStrength = calculateInterferenceStrength(uavStates, interferenceSource);
        attenuationArray(t, :) = interferenceStrength;
        
        % 2.1.1 新增：匹配当前作用的干扰源（后台记录，控制层不感知）
        % 初始化"每架无人机当前匹配的干扰源序号"（0=无干扰）
        % if ~exist('matchedIntfIndexArray', 'var')
        %     matchedIntfIndexArray = zeros(simParams.totalSteps, simParams.numUAVs); % 步长×无人机数，值=干扰源序号（0=无）
        % end
        
        for i = 1:simParams.numUAVs
            matchedIndex = 0; % 初始：无干扰源
            uav_pos_2d = uavStates(i).position(1:2);
            
            % 遍历所有干扰源，匹配唯一可能作用的源（强度超阈值+距离在半径内）
            for j = 1:simParams.numInterference
                intf = interferenceSource{j};
                % 双重验证：1. 强度超当前干扰源阈值；2. 距离在当前干扰源半径内
                dist = norm(uav_pos_2d - intf.center(1:2));
                if interferenceStrength(i) > intf.intensityThreshold && dist < intf.radius
                    matchedIndex = j; % 匹配到唯一干扰源（因无重叠，仅1个）
                    break; % 无需继续遍历
                end
            end
            matchedIntfIndexArray(t, i) = matchedIndex; % 记录当前匹配的干扰源序号
        end
    
        % 2.2 更新损伤因子
        uavStates = updateDamageFactors(uavStates, interferenceStrength, confidence, damageParams, simParams.dt);
    
        % 2.3 更新有效载荷
        uavStates = calculateEffectivePayloads(uavStates);
        totalEffectivePayload = calculateTotalEffectivePayload(uavStates);
        totalEffectivePayloadArray(t) = totalEffectivePayload;
        for i = 1:simParams.numUAVs
            damageFactorsArray(t, i) = uavStates(i).damageFactor;
            effectivePayloadsArray(t, i) = uavStates(i).effectivePayload;
        end
    
    %% -------------------------- 3. STCL激活与干扰估计 --------------------------    
        % 3.1 从控制层（无人机视角）检测是否在干扰区状态。计算每架无人机的在区状态
        % 多干扰源：单机在区判定（只要在任一干扰区，即视为在区）
        uavInZoneArray = false(1, simParams.numUAVs);
        for i = 1:simParams.numUAVs
            if bCheckUAVInZoneByIntf(interferenceStrength(i),globalIntensityThreshold )
                uavInZoneArray(i) = true;  % 任一干扰区在区，即标记为在区
            end
        end
        
        % 记录单机进入/离开干扰区事件（与上一时刻对比）
        if ~exist('prev_uavInZoneArray', 'var') || isempty(prev_uavInZoneArray)
            prev_uavInZoneArray = false(1, simParams.numUAVs);
        end
    
        for i = 1:simParams.numUAVs
            % 进入事件：上一时刻不在区，当前在区
            if uavInZoneArray(i) && ~prev_uavInZoneArray(i)
                researchData.events.uav{i}.entry_time = [researchData.events.uav{i}.entry_time, currentTime];
            end
            % 离开事件：上一时刻在区，当前不在区
            if ~uavInZoneArray(i) && prev_uavInZoneArray(i)
                researchData.events.uav{i}.exit_time = [researchData.events.uav{i}.exit_time, currentTime];
                % 计算单次在区时间（最后一次进入到当前离开）
                if length(researchData.events.uav{i}.entry_time) >= length(researchData.events.uav{i}.exit_time)
                    single_duration = currentTime - researchData.events.uav{i}.entry_time(end);
                    researchData.events.uav{i}.in_zone_duration = researchData.events.uav{i}.in_zone_duration + single_duration;
                end
            end
        end
        prev_uavInZoneArray = uavInZoneArray; % 更新上一时刻状态
        
        % --------------------------4. 编队在区时间统计（区分客观/主观） --------------------------
        % 4.1 客观编队在区时间（仿真层上帝视角：基于真实位置和干扰源参数）
        % 4.1.1 计算每架无人机的客观在区状态（真实位置是否在干扰源物理范围内）
        % matchedIndex>0 表示无人机真实位置在某干扰源半径内（客观在区）
        obj_uavInZoneArray = (matchedIntfIndexArray(t, :) > 0);
    
        % 4.1.2 客观编队在区状态（只要有1架无人机客观在区，编队即客观在区）
        obj_cluster_in_zone = any(obj_uavInZoneArray);
        % 4.3 累加客观编队在区时间（连续在区时累加时间步）
        if ~exist('prev_formationInZone_obj', 'var') || isempty(prev_formationInZone_obj)
            prev_formationInZone_obj = false;
        end
        if obj_cluster_in_zone && prev_formationInZone_obj
            researchData.final_state.obj_interference_zone_total_time = ...
                researchData.final_state.obj_interference_zone_total_time + simParams.dt;
        end
        prev_formationInZone_obj = obj_cluster_in_zone;
        
        % 4.2. 主观编队在区时间（控制层视角：基于传感器强度判定）
        % 4.2.1 主观编队在区状态（控制层判定的编队在区状态）
        sub_cluster_in_zone = stclState.inInterferenceZone;
        % 4.2.2 累加主观编队在区时间
        if ~exist('prev_formationInZone_sub', 'var') || isempty(prev_formationInZone_sub)
            prev_formationInZone_sub = false;
        end
        if sub_cluster_in_zone && prev_formationInZone_sub
            researchData.final_state.sub_interference_zone_total_time = ...
                researchData.final_state.sub_interference_zone_total_time + simParams.dt;
        end
        prev_formationInZone_sub = sub_cluster_in_zone;
        
        % 删除原混淆字段（统一用obj_和sub_前缀区分）
        % researchData.final_state = rmfield(researchData.final_state, 'interference_zone_total_time');
    
        % 4.3.2 更新STCL状态与事件记录（新增：先同步控制层编队在区状态）
        oldState = stclState;  % 保存旧状态用于对比
        % 核心：控制层判定的编队在区状态（任一无人机强度超全局阈值）
        stclState.inInterferenceZone = any(uavInZoneArray); 
        % 再调用函数更新其他STCL状态（如active）
        stclState = updateSTCLState(stclState, uavInZoneArray, currentTime);
    
        % 记录编队进入/离开干扰区事件（控制层视角）
        if stclState.inInterferenceZone && ~oldState.inInterferenceZone
            researchData.events.entry_times = [researchData.events.entry_times, currentTime];
        end
        if ~stclState.inInterferenceZone && oldState.inInterferenceZone
            researchData.events.exit_times = [researchData.events.exit_times, currentTime];
        end
        
        % 记录STCL激活事件（状态从非激活→激活时）
        if stclState.active && ~oldState.active
            researchData.events.stcl_activation_times = [researchData.events.stcl_activation_times, currentTime];
        end
    
        %% -------------------------- 5. 韧性指标计算（每5步执行一次） --------------------------
        resilienceCalculatedThisStep = false; % 标记本步是否已计算
        if mod(t, simParams.ResilenceSimStep) == 0
            % 5.1 准备历史数据
            [allPositions, allAttenuation, allVelocities] = prepareAllHistoricalData(positionsArray, attenuationArray, velocitiesArray, t);
            
            % 5.2 STCL干扰源定位估计
            if stclState.active
                estimatedInterference = STCL_NN_Real(allPositions, allAttenuation, allVelocities, t, simParams, globalIntensityThreshold, estimatedInterference);
                estimatedPos = estimatedInterference.position(1:2);
                confidence = estimatedInterference.confidence;
            end
    
            % 5.3 计算韧性指标（【修改4：扩展计算条件，包含非干扰区场景】）
            % 条件1：干扰区且置信度足够；条件2：非干扰区且已计算过至少一次（t>1）
            if (stclState.active && confidence > 0.3) || (~stclState.active && t > 1)
                % 预测未来载荷（内部需兼容非干扰区场景：无干扰时仅自恢复）
                predictedPayloads = predictFuturePayloads(uavStates, formation, estimatedInterference, simParams, damageParams,stclState.inInterferenceZone);
                % 准备历史载荷
                windowSteps = round(resilienceParams.slidingWindow / simParams.dt);
                historicalSteps = max(1, t - windowSteps + 1):t;
                historicalPayloads = totalEffectivePayloadArray(historicalSteps);
                % 【修改5：传入是否在干扰区的状态，用于韧性计算函数内部区分场景】
                resilienceMetrics = calculateResilienceMetrics(...
                    totalEffectivePayload, historicalPayloads, predictedPayloads, simParams, confidence); 
                resilienceCalculatedThisStep = true;
            end
        end
        estimatedPositionsArray(t, :) = estimatedInterference.position(1:2);
        confidencesArray(t) = estimatedInterference.confidence;
        stclEstimatedPara(t).alpha = estimatedInterference.alpha;
        stclEstimatedPara(t).beta = estimatedInterference.beta;
        stclEstimatedPara(t).d0 = estimatedInterference.d0;
    
        if simParams.ConfAnaFlg
            % 新增：记录置信度中间参数
            confidenceMetricsArray(t).time = t;
            confidenceMetricsArray(t).Q1 = estimatedInterference.confidenceData.Q1;
            confidenceMetricsArray(t).Q2 = estimatedInterference.confidenceData.Q2;
            confidenceMetricsArray(t).DQ = estimatedInterference.confidenceData.DQ;
            confidenceMetricsArray(t).RE = estimatedInterference.confidenceData.RE;
            confidenceMetricsArray(t).SI = estimatedInterference.confidenceData.SI;
        end
        
        % 5.4 记录韧性指标（未计算时保持上一步值）
        if resilienceCalculatedThisStep
            sigmaArray(t) = resilienceMetrics.sigma;
            deltaArray(t) = resilienceMetrics.delta;
            rhoArray(t) = resilienceMetrics.rho;
            tauArray(t) = resilienceMetrics.tau;
            zetaArray(t) = resilienceMetrics.zeta;
            RArray(t) = resilienceMetrics.R;
        elseif t > 1
            % 保持上一次计算的值
            sigmaArray(t) = sigmaArray(t-1);
            deltaArray(t) = deltaArray(t-1);
            rhoArray(t) = rhoArray(t-1);
            tauArray(t) = tauArray(t-1);
            zetaArray(t) = zetaArray(t-1);
            RArray(t) = RArray(t-1);
        end
       
        %% -------------------------- 6. 仿真终止条件 --------------------------
        if checkTerminationCondition(uavStates, simParams.targetPosition, currentTime)
            fprintf('仿真终止：到达目标位置（时间：%.1fs）\n', currentTime);
            % 新增：记录到达目标时的最终状态
            researchData.final_state.total_time = currentTime; % 总飞行时间
            % 到达目标时单机有效载荷
            for i = 1:simParams.numUAVs
                researchData.final_state.uav_effective_payloads(i) = uavStates(i).effectivePayload;
            end
            % 到达目标时集群总载荷
            researchData.final_state.total_effective_payload = sum(researchData.final_state.uav_effective_payloads);
            break;
        end
        
        %% -------------------------- 7. 实时状态显示（每5步输出） --------------------------
        if mod(t, 5) == 0
            displayRealTimeStatus(currentTime, uavStates, interferenceStrength, ...
                estimatedPos, confidence, resilienceMetrics, controlAction);
        end
    end
    
    %% -------------------------- 8. 数据存盘（核心：标注控制模式） --------------------------
    fprintf('\n开始保存科研数据... 控制模式：CtrlMode=%d\n', simParams.CtrlMode);
    % 截断有效数据（去除未执行的步长）
    validSteps = find(timeArray > 0, 1, 'last');
    maxAtt = max(max(attenuationArray(1:validSteps, :)));
    normlizeGain = maxAtt/abs(maxAtt - globalIntensityThreshold);
    researchData = struct(...
        'CtrlMode', simParams.CtrlMode, ... % 标注控制模式（关键对比字段）
        'timeArray', timeArray(1:validSteps), ...
        'positionsArray', positionsArray(1:validSteps, :, :), ...
        'velocitiesArray', velocitiesArray(1:validSteps, :, :), ...
        'attenuationArray', normlizeGain*max(attenuationArray(1:validSteps, :)-globalIntensityThreshold,0), ...
        'damageFactorsArray', damageFactorsArray(1:validSteps, :), ...
        'effectivePayloadsArray', effectivePayloadsArray(1:validSteps, :), ...
        'totalEffectivePayloadArray', totalEffectivePayloadArray(1:validSteps), ...
        'sigmaArray', sigmaArray(1:validSteps), ...
        'deltaArray', deltaArray(1:validSteps), ...
        'rhoArray', rhoArray(1:validSteps), ...
        'tauArray', tauArray(1:validSteps), ...
        'zetaArray', zetaArray(1:validSteps), ...
        'RArray', RArray(1:validSteps), ...
        'estimatedPositionsArray', estimatedPositionsArray(1:validSteps, :), ...
        'confidencesArray', confidencesArray(1:validSteps), ...
        'headingArray', headingArray(1:validSteps, :), ...
        'controlTriggerArray', controlTriggerArray(1:validSteps), ...
        'events', researchData.events, ...
        'final_state', researchData.final_state, ... % 到达目标时的关键状态
        'simParams', simParams, ...
        'interferenceSource', interferenceSource, ...
        'damageParams', damageParams, ...
        'stclEstimatedPara',stclEstimatedPara);
    
    % 根据标志位添加可选字段
    if simParams.ConfAnaFlg && ~isempty(confidenceMetricsArray)
        researchData.confidenceMetricsArray = confidenceMetricsArray.';
    end
    
    % 文件名标注控制模式（C0/C1/C2）
    fileName = sprintf('ResilenceCtrlSimData_C%d.mat', simParams.CtrlMode);
    save(fileName, '-struct', 'researchData', '-v7.3'); % 高版本格式，支持大文件
    fprintf('数据保存完成：%s\n', fileName);
    
    %% -------------------------- 9. 关键事件统计输出 --------------------------
    fprintf('\n=== 仿真关键事件统计（CtrlMode=%d） ===\n', simParams.CtrlMode);
    % 单机事件统计
    for i = 1:simParams.numUAVs
        fprintf('无人机%d：进入干扰区%d次，离开%d次\n', ...
            i, length(researchData.events.uav{i}.entry_time), length(researchData.events.uav{i}.exit_time));
    end
    % STCL与控制事件统计
    fprintf('STCL激活：%d次\n', length(researchData.events.stcl_activation_times));
    fprintf('控制触发：%d次\n', length(researchData.events.control_trigger_times));
    fprintf('总有效载荷范围：[%.2f, %.2f]\n', ...
        min(totalEffectivePayloadArray(1:validSteps)), max(totalEffectivePayloadArray(1:validSteps)));
    fprintf('综合韧性指标范围：[%.3f, %.3f]\n', ...
        min(RArray(1:validSteps)), max(RArray(1:validSteps)));
end   % 结束ResilenceSim函数

%% 以下为初始化函数/核心计算函数（保持不变，确保兼容性）
function simParams = initializeSimulationParameters()
    % 任务参数
    simParams.startPoint = [0, 0];          % 起始点 (km)
    simParams.endPoint = [50, 40];          % 目标点 (km)
    simParams.targetPosition = [50, 40, 2]; % 目标位置 (km)
    
    simParams.targetLoad = 18;              % 目标总载荷需求
    
    % 无人机参数
    simParams.numUAVs = 5;
    simParams.initialPayloads = [4, 4, 4, 8, 10]; % 初始载荷性能
    simParams.formationSpacing = 0.6;              % 编队间隔 (km)
    simParams.uavSpeed = 50 / 1000;                % 飞行速度 (m/s)
    simParams.flightHeight = 2;                    % 飞行高度 (km)
    
    % 干扰源参数
    simParams.numInterference = 1; % 干扰源数量（默认为1个干扰源）
    simParams.intensityThresholds = 0;  

    % 仿真参数
    simParams.totalTime = 4000;    % 总仿真时间 (秒)
    simParams.dt = 0.05;           % 时间步长 (秒)
    simParams.totalSteps = ceil(simParams.totalTime / simParams.dt);
    simParams.T = 1;               % 时间滑窗长度 (秒)
    simParams.ResilenceSimStep = 5;  % 韧性参数输出周期，5个仿真步长输出1次
    simParams.resillenSlidingWindow = 50; % 韧性预测时滑动窗口长度最大值
    
    % 控制参数
    simParams.minResilienceThreshold = 1.2;  % 韧性阈值，低于此值触发控制
    simParams.maxHeadingChange = pi/18;      % 最大航向角变化 (10度)
    % simParams.CtrlMode = 0;                  % 默认为无控模式（可外部修改）

    % 新增σₜ控制参数（放入simParams初始化代码）
    simParams.sigma_threshold = 1.1;      % σₜ触发阈值（核心参数）
    
    simParams.CtrlMode = 3;                % 启用σₜ控制模式
end

function [uavStates, formation] = initializeUAVStates(simParams,globalIntensityThreshold)
    uavStates = struct();
    formation = struct();
    
    % 初始化正五边形编队
    angles = linspace(0, 2*pi, simParams.numUAVs+1);
    angles = angles(1:end-1);
    formation.relativePositions = [simParams.formationSpacing * cos(angles); 
                                  simParams.formationSpacing * sin(angles)]';
    
    % 初始航向向量
    formation.directionVec = (simParams.endPoint - simParams.startPoint) / ...
                            norm(simParams.endPoint - simParams.startPoint);
    
    % 初始化无人机状态
    for i = 1:simParams.numUAVs
        uavStates(i).position = [simParams.startPoint + formation.relativePositions(i, :), simParams.flightHeight];
        uavStates(i).velocity = [formation.directionVec * simParams.uavSpeed, 0];
        uavStates(i).initialPayload = simParams.initialPayloads(i);
        uavStates(i).currentPayload = simParams.initialPayloads(i);
        uavStates(i).damageFactor = 0;
        uavStates(i).effectivePayload = simParams.initialPayloads(i);

        uavStates(i).intensityThreshold = globalIntensityThreshold;   % 初始设定的有效干扰检测阈值
    end
    
    formation.centerPosition = simParams.startPoint;
end

function source = initializeInterferenceSource(seq)
    %% 定义干扰源的基本参数
    switch seq
        case 1
            % 干扰源1参数
            source.center = [44, 33, 0]; % 干扰源中心 (km)
            source.radius = 4;         % 干扰半径 (km)
        case 2
            % 干扰源2参数
            source.center = [70, 50, 0];  % 位置(x,y,z)
            source.radius = 12;             % 干扰半径
    end
    %% 定义干扰源模型参数
    source.alpha = 1.2;          % 干扰模型参数
    source.beta = 1;             % 干扰模型参数
    source.d0 = 1;               % 干扰模型参数
    source.noiseStd = 0.01;      % 噪声标准差

    % 定义干扰源阈值
    source.intensityThreshold = source.beta / (1 + (source.radius / source.d0)^source.alpha);
end

function damageParams = initializeDamageParameters(betaDamage,gammaRecovery)
    % damageParams.beta_damage = 0.03;     % 损伤速率系数
    % damageParams.gamma_recovery = 0.0; % 自恢复速率系数
    damageParams.beta_damage = betaDamage;     % 损伤速率系数
    damageParams.gamma_recovery = gammaRecovery; % 自恢复速率系数
end

function [resilienceMetrics,resilienceParams] = initializeResilienceParameters(simParams)
    resilienceParams.slidingWindow = simParams.resillenSlidingWindow;  % 滑动窗口长度 (秒)
    resilienceParams.windowSteps = ceil(resilienceParams.slidingWindow / simParams.dt);
    resilienceParams.SNR_k = 0.1;         % 波动因子参数
    resilienceParams.SNR_0 = 10;          % 波动因子参数
    % resilienceParams.minResilienceThreshold = 1.1;  % 韧性阈值，低于此值触发控制

    % 初始化韧性指标
    resilienceMetrics.sigma = 1;  % 动态总性能因子
    resilienceMetrics.delta = 0;  % 损伤因子
    resilienceMetrics.rho = 0;    % 动态恢复因子
    resilienceMetrics.tau = 0;    % 恢复时间因子
    resilienceMetrics.zeta = 0;   % 动态波动因子

    resilienceMetrics.R = sum(simParams.initialPayloads)/simParams.targetLoad;      % 综合韧性指标
end

%% 核心计算函数
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

%% 仿真层(上帝视角)干扰检测逻辑。如果用此函数，则不能在控制层中使用bInZone标志和distance距离参数
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

%% 控制层(无人机视角)干扰检测逻辑。
function bInZone = bCheckUAVInZoneByIntf(interference,intensityThreshold)
% bCheckUAVInZone 判定单架无人机是否在干扰区内
    % 输入：uavState（含传感器检测的干扰强度，来自仿真层带噪声的信号）
    % 干扰源参数：interferenceSource.intensityThreshold（预设的边界阈值，先验知识）
    bInZone = (interference > intensityThreshold); % 仅用强度判定
end

function interferenceStrength = calculateInterferenceStrength(uavStates, interferenceSource)
    numUAVs = length(uavStates);
    numIntf = length(interferenceSource); % 多干扰源数量
    interferenceStrength = zeros(1, numUAVs); % 初始化每架无人机的总干扰强度

    % 遍历每架无人机
    for i = 1:numUAVs
        totalStrength = 0; % 单架无人机的总干扰强度（多源叠加）
        % 遍历每个干扰源（cell数组，用{}取出单个结构体）
        for j = 1:numIntf
            singleIntf = interferenceSource{j}; % 取出第j个干扰源结构体
            % 调用bCheckUAVInZone，传递"单个干扰源结构体"
            [bInZone, distance] = bCheckUAVInZone(uavStates(i), singleIntf);
            
            if bInZone
                % 计算当前干扰源对该无人机的强度（需匹配你的干扰模型，示例用alpha/beta模型）
                if distance < 1e-5 % 避免除零
                    distance = 1e-5;
                end
                % 假设干扰模型：strength = beta / (1 + distance^alpha)（与initializeInterferenceSource对应）
                strength = singleIntf.beta / (1 + (distance / singleIntf.d0)^singleIntf.alpha);
                totalStrength = totalStrength + strength; % 多源干扰强度叠加
            end
        end
        interferenceStrength(i) = totalStrength; % 赋值单架无人机的总干扰强度
    end
end

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

        recoveryRate = damageParams.gamma_recovery * currentDamage;  % 不管是否在干扰区，均考虑损伤恢复因子。如果不考虑干扰区的自恢复问题，此行可注释掉或者设置自恢复因子gamma_recovery=0。zqhkd modified in 20250908
        newDamage = currentDamage + (damageRate - recoveryRate) * customDt;
        uavStates(i).damageFactor = max(0, min(1, newDamage));
    end
end

function uavStates = calculateEffectivePayloads(uavStates)
    for i = 1:length(uavStates)
        uavStates(i).effectivePayload = uavStates(i).currentPayload * (1 - uavStates(i).damageFactor);
    end
end

function totalEffectivePayload = calculateTotalEffectivePayload(uavStates)
    % 计算总有效载荷
    totalEffectivePayload = sum([uavStates.effectivePayload]);
    
    % 调试信息：输出总有效载荷
    damageF = uavStates.damageFactor;
    if all( damageF > 0)
        tmps = sprintf('   calculateEffectivePayloads：总有效载荷=%.2f', totalEffectivePayload);
        debugPrintf(tmps, true);
    end
end

function resilienceMetrics = calculateResilienceMetrics(stabPayload,historicalPayloads, predictedPayloads, simParams,confidence)
    % 计算韧性指标
    resilienceMetrics = struct();
    
    % 1. 动态总性能因子 σ(t)
    conf = confidence * 0.5;
    n_hist = length (historicalPayloads); 
    if n_hist > 0
        historical_contribution = sum(historicalPayloads) / n_hist; 
    else
        historical_contribution = 0;
    end
    n_pred = length (predictedPayloads); 
    if n_pred > 0
       predicted_contribution = sum(predictedPayloads) / n_pred; 
    else
        predicted_contribution = 0;
    end
    resilienceMetrics.sigma = ((1-conf) *historical_contribution + conf * predicted_contribution)/ simParams.targetLoad;
    
    % 2. 动态吸收因子 δ(t)
    min_historical = min(historicalPayloads);
    min_predicted = min(predictedPayloads);
    resilienceMetrics.delta = min(min_historical, min_predicted) / simParams.targetLoad;
    
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
    if length(predictedPayloads) > 2
        % 基于预测序列平滑度
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

function terminate = checkTerminationCondition(uavStates, targetPosition, currentTime)
    % 检查是否到达目标位置
    clusterCenter = mean(reshape([uavStates.position], 3, [])', 1);
    distanceToTarget = norm(clusterCenter - targetPosition);
    
    terminate = distanceToTarget < 0.1 || currentTime > 4000;
end

function displayRealTimeStatus(currentTime, uavStates, interferenceStrength, ...
    estimatedPos, confidence, resilienceMetrics, controlAction)
    % 显示仿真实时状态（修复字符串索引错误）
    % 检查controlAction是否有效
    if ~isstruct(controlAction) || ~isfield(controlAction, 'reason') || ~isfield(controlAction, 'CtrlMode')
        % 紧急处理：使用默认值
        controlAction = struct( ...
            'adjustHeading', false, ...
            'reason', '未知控制状态', ...
            'CtrlMode', 0, ...
            'confidence', 0);
    end
    
    % 安全获取控制原因（限制最大长度为50，防止索引错误）
    reasonStr = controlAction.reason;
    maxDisplayLen = 50;
    if length(reasonStr) > maxDisplayLen
        reasonDisplay = reasonStr(1:maxDisplayLen);
    else
        reasonDisplay = reasonStr; % 长度不足时直接使用完整字符串
    end
    
    % 计算集群状态
    clusterCenter = mean(reshape([uavStates.position], 3, [])', 1);
    totalPayload = sum([uavStates.effectivePayload]);
    avgDamage = mean([uavStates.damageFactor]) * 100;
    
    % 生成干扰状态标识
    inIntZone = zeros(1, length(uavStates));
    for i = 1:min(5, length(interferenceStrength))
        inIntZone(i) = interferenceStrength(i) > uavStates(i).intensityThreshold;
    end
    
    % 计算估计误差
    trueInterferencePos = [44, 33]; % 实际干扰源位置
    error = norm(estimatedPos(1:2) - trueInterferencePos);
    
    % 显示实时状态信息
    fprintf('时间: %.1fs | 模式: C%d | 中心: [%.1f, %.1f] | 干扰估计误差: %.2f | 置信度: %.2f | ', ...
            currentTime, ...
            controlAction.CtrlMode, ...
            clusterCenter(1), clusterCenter(2), ...
            error, confidence);
    
    fprintf('干扰状态: [%d,%d,%d,%d,%d] | 总载荷: %.1f/25 | 平均损伤: %.1f%% | 韧性: %.3f | ', ...
            inIntZone(1), inIntZone(2), inIntZone(3), inIntZone(4), inIntZone(5), ...
            totalPayload, ...
            avgDamage, ...
            resilienceMetrics.R);
    
    % 显示控制动作摘要（使用安全处理后的字符串）
    if controlAction.adjustHeading
        fprintf('控制: 已调整 | %s\n', reasonDisplay);
    else
        fprintf('控制: 未调整 | %s\n', reasonDisplay);
    end
end

%% 修改prepareAllHistoricalData函数。注意STCL_NN_Real函数的输入数据positions为二维数据
function [allPositions, allAttenuation, allVelocities] = prepareAllHistoricalData(positionsArray, attenuationArray, velocitiesArray, currentStep)
    % 1. 提取历史数据，但位置/速度仅保留x/y轴（2D），剔除z轴（与ml.m一致）
    allPositions = positionsArray(1:currentStep, :, 1:2);  % 只取x/y，维度变为 currentStep×numUAVs×2
    allAttenuation = attenuationArray(1:currentStep, :)/0.2797;  % 0.2797 为干扰强度量级系数，防止过小，辨识过早收敛
    allVelocities = velocitiesArray(1:currentStep, :, 1:2);% 速度同样保留x/y

    % 新增：筛选有效帧（至少1架无人机衰减>0.01，即进入干扰区的帧）
    % validFrames = sum(allAttenuation, 2) > 0.01;  % 每帧是否有有效衰减
    % allPositions = allPositions(validFrames, :, :);  % 仅保留有效帧的位置
    % allAttenuation = allAttenuation(validFrames, :);  % 仅保留有效帧的衰减
    % allVelocities = allVelocities(validFrames, :, :);% 仅保留有效帧的速度
end

function state = updateSTCLState(state, uavInZoneArray, currentTime)
% 输入新增：
%   state - 关键事件状态及发生时刻
%   uavInZoneArray - 单机在区状态数组（1×numUAVs，布尔值）
%   uavStates - 无人机状态结构体数组（用于获取单机ID，记录单机事件）
%   interferenceSource - 干扰源结构体（冗余，确保与bCheckUAVInZone参数一致）
%   currentTime - 当前时间
    % 添加全局变量声明
    global researchData;
    
    % 1. 计算集群状态（基于单机在区状态的逻辑或）
    clusterInZone = any(uavInZoneArray);
    allLeftZone = all(~uavInZoneArray); % 所有单机都不在区
    
    % 2. 记录单机事件（进入/离开）：对比当前与上一时刻的单机状态
    for i = 1:length(uavInZoneArray)
        currentInZone = uavInZoneArray(i);
        prevInZone = state.prevUavInZoneArray(i);
        
        % 单机进入事件：上一时刻不在，当前在
        if currentInZone && ~prevInZone
            researchData.events.uav{i}.entry_time = [researchData.events.uav{i}.entry_time, currentTime];
            fprintf('无人机 %d 在 %.1fs 进入干扰区\n', i, currentTime);
        end
        
        % 单机离开事件：上一时刻在，当前不在
        if ~currentInZone && prevInZone
            researchData.events.uav{i}.exit_time = [researchData.events.uav{i}.exit_time, currentTime];
            fprintf('无人机 %d 在 %.1fs 离开干扰区\n', i, currentTime);
        end
    end
    
    % 3. 集群状态切换与STCL激活逻辑（保留原逻辑，仅将判定条件改为clusterInZone）
    if clusterInZone && ~state.inInterferenceZone
        % 集群进入干扰区
        state.inInterferenceZone = true;
        state.interferenceStartTime = currentTime;
        state.active = false;
        % （可选）记录集群级进入事件（若需）
        % researchData.events.cluster_entry_times = [researchData.events.cluster_entry_times, currentTime];
    elseif allLeftZone && state.inInterferenceZone
        % 集群离开干扰区
        state.inInterferenceZone = false;
        state.lastInterferenceTime = currentTime;
        state.active = false;
        % （可选）记录集群级离开事件（若需）
        % researchData.events.cluster_exit_times = [researchData.events.cluster_exit_times, currentTime];
    end
    
    % STCL激活条件（保留原逻辑）
    if state.inInterferenceZone && (currentTime - state.interferenceStartTime >= 1.0)
        state.active = true;
        % 记录STCL激活事件（确保只记录1次）
        if ~state.prevActive  % 需在stclState中新增prevActive字段，避免重复记录
            researchData.events.stcl_activation_times = [researchData.events.stcl_activation_times, currentTime];
            state.prevActive = true;
        end
    else
        state.prevActive = false;
    end
    
    % 离开后保持激活逻辑（保留原逻辑）
    LEAVE_HOLD_TIME = 20;
    if ~state.inInterferenceZone && (currentTime - state.lastInterferenceTime < LEAVE_HOLD_TIME) && state.active
        state.active = true;
    elseif ~state.inInterferenceZone
        state.active = false;
    end
    
    % 保存当前单机在区状态，用于下一时刻对比
    state.prevUavInZoneArray = uavInZoneArray;
end

function predictedPayloads = predictFuturePayloads(uavStates, formation, estimatedInterference, simParams, damageParams, isInInterferenceZone)
    % 新增参数默认值：未传入时默认非干扰区，避免报错
    if nargin < 6
        isInInterferenceZone = false;
    end
    % 使用当前时刻t的实际状态作为预测起点
    predUavStates = uavStates;
    predFormation = formation;
    
    % 预测参数 - 加大步长减少计算量
    predDt = 5 * simParams.dt; % 主步长的5倍（0.25秒）
    maxPredictionTime = 50;    % 最大预测50秒
    leaveBufferTime = 10;      % 离开后额外预测10秒
    
    maxPredictionSteps = round(maxPredictionTime / predDt);
    predictedPayloads = zeros(maxPredictionSteps, 1);
    
    % 记录离开时间
    leaveTime = inf;
    hasLeft = false;
    
    for step = 1:maxPredictionSteps
        % 预测运动（使用加大步长）
        [predUavStates, predFormation] = updateUAVPositions(predUavStates, predFormation, simParams, predDt);

        % 核心：按场景计算干扰强度（仅计算一次）
        if isInInterferenceZone
            % 干扰区：正常计算干扰
            predInterference = calculatePredictedInterference(predUavStates, estimatedInterference);
        else
            % 非干扰区：强制干扰强度为0
            predInterference = zeros(length(predUavStates), 1);
        end

        % 检查是否全部离开干扰区（基于当前有效的predInterference）
        allOutside = true;
        for i = 1:length(predUavStates)
            if predInterference(i) >= predUavStates(i).intensityThreshold
                allOutside = false;
                break;
            end
        end
        
        % 记录离开时间（仅在首次离开时更新）
        if allOutside && ~hasLeft
            leaveTime = step * predDt;
            hasLeft = true;
        end
        
        % 更新损伤（使用当前有效的predInterference）
        predUavStates = updateDamageFactors(predUavStates, predInterference, estimatedInterference.confidence, damageParams, predDt);
        predUavStates = calculateEffectivePayloads(predUavStates);
        
        % 记录预测载荷
        predictedPayloads(step) = calculateTotalEffectivePayload(predUavStates);
        
        % 终止条件1：离开干扰区超过缓冲时间
        if hasLeft && (step * predDt - leaveTime >= leaveBufferTime)
            predictedPayloads = predictedPayloads(1:step);
            break;
        end
        
        % 终止条件2：达到最大预测时间
        if step * predDt >= maxPredictionTime
            predictedPayloads = predictedPayloads(1:step);
            break;
        end
    end
    
    % 确保至少有一个预测值（覆盖异常情况）
    if isempty(predictedPayloads) || all(predictedPayloads == 0)
        predictedPayloads = calculateTotalEffectivePayload(uavStates);
    end
end

function predInterference = calculatePredictedInterference(uavStates, estimatedInterference)
    % 使用估计参数计算预测干扰
    predInterference = zeros(1, length(uavStates));
    estPos = estimatedInterference.position;
    estAlpha = estimatedInterference.alpha;
    estBeta = estimatedInterference.beta;
    estD0 =   estimatedInterference.d0;
    
    for i = 1:length(uavStates)
        dx = uavStates(i).position(1) - estPos(1);
        dy = uavStates(i).position(2) - estPos(2);
        dz = uavStates(i).position(3);
        distance = sqrt(dx^2 + dy^2 + dz^2);
        predInterference(i) = estBeta / (1 + (distance / estD0) ^ estAlpha);
    end
end
