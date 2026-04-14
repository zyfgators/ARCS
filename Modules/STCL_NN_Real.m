function estimatedInterference = STCL_NN_Real(positions, attenuation, velocities, t, simParams, MIN_ATTENUATION, prevEstimatedInterference)
 % 算法名称：时空协同检测神经网络，
 %    Spatio-Temporal Collaborative Location Detection Neural Network, STCL_NN

 % positions   -- 无人机位置信息，仅包含x,y坐标信息，维度为zeros(timeSteps, numUAVs, 2);
 % attenuation -- 干扰量
 % velocities  --  速度信息
 % t  -- 当前场景时间
 % T  -- 滑动窗口大小
 % dt -- 仿真时间步长 
 
    %% 参数设置
    % 仿真参数处理
    T = simParams.T; dt = simParams.dt; enableConfidenceMetrics = simParams.ConfAnaFlg;

    window_size = max(10,T/dt);
    % MIN_ATTENUATION = 0.05;
    disturbedUAVs = find(attenuation(t, :) > MIN_ATTENUATION);
    
    % 初始化必要字段（确保任何分支都有基础结构）
    estimatedInterference = struct();
    estimatedInterference.alpha = 0;    % 初始化alpha
    estimatedInterference.beta = 0;     % 初始化beta
    estimatedInterference.d0 = 1.0;     % 初始化d0（保持原有）
    estimatedInterference.fiterror = NaN;
    estimatedInterference.position = [NaN, NaN, 0];
    estimatedInterference.confidence = 0;

    % 始终初始化confidenceData，避免字段不存在
    estimatedInterference.confidenceData = struct('Q1', 0, 'Q2', 0, 'DQ', 0, 'RE', 0, 'SI', 0);
    
    if isempty(disturbedUAVs)
        % 无干扰无人机时（离开干扰区）：复用历史alpha/beta
        if nargin >= 7 && isfield(prevEstimatedInterference, 'alpha')
            % 优先复用上一步的估计值
            estimatedInterference.alpha = prevEstimatedInterference.alpha;
            estimatedInterference.beta = prevEstimatedInterference.beta;
            estimatedInterference.d0 = prevEstimatedInterference.d0;
        else
            % 无历史值时用默认值（根据你的仿真设置调整）
            estimatedInterference.alpha = 2.0;  % 典型衰减系数
            estimatedInterference.beta = 10.0;  % 典型基础强度
        end
        if enableConfidenceMetrics && isfield(prevEstimatedInterference, 'confidenceData')
            estimatedInterference.confidenceData = prevEstimatedInterference.confidenceData;
        end        
        return;  % 保持原有逻辑，但已包含alpha字段
    end
    
    %% 步骤1: 计算累积干扰量并筛选高置信度点
    [high_confidence_points, confidence_scores, observed_attenuations] = findHighConfidencePoints(...
        positions, attenuation, disturbedUAVs, t, window_size, dt, MIN_ATTENUATION);
    
    if size(high_confidence_points, 1) < 3
        % 点太少，使用备用方法：复用历史或默认alpha
        [estimated_position, confidence] = fallbackMethod(...
            positions, attenuation, velocities, t, disturbedUAVs);
        fit_error = NaN;  
        
        % 补充alpha/beta（复用历史或默认）
        if nargin >=7 && isfield(prevEstimatedInterference, 'alpha')
            estimatedInterference.alpha = prevEstimatedInterference.alpha;
            estimatedInterference.beta = prevEstimatedInterference.beta;
        else
            estimatedInterference.alpha = 2.0;
            estimatedInterference.beta = 10.0;
        end
    else
        %% 步骤2: 非线性最小二乘拟合（原有逻辑）
        [estimated_position, fit_error, alpha_est, beta_est] = nonlinearLeastSquaresFit(...
            high_confidence_points, observed_attenuations, confidence_scores, ...
            positions, velocities, t, disturbedUAVs);
        % 保留拟合得到的alpha/beta（核心修改：确保赋值）
        estimatedInterference.alpha = alpha_est;  
        estimatedInterference.beta = beta_est;
        estimatedInterference.d0 = 1.0;
        estimatedInterference.fiterror = fit_error;

        %% 步骤3: 计算方位和置信度
        confidence = calculateFinalConfidence(fit_error, size(high_confidence_points, 1), confidence_scores);

        % 仅当开关开启时才计算额外参数。stcl置信度实验添加代码，20250919
        if enableConfidenceMetrics
            estimatedInterference.confidenceData = computeConfidenceMetrics(...
                high_confidence_points, observed_attenuations, fit_error, ...
                size(high_confidence_points, 1), confidence_scores, dt);
        end
    end

    % 更新位置和置信度
    estimatedInterference.position = estimated_position;
    estimatedInterference.confidence = confidence;

    % 更新高置信度结果缓存
    global high_conf_pos_cache;
    if estimatedInterference.confidence > 0.7  
        new_entry = [t, estimatedInterference.position(1), estimatedInterference.position(2), estimatedInterference.confidence];
        high_conf_pos_cache = [high_conf_pos_cache; new_entry];
        if size(high_conf_pos_cache, 1) > 10
            high_conf_pos_cache = high_conf_pos_cache(2:end, :);
        end
    end

    % 打印日志（补充alpha/beta信息）
    fprintf('时间：%.2f, 估计位置: (%.2f, %.2f), 误差：%.4f, (alp,beta):%0.4f,%.4f, 置信度: %.2f\n\n',...
            t, estimated_position(1), estimated_position(2), fit_error,estimatedInterference.alpha,estimatedInterference.beta, confidence);
end

function [points, confidences, attenuations] = findHighConfidencePoints(...
    positions, attenuation, disturbedUAVs, current_time, window_size, dt, MIN_ATTENUATION)
    
    % 存储所有无人机的所有窗口数据
    all_points = [];
    all_attenuations = [];
    all_confidences = [];
    all_uav_ids = [];
    
    % 处理每个受扰无人机的数据
    for i = 1:length(disturbedUAVs)
        uav_idx = disturbedUAVs(i);
        
        % 从当前时刻倒着处理滑动窗口
        [window_data, window_confidences] = optimizedSlidingWindow(...
            positions, attenuation, uav_idx, current_time, window_size, dt,MIN_ATTENUATION);
        
        if ~isempty(window_data)
            all_points = [all_points; window_data.points];
            all_attenuations = [all_attenuations; window_data.attenuations];
            all_confidences = [all_confidences; window_confidences];
            all_uav_ids = [all_uav_ids; repmat(uav_idx, size(window_data.points, 1), 1)];
        end
    end
    
    % 多机数据筛选
    if isempty(all_points) || (size(all_points,1) < 3)   % 所有数据点数小于3个
        points = [];
        confidences = [];
        attenuations = [];
        return;
    end
    
    [points, confidences, attenuations] = selectDiverseHighConfidencePoints(...
        all_points, all_attenuations, all_confidences, all_uav_ids);
end

function [window_data, confidences] = optimizedSlidingWindow(...
    positions, attenuation, uav_idx, current_time, window_size, dt,MIN_ATTENUATION)
    % 替代原processUAVSlidingWindow函数，修复维度不匹配问题
    window_data.points = [];
    window_data.attenuations = [];
    confidences = [];
    
    start_time = current_time;
    
    while start_time >= 1
        end_time = max(1, start_time - window_size + 1);
        time_steps = end_time:start_time;
        
        % 获取窗口内的衰减数据
        att_win = attenuation(time_steps, uav_idx);
        disturbed_mask = att_win > MIN_ATTENUATION;
        
        if sum(disturbed_mask) < max(2, window_size * 0.5)
            start_time = end_time - 1;
            continue;
        end
        
        % 关键修复：确保pos_win是二维矩阵（n×2）
        pos_win = squeeze(positions(time_steps, uav_idx, :));
        % 当窗口大小为1时，squeeze会导致pos_win变成向量，这里强制转为矩阵
        if isvector(pos_win)
            pos_win = reshape(pos_win, 1, 2);  % 确保是1×2的矩阵
        end
        
        % 确保时间戳是列向量，与位置矩阵的行数匹配
        time_win = time_steps * dt;
        time_win_col = time_win(:);  % 强制转为列向量（n×1）
        
        % 确保衰减数据是列向量
        att_win_col = att_win(:);  % 强制转为列向量（n×1）
        
        % 现在所有组件都是n×1或n×2，可以安全拼接
        uav_data = [time_win_col, pos_win, att_win_col];
        
        % 窗口质量评分
        sum_att = sum(uav_data(:,4));
        std_att = std(uav_data(:,4));
        
        if sum_att > 0
            stability_score = max(1 - std_att / sum_att, 0.3);
            window_score = 0.7 * sum_att + 0.3 * stability_score;
        else
            window_score = 0;
        end
        
        if window_score < 0.5 * mean(att_win) * window_size
            start_time = end_time - 1;
            continue;
        end
        
        % 选择中间值点
        [sorted_att, sort_pos] = sort(att_win);
        mid_pos = ceil(length(att_win) / 2);
        selected_pos = sort_pos(mid_pos);
        
        win_point = pos_win(selected_pos, :);
        win_att = sorted_att(mid_pos);
        
        % 计算置信度
        win_confidence = calculateWindowConfidence(att_win, pos_win, dt, MIN_ATTENUATION);
        
        % 保存结果
        window_data.points = [window_data.points; win_point];
        window_data.attenuations = [window_data.attenuations; win_att];
        confidences = [confidences; win_confidence];
        
        start_time = end_time - 1;
        
        if current_time - start_time > 50 * window_size
            break;
        end
    end
end

function [window_data, confidences] = processUAVSlidingWindow(...
    positions, attenuation, uav_idx, current_time, window_size, dt, MIN_ATTENUATION)
    
    window_data.points = [];
    window_data.attenuations = [];
    confidences = [];
    
    % 从当前时刻开始，倒着处理滑动窗口
    start_time = current_time;
    
    while start_time >= 1
        % 计算窗口结束时间（当前时刻或更早）
        end_time = max(1, start_time - window_size + 1);
        time_steps = end_time:start_time;
        
        % 检查窗口内是否有足够的受扰数据
        att_win = attenuation(time_steps, uav_idx);
        disturbed_mask = att_win > MIN_ATTENUATION;
        
        if sum(disturbed_mask) < max(2, window_size * 0.5)  % 至少50%的数据受扰
            % 数据不足，移动到前一个窗口
            start_time = end_time - 1;
            continue;
        end
        
        % 获取窗口数据
        pos_win = squeeze(positions(time_steps, uav_idx, :));
        
        % 处理当前窗口
        [win_point, win_att, win_confidence] = processSingleWindow(...
            att_win, pos_win, dt, MIN_ATTENUATION);
        
        if ~isempty(win_point)
            window_data.points = [window_data.points; win_point];
            window_data.attenuations = [window_data.attenuations; win_att];
            confidences = [confidences; win_confidence];
        end
        
        % 移动到下一个窗口（向前移动一个时间步）
        start_time = end_time - 1;
        
        % 限制处理的历史长度（最多处理最近50个窗口）
        if current_time - start_time > 50 * window_size
            break;
        end
    end
end

function [win_point, win_att, win_confidence] = processSingleWindow(...
    att_win, pos_win, dt, MIN_ATTENUATION)
    
    n = length(att_win);
    if n < 2  % 降低要求，允许较小的窗口
        win_point = [];
        win_att = [];
        win_confidence = [];
        return;
    end
    
    % 选择窗口内衰减最大的点
    [max_att, max_idx] = max(att_win);
    win_point = pos_win(max_idx, :);
    win_att = max_att;
    
    % 计算窗口置信度（适应不同的窗口大小）
    win_confidence = calculateWindowConfidence(att_win, pos_win, dt, MIN_ATTENUATION);
end

function confidence = calculateWindowConfidence(att_win, pos_win, dt, MIN_ATTENUATION)
    n = length(att_win);
    
    % 特征1: 平均衰减强度
    att_mean = mean(att_win);
    
    % 特征2: 受扰数据比例
    disturbed_ratio = sum(att_win > MIN_ATTENUATION) / n;
    
    % 特征3: 衰减趋势（对于小窗口使用简化计算）
    if n >= 3
        time_vec = (1:n)' * dt;
        X = [ones(n,1), time_vec];
        [~, ~, ~, ~, stats] = regress(att_win, X);
        trend_consistency = stats(1);
    else
        trend_consistency = 0.5;
    end
    
    % 特征4: 运动-衰减相关性（简化版）
    motion_consistency = 0.5;
    if n >= 2
        vel = (pos_win(end, :) - pos_win(1, :)) / ((n-1) * dt);
        att_change = att_win(end) - att_win(1);
        
        if norm(vel) > 1e-5 && abs(att_change) > 1e-4
            % 简单的相关性判断
            motion_consistency = (sign(att_change) == sign(mean(vel))) * 0.8 + 0.2;
        end
    end
    
    % 综合置信度（加权平均）
    confidence = att_mean * 0.2 + ...
                 disturbed_ratio * 0.3 + ...
                 trend_consistency * 0.2 + ...
                 motion_consistency * 0.4;
    
    confidence = min(max(confidence, 0.1), 1.0);
end

function [selected_points, selected_confidences, selected_attenuations] = ...
    selectDiverseHighConfidencePoints(all_points, all_attenuations, all_confidences, all_uav_ids)
    
    % 首先按衰减值排序
    [sorted_attenuations, sort_idx] = sort(all_attenuations, 'descend');
    sorted_points = all_points(sort_idx, :);
    sorted_confidences = all_confidences(sort_idx);
    sorted_uav_ids = all_uav_ids(sort_idx);
    
    selected_indices = [];
    
    % fprintf('总可用数据点: %d个, 来自%d架无人机\n', ...
    %         length(sorted_points), length(unique(sorted_uav_ids)));
    
    % 显示各无人机可用数据点数
    unique_uavs = unique(sorted_uav_ids);
    % for i = 1:length(unique_uavs)
    %     uav_mask = (sorted_uav_ids == unique_uavs(i));
    %     if any(uav_mask)
    %         fprintf('无人机%d: %d个数据点, 衰减范围[%.3f, %.3f]\n', ...
    %                 unique_uavs(i), sum(uav_mask), ...
    %                 min(sorted_attenuations(uav_mask)), max(sorted_attenuations(uav_mask)));
    %     end
    % end
    
    % 策略1: 首先选择每个无人机的最大干扰点
    for i = 1:length(unique_uavs)
        uav_idx = unique_uavs(i);
        uav_mask = (sorted_uav_ids == uav_idx);
        uav_indices = find(uav_mask);
        if ~isempty(uav_indices)
            % 选择该无人机中干扰最大的点
            [~, max_idx] = max(sorted_attenuations(uav_mask));
            selected_indices = [selected_indices; uav_indices(max_idx)];
        end
    end
    
    % 策略2: 按空间距离最大原则选择剩余点
    remaining_indices = setdiff(1:length(sorted_points), selected_indices);
    
    % 选择距离已选点最远的点
    for i = 1:min(10, length(remaining_indices))
        if isempty(selected_indices) || isempty(remaining_indices)
            break;
        end
        
        max_min_dist = -1;
        best_idx = -1;
        
        for j = 1:length(remaining_indices)
            idx = remaining_indices(j);
            current_point = sorted_points(idx, :);
            
            % 计算到所有已选点的最小距离
            dists = sqrt(sum((sorted_points(selected_indices, :) - current_point).^2, 2));
            min_dist = min(dists);
            
            if min_dist > max_min_dist
                max_min_dist = min_dist;
                best_idx = idx;
            end
        end
        
        if best_idx > 0 && max_min_dist > 0.1 % 至少0.1km距离
            selected_indices = [selected_indices; best_idx];
            remaining_indices = setdiff(remaining_indices, best_idx);
        end
    end
    
    % 确保至少选择8个点
    if length(selected_indices) < 8
        needed = 8 - length(selected_indices);
        additional_indices = setdiff(1:min(needed, length(sorted_points)), selected_indices);
        selected_indices = [selected_indices; additional_indices(:)];
    end
    
    % 提取最终数据
    selected_points = sorted_points(selected_indices, :);
    selected_confidences = sorted_confidences(selected_indices);
    selected_attenuations = sorted_attenuations(selected_indices);
    % selected_uav_ids = sorted_uav_ids(selected_indices);
    
    % % 详细调试信息
    % fprintf('最终选点: %d个点, 来自%d架无人机\n', ...
    %         length(selected_indices), length(unique(selected_uav_ids)));
    
    % if length(selected_indices) >= 2
    %     distances = pdist(selected_points);
    %     fprintf('空间分布: 最小距离=%.2fkm, 最大距离=%.2fkm, 平均距离=%.2fkm\n', ...
    %             min(distances), max(distances), mean(distances));
    % 
    %     % 显示各无人机选点数量
    %     for i = 1:length(unique_uavs)
    %         uav_idx = unique_uavs(i);
    %         uav_mask = (selected_uav_ids == uav_idx);
    %         if any(uav_mask)
    %             fprintf('无人机%d: %d个点, 衰减[%.3f, %.3f]\n', ...
    %                     uav_idx, sum(uav_mask), ...
    %                     min(selected_attenuations(uav_mask)), max(selected_attenuations(uav_mask)));
    %         end
    %     end
    % end
    % 
    % fprintf('总体衰减范围: [%.4f, %.4f]\n', ...
    %         min(selected_attenuations), max(selected_attenuations));   
end

function [estimated_pos, fit_error, alpha_est, beta_est] = nonlinearLeastSquaresFit(...
    points, observed_attenuations, confidences, positions, velocities, current_time, disturbedUAVs)

     global high_conf_pos_cache;
    
    if size(points, 1) < 3
        centroid = mean(points, 1);
        estimated_pos = [centroid(1:2), 0];
        fit_error = Inf;
        alpha_est = 1.2;
        beta_est = 2.4;
        return;
    end
    
    % 新增：优先用历史高置信度结果计算初值
    initial_pos = [];
    if ~isempty(high_conf_pos_cache)
        % 提取缓存中的位置和置信度
        cache_x = high_conf_pos_cache(:, 2);
        cache_y = high_conf_pos_cache(:, 3);
        cache_conf = high_conf_pos_cache(:, 4);
        
        % 加权平均：置信度越高、时间越近（t越大），权重越高
        % 时间权重：近期数据权重=1，最早数据权重=0.5（线性衰减）
        time_weights = linspace(0.5, 1, size(high_conf_pos_cache, 1))';
        % 总权重=置信度权重×时间权重
        total_weights = cache_conf .* time_weights;
        total_weights = total_weights / sum(total_weights);  % 归一化
        
        % 计算加权平均初值
        initial_pos(1) = sum(cache_x .* total_weights);
        initial_pos(2) = sum(cache_y .* total_weights);
        % fprintf('使用历史高置信度结果作为初值: (%.2f, %.2f)\n', initial_pos(1), initial_pos(2));
    end
    
    % 若缓存为空，退回到原有逻辑（当前高衰减点均值）
    if isempty(initial_pos)
        [max_att, max_idx] = max(observed_attenuations);
        high_atten_mask = observed_attenuations > 0.3 * max_att;
        if sum(high_atten_mask) >= 2
            initial_pos = mean(points(high_atten_mask, :), 1);
        else
            initial_pos = points(max_idx, :);
        end
    end
    
    % 2. 重新设计参数边界（更合理且确保初始点在范围内）
    % 位置边界：以初始位置为中心，根据数据分布动态调整
    pos_range = 0.5 * range(points); % 数据范围的一半作为搜索半径
    pos_range(pos_range < 3) = 3; % 最小搜索半径保证3个单位
    
    % alpha和beta的边界（基于先验知识，更宽松但合理）
    lb = [initial_pos(1) - pos_range(1), initial_pos(2) - pos_range(2), 0.8, 1.5];
    ub = [initial_pos(1) + pos_range(1), initial_pos(2) + pos_range(2), 2.0, 3.5];
    
    % 3. 确保初始参数在边界内（核心修复）
    initial_params = [
        clamp(initial_pos(1), lb(1), ub(1)), ...  % x坐标
        clamp(initial_pos(2), lb(2), ub(2)), ...  % y坐标
        1.2, ...  % alpha（在[0.8,2.0]范围内）
        2.4       % beta（在[1.5,3.5]范围内）
    ];
    
    % 4. 优化选项（更接近原始设置，保证计算量）
    options = optimoptions('lsqnonlin', ...
                          'Display', 'off', ...
                          'Algorithm', 'levenberg-marquardt', ...
                          'MaxIterations', 50, ...  % 适当增加迭代次数
                          'MaxFunctionEvaluations', 150, ...
                          'FunctionTolerance', 1e-3);  % 恢复原始精度要求
    
    % 5. 执行拟合（保留原始逻辑的稳定性）
    try
        [opt_params, resnorm] = lsqnonlin(...
            @(p) computeResidualsSimplified(p, points, observed_attenuations),...
            initial_params, lb, ub, options);
        
        estimated_pos = [opt_params(1:2), 0];
        alpha_est = opt_params(3);
        beta_est = opt_params(4);
        fit_error = resnorm;

        % fprintf('成功估计位置为：(%.1f,%.1f),误差为：%.3f,观测数据:%d个\n', ...
        %         estimated_pos(1), estimated_pos(2), fit_error, size(points, 1));
        
    catch
        % 失败时使用备用估计（与原始逻辑一致）
        centroid = mean(points, 1);
        estimated_pos = [centroid(1:2), 0];
        alpha_est = 1.2;
        beta_est = 2.4;
        fit_error = Inf;
    end
end

% 辅助函数：确保值在[min_val, max_val]范围内
function val = clamp(val, min_val, max_val)
    val = max(min(val, max_val), min_val);
end

function residuals = computeResidualsSimplified(params, points, observed_attenuations)
    % 简洁的最小二乘残差计算
    % params: [x_j, y_j, alpha, beta]
    x_j = params(1);
    y_j = params(2);
    alpha = params(3);
    beta = params(4);
    
    n_points = size(points, 1);
    residuals = zeros(n_points, 1);
    
    for i = 1:n_points
        dist = norm(points(i, :) - [x_j, y_j]);
        dist = max(dist, 0.1);  % 避免除零
        
        predicted_att = beta / (1 + dist^alpha);
        residuals(i) = observed_attenuations(i) - predicted_att;
    end
end

function [estimated_position, confidence] = fallbackMethod(...
    positions, attenuation, velocities, current_time, disturbedUAVs)
    
    % 备用方法：当高置信度点不足时，使用受扰点的速度方向作为方位估计值
    
    N_int = length(disturbedUAVs);
    
    if N_int == 0
        % estimated_direction = 2*pi*rand(); % 随机方向
        confidence = 0.1;
        estimated_position = [NaN, NaN, 0];
        return;
    end
    
    % 方法: 使用速度方向加权平均
    direction_estimates = zeros(N_int, 1);
    weights = zeros(N_int, 1);
    
    for i = 1:N_int
        uav_idx = disturbedUAVs(i);
        current_vel = squeeze(velocities(current_time, uav_idx, :));
        
        if norm(current_vel) > 1e-5
            vel_dir = current_vel / norm(current_vel);
            direction_estimates(i) = atan2(vel_dir(2), vel_dir(1));
            weights(i) = attenuation(current_time, uav_idx); % 衰减越大权重越高
        else
            direction_estimates(i) = 2*pi*rand();
            weights(i) = 0.1;
        end
    end
    
    % 加权平均方向
    if sum(weights) > 0
        weights = weights / sum(weights);
        sin_sum = sum(weights .* sin(direction_estimates));
        cos_sum = sum(weights .* cos(direction_estimates));
        estimated_direction = atan2(sin_sum, cos_sum);
    else
        estimated_direction = mean(direction_estimates);
    end
    
    % 估计位置（使用衰减最大的无人机位置作为近似）
    [max_att, max_idx] = max(attenuation(current_time, disturbedUAVs));
    % 优化后代码
    pos_temp = squeeze(positions(current_time, disturbedUAVs(max_idx), :));
    % 确保pos_temp为2维或3维向量
    if isscalar(pos_temp)
        estimated_position = [0, 0, 0];  % 极端情况兜底
    else
        estimated_position = pos_temp(:)';  % 转为行向量
    end
    if numel(estimated_position) < 3
        estimated_position = [estimated_position, zeros(1, 3 - numel(estimated_position))];
    end

    intR0 = 2;   % 干扰半径初始估计值为2km
    % 置信度基于无人机数量和衰减强度
    confidence = 0.15 * N_int + 0.2 * min(max_att, 1.0);
    confidence = min(confidence, 0.6); % 备用方法置信度上限
    
    estimated_position(1) = estimated_position(1) + intR0 * cos(estimated_direction(1));
    estimated_position(2) = estimated_position(2) + intR0 * sin(estimated_direction(1));

    % fprintf('使用备用方法: 位置=(%.3f,%.3f)°, 置信度=%.2f\n',...
    %         estimated_position(1), estimated_position(2), confidence);
end

function confidence = calculateFinalConfidence(fit_error, n_points, confidence_scores)
    % 计算最终置信度
    
    if n_points < 3
        confidence = 0.3;
        return;
    end
    
    % 基于拟合误差的置信度计算
    % 参数设置
    e_opt = 0.01;    % 最优误差目标
    lambda = 0.05;     % 灵敏度参数  
    kappa = 1.5;        % 衰减陡度参数
    % 计算RE置信度
    tmp = max(1 + lambda * (fit_error / e_opt - 1), 1e-6);
    RE = min(tmp^(-kappa), 1.0);   % 确保RE在[0,1]

    % 基于点数量的置信度
    % point_confidence = min(n_points / 8, 1.0);
    SI = max(min(1 - exp(-(n_points*1.5-6)/6), 1.0),0);
    
    % 基于点质量的置信度
    DQ = mean(confidence_scores);
    
    % 综合置信度
    confidence = 0.3 * DQ + 0.3 * RE + 0.4 * SI;
    confidence = min(max(confidence, 0.1), 0.95);

    % fprintf('综合置信度 DQ: %.3f,RE: %.3f(err%.4f), SI: %.3f(PN%d)\n', ...
    %       DQ, RE, fit_error, SI,n_points);
end

% 局部函数：计算并封装所有置信度中间参数
function confidenceData = computeConfidenceMetrics(high_confidence_points, observed_attenuations, ...
                                                  fit_error, n_points, confidence_scores, dt)
    % 初始化结构体
    confidenceData = struct('Q1', 0, 'Q2', 0, 'DQ', 0, 'RE', 0, 'SI', 0);
    
    % 计算DQ、RE、SI（复用calculateFinalConfidence的逻辑）
    if n_points < 3
        confidenceData.DQ = 0.3;
        confidenceData.RE = 0.3;
        confidenceData.SI = 0.3;
    else
        e_opt = 0.01;
        lambda = 0.05;
        kappa = 1.5;
        tmp = max(1 + lambda * (fit_error / e_opt - 1), 1e-6);
        confidenceData.RE = min(tmp^(-kappa), 1.0);
        confidenceData.SI = max(min(1 - exp(-(n_points*1.5-6)/6), 1.0), 0);
        confidenceData.DQ = mean(confidence_scores);
    end
    
    % 计算Q1（信息一致性指数）
    n = length(observed_attenuations);
    pos_win = high_confidence_points;
    att_win = observed_attenuations;
    
    % 运动-衰减相关性
    motion_consistency = 0.5;
    if n >= 2
        vel = (pos_win(end, :) - pos_win(1, :)) / ((n-1) * dt);
        att_change = att_win(end) - att_win(1);
        if norm(vel) > 1e-5 && abs(att_change) > 1e-4
            motion_consistency = (sign(att_change) == sign(mean(vel))) * 0.8 + 0.2;
        end
    end
    
    % 衰减趋势稳定性
    trend_consistency = 0.5;
    if n >= 3
        time_vec = (1:n)' * dt;
        X = [ones(n,1), time_vec];
        [~, ~, ~, ~, stats] = regress(att_win, X);
        trend_consistency = stats(1);
    end
    confidenceData.Q1 = motion_consistency * 0.5 + trend_consistency * 0.5;
    
    % 计算Q2（空间构型熵）
    if size(pos_win, 1) >= 3
        P = pos_win(:, 1:2)';
        [~, S] = svd(P' * P);
        lambda_min = min(diag(S));
        lambda_max = max(diag(S));
        confidenceData.Q2 = 1 - lambda_min / lambda_max;
    else
        confidenceData.Q2 = 0.5;
    end
end