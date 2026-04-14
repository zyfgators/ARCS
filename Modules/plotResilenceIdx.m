function plotResilenceIdx(fileName)
% PLOTRESILENCEIDX 绘制无人机集群韧性指标综合分析图（模块化版本）
% 调用方式：plotResilenceIdx('ResilenceCtrlSimData_C0.mat')

    % 调用配置函数（用于控制台输出）
    const = Constants();

    % 1. 加载并验证数据
    data = loadAndValidateData(fileName);
    
    % 2. 确定时间范围和索引
    [timeParams, indices] = determineTimeRange(data);
    start_idx = indices.start_idx;
    end_idx = indices.end_idx;
    time_segment = data.timeArray(start_idx:end_idx);
    
    % 3. 数据预处理
    processedData = preprocessData(data, timeParams, indices);
    
    % 4. 绘制图表
    generatePlots(data, processedData, timeParams, time_segment, start_idx, end_idx, fileName);
    
    if const.bChinese
        fprintf('绘图完成！\n');
    else
        fprintf('Plotting completed!\n');
    end
end

%% 模块1：数据加载与验证
function data = loadAndValidateData(fileName)
% 加载数据文件并验证关键字段

    % 调用配置函数
    const = Constants();

    % 输入格式检查
    if ~ischar(fileName) || ~endsWith(fileName, '.mat')
        if const.bChinese
            error('输入错误：fileName必须是后缀为.mat的字符串（如：''ResilenceCtrlSimData_C0.mat''）');
        else
            error('Input Error: fileName must be a string ending with .mat (e.g., ''ResilenceCtrlSimData_C0.mat'')');
        end
    end
    
    % 文件存在性检查
    if ~exist(fileName, 'file')
        if const.bChinese
            error('未找到数据文件：%s，请先运行Resilience仿真程序生成数据', fileName);
        else
            error('Data file not found: %s. Please run the Resilience simulation to generate data first.', fileName);
        end
    end
    
    % 加载数据
    if const.bChinese
        fprintf('正在加载数据文件：%s\n', fileName);
    else
        fprintf('Loading data file: %s\n', fileName);
    end
    data = load(fileName);
    
    % 验证关键字段
    requiredFields = {'timeArray', 'attenuationArray', 'RArray', 'events', 'simParams'};
    missingFields = {};
    for idx = 1:length(requiredFields)
        field = requiredFields{idx};
        if ~isfield(data, field)
            missingFields = [missingFields, field];
        elseif strcmp(field, 'timeArray') && (~isnumeric(data.timeArray) || isempty(data.timeArray))
            if const.bChinese
                error('数据文件中timeArray不是合法数值数组，请检查仿真数据');
            else
                error('timeArray in the data file is not a valid numeric array. Please check the simulation data.');
            end
        end
    end
    
    if ~isempty(missingFields)
        if const.bChinese
            error('数据文件缺失关键字段：%s，请检查仿真数据格式', strjoin(missingFields, ', '));
        else
            error('Data file missing key fields: %s. Please check the simulation data format.', strjoin(missingFields, ', '));
        end
    end
    
    if const.bChinese
        fprintf('数据加载完成！时间范围: %.1fs 到 %.1fs\n', ...
            data.timeArray(1), data.timeArray(end));
    else
        fprintf('Data loading completed! Time range: %.1fs to %.1fs\n', ...
            data.timeArray(1), data.timeArray(end));
    end
end

%% 模块2：时间范围与索引计算
function [timeParams, indices] = determineTimeRange(data)
% 确定干扰区时间范围和数据索引

    % 调用配置函数
    const = Constants();

    % 计算进入和离开时间
    first_entry_time = calculateFirstEntryTime(data);
    last_exit_time = calculateLastExitTime(data);
    
    % 计算显示时间范围
    tx = -20; % 时间偏移量
    start_time = first_entry_time + tx;
    end_time = last_exit_time;
    
    % 边界调整
    start_time = max(start_time, data.timeArray(1));
    end_time = min(end_time, data.timeArray(end));
    
    % 计算索引
    start_idx = find(data.timeArray >= start_time, 1, 'first');
    if isempty(start_idx)
        start_idx = 1;
    end
    
    end_idx = find(data.timeArray <= end_time, 1, 'last');
    if isempty(end_idx)
        end_idx = length(data.timeArray);
    end
    
    % 确保索引有效性
    start_idx = max(1, min(start_idx, length(data.timeArray)));
    end_idx = max(start_idx, min(end_idx, length(data.timeArray)));
    
    % 输出配置信息
    time_segment = data.timeArray(start_idx:end_idx);
    if const.bChinese
        fprintf('显示配置：\n');
        fprintf('  - 首次进入: %.1fs | 最后离开: %.1fs\n', first_entry_time, last_exit_time);
        fprintf('  - 时间偏移: %.1fs | 显示范围: %.1fs 到 %.1fs\n', tx, time_segment(1), time_segment(end));
    else
        fprintf('Display Configuration:\n');
        fprintf('  - First Entry: %.1fs | Last Exit: %.1fs\n', first_entry_time, last_exit_time);
        fprintf('  - Time Offset: %.1fs | Display Range: %.1fs to %.1fs\n', tx, time_segment(1), time_segment(end));
    end
    
    % 打包返回结果
    timeParams = struct( ...
        'first_entry', first_entry_time, ...
        'last_exit', last_exit_time, ...
        'tx', tx, ...
        'start_time', start_time, ...
        'end_time', end_time ...
    );
    
    indices = struct( ...
        'start_idx', start_idx, ...
        'end_idx', end_idx ...
    );
end

%% 模块3：数据预处理
function processedData = preprocessData(data, timeParams, indices)
% 对原始数据进行预处理（平滑、补全等）

    % 调用配置函数
    const = Constants();

    if const.bChinese
        fprintf('数据预处理中...\n');
    else
        fprintf('Preprocessing data...\n');
    end
    start_idx = indices.start_idx;
    end_idx = indices.end_idx;
    
    % 1. 平滑干扰强度（当前保留原始数据，可按需启用平滑）
    smoothed_attenuation = data.attenuationArray;

    % 2. 确定指标补全基准点
    T0 = 5;
    entry_idx = find(data.timeArray >= timeParams.first_entry + T0, 1, 'first');
    if isempty(entry_idx)
        entry_idx = ceil(length(data.timeArray)/2);
    end
    
    % 3. 提取基准值
    sigma_entry_value = getSafeValue(data, 'sigmaArray', entry_idx);
    delta_entry_value = getSafeValue(data, 'deltaArray', entry_idx);
    rho_entry_value = getSafeValue(data, 'rhoArray', entry_idx);
    tau_entry_value = getSafeValue(data, 'tauArray', entry_idx);
    zeta_entry_value = getSafeValue(data, 'zetaArray', entry_idx);
    R_entry_value = getSafeValue(data, 'RArray', entry_idx);
    
    % 4. 补全指标数据
    preprocessed_sigma = preprocessMetricsWithValue(data.sigmaArray, entry_idx, sigma_entry_value);
    preprocessed_delta = preprocessMetricsWithValue(data.deltaArray, entry_idx, delta_entry_value);
    preprocessed_rho = preprocessMetricsWithValue(data.rhoArray, entry_idx, rho_entry_value);
    preprocessed_tau = preprocessMetricsWithValue(data.tauArray, entry_idx, tau_entry_value);
    preprocessed_zeta = preprocessMetricsWithValue(data.zetaArray, entry_idx, zeta_entry_value);
    preprocessed_R = preprocessMetricsWithValue(data.RArray, entry_idx, R_entry_value);
    
    % 5. 补全总有效载荷和损伤因子
    preprocessed_totalPayload = data.totalEffectivePayloadArray;
    if entry_idx > 1 && length(preprocessed_totalPayload) >= entry_idx
        preprocessed_totalPayload(1:entry_idx-1) = preprocessed_totalPayload(entry_idx);
    end
    
    preprocessed_damage = data.damageFactorsArray;
    if entry_idx > 1 && size(preprocessed_damage, 1) >= entry_idx
        for i = 1:size(preprocessed_damage, 2)
            preprocessed_damage(1:entry_idx-1, i) = preprocessed_damage(entry_idx, i);
        end
    end
    
    % 打包返回预处理结果
    processedData = struct( ...
        'smoothed_attenuation', smoothed_attenuation, ...
        'preprocessed_sigma', preprocessed_sigma, ...
        'preprocessed_delta', preprocessed_delta, ...
        'preprocessed_rho', preprocessed_rho, ...
        'preprocessed_tau', preprocessed_tau, ...
        'preprocessed_zeta', preprocessed_zeta, ...
        'preprocessed_R', preprocessed_R, ...
        'preprocessed_totalPayload', preprocessed_totalPayload, ...
        'preprocessed_damage', preprocessed_damage, ...
        'entry_idx', entry_idx ...
    );
end

%% 模块4：图表生成
function generatePlots(data, processedData, timeParams, time_segment, start_idx, end_idx, fileName)
% 生成所有子图并组合成综合图表

    % 调用配置函数
    const = Constants();

    if const.bChinese
        fprintf('生成韧性指标图表...\n');
    else
        fprintf('Generating resilience index plots...\n');
    end
    
    % 创建主图
    fig = figure('Position', [100, 100, 1400, 900], 'Color', 'w');
    % 设置总标题
    if const.bChinese
        sgtitle(sprintf('无人机集群韧性指标分析\n（数据文件：%s）', fileName), ...
            'FontSize', 16, 'FontWeight', 'bold');
    else
        tmpfileName =  strrep(fileName, '\', '\\');
        sgtitle(sprintf('UAV Swarm Resilience Index Analysis\n(Data File: %s)', tmpfileName), ...
            'FontSize', 16, 'FontWeight', 'bold');
    end
    
    % 字体设置（适配中英文）
    if const.bChinese
        set(groot, 'DefaultAxesFontName', 'SimSun');
        set(groot, 'DefaultTextFontName', 'SimSun');
    else
        set(groot, 'DefaultAxesFontName', 'Times New Roman');
        set(groot, 'DefaultTextFontName', 'Times New Roman');
    end
    set(groot, 'DefaultAxesFontSize', 11);
    set(groot, 'DefaultTextFontSize', 11);
    
    % 颜色方案
    colors = lines(min(5, size(processedData.smoothed_attenuation, 2)));
    
    % 绘制各子图
    plotInterferenceStrength(1, time_segment, processedData.smoothed_attenuation, ...
        start_idx, end_idx, colors, timeParams);
    
    plotDamageFactors(2, time_segment, processedData.preprocessed_damage, ...
        start_idx, end_idx, colors, timeParams);
    
    plotTotalPayload(3, time_segment, processedData.preprocessed_totalPayload, ...
        start_idx, end_idx, data.simParams, timeParams);
    
    plotMetric(4, time_segment, processedData.preprocessed_sigma, ...
        start_idx, end_idx, '\sigma(t)', timeParams);
    
    plotMetric(5, time_segment, processedData.preprocessed_delta, ...
        start_idx, end_idx, '\delta(t)', timeParams);
    
    plotMetric(6, time_segment, processedData.preprocessed_rho, ...
        start_idx, end_idx, '\rho(t)', timeParams);
    
    plotDualMetrics(7, time_segment, processedData.preprocessed_tau, ...
        processedData.preprocessed_zeta, start_idx, end_idx, timeParams);
    
    plotResilienceMetric(8, time_segment, processedData.preprocessed_R, ...
        start_idx, end_idx, data.simParams, timeParams);

    [~, name, ~] = fileparts(fileName);
    num = str2double(name(end)) + 1;
    newFileName = sprintf('exp03Fig%02dB', num);
    % 保存图形
    figsDir = fullfile(pwd,'Figs');
    savefig(fig, fullfile(figsDir, newFileName));
end

%% 子模块：子图绘制函数 - 干扰强度
function plotInterferenceStrength(subplotIdx, time_segment, data, start_idx, end_idx, colors, timeParams)
% 绘制干扰强度子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    hold on; grid on; box on;
    for i = 1:min(5, size(data, 2))
        if const.bChinese
            plot(time_segment, data(start_idx:end_idx, i), ...
                'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', sprintf('无人机 %d', i));
        else
            plot(time_segment, data(start_idx:end_idx, i), ...
                'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', sprintf('UAV %d', i));
        end
    end
    y_lim = ylim;
    % 绘制进入/离开干扰区参考线
    if const.bChinese
        plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5, 'DisplayName', '进入干扰区');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5, 'DisplayName', '离开干扰区');
    else
        plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Enter Interference Zone');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Exit Interference Zone');
    end
    hold off;
    % 设置轴标签和标题
    if const.bChinese
        xlabel('时间 (s)'); 
        ylabel('干扰强度');
        title('(a) 各无人机干扰强度');
    else
        xlabel('Time (s)'); 
        ylabel('Interference Strength');
        title('(a) Interference Strength of Each UAV');
    end
    legend('Location', 'bestoutside');
    xlim([time_segment(1), time_segment(end)]);
end

%% 子模块：子图绘制函数 - 损伤因子
function plotDamageFactors(subplotIdx, time_segment, data, start_idx, end_idx, colors, timeParams)
% 绘制损伤因子子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    hold on; grid on; box on;
    for i = 1:min(5, size(data, 2))
        if const.bChinese
            plot(time_segment, data(start_idx:end_idx, i), ...
                'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', sprintf('无人机 %d', i));
        else
            plot(time_segment, data(start_idx:end_idx, i), ...
                'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', sprintf('UAV %d', i));
        end
    end
    y_lim = ylim;
    % 绘制进入/离开干扰区参考线（无图例，避免重复）
    plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5);
    plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5);
    hold off;
    % 设置轴标签和标题
    if const.bChinese
        xlabel('时间 (s)'); 
        ylabel('损伤因子');
        title('(b) 各无人机损伤因子');
    else
        xlabel('Time (s)'); 
        ylabel('Damage Factor');
        title('(b) Damage Factor of Each UAV');
    end
    legend('Location', 'bestoutside');
    xlim([time_segment(1), time_segment(end)]);
end

%% 子模块：子图绘制函数 - 总有效载荷
function plotTotalPayload(subplotIdx, time_segment, data, start_idx, end_idx, simParams, timeParams)
% 绘制总有效载荷子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    hold on; grid on; box on;
    % 绘制实际总载荷
    if const.bChinese
        plot(time_segment, data(start_idx:end_idx), 'b-', 'LineWidth', 2, 'DisplayName', '实际总载荷');
    else
        plot(time_segment, data(start_idx:end_idx), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Total Payload');
    end
    
    % 绘制目标载荷（如果存在）
    if isfield(simParams, 'targetLoad')
        if const.bChinese
            yline(simParams.targetLoad, 'r--', sprintf('目标载荷 (%.1f)', simParams.targetLoad), ...
                'LineWidth', 2, 'FontSize', 9);
        else
            yline(simParams.targetLoad, 'r--', sprintf('Target Load (%.1f)', simParams.targetLoad), ...
                'LineWidth', 2, 'FontSize', 9);
        end
    end
    
    % 绘制进入/离开干扰区参考线（无图例，避免重复）
    y_lim = ylim;
    plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5);
    plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5);
    hold off;
    
    % 设置轴标签和标题
    if const.bChinese
        xlabel('时间 (s)'); 
        ylabel('总有效载荷');
        title('(c) 集群总有效载荷');
    else
        xlabel('Time (s)'); 
        ylabel('Total Effective Payload');
        title('(c) Total Effective Payload of the Swarm');
    end
    legend('Location', 'best');
    xlim([time_segment(1), time_segment(end)]);
end

%% 子模块：子图绘制函数 - 单一指标
function plotMetric(subplotIdx, time_segment, data, start_idx, end_idx, yLabel, timeParams)
% 绘制单一指标子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    hold on; grid on; box on;
    plot(time_segment, data(start_idx:end_idx), 'LineWidth', 2);
    
    % 绘制进入/离开干扰区参考线（无图例）
    y_lim = ylim;
    plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5);
    plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5);
    hold off;
    
    % 设置轴标签和标题
    if const.bChinese
        xlabel('时间 (s)'); 
        ylabel(yLabel);
        % 根据ylabel自动设置标题
        if strcmp(yLabel, '\sigma(t)')
            title('(d) 动态总性能因子');
        elseif strcmp(yLabel, '\delta(t)')
            title('(e) 动态吸收因子');
        elseif strcmp(yLabel, '\rho(t)')
            title('(f) 动态恢复因子');
        end
    else
        xlabel('Time (s)'); 
        ylabel(yLabel);
        % 根据ylabel自动设置标题
        if strcmp(yLabel, '\sigma(t)')
            title('(d) Dynamic Total Performance Factor');
        elseif strcmp(yLabel, '\delta(t)')
            title('(e) Dynamic Absorption Factor');
        elseif strcmp(yLabel, '\rho(t)')
            title('(f) Dynamic Recovery Factor');
        end
    end
    xlim([time_segment(1), time_segment(end)]);
end

%% 子模块：子图绘制函数 - 双Y轴指标
function plotDualMetrics(subplotIdx, time_segment, data1, data2, start_idx, end_idx, timeParams)
% 绘制双Y轴指标子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    % 左Y轴：恢复时间因子 tau(t)
    yyaxis left;
    hold on; grid on; box on;
    plot(time_segment, data1(start_idx:end_idx), 'LineWidth', 2, 'DisplayName', '\tau(t)');
    
    % 绘制进入/离开干扰区参考线
    y_lim_left = ylim;
    if const.bChinese
        plot([timeParams.first_entry, timeParams.first_entry], y_lim_left, 'r--', 'LineWidth', 1.5, 'DisplayName', '进入干扰区');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim_left, 'g--', 'LineWidth', 1.5, 'DisplayName', '离开干扰区');
    else
        plot([timeParams.first_entry, timeParams.first_entry], y_lim_left, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Enter Zone');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim_left, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Exit Zone');
    end
    ylabel('\tau(t)');

    % 右Y轴：动态波动因子 zeta(t)
    yyaxis right;
    plot(time_segment, data2(start_idx:end_idx), 'LineWidth', 2, 'DisplayName', '\zeta(t)');
    
    % 绘制进入/离开干扰区参考线（隐藏图例，避免重复）
    y_lim_right = ylim;
    plot([timeParams.first_entry, timeParams.first_entry], y_lim_right, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot([timeParams.last_exit, timeParams.last_exit], y_lim_right, 'g--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    ylabel('\zeta(t)');

    % 设置轴标签和标题
    xlabel('时间 (s)');
    if const.bChinese
        title('(g) 恢复时间与动态波动因子');
    else
        title('(g) Recovery Time & Dynamic Fluctuation Factor');
    end
    legend('Location', 'best');
    xlim([time_segment(1), time_segment(end)]);
end

%% 子模块：子图绘制函数 - 综合韧性指标
function plotResilienceMetric(subplotIdx, time_segment, data, start_idx, end_idx, simParams, timeParams)
% 绘制综合韧性指标子图

    % 调用配置函数
    const = Constants();

    subplot(4, 2, subplotIdx);
    hold on; grid on; box on;
    % 绘制综合韧性指标 R(t)
    if const.bChinese
        plot(time_segment, data(start_idx:end_idx), 'LineWidth', 2, 'DisplayName', 'R(t)');
    else
        plot(time_segment, data(start_idx:end_idx), 'LineWidth', 2, 'DisplayName', 'R(t)');
    end
    
    % 绘制韧性阈值（如果存在）
    if isfield(simParams, 'minResilienceThreshold')
        if const.bChinese
            yline(simParams.minResilienceThreshold, 'r--', '韧性阈值', 'LineWidth', 2);
        else
            yline(simParams.minResilienceThreshold, 'r--', 'Resilience Threshold', 'LineWidth', 2);
        end
    end
    
    % 绘制进入/离开干扰区参考线
    y_lim = ylim;
    if const.bChinese
        plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5, 'DisplayName', '进入干扰区');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5, 'DisplayName', '离开干扰区');
    else
        plot([timeParams.first_entry, timeParams.first_entry], y_lim, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Enter Zone');
        plot([timeParams.last_exit, timeParams.last_exit], y_lim, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Exit Zone');
    end
    hold off;
    
    % 设置轴标签和标题
    if const.bChinese
        xlabel('时间 (s)'); 
        ylabel('R(t)');
        title('(h) 综合韧性指标');
    else
        xlabel('Time (s)'); 
        ylabel('R(t)');
        title('(h) Comprehensive Resilience Index');
    end
    legend('Location', 'best');
    xlim([time_segment(1), time_segment(end)]);
end

%% 辅助函数：计算首次进入时间
function first_entry_time = calculateFirstEntryTime(data)
    % 关键修改：读取 data.events.uav{i}.entry_time（与主仿真存储字段匹配）
    if isfield(data.events, 'uav') && ~isempty(data.events.uav)
        allEntryTimes = [];
        % 遍历每架无人机的进入时间
        for i = 1:length(data.events.uav)
            % 检查当前无人机是否有 entry_time 字段且非空
            if isfield(data.events.uav{i}, 'entry_time') && ~isempty(data.events.uav{i}.entry_time)
                allEntryTimes = [allEntryTimes, data.events.uav{i}.entry_time];
            end
        end
        % 若收集到有效进入时间，取最小值（首次进入）
        if ~isempty(allEntryTimes)
            first_entry_time = min(allEntryTimes);
            return;
        end
    end
    
    % 未检测到事件时的默认时间（保留原有逻辑）
    first_entry_time = data.timeArray(ceil(length(data.timeArray)/3));
    
    % 调用配置函数
    const = Constants();
    if const.bChinese
        warning('未检测到进入干扰区事件，使用默认时间：%.1fs', first_entry_time);
    else
        warning('No entry events detected, using default time: %.1fs', first_entry_time);
    end
end

%% 辅助函数：计算最后离开时间
function last_exit_time = calculateLastExitTime(data)
    % 关键修改：读取 data.events.uav{i}.exit_time（与主仿真存储字段匹配）
    if isfield(data.events, 'uav') && ~isempty(data.events.uav)
        allExitTimes = [];
        % 遍历每架无人机的离开时间
        for i = 1:length(data.events.uav)
            % 检查当前无人机是否有 exit_time 字段且非空
            if isfield(data.events.uav{i}, 'exit_time') && ~isempty(data.events.uav{i}.exit_time)
                allExitTimes = [allExitTimes, data.events.uav{i}.exit_time];
            end
        end
        % 若收集到有效离开时间，取最大值（最后离开）
        if ~isempty(allExitTimes)
            last_exit_time = max(allExitTimes);
            return;
        end
    end
    
    % 未检测到事件时的默认时间（保留原有逻辑）
    last_exit_time = data.timeArray(floor(length(data.timeArray)*2/3));
    
    % 调用配置函数
    const = Constants();
    if const.bChinese
        warning('未检测到离开干扰区事件，使用默认时间：%.1fs', last_exit_time);
    else
        warning('No exit events detected, using default time: %.1fs', last_exit_time);
    end
end

%% 辅助函数：安全获取数据字段值
function val = getSafeValue(data, fieldName, idx)
    % 调用配置函数
    const = Constants();

    if ~isfield(data, fieldName)
        val = 0;
        if const.bChinese
            warning('数据缺失字段 %s，使用默认值 0', fieldName);
        else
            warning('Data missing field %s, using default value 0', fieldName);
        end
        return;
    end

    arr = data.(fieldName);
    if idx < 1 || idx > length(arr)
        val = mean(arr); % 索引越界时使用均值
        if const.bChinese
            warning('%s 索引越界，使用均值替代', fieldName);
        else
            warning('%s index out of bounds, using mean value instead', fieldName);
        end
    else
        val = arr(idx);
    end
end

%% 辅助函数：平滑干扰强度
function smoothed_data = smoothInterferenceStrengthWithBounds(attenuationArray, timeArray, events, varargin)
    smoothed_data = attenuationArray;
    if ~isfield(events, 'entry_times_uav') || ~isfield(events, 'exit_times_uav')
        % 调用配置函数
        const = Constants();
        if const.bChinese
            warning('事件数据不完整，返回原始干扰强度');
        else
            warning('Event data is incomplete, returning original interference strength');
        end
        return;
    end

    T = 30;
    for k = 1:2:length(varargin)
        if strcmpi(varargin{k}, 'T')
            T = max(25, varargin{k+1});
        end
    end

    uav_count = min([length(events.entry_times_uav), length(events.exit_times_uav), size(attenuationArray, 2)]);
    if uav_count < 1
        % 调用配置函数
        const = Constants();
        if const.bChinese
            warning('无有效无人机数据，返回原始干扰强度');
        else
            warning('No valid UAV data, returning original interference strength');
        end
        return;
    end

    for i = 1:uav_count
        ti0 = events.entry_times_uav{i};
        ti1 = events.exit_times_uav{i};
        if isempty(ti0) || isempty(ti1)
            continue;
        end
        
        t0_start = ti0 - T;
        t0_end = ti0 + T;
        t1_start = ti1 - T;
        t1_end = ti1 + T;
        
        idx_t0_start = find(timeArray >= t0_start, 1);
        idx_t0_end = find(timeArray >= t0_end, 1);
        idx_t1_start = find(timeArray >= t1_start, 1);
        idx_t1_end = find(timeArray >= t1_end, 1);
        
        if any([isempty(idx_t0_start), isempty(idx_t0_end), isempty(idx_t1_start), isempty(idx_t1_end)])
            continue;
        end
        
        enter_indices = idx_t0_start:idx_t0_end;
        exit_indices = idx_t1_start:idx_t1_end;
        middle_indices = idx_t0_end:idx_t1_start;
        
        if length(enter_indices) < 5 || length(exit_indices) < 5 || length(middle_indices) < 5
            continue;
        end
        
        enter_junction_value = mean(attenuationArray(max(idx_t0_end-4, idx_t0_start):min(idx_t0_end+4, idx_t1_start), i));
        exit_junction_value = mean(attenuationArray(max(idx_t1_start-4, idx_t0_end):min(idx_t1_start+4, idx_t1_end), i));
        
        enter_time = timeArray(enter_indices);
        t_relative_rev = (t0_end - enter_time) / (2*T);
        enter_curve = enter_junction_value * exp(-5*t_relative_rev.^2);
        enter_curve(1) = 0;
        enter_curve(end) = enter_junction_value;
        
        noisy_enter_curve = enter_curve;
        for j = 1:length(enter_indices)
            noise = randn * 0.04 * noisy_enter_curve(j);
            noisy_enter_curve(j) = max(0, noisy_enter_curve(j) + noise);
        end
        
        smoothed_data(enter_indices, i) = noisy_enter_curve;
        smoothed_data(middle_indices, i) = attenuationArray(middle_indices, i);
        
        exit_time = timeArray(exit_indices);
        t_relative = (exit_time - t1_start) / (2*T);
        exit_curve = exit_junction_value * exp(-5*t_relative.^2);
        exit_curve(1) = exit_junction_value;
        exit_curve(end) = 0;
        
        noisy_exit_curve = exit_curve;
        for j = 1:length(exit_indices)
            noise = randn * 0.04 * noisy_exit_curve(j);
            noisy_exit_curve(j) = max(0, noisy_exit_curve(j) + noise);
        end
        
        smoothed_data(exit_indices, i) = noisy_exit_curve;
        
        overlap_enter = max(idx_t0_end - 10, idx_t0_start):idx_t0_end;
        if length(overlap_enter) > 1
            weights = linspace(0, 1, length(overlap_enter))';
            smoothed_data(overlap_enter, i) = ...
                smoothed_data(overlap_enter, i) .* (1 - weights) + ...
                attenuationArray(overlap_enter, i) .* weights;
        end
        
        overlap_exit = idx_t1_start:min(idx_t1_start + 10, idx_t1_end);
        if length(overlap_exit) > 1
            weights = linspace(1, 0, length(overlap_exit))';
            smoothed_data(overlap_exit, i) = ...
                attenuationArray(overlap_exit, i) .* weights + ...
                smoothed_data(overlap_exit, i) .* (1 - weights);
        end
        
        if length(enter_indices) > 10
            smoothed_data(enter_indices, i) = movmean(smoothed_data(enter_indices, i), 3);
        end
        if length(exit_indices) > 10
            smoothed_data(exit_indices, i) = movmean(smoothed_data(exit_indices, i), 3);
        end
    end
end

%% 辅助函数：预处理指标数据
function preprocessed_data = preprocessMetricsWithValue(metricArray, entry_idx, target_value)
preprocessed_data = metricArray;
if entry_idx > 1 && length(preprocessed_data) >= entry_idx
    preprocessed_data(1:entry_idx-1) = target_value;
end
end