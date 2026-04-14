%% 运行集群飞行轨迹绘图（支持多模式切换）
function fig = plotUavTrajectory(fileName, plotMode, varargin)
    % 无人机集群轨迹专业科研绘图 - 支持3种模式：全程/指定时间段/关键时间点（离开干扰区后）
    % 输入参数：
    %   fileName    - 数据文件路径（如'research_plot_data.mat'）
    %   plotMode    - 绘图模式（字符串）：
    %                 'full'       - 全程轨迹
    %                 'custom'     - 指定时间段（需额外传入startTime和endTime）
    %                 'postExit'   - 离开干扰区后（自动识别exit_time）
    %   varargin    - 可选参数：plotMode='custom'时需传入[startTime, endTime]
    
    % 调用配置函数（此处仅用于控制台输出）
    const = Constants();

    if const.bChinese
        fprintf('=== 无人机集群轨迹科研绘图（模式：%s） ===\n', plotMode);
        fprintf('加载科研绘图数据...\n');
    else
        fprintf('=== UAV Swarm Trajectory Research Plotting (Mode: %s) ===\n', plotMode);
        fprintf('Loading research plotting data...\n');
    end
    
    % 1. 数据加载与合法性检查
    if ~exist(fileName, 'file')
        if const.bChinese
            error('未找到 %s 文件，请先运行 Resilience.m', fileName);
        else
            error('File %s not found. Please run Resilience.m first.', fileName);
        end
    end
    data = load(fileName);
    
    % 2. 模式参数初始化与检查
    [timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max] = initPlotParams(data, plotMode, varargin);
    
    % 3. 创建专业图形窗口
    fig = figure('Position', [100, 100, 1600, 1000], 'Color', 'w', 'InvertHardcopy', 'off');
    if const.bChinese
        set(fig, 'Name', sprintf('无人机集群轨迹（模式：%s）', plotMode), 'NumberTitle', 'off');
    else
        set(fig, 'Name', sprintf('UAV Trajectory (Mode: %s)', plotMode), 'NumberTitle', 'off');
    end
    
    % 4. 学术字体设置
    if const.bChinese
        set(groot, 'DefaultAxesFontName', 'SimSun'); % 中文宋体
        set(groot, 'DefaultTextFontName', 'SimSun');  % 中文宋体
    else
        set(groot, 'DefaultAxesFontName', 'Times New Roman');
        set(groot, 'DefaultTextFontName', 'Times New Roman');
    end
    set(groot, 'DefaultAxesFontSize', 12);
    set(groot, 'DefaultTextFontSize', 12);
    
    % 5. 关键时间点与位置计算（进入/离开干扰区）
    [entry_time, entry_idx, exit_time, exit_idx, start_point, end_point, interference_center] = ...
        calcKeyPoints(data, plotMode, timeMask);
    
    % 6. 创建坐标轴（适配当前绘图模式的范围）
    axes('Position', [0.1, 0.1, 0.8, 0.8]);
    hold on;
    grid off;
    box on;
    xlim([focus_x_min, focus_x_max]);
    ylim([focus_y_min, focus_y_max]);
    axis equal; % 保持纵横比，确保干扰源为圆形
    
    % 7. 绘制背景：干扰场强度梯度云图（适配当前绘图范围）
    plotInterferenceField(data, focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    % 1. 获取"干扰区域"的句柄（contourf绘制的云图，通常是坐标轴最后一个子对象）
    h_contour = get(gca, 'Children');
    h_interference = h_contour(end); 


    % 8. 绘制轨迹：根据timeMask筛选对应时间段的轨迹
    plotUavTrajectories(data, timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    
    % 9. 绘制关键时间点编队（正五边形）
    h_formations = plotKeyFormations(data, plotMode, entry_time, exit_time, focus_x_min, focus_x_max, focus_y_min, focus_y_max, varargin);
    % 9.1 只保留需要的4个编队句柄（进入、峰值、离开、到达，对应h_formations的第2-5个）
    % 注：h_formations(1)是"起点"，如需隐藏则不加入
    h_needed = [h_interference, h_formations(2:5)]; 

    % 10. 标注层：干扰源、目标点、比例尺等
    addAnnotations(data, entry_time, exit_time, start_point, end_point, interference_center, ...
                   focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    
    % 11. 图形美化与标题
    % 调用子函数设置标题
    setTitle(plotMode, data, exit_time, varargin);
    
    % 12. 颜色条与图例
    c = colorbar('eastoutside');
    const = Constants(); % 在需要的地方调用
    if const.bChinese
        c.Label.String = '干扰强度';
        legend(h_needed, {'干扰区域', '无人机进入','无人机干扰峰值点', '无人机离开', '无人机到达'}, ...
           'Location', 'northwest', 'FontSize', 11);
    else
        c.Label.String = 'Interference Intensity';
        legend(h_needed, {'Interference Zone', 'Swarm Entry', 'Swarm Peak','Swarm Exit', 'Swarm Target'}, ...
           'Location', 'northwest', 'FontSize', 11);
    end
    c.Label.FontWeight = 'bold';
    c.Label.FontSize = 12;
    
    if const.bChinese
        fprintf('绘图完成！当前模式范围: X[%d,%d], Y[%d,%d]\n', focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    else
        fprintf('Plotting completed! Current range: X[%d,%d], Y[%d,%d]\n', focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    end

    % 13. 单机-集群-干扰源分类图表
    showStatisticTable(data);

    % 14. 总体统计数据图形展示
    plotStatisticGraphs(data);
end

%% 子函数1：初始化绘图参数（根据模式动态调整时间掩码和坐标范围）
function [timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max] = initPlotParams(data, plotMode, varargin)
    % 初始化时间掩码（true表示该时间步需要绘制）和坐标轴范围
    timeArray = data.timeArray;
    numTimeSteps = length(timeArray);
    timeMask = true(1, numTimeSteps); % 默认全程绘制
    
    % 调用配置函数
    const = Constants();
    
    % 处理不同模式的参数
    switch plotMode
        case 'full'
            % 模式1：全程轨迹 - 范围覆盖所有轨迹点
            allX = data.positionsArray(:, :, 1);
            allY = data.positionsArray(:, :, 2);
            focus_x_min = floor(min(allX(:)) - 5);
            focus_x_max = ceil(max(allX(:)) + 5);
            focus_y_min = floor(min(allY(:)) - 5);
            focus_y_max = ceil(max(allY(:)) + 5);
        
        case 'custom'
            % 模式2：指定时间段 - 需传入[startTime, endTime]
            if length(varargin) ~= 1 || length(varargin{1}{1}) ~= 2
                if const.bChinese
                    error('plotMode=custom时，需传入[startTime, endTime]作为第3个参数');
                else
                    error('For plotMode=custom, pass [startTime, endTime] as the 3rd parameter.');
                end
            end
            [startTime, endTime] = deal(varargin{1}{1}(1), varargin{1}{1}(2));
            if startTime >= endTime || startTime < timeArray(1)
                if const.bChinese
                    error('指定时间段[%g,%g]无效，需满足：%g ≤ startTime < endTime ≤ %g', ...
                          startTime, endTime, timeArray(1), timeArray(end));
                else
                    error('Specified time range [%g,%g] is invalid. Requirement: %g ≤ startTime < endTime ≤ %g', ...
                          startTime, endTime, timeArray(1), timeArray(end));
                end
            end
            % 生成时间掩码（仅保留指定时间段内的时间步）
            endTime = min(endTime,timeArray(end));

            timeMask = (timeArray >= startTime) & (timeArray <= endTime);
            % 范围覆盖指定时间段内的轨迹点
            validX = data.positionsArray(timeMask, :, 1);
            validY = data.positionsArray(timeMask, :, 2);
            focus_x_min = floor(min(validX(:)) - 2);
            focus_x_max = ceil(max(validX(:)) + 2);
            focus_y_min = floor(min(validY(:)) - 2);
            focus_y_max = ceil(max(validY(:)) + 2);
        
        case 'postExit'
            % 模式3：离开干扰区后 - 自动识别exit_time
            if isfield(data, 'events') && isfield(data.events, 'exit_times') && ~isempty(data.events.exit_times)
                exit_time = data.events.exit_times(1);
            else
                % 启发式计算exit_time：超出干扰半径1.05倍的第一个时间点
                distances = sqrt((data.positionsArray(:,1,1) - data.interferenceSource.center(1)).^2 + ...
                                 (data.positionsArray(:,1,2) - data.interferenceSource.center(2)).^2);
                exit_idx = find(distances > data.interferenceSource.radius * 1.05, 1, 'first');
                exit_time = data.timeArray(exit_idx);
            end
            % 生成时间掩码（仅保留离开干扰区后的时间步）
            timeMask = timeArray >= exit_time;
            % 范围覆盖离开干扰区后的轨迹点+干扰源
            validX = data.positionsArray(timeMask, :, 1);
            validY = data.positionsArray(timeMask, :, 2);
            focus_x_min = floor(min([validX(:); data.interferenceSource.center(1)]) - 2);
            focus_x_max = ceil(max(validX(:)) + 2);
            focus_y_min = floor(min([validY(:); data.interferenceSource.center(2)]) - 2);
            focus_y_max = ceil(max(validY(:)) + 2);
        
        otherwise
            if const.bChinese
                error('无效plotMode：%s，可选模式为full/custom/postExit', plotMode);
            else
                error('Invalid plotMode: %s. Valid modes are ''full'', ''custom'', ''postExit''.', plotMode);
            end
    end
end

%% 子函数2：计算关键时间点（进入/离开干扰区、起点/终点）
function [entry_time, entry_idx, exit_time, exit_idx, start_point, end_point, interference_center] = ...
    calcKeyPoints(data, plotMode, timeMask)
    % 此函数无文本输出，无需改动
    timeArray = data.timeArray;
    start_point = data.simParams.startPoint;
    end_point = data.simParams.endPoint;
    interference_center = data.interferenceSource.center(1:2);
    
    % 计算进入干扰区时间（entry_time）
    if isfield(data, 'events') && isfield(data.events, 'entry_times') && ~isempty(data.events.entry_times)
        entry_time = data.events.entry_times(1);
    else
        distances = sqrt((data.positionsArray(:,1,1) - interference_center(1)).^2 + ...
                         (data.positionsArray(:,1,2) - interference_center(2)).^2);
        entry_idx = find(distances < data.interferenceSource.radius * 0.95, 1, 'first');
        entry_time = timeArray(entry_idx);
    end
    [~, entry_idx] = min(abs(timeArray - entry_time));
    
    % 计算离开干扰区时间（exit_time）
    if isfield(data, 'events') && isfield(data.events, 'exit_times') && ~isempty(data.events.exit_times)
        maxExitTimeNo = length(data.events.exit_times);
        exit_time = data.events.exit_times(maxExitTimeNo);
    else
        distances = sqrt((data.positionsArray(:,1,1) - interference_center(1)).^2 + ...
                         (data.positionsArray(:,1,2) - interference_center(2)).^2);
        exit_idx = find(distances > data.interferenceSource.radius * 1.05, 1, 'first');
        exit_time = timeArray(exit_idx);
    end
    [~, exit_idx] = min(abs(timeArray - exit_time));
    
    % 模式3（postExit）时，调整关键时间点为离开后的时间
    if strcmp(plotMode, 'postExit')
        entry_time = exit_time + (timeArray(end) - exit_time)/3; % 离开后1/3处
        exit_time = exit_time + 2*(timeArray(end) - exit_time)/3; % 离开后2/3处
    end
end

%% 子函数3：绘制干扰场强度梯度云图
function plotInterferenceField(data, focus_x_min, focus_x_max, focus_y_min, focus_y_max)
    % 调用配置函数
    const = Constants();
    if const.bChinese
        fprintf('绘制干扰场背景...\n');
    else
        fprintf('Plotting interference field background...\n');
    end

    % 创建适配配当前范围的网格（保持纵横比）
    x_range = focus_x_max - focus_x_min;
    y_range = focus_y_max - focus_y_min;
    aspect_ratio = y_range / x_range;
    grid_points_x = 150;
    grid_points_y = round(grid_points_x * aspect_ratio);
    
    [X, Y] = meshgrid(linspace(focus_x_min, focus_x_max, grid_points_x), ...
                      linspace(focus_y_min, focus_y_max, grid_points_y));
    
    % 计算干扰场强度
    interferenceField = zeros(size(X));
    for i = 1:numel(X)
        dx = X(i) - data.interferenceSource.center(1);
        dy = Y(i) - data.interferenceSource.center(2);
        distance = sqrt(dx^2 + dy^2);
        if distance <= data.interferenceSource.radius
            interferenceField(i) = data.interferenceSource.beta / ...
                (1 + (distance / data.interferenceSource.d0) ^ data.interferenceSource.alpha);
        end
    end
    
    % 绘制半透明云图+干扰区边界
    contour_levels = linspace(0, max(interferenceField(:)), 20);
    contourf(X, Y, interferenceField, contour_levels, 'LineStyle', 'none');
    colormap(flipud(hot));
    alpha(0.3);
    caxis([0, 1]);
    
    % 绘制干扰区红色虚线边界
    theta = linspace(0, 2*pi, 100);
    x_circle = data.interferenceSource.center(1) + data.interferenceSource.radius * cos(theta);
    y_circle = data.interferenceSource.center(2) + data.interferenceSource.radius * sin(theta);
    plot(x_circle, y_circle, 'r--', 'LineWidth', 3, 'Color', [0.8, 0.1, 0.1]);
    
    % 绘制等高线标签
    [C, h] = contour(X, Y, interferenceField, 8, 'LineColor', [0.5, 0.5, 0.5], 'LineWidth', 1);
    clabel(C, h, 'FontSize', 9, 'Color', [0.4, 0.4, 0.4]);
end

%% 子函数4：绘制无人机轨迹（根据timeMask筛选时间段）
function plotUavTrajectories(data, timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max)
    % 调用配置函数
    const = Constants();
    if const.bChinese
        fprintf('绘制无人机轨迹...\n');
    else
        fprintf('Plotting UAV trajectories...\n');
    end
    
    colors = lines(data.simParams.numUAVs);
    
    for uav_id = 1:data.simParams.numUAVs
        % 提取当前无人机的全轨迹
        x_traj = data.positionsArray(:, uav_id, 1);
        y_traj = data.positionsArray(:, uav_id, 2);
        
        % 1. 按时间掩码筛选当前模式的轨迹
        x_traj_mode = x_traj(timeMask);
        y_traj_mode = y_traj(timeMask);
        
        % 2. 按坐标范围筛选（避免超出当前绘图范围的零散点）
        in_range = (x_traj_mode >= focus_x_min) & (x_traj_mode <= focus_x_max) & ...
                   (y_traj_mode >= focus_y_min) & (y_traj_mode <= focus_y_max);
        
        % 3. 绘制有效轨迹
        if any(in_range)
            plot(x_traj_mode(in_range), y_traj_mode(in_range), ...
                 '-', 'Color', colors(uav_id, :), 'LineWidth', 2, 'HandleVisibility', 'off');
        end
    end
end

%% 子函数5：绘制关键时间点的正五边形编队
function h_formations = plotKeyFormations(data, plotMode, entry_time, exit_time, focus_x_min, focus_x_max, focus_y_min, focus_y_max, varargin)
    % 调用配置函数
    const = Constants();
    if const.bChinese
        fprintf('绘制关键时间点编队...\n');
    else
        fprintf('Plotting key time point formations...\n');
    end

    timeArray = data.timeArray;
    
    % 根据模式调整关键时间点（确保在当前绘图范围内）
    switch plotMode
        case 'full'
            key_times = [timeArray(1), entry_time, (entry_time+exit_time)/2, exit_time, timeArray(end)];
            if const.bChinese
                key_labels = {'起点', '进入', '峰值', '离开', '目标'};
            else
                key_labels = {'Start', 'Entry', 'Peak', 'Exit', 'Target'};
            end
        case 'custom'
            % 从varargin获取自定义时间段参数
            if ~isempty(varargin)
                startTime = varargin{1}{1}(1);
                endTime = min(varargin{1}{1}(2),timeArray(end));
            else
                if const.bChinese
                    error('plotMode=custom时需传入时间范围参数');
                else
                    error('Time range parameter is required for plotMode=custom.');
                end
            end
            
            % 筛选在自定义时间范围内的关键时间点
            key_times = [startTime, ...
                         entry_time*(entry_time>=startTime&&entry_time<=endTime), ...
                         (startTime+endTime)/2, ...
                         exit_time*(exit_time>=startTime&&exit_time<=endTime), ...
                         endTime];
            key_times = key_times(~isnan(key_times) & ~isinf(key_times)); % 移除无效值
            if const.bChinese
                key_labels = {'起点', '进入', '中期', '离开', '结束'};
            else
                key_labels = {'Start', 'Entry', 'Mid', 'Exit', 'End'};
            end
            key_labels = key_labels(1:length(key_times)); % 匹配标签长度
        case 'postExit'
            key_times = [exit_time, exit_time+(timeArray(end)-exit_time)/2, timeArray(end)];
            if const.bChinese
                key_labels = {'离开', '离开后', '目标'};
            else
                key_labels = {'Exit', 'Post-Exit', 'Target'};
            end
    end
    
    % 定义关键时间点的颜色plot_pentagon_formation_corrected
    key_colors = {[0.2,0.8,0.2], [0.8,0.2,0.2], [1,0.5,0], [0.2,0.2,0.8], [0,0.5,0]};
    % 确保颜色数组长度与关键时间点匹配
    if length(key_colors) < length(key_times)
        key_colors = repmat(key_colors, 1, ceil(length(key_times)/length(key_colors)));
    end
    
        % 初始化句柄数组
    h_formations = [];
    % 绘制每个关键时间点的正五边形编队
    for i = 1:length(key_times)
        % 找到最接近关键时间点的索引
        [~, time_idx] = min(abs(timeArray - key_times(i)));
        % 计算编队中心
        center_x = mean(data.positionsArray(time_idx, :, 1));
        center_y = mean(data.positionsArray(time_idx, :, 2));
        
        % 仅绘制当前坐标范围内的编队
        if center_x >= focus_x_min && center_x <= focus_x_max && ...
           center_y >= focus_y_min && center_y <= focus_y_max
            % 调用正五边形绘制函数
            hwnd = plot_pentagon_formation_corrected( ...
                    time_idx, ...
                    key_labels{i}, ...
                    key_colors{i}, ...
                    data ...
                );
            % 存入句柄数组
            h_formations = [h_formations, hwnd];
        end
    end
end

%% 子函数6：添加标注（干扰源、目标点、比例尺等）
function addAnnotations(data, entry_time, exit_time, start_point, end_point, interference_center, ...
                       focus_x_min, focus_x_max, focus_y_min, focus_y_max)
    % 调用配置函数
    const = Constants();

    % 干扰源标注
    if interference_center(1) >= focus_x_min && interference_center(1) <= focus_x_max && ...
       interference_center(2) >= focus_y_min && interference_center(2) <= focus_y_max
        plot(interference_center(1), interference_center(2), ...
             'rp', 'MarkerSize', 25, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        if const.bChinese
            text(interference_center(1), interference_center(2) - 1.5, ...
                 '干扰源', 'Color', 'r', 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center', 'FontSize', 12);
        else
            text(interference_center(1), interference_center(2) - 1.5, ...
                 'Interference Source', 'Color', 'r', 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
    end
    
    % 目标点标注
    if end_point(1) >= focus_x_min && end_point(1) <= focus_x_max && ...
       end_point(2) >= focus_y_min && end_point(2) <= focus_y_max
        plot(end_point(1), end_point(2), 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'LineWidth', 2);
        if const.bChinese
            text(end_point(1), end_point(2) + 1.5, sprintf('目标点 (%d,%d)', end_point(1), end_point(2)), ...
                 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
        else
            text(end_point(1), end_point(2) + 1.5, sprintf('Target (%d,%d)', end_point(1), end_point(2)), ...
                 'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
    end
    
    % 起点标注（仅在全程模式显示）
    if start_point(1) >= focus_x_min && start_point(1) <= focus_x_max && ...
       start_point(2) >= focus_y_min && start_point(2) <= focus_y_max
        plot(start_point(1), start_point(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
        if const.bChinese
            text(start_point(1), start_point(2) - 1.5, sprintf('起点 (%d,%d)', start_point(1), start_point(2)), ...
                 'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
        else
            text(start_point(1), start_point(2) - 1.5, sprintf('Start (%d,%d)', start_point(1), start_point(2)), ...
                 'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
    end
    
    % 比例尺
    % scale_x = focus_x_min + 2;
    % scale_y = focus_y_min + 1;
    % plot([scale_x, scale_x + 2], [scale_y, scale_y], 'k-', 'LineWidth', 3);
    % if const.bChinese
    %     text(scale_x + 1, scale_y - 0.5, '2 km', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);
    % else
    %     text(scale_x + 1, scale_y - 0.5, '2 km', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 11);
    % end
    % 
    % % 区域标注
    % if const.bChinese
    %     text(focus_x_min + 2, focus_y_min + 1.5, sprintf('关注区域: (%d,%d) 至 (%d,%d)', ...
    %          focus_x_min, focus_y_min, focus_x_max, focus_y_max), ...
    %          'FontWeight', 'bold', 'BackgroundColor', [0.9,0.9,0.9], 'FontSize', 11);
    % else
    %     text(focus_x_min + 2, focus_y_min + 1.5, sprintf('Focus Region: (%d,%d) to (%d,%d)', ...
    %          focus_x_min, focus_y_min, focus_x_max, focus_y_max), ...
    %          'FontWeight', 'bold', 'BackgroundColor', [0.9,0.9,0.9], 'FontSize', 11);
    % end
end

%% 子函数7：设置图表标题（根据模式动态生成）
function setTitle(plotMode, data, exit_time, varargin)
    % 调用配置函数
    const = Constants();

    switch plotMode
        case 'full'
            if const.bChinese
                title('无人机集群轨迹 - 完整任务', 'FontSize', 16, 'FontWeight', 'bold');
            else
                title('UAV Swarm Trajectory - Full Mission', 'FontSize', 16, 'FontWeight', 'bold');
            end
        case 'custom'
            startTime = varargin{1}{1}(1);
            endTime = min(varargin{1}{1}(2),data.timeArray(end));
            if const.bChinese
                title(sprintf('无人机轨迹 - 自定义时段 (%.1fs 至 %.1fs)', startTime, endTime), ...
                      'FontSize', 16, 'FontWeight', 'bold');
            else
                title(sprintf('UAV Trajectory - Custom Period (%.1fs to %.1fs)', startTime, endTime), ...
                      'FontSize', 16, 'FontWeight', 'bold');
            end
        case 'postExit'
            if const.bChinese
                title(sprintf('无人机轨迹 - 离开干扰区后 (%.1fs 之后)', exit_time), ...
                      'FontSize', 16, 'FontWeight', 'bold');
            else
                title(sprintf('UAV Trajectory - Post Exit (After %.1fs)', exit_time), ...
                      'FontSize', 16, 'FontWeight', 'bold');
            end
    end
end

%% 辅助函数：plot_pentagon_formation_corrected
function h_legend = plot_pentagon_formation_corrected(time_idx, label, color, data)
    x_points = data.positionsArray(time_idx, :, 1);
    y_points = data.positionsArray(time_idx, :, 2);
    center_x = mean(x_points);
    center_y = mean(y_points);
    
    % 1. 计算并绘制无人机关键点（颜色实心，与图例一致）
    pentagon_radius = 0.6;
    for i = 1:data.simParams.numUAVs
        angle = (i-1) * (2*pi/data.simParams.numUAVs);
        correct_x = center_x + pentagon_radius * cos(angle);
        correct_y = center_y + pentagon_radius * sin(angle);
        
        % 无人机关键点：颜色实心（与图例一致）
        plot(correct_x, correct_y, 'o', 'MarkerSize', 10, ...
             'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        x_points(i) = correct_x;
        y_points(i) = correct_y;
    end
    
    % 2. 绘制五边形连线
    pentagon_x = [x_points, x_points(1)];
    pentagon_y = [y_points, y_points(1)];
    plot(pentagon_x, pentagon_y, '-', 'LineWidth', 2.5, 'Color', color);
    
    % 3. 绘制集群中心点：白色空心（按你的需求保留）
    plot(center_x, center_y, 'o', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'w', 'MarkerEdgeColor', color, 'LineWidth', 1.5);
    
    % 4. 关键：创建一个"隐藏的颜色实心点"作为图例图标（不显示在图上，但用于图例）
    h_legend = plot(NaN, NaN, 'o', 'MarkerSize', 8, ...
                    'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    h_legend.DisplayName = label; % 关联标签，供图例识别
    h_legend.HandleVisibility = 'off'; % 不在图上显示这个点，只用于图例
    
    % 时间标注
    text(center_x, center_y - 1.5, sprintf('%s\n%.1fs', label, data.timeArray(time_idx)), ...
         'HorizontalAlignment', 'center', 'BackgroundColor', 'w', ...
         'FontWeight', 'bold', 'Margin', 2, 'FontSize', 10);
end

function showStatisticTable(data)
    % 调用配置函数
    const = Constants();

    % 生成总体统计数据表格
    fig = figure('Position', [200, 200, 1000, 600], 'Color', 'w');
    if const.bChinese
        title('无人机集群仿真总体统计数据', 'FontSize', 16, 'FontWeight', 'bold');
    else
        title('UAV Swarm Simulation Overall Statistics', 'FontSize', 16, 'FontWeight', 'bold');
    end
    
    % ---------------------- 1. 单机统计数据 ----------------------
    uavStats = [];
    for i = 1:length(data.events.uav)
        % 单机在区时长（计算entry_time与exit_time的差值之和）
        entryTimes = data.events.uav{i}.entry_time;
        exitTimes = data.events.uav{i}.exit_time;
        inZoneDur = 0;
        for j = 1:min(length(entryTimes), length(exitTimes))
            inZoneDur = inZoneDur + exitTimes(j) - entryTimes(j);
        end
        % 单机最大损伤因子
        maxDamage = max(data.damageFactorsArray(:, i));
        % 单机平均有效载荷
        avgPayload = mean(data.effectivePayloadsArray(:, i));
        
        uavStats = [uavStats; 
                    i, ...                  % 无人机ID
                    round(inZoneDur, 1), ... % 在区时长(s)
                    round(maxDamage, 3), ... % 最大损伤因子
                    round(avgPayload, 2)];   % 平均有效载荷
    end
    
    % ---------------------- 2. 集群统计数据 ----------------------
    % 集群首次进入/最后离开时间
    allEntry = []; allExit = [];
    for i = 1:length(data.events.uav)
        allEntry = [allEntry, data.events.uav{i}.entry_time];
        allExit = [allExit, data.events.uav{i}.exit_time];
    end
    firstEntry = min(allEntry);
    lastExit = max(allExit);
    % STCL激活时长
    stclActTimes = data.events.stcl_activation_times;
    stclDur = 0;
    if ~isempty(stclActTimes)
        stclDur = data.timeArray(end) - stclActTimes(1); % 从首次激活到仿真结束
    end
    % 集群总有效载荷积分（时间步长*载荷求和）
    totalPayloadInt = sum(data.totalEffectivePayloadArray) * data.simParams.dt;
    
    clusterStats = [round(firstEntry, 1), ...  % 首次进入时间(s)
                    round(lastExit, 1), ...     % 最后离开时间(s)
                    round(stclDur, 1), ...      % STCL激活时长(s)
                    round(totalPayloadInt, 2)]; % 总有效载荷积分(kg·s)
    
    % ---------------------- 3. 干扰源统计数据 ----------------------
    sourceStats = [];
    if iscell(data.interferenceSource)
        numSources = length(data.interferenceSource);
    else
        numSources = 1;
        data.interferenceSource = {data.interferenceSource};
    end
    
    for k = 1:numSources
        src = data.interferenceSource{k};
        % 覆盖无人机数量（统计所有进入该干扰源的无人机）
        coveredUAV = 0;
        for i = 1:length(data.events.uav)
            entryTimes = data.events.uav{i}.entry_time;
            if ~isempty(entryTimes)
                coveredUAV = coveredUAV + 1;
            end
        end
        % 平均干扰强度（该干扰源覆盖区域内的平均场强）
        avgIntensity = mean(data.attenuationArray(:, k)); % 假设attenuationArray每列对应一个干扰源
        
        sourceStats = [sourceStats; 
                       k, ...                  % 干扰源ID
                       coveredUAV, ...         % 覆盖无人机数量
                       round(avgIntensity, 3), ... % 平均干扰强度
                       src.radius];            % 干扰半径(km)
    end
    
    % ---------------------- 4. 创建表格 ----------------------
    % 单机统计表格
    uavTable = uitable('Parent', fig, 'Position', [50, 350, 900, 200]);
    if const.bChinese
        uavTable.ColumnName = {'无人机ID', '在区时长(s)', '最大损伤因子', '平均有效载荷(kg)'};
    else
        uavTable.ColumnName = {'UAV ID', 'In-Zone Duration(s)', 'Max Damage Factor', 'Avg Payload(kg)'};
    end
    uavTable.Data = uavStats;
    if const.bChinese
        uavTable.FontName = 'SimSun';
    else
        uavTable.FontName = 'Times New Roman';
    end
    uavTable.FontSize = 11;
    uavTable.ColumnWidth = {80, 120, 120, 150};
    
    % 集群统计表格
    clusterTable = uitable('Parent', fig, 'Position', [50, 200, 900, 80]);
    if const.bChinese
        clusterTable.ColumnName = {'首次进入时间(s)', '最后离开时间(s)', 'STCL激活时长(s)', '总有效载荷积分(kg·s)'};
    else
        clusterTable.ColumnName = {'First Entry (s)', 'Last Exit (s)', 'STCL Active (s)', 'Total Payload Integral(kg·s)'};
    end
    clusterTable.Data = clusterStats;
    if const.bChinese
        clusterTable.FontName = 'SimSun';
    else
        clusterTable.FontName = 'Times New Roman';
    end
    clusterTable.FontSize = 11;
    clusterTable.ColumnWidth = {150, 150, 150, 200};
    
    % 干扰源统计表格
    sourceTable = uitable('Parent', fig, 'Position', [50, 50, 900, 100]);
    if const.bChinese
        sourceTable.ColumnName = {'干扰源ID', '覆盖无人机数量', '平均干扰强度', '干扰半径(km)'};
    else
        sourceTable.ColumnName = {'Source ID', 'Covered UAVs', 'Avg Intensity', 'Radius(km)'};
    end
    sourceTable.Data = sourceStats;
    if const.bChinese
        sourceTable.FontName = 'SimSun';
    else
        sourceTable.FontName = 'Times New Roman';
    end
    sourceTable.FontSize = 11;
    sourceTable.ColumnWidth = {80, 150, 150, 120};
end

function plotStatisticGraphs(data)
    % 调用配置函数
    const = Constants();

    % 总体统计数据图形展示
    fig = figure('Position', [300, 200, 1200, 850], 'Color', 'w', 'InvertHardcopy', 'off');
    
    % 设置总标题
    if const.bChinese
        mainTitle = sgtitle('无人机集群仿真总体统计数据图形分析');
    else
        mainTitle = sgtitle('UAV Swarm Simulation Statistical Analysis');
    end
    set(mainTitle, 'FontSize', 16, 'FontWeight', 'bold');

    % 全局字体设置
    if const.bChinese
        set(groot, 'DefaultAxesFontName', 'SimSun');
        set(groot, 'DefaultTextFontName', 'SimSun');
    else
        set(groot, 'DefaultAxesFontName', 'Times New Roman');
        set(groot, 'DefaultTextFontName', 'Times New Roman');
    end
    set(groot, 'DefaultAxesFontSize', 11);
    set(groot, 'DefaultTextFontSize', 11);

    % ---------------------- 子图1：单机关键指标对比 ----------------------
    subplot(2, 2, 1);
    set(gca, 'Position', [0.1, 0.55, 0.45, 0.35]); % [左, 下, 宽, 高]
    uavIDs = 1:length(data.events.uav);
    numUAVs = length(uavIDs);

    inZoneDurs = zeros(1, numUAVs);
    maxDamages = zeros(1, numUAVs);
    avgPayloads = zeros(1, numUAVs);
    
    for i = uavIDs
        % 计算在区时长（处理多次进入/离开）
        inZoneDurs(i) = calculateInZoneDuration(data.events.uav{i});
        
        % 最大损伤因子
        maxDamages(i) = max(data.damageFactorsArray(:, i));
        
        % 平均有效载荷
        avgPayloads(i) = mean(data.effectivePayloadsArray(data.timeArray >= 10, i));
    end

    % 双Y轴绘制
    yyaxis left;
    bar(uavIDs, inZoneDurs, 0.6, 'FaceColor', [0.2, 0.7, 0.8], 'EdgeColor', 'k');
    if const.bChinese
        ylabel('在区时长 (s)', 'FontWeight', 'bold', 'FontSize', 12);
    else
        ylabel('In-Zone Duration (s)', 'FontWeight', 'bold', 'FontSize', 12);
    end
    ylim([0, max(inZoneDurs) * 1.1]);
    grid on; box on;

    yyaxis right;
    plot(uavIDs, avgPayloads, 'ro-', 'MarkerSize', 6, 'LineWidth', 2, 'DisplayName', '平均有效载荷');
    bar(uavIDs, maxDamages * 10, 0.4, 'FaceColor', [0.8, 0.3, 0.3], 'EdgeColor', 'k', 'DisplayName', '最大损伤因子×10');
    if const.bChinese
        ylabel('平均有效载荷 (kg) / 最大损伤因子×10', 'FontWeight', 'bold');
        legend('Location', 'best');
    else
        ylabel('Avg Payload (kg) / Max Damage×10', 'FontWeight', 'bold');
        legend({'Avg Payload', 'Max Damage×10'}, 'Location', 'best');
    end
    ylim([0, max([avgPayloads, maxDamages * 10]) * 1.1]);

    if const.bChinese
        xlabel('无人机ID', 'FontWeight', 'bold');
        title('(a) 单机在区时长、损伤与载荷对比', 'FontWeight', 'bold');
    else
        xlabel('UAV ID', 'FontWeight', 'bold');
        title('(a) UAV In-Zone Duration, Damage & Payload', 'FontWeight', 'bold');
    end
    xticks(uavIDs);

    % ---------------------- 子图2：集群时间维度统计 ----------------------
    subplot(2, 2, 2);
    set(gca, 'Position', [0.55, 0.55, 0.4, 0.35]);
    % 提取所有进入/离开时间
    [allEntryTimes, allExitTimes] = extractAllEventTimes(data.events.uav);
    
    % 计算时间指标
    totalSimDur = data.timeArray(end) - data.timeArray(1);
    firstEntry = getFirstValidTime(allEntryTimes, data.timeArray(1));
    lastExit = getLastValidTime(allExitTimes, data.timeArray(end));
    inZoneDur = lastExit - firstEntry;
    
    % STCL激活时间处理
    [stclActTime, stclDur] = getSTCLActivationTime(data.events, data.timeArray);

    % 时间阶段划分
    if const.bChinese
        phases = {
            '干扰前阶段',   firstEntry - data.timeArray(1);
            '干扰区阶段',   inZoneDur;
            '干扰后阶段',   totalSimDur - lastExit;
            'STCL激活阶段', stclDur;
        };
    else
        phases = {
            'Pre-Interference',   firstEntry - data.timeArray(1);
            'In-Interference',   inZoneDur;
            'Post-Interference',   totalSimDur - lastExit;
            'STCL Activation', stclDur;
        };
    end
    phaseNames = {phases{:, 1}};
    phaseValues = [phases{:, 2}];

    % 绘制横向柱状图
    colors = [0.6, 0.6, 0.6; 0.2, 0.7, 0.8; 0.3, 0.9, 0.3; 1, 0.6, 0];
    h = barh(1:4, phaseValues, 0.7, 'FaceColor', 'flat', 'EdgeColor', 'k');
    for i = 1:4
        h.CData(i, :) = colors(i, :);
    end

    % 数值标注
    for i = 1:4
        text(phaseValues(i) + 5, i, sprintf('%.1fs', phaseValues(i)), ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
    end

    yticks(1:4);
    yticklabels(phaseNames);
    if const.bChinese
        xlabel('时间 (s)', 'FontWeight', 'bold');
        title('(b) 集群时间阶段统计（含STCL激活）', 'FontWeight', 'bold');
    else
        xlabel('Time (s)', 'FontWeight', 'bold');
        title('(b) Swarm Time Phases (Incl. STCL)', 'FontWeight', 'bold');
    end
    grid on; box on;
    xlim([0, max(phaseValues) * 1.2]);

    % ---------------------- 子图3：干扰源覆盖与强度统计 ----------------------
    subplot(2, 2, 3);
    set(gca, 'Position', [0.1, 0.1, 0.45, 0.35]);
    % 处理多干扰源
    if iscell(data.interferenceSource)
        numSources = length(data.interferenceSource);
        srcIDs = 1:numSources;
    else
        numSources = 1;
        srcIDs = 1;
        data.interferenceSource = {data.interferenceSource};
    end

    coveredUAVs = zeros(1, numSources);
    avgIntensities = zeros(1, numSources);
    for k = srcIDs
        singleSrc = data.interferenceSource{k};
        % 覆盖无人机数量
        coveredCnt = 0;
        for i = 1:numUAVs
            if ~isempty(data.events.uav{i}.entry_time)
                coveredCnt = coveredCnt + 1;
            end
        end
        coveredUAVs(k) = coveredCnt;

        % 平均干扰强度
        srcCenter = singleSrc.center(1:2);
        dists = sqrt((data.positionsArray(:, :, 1) - srcCenter(1)).^2 + ...
                     (data.positionsArray(:, :, 2) - srcCenter(2)).^2);
        inZoneMask = dists < singleSrc.radius;
        validIntensities = data.attenuationArray(inZoneMask);
        avgIntensities(k) = mean(validIntensities);
    end

    % 左侧饼图
    ax1 = axes('Position', [0.1, 0.1, 0.2, 0.35]);
    if const.bChinese
        pie(coveredUAVs, sprintf('干扰源%d\n(%d架)', srcIDs, coveredUAVs));
        title('(c1) 各干扰源覆盖无人机占比', 'FontWeight', 'bold');
    else
        pie(coveredUAVs, sprintf('Source %d\n(%d UAVs)', srcIDs, coveredUAVs));
        title('(c1) UAV Coverage per Source', 'FontWeight', 'bold');
    end
    colormap(colors(1:numSources, :));

    % 右侧柱状图
    ax2 = axes('Position', [0.35, 0.1, 0.2, 0.35]);
    bar(srcIDs, avgIntensities, 0.6, 'FaceColor', [0.8, 0.6, 0.2], 'EdgeColor', 'k');
    if const.bChinese
        xlabel('干扰源ID', 'FontWeight', 'bold');
        ylabel('平均干扰强度', 'FontWeight', 'bold');
        title('(c2) 各干扰源平均干扰强度', 'FontWeight', 'bold');
    else
        xlabel('Source ID', 'FontWeight', 'bold');
        ylabel('Avg Interference', 'FontWeight', 'bold');
        title('(c2) Avg Intensity per Source', 'FontWeight', 'bold');
    end
    xticks(srcIDs);
    grid on; box on;
    for k = srcIDs
        text(k, avgIntensities(k) + 0.02, sprintf('%.3f', avgIntensities(k)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end

    % ---------------------- 子图4：集群载荷-时间积分与韧性趋势 ----------------------
    subplot(2, 2, 4);
    set(gca, 'Position', [0.55, 0.1, 0.4, 0.35]);
    % 载荷积分
    payloadIntegral = cumtrapz(data.timeArray, data.totalEffectivePayloadArray);
    normPayloadIntegral = payloadIntegral / max(payloadIntegral);

    % 韧性指标
    resilience = data.RArray;
    normResilience = resilience / max(resilience);

    % 绘图
    area(data.timeArray, normPayloadIntegral, 'FaceColor', [0.2, 0.7, 0.8], 'FaceAlpha', 0.3, ...
         'EdgeColor', [0.2, 0.7, 0.8], 'LineWidth', 2);
    hold on;
    plot(data.timeArray, normResilience, 'r-', 'LineWidth', 2.5);

    % 关键时间点标注
    firstEntry = getFirstValidTime(extractAllEventTimes(data.events.uav), data.timeArray(1));
    [stclActTime, ~] = getSTCLActivationTime(data.events, data.timeArray);
    lastExit = getLastValidTime(extractAllEventTimes(data.events.uav), data.timeArray(end));
    
    if const.bChinese
        vline(firstEntry, 'g--', 1.5, '首次进入干扰区');
        vline(stclActTime, 'm--', 1.5, 'STCL激活');
        vline(lastExit, 'c--', 1.5, '最后离开干扰区');
        legend({'归一化载荷积分', '归一化韧性指标'}, 'Location', 'best');
    else
        vline(firstEntry, 'g--', 1.5, 'First Entry');
        vline(stclActTime, 'm--', 1.5, 'STCL On');
        vline(lastExit, 'c--', 1.5, 'Last Exit');
        legend({'Norm. Payload Integral', 'Norm. Resilience'}, 'Location', 'best');
    end

    if const.bChinese
        xlabel('时间 (s)', 'FontWeight', 'bold');
        ylabel('归一化值（0~1）', 'FontWeight', 'bold');
        title('(d) 集群载荷积分与韧性指标趋势', 'FontWeight', 'bold');
    else
        xlabel('Time (s)', 'FontWeight', 'bold');
        ylabel('Normalized Value (0~1)', 'FontWeight', 'bold');
        title('(d) Payload Integral & Resilience Trend', 'FontWeight', 'bold');
    end
    grid on; box on;
    hold off;
end

% 辅助函数1：计算单机在区总时长（处理多次进入/离开）
function dur = calculateInZoneDuration(uavEvents)
    dur = 0;
    if ~isfield(uavEvents, 'entry_time') || ~isfield(uavEvents, 'exit_time')
        return; % 无事件记录时返回0
    end
    entryTimes = uavEvents.entry_time(:); % 转为列向量
    exitTimes = uavEvents.exit_time(:);   % 转为列向量
    validPairs = min(length(entryTimes), length(exitTimes));
    for j = 1:validPairs
        dur = dur + exitTimes(j) - entryTimes(j);
    end
end

% 辅助函数2：提取所有无人机的所有进入/离开时间
function [allEntry, allExit] = extractAllEventTimes(uavArray)
    allEntry = [];
    allExit = [];
    for i = 1:length(uavArray)
        % 提取进入时间
        if isfield(uavArray{i}, 'entry_time') && ~isempty(uavArray{i}.entry_time)
            allEntry = [allEntry; uavArray{i}.entry_time(:)]; % 纵向拼接
        end
        % 提取离开时间
        if isfield(uavArray{i}, 'exit_time') && ~isempty(uavArray{i}.exit_time)
            allExit = [allExit; uavArray{i}.exit_time(:)]; % 纵向拼接
        end
    end
end

% 辅助函数3：获取首个有效时间（无数据时用默认值）
function firstTime = getFirstValidTime(timeArray, defaultValue)
    if isempty(timeArray)
        firstTime = defaultValue;
    else
        firstTime = min(timeArray);
    end
end

% 辅助函数4：获取最后一个有效时间（无数据时用默认值）
function lastTime = getLastValidTime(timeArray, defaultValue)
    if isempty(timeArray)
        lastTime = defaultValue;
    else
        lastTime = max(timeArray);
    end
end

% 辅助函数5：处理STCL激活时间
function [actTime, dur] = getSTCLActivationTime(events, timeArray)
    if isfield(events, 'stcl_activation_times') && ~isempty(events.stcl_activation_times)
        actTime = events.stcl_activation_times(1);
        dur = timeArray(end) - actTime;
    else
        actTime = timeArray(end);
        dur = 0;
    end
end

% 辅助函数6：绘制垂直参考线
function vline(x, style, lineWidth, displayName)
    y = ylim;
    plot([x, x], y, style, 'LineWidth', lineWidth, 'DisplayName', displayName);
    ylim(y);
end