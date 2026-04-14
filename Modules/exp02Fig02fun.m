function exp02Fig02fun(fileName, scenario, psi)
    % 功能：基于主仿真数据绘制轨迹主图，支持动态虚拟窗口和多场景兼容
    % 输入：fileName-仿真数据文件路径；scenario-场景类型（1=T0+10s, 2=T1, 3=离开前10s）
    
    % ====================== 1. 加载配置与数据 ======================
    const = Constants();  % 常量配置（包含语言设置等）

    % 默认参数处理
    if nargin == 0
        fileName = 'ResilenceCtrlSimExp02Fig02.mat';  % 默认数据文件
    end
    if ~exist(fileName, 'file')
        error('数据文件不存在：%s，请检查路径', fileName);
    end
    if nargin < 2
        scenario = 1;  % 默认场景1
    end
    if nargin < 3
        psi = 0;  % 默认不调整航向（预测时使用原航向）
    end

    data = load(fileName);
    plotMode = 'custom'; 
    timeZone = [900, 1278.6];  % 自定义模式时间范围

    % 2. 关键时间点与位置计算（含动态窗口参数）
   [scene_times, window_params, curTimIdx] = calcKeyPoints(data, scenario);

    % 3. 获取统一的Tw参数
    if isfield(data.simParams, 'resillenSlidingWindow') && ~isempty(data.simParams.resillenSlidingWindow)
        Tw = data.simParams.resillenSlidingWindow;  % 统一使用此Tw
    else
        Tw = 50;  % 强制默认Tw=50s（根据需求设定）
        warning('未找到resillenSlidingWindow参数，使用默认Tw=50s');
    end

    % 4. 模式参数初始化与检查（不修改initPlotParams）
    [timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max] = initPlotParams(data, plotMode, timeZone);

    % 5. 调用独立预测模块（无需依赖其他文件）
    % [data, timeMask,Texit] = resiliencePredictionModule(data, curTimIdx, psi, timeZone);
    [data, timeMask, Tw_used] = resiliencePredictionModule(data, curTimIdx, psi, timeZone); % 新代码

    % 6. 创建专业图形窗口
    fig = figure('Position', [100, 100, 1600, 1000], 'Color', [0.98, 0.98, 0.98], ...
                 'InvertHardcopy', 'off', 'Renderer', 'Painters');
    set(fig, 'Name', sprintf('UAV Trajectory (Scenario: %d)', scenario), 'NumberTitle', 'off');
    set(groot, 'DefaultAxesFontName', 'Times New Roman');
    set(groot, 'DefaultTextFontName', 'Times New Roman');
    set(groot, 'DefaultAxesFontSize', 12);
    set(groot, 'DefaultTextFontSize', 12);    
    
    % 7. 创建主坐标轴
    ax_main = axes('Position', [0.15, 0.15, 0.65, 0.75]);
    hold on; grid off; box on; axis equal;
    xlim(ax_main, [focus_x_min, focus_x_max]);
    ylim(ax_main, [focus_y_min, focus_y_max]);
    set(ax_main, 'FontName', 'Times New Roman', 'FontSize', 12);
    
    % 8. 绘制背景：干扰场强度梯度云图
    h_interference = plotInterferenceField(data, focus_x_min, focus_x_max, focus_y_min, focus_y_max);
    
    % 9. 绘制轨迹：区分实线和虚线
    plotUavTrajectoriesEnhanced(data, timeMask, scene_times.curTim);
    
    % 10. 绘制动态虚拟窗口（使用从模块返回的Tw_used）
    drawDynamicWindow(ax_main, data, scene_times, struct('start', window_params.start, 'end', window_params.start+Tw_used, 'duration', Tw_used));
    
    % 11. 绘制指定场景的正五边形编队（简化节点显示）
    plotSceneFormation(ax_main, data, scene_times, scenario);
    
    % 12. 标注层：干扰源、目标点等
    addAnnotationsWithLang(ax_main, data, const);
    
    % 13. 颜色条（移至左侧）
    c = colorbar(ax_main, 'westoutside');
    if const.bChinese
        c.Label.String = '干扰强度';
    else
        c.Label.String = 'Interference Intensity';
    end
    c.Label.FontWeight = 'bold';
    c.Label.FontSize = 12;
    set(c, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    % 调试时观测所有图例元素
    % h_all = findobj(gca, '-property', 'DisplayName');
    % for i = 1:length(h_all)
    %     fprintf('对象%d: DisplayName=%s, 类型=%s\n', i, h_all(i).DisplayName, h_all(i).Type);
    % end

    % 14. 坐标轴标签与图例（专业版）
    % 获取当前场景颜色
    scenario_colors = [1 0 0; 0 1 0; 0 0 1; 1 0.5 0; 0.5 0 1];
    current_color = scenario_colors(scenario, :);
    
    % 创建专门的图例对象（不显示在实际图中）
    h_history = plot(NaN, NaN, '-', 'Color', current_color, 'LineWidth', 2, 'DisplayName', 'History Traj.');
    h_predict = plot(NaN, NaN, '--', 'Color', current_color, 'LineWidth', 2, 'DisplayName', 'Predict Traj.');
    
    % % 滑动窗口图例 - 正方形标记+虚线
    % h_window = plot(NaN, NaN, 's--', 'MarkerSize', 8, 'MarkerEdgeColor', 'm', ...
    %                'MarkerFaceColor', 'none', 'LineWidth', 1, 'DisplayName', 'SlideWindow');

    % 滑动窗口图例 - 使用正方形标记
    h_window = plot(NaN, NaN, 's', 'MarkerSize', 8, 'MarkerEdgeColor', 'm', ...
                   'MarkerFaceColor', 'none', 'LineStyle', '--', 'LineWidth', 1, ...
                   'DisplayName', 'SlideWindow');
    
    if const.bChinese
        xlabel(ax_main, 'X坐标 (km)', 'FontWeight', 'bold', 'FontSize', 14);
        ylabel(ax_main, 'Y坐标 (km)', 'FontWeight', 'bold', 'FontSize', 14);
        legend([h_interference,h_history, h_predict, h_window], {'干扰区', '历史轨迹(History Traj.)', '预测轨迹(Predict Traj.)', '滑动窗口(SlideWindow)'}, ...
               'Location', 'northwest', 'FontSize', 11, 'Box', 'on', 'Interpreter', 'latex');
    else
        xlabel(ax_main, 'X Coordinate (km)', 'FontWeight', 'bold', 'FontSize', 14);
        ylabel(ax_main, 'Y Coordinate (km)', 'FontWeight', 'bold', 'FontSize', 14);
        legend([h_interference,h_history, h_predict, h_window], {'Interference Zone', 'History Traj.', 'Predict Traj.', 'SlideWindow'}, ...
               'Location', 'northwest', 'FontSize', 11, 'Box', 'on', 'Interpreter', 'latex');
    end    
    % 15. 设置标题
    setSceneTitle(ax_main, scene_times, scenario, const);
    
    % 16.
    % 获取所有的韧性性能相关指标，包括：性能因子sigma、吸收因子delta、恢复因子rho、恢复时间因子tau、波动因子zeta、综合韧性R
    metrics = calculateSwarmMetrics(data, scene_times.curTim);
    
    % 17. 参数文本框（希腊字母+紧凑布局）
    addParameterTextBox(ax_main, data, metrics, scene_times.curTim);
    
    % 18. 调用动态韧性过程绘图函数（如果需要，也传递Tw_used）
    % 注意：如果dynamicResilienceProcess内部需要Texit，你可能还需要从resiliencePredictionModule额外返回它，
    % 或者在该函数内部重新计算。但根据你的描述，它主要用Tw，所以传递Tw_used。
    dynamicResilienceProcess(fig, data, scene_times, Tw_used, Tw_used); 

    figsDir = fullfile(pwd,'Figs');
    fileNames = {'exp02Fig02A.fig', 'exp02Fig02B.fig', 'exp02Fig02C.fig'};
    fileName = fileNames{scenario}; % 直接索引
    savefig(fig, fullfile(figsDir,fileName));
    % 打印信息也使用Tw_used
    fprintf('绘图完成！虚拟窗口范围: %.1fs - %.1fs (时长: %.1fs)\n', ...
            window_params.start, window_params.start+Tw_used, Tw_used);
end

%% 修正的子函数：绘制动态虚拟窗口（基于集群运动方向）- 修复图例小图标
function drawDynamicWindow(ax, data, scene_times, window_params)
    % 获取集群中心在curTim+10s时刻的坐标
    [~, time_idx] = min(abs(data.timeArray - scene_times.curTim));
    center_x = mean(data.positionsArray(time_idx, :, 1));
    center_y = mean(data.positionsArray(time_idx, :, 2));
    
    % 计算集群运动方向（假设后续轨迹为直线，取T0+10s和T0+20s的位置）
    [~, time_idx_next] = min(abs(data.timeArray - (scene_times.curTim + 10)));
    next_center_x = mean(data.positionsArray(time_idx_next, :, 1));
    next_center_y = mean(data.positionsArray(time_idx_next, :, 2));
    
    % 计算运动方向向量
    dir_x = next_center_x - center_x;
    dir_y = next_center_y - center_y;
    dir_length = sqrt(dir_x^2 + dir_y^2);
    if dir_length > 0
        unit_dir_x = dir_x / dir_length;
        unit_dir_y = dir_y / dir_length;
    else
        % 如果方向向量长度为0，默认向右
        unit_dir_x = 1;
        unit_dir_y = 0;
    end
    
    % 计算垂直于运动方向的单位向量
    perp_dir_x = -unit_dir_y;
    perp_dir_y = unit_dir_x;
    
    % 窗口参数
    width = 1.5;  % 窗口宽度1.5km
    speed = data.simParams.uavSpeed;  % 获取无人机速度
    length = window_params.duration * speed;  % 窗口长度（50s * 速度）
    
    % 计算窗口四个顶点
    p1_x = center_x - perp_dir_x * width/2;
    p1_y = center_y - perp_dir_y * width/2;
    p2_x = center_x + perp_dir_x * width/2;
    p2_y = center_y + perp_dir_y * width/2;
    p3_x = p2_x + unit_dir_x * length;
    p3_y = p2_y + unit_dir_y * length;
    p4_x = p1_x + unit_dir_x * length;
    p4_y = p1_y + unit_dir_y * length;
    
    % 绘制虚拟窗口（多边形），使用虚线矩形样式，保留DisplayName为SlideWindow
    plot(ax, [p1_x, p2_x, p3_x, p4_x, p1_x], [p1_y, p2_y, p3_y, p4_y, p1_y], ...
         'm--', 'LineWidth', 2.5, 'DisplayName', 'SlideWindow');

    % 窗口时间标注
    label_x = center_x + unit_dir_x * length/2;
    label_y = center_y;
    text(ax, label_x, label_y, ...
         sprintf('window: %.1fs - %.1fs (%.1fs)', ...
                 window_params.start, window_params.end, window_params.duration), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', ...
         'Color', 'm', 'BackgroundColor', [1, 1, 1, 0.7]);
end

%% 修正的子函数：绘制场景编队（简化节点显示）
function plotSceneFormation(ax, data, scene_times, scenario)
    timeArray = data.timeArray;
    [~, time_idx] = min(abs(timeArray - scene_times.curTim));
    
    % 编队中心
    center_x = mean(data.positionsArray(time_idx, :, 1));
    center_y = mean(data.positionsArray(time_idx, :, 2));
    
    % 编队颜色: 定义5行颜色，对应5个场景（可根据论文配色需求调整RGB值）
    colors = [1 0 0;     % 场景1：红色
              0 1 0;     % 场景2：绿色
              0 0 1;     % 场景3：蓝色
              1 0.5 0;   % 场景4：橙色
              0.5 0 1];  % 场景5：紫色
    color = colors(scenario, :);
    
    % 绘制正五边形（仅显示五个顶点）
    numUAVs = 5;  % 只显示5个节点
    radius = 0.6;
    angles = linspace(0, 2*pi, numUAVs+1);
    pentagon_x = center_x + radius * cos(angles);
    pentagon_y = center_y + radius * sin(angles);
    
    % 隐藏Current Formation的DisplayName，避免图例显示
    plot(ax, pentagon_x, pentagon_y, '-', 'LineWidth', 2.5, 'Color', color, ...
         'HandleVisibility', 'off');
    
    % 无人机位置标记（仅五个顶点）
    for i = 1:numUAVs
        plot(ax, pentagon_x(i), pentagon_y(i), 'o', 'MarkerSize', 10, ...
             'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
             'HandleVisibility', 'off');   % 关键：隐藏节点
    end
    
    % 中心标记
    plot(ax, center_x, center_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w', ...
         'MarkerEdgeColor', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');   % 关键：隐藏中心点
end

%% 修正的子函数：绘制无人机轨迹 (返回句柄版) - 修复图例小图标
function [h_history, h_prediction] = plotUavTrajectoriesEnhanced(data, timeMask, split_time)
    colors = lines(data.simParams.numUAVs);
    timeArray = data.timeArray;
    
    % 初始化句柄变量
    h_history = [];
    h_prediction = [];
    
    % 获取当前场景对应的颜色（用于历史轨迹和预测轨迹）
    scenario_colors = [1 0 0; 0 1 0; 0 0 1; 1 0.5 0; 0.5 0 1];
    current_color = scenario_colors(1, :); % 默认使用场景1的颜色，可根据需要调整
    
    for uav_id = 1:data.simParams.numUAVs
        x_traj = data.positionsArray(:, uav_id, 1);
        y_traj = data.positionsArray(:, uav_id, 2);
        valid_idx = timeMask;
        x_valid = x_traj(valid_idx);
        y_valid = y_traj(valid_idx);
        t_valid = timeArray(valid_idx);
        
        % 绘制历史轨迹：使用当前场景颜色，实线样式
        before_idx = t_valid <= split_time;
        if any(before_idx)
            plot(x_valid(before_idx), y_valid(before_idx), '-', 'Color', current_color, ...
                'LineWidth', 1.5, 'HandleVisibility', 'off');
        end

        % 绘制预测轨迹：使用当前场景颜色，虚线样式
        after_idx = t_valid > split_time;
        if any(after_idx)
            x_pred = x_valid(after_idx);
            y_pred = y_valid(after_idx);
            plot(x_pred, y_pred, '--', 'Color', current_color, ...
                 'LineWidth', 1.5, 'HandleVisibility', 'off');
            
            % 延长线保持隐藏
            if length(x_pred) >= 2
                [extend_x, extend_y] = generateExtendTraj(x_pred, y_pred);
                plot(extend_x, extend_y, '-.', 'Color', current_color, ...
                     'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end
    end
end

% 子函数：生成轨迹延长段的点（仅用预测轨迹最后2个点的斜率）
function [extend_x, extend_y] = generateExtendTraj(x_pred, y_pred)
    % 1. 计算预测轨迹的总长度（所有点之间的距离和）
    total_len = 0;
    for i = 2:length(x_pred)
        dx = x_pred(i) - x_pred(i-1);
        dy = y_pred(i) - y_pred(i-1);
        total_len = total_len + sqrt(dx^2 + dy^2);
    end
    
    % 2. 延长长度 = 预测轨迹总长度的0.5倍
    extend_len = 0.5 * total_len;
    
    % 3. 用最后一段的方向作为延长方向
    dx_last = x_pred(end) - x_pred(end-1);
    dy_last = y_pred(end) - y_pred(end-1);
    dir_vec = [dx_last, dy_last] / norm([dx_last, dy_last]);  % 归一化
    
    % 4. 计算延长终点（仅一个点，画直线）
    end_pt = [x_pred(end), y_pred(end)] + dir_vec * extend_len;
    
    % 5. 输出两点（最后一点 + 延长终点）
    extend_x = [x_pred(end); end_pt(1)];
    extend_y = [y_pred(end); end_pt(2)];
end

%% 修正的子函数：绘制干扰场 - 确保干扰区图例正确
function hInterfenernce = plotInterferenceField(data, focus_x_min, focus_x_max, focus_y_min, focus_y_max)
    % 创建网格
    x_range = focus_x_max - focus_x_min;
    y_range = focus_y_max - focus_y_min;
    aspect_ratio = y_range / x_range;
    grid_points_x = 150;
    grid_points_y = round(grid_points_x * aspect_ratio);
    
    [X, Y] = meshgrid(linspace(focus_x_min, focus_x_max, grid_points_x), ...
                      linspace(focus_y_min, focus_y_max, grid_points_y));
    
    % 计算干扰强度
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
    
    % 绘制云图和边界
    contour_levels = linspace(0, max(interferenceField(:)), 20);
    [~, hInterfenernce] = contourf(X, Y, interferenceField, contour_levels, 'LineStyle', 'none', 'DisplayName', 'Interference Zone');
    colormap(flipud(hot));
    alpha(0.3);
    caxis([0, 1]);
    
    % 干扰区边界：添加DisplayName，让图例识别 - 使用红色虚线样式
    theta = linspace(0, 2*pi, 100);
    x_circle = data.interferenceSource.center(1) + data.interferenceSource.radius * cos(theta);
    y_circle = data.interferenceSource.center(2) + data.interferenceSource.radius * sin(theta);
    plot(x_circle, y_circle, 'r--', 'LineWidth', 2.5, 'Color', [0.8, 0.1, 0.1], ...
         'HandleVisibility', 'off');  % 关键：添加干扰区图例名称
    
    % 等高线标签
    [C,h] = contour(X, Y, interferenceField, 8, 'LineColor', [0.5, 0.5, 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
    clabel(C, h, 'FontSize', 9, 'Color', [0.4, 0.4, 0.4]);
end

% 以下子函数保持不变（initPlotParams, calcKeyPoints, addAnnotationsWithLang等）
function [timeMask, focus_x_min, focus_x_max, focus_y_min, focus_y_max] = initPlotParams(data, plotMode, timeZone)
    % 保持原有实现...
    timeArray = data.timeArray;
    numTimeSteps = length(timeArray);
    timeMask = true(1, numTimeSteps);  % 默认全程绘制
    
    switch plotMode
        case 'full'
            allX = data.positionsArray(:, :, 1);
            allY = data.positionsArray(:, :, 2);
            focus_x_min = floor(min(allX(:)) - 5);
            focus_x_max = ceil(max(allX(:)) + 5);
            focus_y_min = floor(min(allY(:)) - 5);
            focus_y_max = ceil(max(allY(:)) + 5);
        
        case 'custom'
            [startTime, endTime] = deal(timeZone(1), timeZone(2));
            if startTime >= endTime || startTime < timeArray(1) || endTime > timeArray(end)
                error('无效时间段[%g,%g]', startTime, endTime);
            end
            timeMask = (timeArray >= startTime) & (timeArray <= endTime);
            validX = data.positionsArray(timeMask, :, 1);
            validY = data.positionsArray(timeMask, :, 2);
            focus_x_min = floor(min(validX(:)) - 2);
            focus_x_max = ceil(max(validX(:)) + 2);
            focus_y_min = floor(min(validY(:)) - 2);
            focus_y_max = ceil(max(validY(:)) + 2);
        
        case 'postExit'
            distances = sqrt((data.positionsArray(:,1,1) - data.interferenceSource.center(1)).^2 + ...
                             (data.positionsArray(:,1,2) - data.interferenceSource.center(2)).^2);
            exit_idx = find(distances > data.interferenceSource.radius * 1.05, 1, 'first');
            exit_time = data.timeArray(exit_idx);
            timeMask = timeArray >= exit_time;
            validX = data.positionsArray(timeMask, :, 1);
            validY = data.positionsArray(timeMask, :, 2);
            focus_x_min = floor(min([validX(:); data.interferenceSource.center(1)]) - 2);
            focus_x_max = ceil(max(validX(:)) + 2);
            focus_y_min = floor(min([validY(:); data.interferenceSource.center(2)]) - 2);
            focus_y_max = ceil(max(validY(:)) + 2);
        
        otherwise
            error('无效plotMode：%s', plotMode);
    end
end

function [scene_times, window_params, curTimIdx] = calcKeyPoints(data, scenario)
    % 保持原有实现并修复时间范围错误...
    timeArray = data.timeArray;
    
    % 计算进入/离开干扰区时间
    entry_time = data.events.entry_times;   % T0 进入干扰区时间
    exit_time = data.events.exit_times ;  % Texit 离开干扰区时间
    
    % 场景时间点定义
    scene_times = struct();
    scene_times.T0 = entry_time;
    scene_times.T0_plus_10 = entry_time + 10;
    scene_times.T0_plus_10 = clamp(scene_times.T0_plus_10, timeArray(1), timeArray(end));
    
    % 干扰峰值时间T1
    if isfield(data, 'attenuationArray') && ~isempty(data.attenuationArray)
        [~, linearIdx] = max(data.attenuationArray(:));
        [time_idx, ~] = ind2sub(size(data.attenuationArray), linearIdx);
        scene_times.T1 = timeArray(time_idx);
    else
        scene_times.T1 = (entry_time + exit_time) / 2;
    end
    scene_times.T1 = clamp(scene_times.T1, timeArray(1), timeArray(end));
    % peak_time = scene_times.T1;
    
    % 离开前10s
    scene_times.exit_minus_10 = exit_time - 10;
    scene_times.exit_minus_10 = clamp(scene_times.exit_minus_10, timeArray(1), timeArray(end));
    
    % 当前场景时间点
    switch scenario
        case 1
            scene_times.curTim = scene_times.T0_plus_10;
        case 2
            scene_times.curTim = scene_times.T1;
        case 3
            % scene_times.curTim = 1140;
            scene_times.curTim = exit_time - 30;   % 离开干扰区前30s
        case 4
            scene_times.curTim = scene_times.exit_minus_10;
        otherwise
            scene_times.curTim = scene_times.T0_plus_10;
    end
    
    % 动态虚拟窗口参数（确保时间范围有效）
    window_params = struct();
    window_params.default_duration = data.simParams.resillenSlidingWindow;
    window_params.start = scene_times.curTim;
    
    % 确保结束时间不早于开始时间
    window_params.candidate_end = window_params.start + window_params.default_duration;
    window_params.end = max(window_params.start, min(window_params.candidate_end, exit_time));
    % window_params.duration = window_params.end - window_params.start;

    [~, curTimIdx] = min(abs(data.timeArray - scene_times.curTim));
    curTimIdx = max(curTimIdx, 1);  % 边界保护（避免索引小于1）
end

%% 子函数7：添加标注
function addAnnotationsWithLang(ax, data, const)
    start_point = data.simParams.startPoint;    end_point = data.simParams.endPoint;
    interference_center = data.interferenceSource.center(1:2);
    % entry_time = data.envents.entry_times;   exit_time = data.envents.exit_times;

    % 干扰源标注
    plot(ax, interference_center(1), interference_center(2), 'rp', 'MarkerSize', 20, ...
         'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    
    if const.bChinese
        text(ax, interference_center(1), interference_center(2) - 1.2, '干扰源', ...
             'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
    else
        text(ax, interference_center(1), interference_center(2) - 1.2, 'Interference Source', ...
             'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 12);
    end
    
    % 目标点和起点
    % plot(ax, end_point(1), end_point(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 2);
    plot(ax, start_point(1), start_point(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'HandleVisibility', 'off');
    
    if const.bChinese
        % text(ax, end_point(1), end_point(2) + 1, sprintf('目标点'), ...
        %      'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(ax, start_point(1), start_point(2) - 1, sprintf('起点'), ...
             'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    else
        % text(ax, end_point(1), end_point(2) + 1, sprintf('Target'), ...
        %      'Color', 'b', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(ax, start_point(1), start_point(2) - 1, sprintf('Start'), ...
             'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    
    % 比例尺（增加安全检查）
    scale_len = 2;  % 2km
    
    % 安全获取坐标轴范围
    try
        x_lim = get(ax, 'XLim');
        y_lim = get(ax, 'YLim');
        
        % 验证范围有效性
        if length(x_lim) ~= 2 || length(y_lim) ~= 2 || any(isnan(x_lim)) || any(isnan(y_lim))
            error('无效的坐标轴范围');
        end
        
        % 计算比例尺位置（确保在可见范围内）
        scale_x = x_lim(1) + (x_lim(2) - x_lim(1)) * 0.05;  % 左边界+5%宽度
        scale_y = y_lim(1) + (y_lim(2) - y_lim(1)) * 0.05;  % 下边界+5%高度
        
        % 确保比例尺完全可见
        if scale_x + scale_len > x_lim(2)
            scale_x = x_lim(2) - scale_len - (x_lim(2) - x_lim(1)) * 0.05;
        end
        if scale_y - 0.5 < y_lim(1)
            scale_y = y_lim(1) + (y_lim(2) - y_lim(1)) * 0.1;
        end
        
        % 绘制比例尺
        % plot(ax, [scale_x, scale_x + scale_len], [scale_y, scale_y], 'k-', 'LineWidth', 3);
        % text(ax, scale_x + scale_len/2, scale_y - 0.5, '2 km', ...
        %      'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10);
         
    catch ME
        warning('无法绘制比例尺: %s', ME.message);
    end
end

%% 子函数8：添加参数文本框
function addParameterTextBox(ax, data, metrics, current_time)
    % 获取编队中心位置
    [~, time_idx] = min(abs(data.timeArray - current_time));
    center_x = mean(data.positionsArray(time_idx, :, 1));
    center_y = mean(data.positionsArray(time_idx, :, 2));
    
    % 文本框位置（编队下方）
    text_pos_x = center_x - 6;
    text_pos_y = center_y - 4;
    
    % 文本内容（希腊字母+多列布局）
    text_content = {
        ['T: ', sprintf('%.1fs', metrics.time)], ...
        ['$\hat{s}$: ', sprintf('%.4f', metrics.attenuation), '  C: ', sprintf('%.4f', metrics.confidences), ...
         '  $\eta$: ', sprintf('%.4f', metrics.damageFactors)], ...
        ['L: ', sprintf('%.2f', metrics.totalEffectivePayload), '  $\sigma$: ', sprintf('%.4f', metrics.sigma), ...
         '  $\delta$: ', sprintf('%.4f', metrics.delta)], ...
        ['$\rho$: ', sprintf('%.4f', metrics.rho), '  $\tau$: ', sprintf('%.2f', metrics.tau), ...
         '  $\zeta$: ', sprintf('%.4f', metrics.zeta)], ...
        ['R: ', sprintf('%.4f', metrics.R)]
    };
    
    % 转换坐标为标准化坐标
    ax_pos = get(ax, 'Position');
    x_lim = get(ax, 'XLim');
    y_lim = get(ax, 'YLim');
    
    norm_x = ax_pos(1) + (text_pos_x - x_lim(1)) / (x_lim(2) - x_lim(1)) * ax_pos(3);
    norm_y = ax_pos(2) + (text_pos_y - y_lim(1)) / (y_lim(2) - y_lim(1)) * ax_pos(4);
    
    % 创建文本框
    annotation('textbox', [norm_x, norm_y, 0.22, 0.15], ...
        'String', text_content, ...
        'Interpreter', 'latex', ...
        'BackgroundColor', 'none', ...
        'EdgeColor', 'none', ...
        'FontName', 'Times New Roman', ...
        'FontSize', 10, ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left');
end

%% 子函数9：添加性能子图
function addPerformanceSubplot(fig, data, scene_times, metrics)
    % 创建子图（右侧，增大尺寸）
    ax_perf = axes('Parent', fig, 'Position', [0.82, 0.25, 0.16, 0.6]);
    hold on; grid on; box on;
    set(ax_perf, 'FontName', 'Times New Roman', 'FontSize', 10, 'GridLineStyle', ':');
    
    % 左Y轴：σ曲线
    plot(ax_perf, data.timeArray, data.sigmaArray, 'b-', 'LineWidth', 1.5, ...
         'DisplayName', '$\sigma$ (Performance)');
    ylabel(ax_perf, 'Performance Index', 'FontWeight', 'bold', 'FontSize', 11);
    xlabel(ax_perf, 'Time (s)', 'FontWeight', 'bold', 'FontSize', 11);
    
    % 右Y轴：R曲线
    ax_perf_right = axes('Parent', fig, 'Position', get(ax_perf, 'Position'), ...
        'XAxisLocation', 'bottom', 'YAxisLocation', 'right', ...
        'Color', 'none', 'XColor', 'k', 'YColor', 'r', 'XTickLabel', []);
    hold on; set(ax_perf_right, 'FontName', 'Times New Roman', 'FontSize', 10);
    plot(ax_perf_right, data.timeArray, data.RArray, 'r-', 'LineWidth', 1.5, ...
         'DisplayName', '$R$ (Resilience)');
    ylabel(ax_perf_right, 'Resilience Index', 'FontWeight', 'bold', 'FontSize', 11);
    
    % 当前时间点标记
    plot(ax_perf, scene_times.curTim, metrics.sigma, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(ax_perf_right, scene_times.curTim, metrics.R, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % 滑动窗口范围
    window = 30;
    xlim(ax_perf, [scene_times.curTim - window, scene_times.curTim + window]);
    xlim(ax_perf_right, [scene_times.curTim - window, scene_times.curTim + window]);
    
    % 标题和图例
    title(ax_perf, 'Resilience Metrics', 'FontSize', 12, 'FontWeight', 'bold');
    legend(ax_perf, 'Location', 'best', 'FontSize', 9, 'Interpreter', 'latex', 'Box', 'on');
end

%% 子函数10：设置标题
function setSceneTitle(ax, scene_times, scenario, const)
    if const.bChinese
        scene_names = {'进入干扰区后10秒', '干扰峰值时刻', '指定时间段', '离开干扰区前10秒'};
        main_title = '无人机集群轨迹与动态韧性分析';
    else
        scene_names = {'10s after entering zone', 'Peak interference time', 'Custom time','10s before exiting'};
        main_title = 'UAV Swarm Trajectory & Dynamic Resilience';
    end
    title(ax, sprintf('%s\n%s (%.1fs)', main_title, scene_names{scenario}, scene_times.curTim), ...
          'FontSize', 16, 'FontWeight', 'bold');
end

%% 辅助函数：计算集群性能指标
function [metrics] = calculateSwarmMetrics(data, target_time)
    [~, time_idx] = min(abs(data.timeArray - target_time));
    time_idx = clamp(time_idx, 1, length(data.timeArray));
    
    metrics = struct();
    metrics.time = target_time;
    
    % 提取标量指标
    metrics.sigma = getArrayValue(data, 'sigmaArray', time_idx);
    metrics.delta = getArrayValue(data, 'deltaArray', time_idx);
    metrics.rho = getArrayValue(data, 'rhoArray', time_idx);
    metrics.tau = getArrayValue(data, 'tauArray', time_idx);
    metrics.zeta = getArrayValue(data, 'zetaArray', time_idx);
    metrics.R = getArrayValue(data, 'RArray', time_idx);
    metrics.totalEffectivePayload = getArrayValue(data, 'totalEffectivePayloadArray', time_idx);
    
    % 提取向量指标（平均值）
    metrics.attenuation = mean(getMatrixValue(data, 'attenuationArray', time_idx));
    metrics.damageFactors = mean(getMatrixValue(data, 'damageFactorsArray', time_idx));
    metrics.confidences = mean(getMatrixValue(data, 'confidencesArray', time_idx));  % 假设存在该数组
end

%% 工具函数：安全获取数组值
function val = getArrayValue(data, fieldname, idx)
    if isfield(data, fieldname) && length(data.(fieldname)) >= idx
        val = data.(fieldname)(idx);
    else
        val = NaN;
    end
end

%% 工具函数：安全获取矩阵值
function val = getMatrixValue(data, fieldname, idx)
    if isfield(data, fieldname) && size(data.(fieldname), 1) >= idx
        val = data.(fieldname)(idx, :);
    else
        val = NaN;
    end
end

%% 工具函数：数值范围限制
function out = clamp(in, min_val, max_val)
    out = max(min(in, max_val), min_val);
end

function dynamicResilienceProcess(fig, data, scene_times, Texit, Tw)
    % 动态韧性过程曲线绘制（上下子图：上=L(t)，下=σ(t)）
    % 新增：sigmaHatArray计算与绘制，时间垂线标注
    % 输入：
    %   Tw - 统一的虚拟窗口时长（新增参数）
    
    % ====================== 1. 核心参数计算 ======================
    Tk = scene_times.curTim;  % 预测起始时刻
    max_sim_time = max(data.timeArray);  % 总仿真时间
    % time_offset = -0.2;  % 结束垂线偏移量（2个步长单位，0.2s）
    
    % 1.1 判定Texit有效性（保持原逻辑）
    is_Texit_valid = (Texit > Tk) && (Texit <= max_sim_time);
    if is_Texit_valid
        tw_actual = Texit - Tk;  % 实际窗口时长（可能小于Tw）
        tw_correction_note = ' (Corrected by Texit-Tk)';
    else
        tw_actual = Tw;
        if (Tk + tw_actual) > max_sim_time
            tw_actual = max_sim_time - Tk;
            tw_correction_note = ' (Corrected by max sim time)';
        else
            tw_correction_note = '';
        end
    end
    tw_actual = max(tw_actual, 0);  % 确保非负

    % 1.2 确定绘图时间范围
    % plot_start_time = max(Tk - 10, 0);
    plot_start_time = data.events.entry_times;  % 从无人机集群进入干扰区开始绘制
    plot_end_time = Tk + tw_actual;  % 核心修改：向右偏移0.1s

    % ====================== 2. 数据提取与处理（修正sigmaHatArray索引） ======================
    time_mask = (data.timeArray >= plot_start_time) & (data.timeArray <= plot_end_time);
    time_plot = data.timeArray(time_mask);
    L_full = data.totalEffectivePayloadArray(time_mask);
    
    % 2.1 计算sigmaHatArray（核心修正）
    LD = data.simParams.targetLoad;
    % [~, tk_data_idx] = min(abs(data.timeArray - Tk));  % Tk在原始数据中的索引
    % confidence = data.confidencesArray(tk_data_idx);   % 获取Tk时刻置信度
    
    % 步骤1：提取窗口内的有效载荷数据（长度=length(time_plot)）
    effective_payload_valid = data.totalEffectivePayloadArray(time_mask);
    % 步骤2：初始化sigmaHatArray（与time_plot同长度）
    sigmaHatArray = zeros(size(time_plot));
    % 步骤3：分段计算（维度完全匹配）
    for i = 1:length(time_plot)
        % if time_plot(i) <= Tk
        %     sigmaHatArray(i) = (1 - 0.5*confidence) * effective_payload_valid(i) / LD;
        % else
        %     sigmaHatArray(i) = 0.5 * confidence * effective_payload_valid(i) / LD;
        % end
        sigmaHatArray(i) = effective_payload_valid(i) / LD;
    end

    % 2.2 数据长度对齐（确保所有数组长度一致）
    min_len = min([length(L_full), length(sigmaHatArray), length(time_plot)]);
    time_plot = time_plot(1:min_len);
    L_full = L_full(1:min_len);
    sigmaHatArray = sigmaHatArray(1:min_len);
    % effective_payload_valid = effective_payload_valid(1:min_len);  % 同步截断（若需）

    % 2.3 拆分历史与预测数据
    Tk_idx = find(time_plot >= Tk, 1, 'first');
    Tk_idx = isempty(Tk_idx) * length(time_plot) + ~isempty(Tk_idx) * Tk_idx;
    time_hist = time_plot(1:Tk_idx);
    time_pred = time_plot(Tk_idx:end);
    L_hist = L_full(1:Tk_idx);
    L_pred = L_full(Tk_idx:end);
    sigmaHat_hist = sigmaHatArray(1:Tk_idx);
    sigmaHat_pred = sigmaHatArray(Tk_idx:end);

    % ====================== 3. 上子图：L(t)绘制 ======================
    ax_L = axes('Parent', fig, 'Position', [0.82, 0.55, 0.16, 0.28]);
    hold on; grid on; box on;
    set(ax_L, 'FontName', 'Times New Roman', 'FontSize', 10, 'GridLineStyle', ':');

    plot(ax_L, time_hist, L_hist, 'b-', 'LineWidth', 1.5, 'DisplayName', '$L(t)$ (Historical)');
    if tw_actual > 0 && length(time_pred) > 1
        plot(ax_L, time_pred, L_pred, 'b--', 'LineWidth', 1.5, 'DisplayName', '$L(t)$ (Predicted)');
    end
    L_Tk = L_hist(end);
    plot(ax_L, Tk, L_Tk, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', '$T_k$');

    time_offset = 0.1;  % 偏移量（2个步长单位）
    shifted_end_time = plot_end_time - time_offset;  % 向左移动0.1s

    % 添加偏移后的时间垂线（核心修改）
    plot(ax_L, [Tk, Tk], ylim(ax_L), 'k:', 'LineWidth', 1);  % Tk时刻垂线（不变）
    plot(ax_L, [shifted_end_time, shifted_end_time], ylim(ax_L), 'r:', 'LineWidth', 1);  % 偏移后的结束垂线
    
    % Tw标注位置同步调整
    text(Tk + tw_actual/20 + time_offset/2, max(ylim(ax_L))*0.9, ...
         sprintf('$T_w=%.1fs$', tw_actual), ...
         'Color', 'r', 'FontWeight', 'bold', 'Interpreter', 'latex');

    xlim(ax_L, [plot_start_time, plot_end_time]);
    xlabel(ax_L, 'Time (s)', 'FontWeight', 'bold', 'FontSize', 10);
    ylabel(ax_L, 'Task Load $L(t)$', 'FontWeight', 'bold', 'FontSize', 10, 'Interpreter', 'latex');
    title(ax_L, 'Task Load Dynamics', 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax_L, 'Location', 'best', 'FontSize', 8, 'Interpreter', 'latex', 'Box', 'off');

    % ====================== 4. 下子图：sigmaHat(t)绘制 ======================
    ax_sigma = axes('Parent', fig, 'Position', [0.82, 0.25, 0.16, 0.28]);
    hold on; grid on; box on;
    set(ax_sigma, 'FontName', 'Times New Roman', 'FontSize', 10, 'GridLineStyle', ':');

    plot(ax_sigma, time_hist, sigmaHat_hist, 'k-', 'LineWidth', 1.5, 'DisplayName', '$\hat{\sigma}(t)$ (Historical)');
    if tw_actual > 0 && length(time_pred) > 1
        plot(ax_sigma, time_pred, sigmaHat_pred, 'k--', 'LineWidth', 1.5, 'DisplayName', '$\hat{\sigma}(t)$ (Predicted)');
    end
    sigmaHat_Tk = sigmaHat_hist(end);
    plot(ax_sigma, Tk, sigmaHat_Tk, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', '$T_k$');

    % 添加偏移后的时间垂线（核心修改）
    plot(ax_sigma, [Tk, Tk], ylim(ax_sigma), 'k:', 'LineWidth', 1);  % Tk时刻垂线（不变）
    plot(ax_sigma, [shifted_end_time, shifted_end_time], ylim(ax_sigma), 'r:', 'LineWidth', 1);  % 偏移后的结束垂线

    xlim(ax_sigma, [plot_start_time, plot_end_time]);
    xlabel(ax_sigma, 'Time (s)', 'FontWeight', 'bold', 'FontSize', 10);
    ylabel(ax_sigma, 'Performance Index $\hat{\sigma}(t)$', 'FontWeight', 'bold', 'FontSize', 10, 'Interpreter', 'latex');
    title(ax_sigma, 'Performance Dynamics', 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax_sigma, 'Location', 'best', 'FontSize', 8, 'Interpreter', 'latex', 'Box', 'off');

    % ====================== 5. 其他标注与联动 ======================
    tw_text = sprintf('Prediction Window $T_w$ = %.1fs', tw_actual);
    tw_text = [tw_text tw_correction_note];
    if is_Texit_valid
        tw_text = [tw_text sprintf('\n$T_{exit}$ = %.1fs', Texit)];
    end
    text(0.05, 0.92, tw_text, 'Parent', ax_L, 'Units', 'normalized', ...
         'FontSize', 9, 'FontWeight', 'bold', 'Interpreter', 'latex', ...
         'BackgroundColor', [1 1 1 0.7], 'EdgeColor', 'none', 'VerticalAlignment', 'top');

    linkaxes([ax_L, ax_sigma], 'x');
end
   