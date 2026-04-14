function plotDroneEntryTimeline(events)
% plotDroneEntryTimeline 从传入的events提取并展示无人机进入干扰区的时序
% 输入参数:
%   events - 包含无人机事件的结构，需有uav字段（1×5 cell，每个元素为包含entry_time的struct）

    % 检查events结构完整性
    if ~isfield(events, 'uav') || ~iscell(events.uav) || length(events.uav) ~= 5
        error('events结构中uav字段应为1×5的cell数组');
    end
    
    % 提取进入时间（只保留有有效数据的无人机）
    enterTimes = [];
    droneIDs = [];
    
    for i = 1:5  % 遍历5架无人机
        uavEvent = events.uav{i};
        % 检查是否存在entry_time且不为空
        if isfield(uavEvent, 'entry_time') && ~isempty(uavEvent.entry_time)
            % 取首次进入时间
            firstEntry = min(uavEvent.entry_time);
            enterTimes = [enterTimes, firstEntry];
            droneIDs = [droneIDs, i];
        end
    end
    
    % 检查是否有有效数据
    if isempty(enterTimes)
        error('未找到任何无人机的进入时间数据');
    end
    
    % 按时间排序（获取进入顺序）
    [sortedTimes, sortIndices] = sort(enterTimes);
    sortedIDs = droneIDs(sortIndices);
    entryOrder = 1:length(sortedIDs);  % 进入顺序编号
    
    % 限制时间范围在1020 - 1060
    timeRange = [1020, 1060];
    
    % 创建更大的新图
    figure('Name', '无人机进入干扰区时序', 'Position', [100, 100, 1200, 800]); % 增大图幅尺寸
    ax = axes('Position', [0.1, 0.3, 0.85, 0.5]);
    hold on; grid on;
    box on;  % 增加边框使图表更规整
    
    % 绘制时间轴基线
    plot(ax, timeRange, [0, 0], 'k-', 'LineWidth', 2.5);
    
    % 绘制每个无人机的进入标记
    for i = 1:length(sortedIDs)
        % 时间点标记
        plot(ax, sortedTimes(i), 0, 'o', ...
             'MarkerSize', 14, ...
             'MarkerFaceColor', [0.2, 0.6, 0.9], ...
             'MarkerEdgeColor', 'k', ...
             'LineWidth', 1.5);
        
        % 连接标记到标签的竖线
        plot(ax, [sortedTimes(i), sortedTimes(i)], [0, 1.2], 'k--', 'LineWidth', 1); % 延长竖线
        
        % 无人机编号标签
        text(ax, sortedTimes(i), 1.25, ...
             sprintf('无人机%d', sortedIDs(i)), ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', ...
             'FontSize', 11, ...
             'FontWeight', 'bold');
        
        % 时间标签（显示具体时间，不用科学计数法）
        text(ax, sortedTimes(i), -0.3, ...
             sprintf('%.1f', sortedTimes(i)), ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', ...
             'FontSize', 10, ...
             'Rotation', 0);  % 不旋转文字更易读
        
        % 进入顺序标签
        text(ax, sortedTimes(i), 0.6, ...
             sprintf('第%d个', entryOrder(i)), ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle', ...
             'FontSize', 9, ...
             'Color', [0.6, 0, 0]);
    end
    
    % 设置坐标轴和标题
    xlabel(ax, '时间 (s)', 'FontSize', 12);
    title(ax, '无人机进入干扰区的时序图（1020 - 1060s）', 'FontSize', 14, 'FontWeight', 'bold');
    ylim(ax, [-0.5, 1.5]);  % 进一步增大Y轴范围
    xlim(ax, timeRange);
    set(ax, 'YTick', [], 'FontSize', 10);  % 隐藏Y轴刻度
    % 设置X轴刻度，更清晰展示
    set(ax, 'XTick', 1020:5:1060, 'FontSize', 10);
    
    % 添加时间流向指示
    text(ax, mean(timeRange), -0.4, ...
         '时间流向 →', ...
         'HorizontalAlignment', 'center', ...
         'FontSize', 11, ...
         'Color', [0.4, 0.4, 0.4]);
    
    % 处理表格数据，将时间格式化为字符串（确保非科学计数法）
    tblData = cell(length(sortedIDs), 3);
    for i = 1:length(sortedIDs)
        tblData{i, 1} = sortedIDs(i);
        tblData{i, 2} = sprintf('%.1f', sortedTimes(i));  % 时间格式化为字符串
        tblData{i, 3} = entryOrder(i);
    end
    
    % 添加详细数据表格（底部，进一步增大尺寸）
    rowNames = arrayfun(@(x) sprintf('记录%d', x), 1:length(sortedIDs), 'UniformOutput', false);
    colNames = {'无人机编号', '进入时间 (s)', '进入顺序'};
    
    uitable('Parent', gcf, ...
            'Data', tblData, ...
            'RowName', rowNames, ...
            'ColumnName', colNames, ...
            'Position', [100, 50, 1000, 200], ...  % 大幅增大表格尺寸
            'FontSize', 11, ...
            'ColumnWidth', {120, 150, 100}, ...
            'BackgroundColor', [0.95, 0.95, 0.95]);
end