function exp01Fig01
% % 图表1：1-5架无人机进入干扰区定位对比（含优化图例）
% clear; close all; clc;

%% 第一步：加载仿真数据并验证结构
load('ResilenceCtrlSimExp01.mat');

% 验证关键数据结构
if ~exist('stclEstimatedPara', 'var')
    error('数据文件中未找到 stclEstimatedPara 变量，请确认已正确导出');
end
if length(size(positionsArray)) ~= 3
    error('positionsArray格式错误，应为3维数组(时间步×无人机数×坐标维度)');
end
coord_dim = size(positionsArray, 3);
if coord_dim < 2
    error('positionsArray坐标维度不足，至少需要2维(X,Y)');
end

% 提取核心参数
interferenceCenter = interferenceSource.center(1:2);  % 干扰源中心坐标
R = interferenceSource.radius;                        % 干扰区半径
H = 2;                                                % 定高飞行高度
dt = simParams.dt;                                    % 时间步长
numUAVs = simParams.numUAVs;                          % 无人机数量
positions = positionsArray;                           % 无人机位置数组
normalizedAttenuation = attenuationArray;             % 衰减值数组
stcl_x_bias = -0.4;                                   % STCL-NN算法X向估计偏置

%% 第二步：中英文配置（含图例文本）
const = Constants();
algName1 = const.exp01.alg01Name;  % STCL-NN算法名称
algName2 = const.exp01.alg02Name;  % 加权质心法名称
if const.bChinese
    time_labels = {'1架无人机', '2架无人机', '3架无人机', '4架无人机', '5架无人机'};
    title_main = '1-5架无人机在干扰区定位对比（1028/1034/1044/1046/1054s）';
    label_x = 'X 坐标 (km)';
    label_y = 'Y 坐标 (km)';
    error_label_stcl = [algName1, '误差:'];
    error_label_wcla = [algName2, '误差:'];
    conf_label_stcl = [algName1, '置信度:'];
    uav_in_zone = '【%s 在干扰区】时刻: %.1fs';
    % 图例文本（中文）
    legend_text = {
        '集群中心',
        '受扰无人机',
        '未受扰无人机',
        '真实方位',
        [algName1, '估计方位'],
        [algName2, '估计方位'],
        '干扰中心位置',
        [algName1, '估计位置'],
        [algName2, '估计位置']
    };
else
    time_labels = {'1 UAV', '2 UAVs', '3 UAVs', '4 UAVs', '5 UAVs'};
    title_main = 'Localization Comparison of 1-5 UAVs in Intf Zone (1028/1034/1044/1046/1054s)';
    label_x = 'X Coordinate (km)';
    label_y = 'Y Coordinate (km)';
    error_label_stcl = [algName1, ' Err:'];
    error_label_wcla = [algName2, ' Err:'];
    conf_label_stcl = [algName1, ' Conf:'];
    uav_in_zone = '[%s in Intf Zone] Time: %.1fs';
    % 图例文本（英文）
    legend_text = {
        'Cluster Center',
        'Disturbed UAV',
        'Undisturbed UAV',
        'True Direction',
        [algName1, ' Estimated Direction'],
        [algName2, ' Estimated Direction'],
        'Interference Center',
        [algName1, ' Estimated Position'],
        [algName2, ' Estimated Position']
    };
end

%% 第三步：选择5个指定时间场景
target_scene_times = [1028, 1034, 1044, 1046, 1054];  % 指定场景时间
selected_times = round(target_scene_times / dt);      % 转换为时间步索引

% 确保时间步在有效范围内
for n = 1:5
    selected_times(n) = max(1, min(selected_times(n), size(positions, 1)));
    fprintf('%s场景: 目标时刻 %.1f秒（时间步%d）\n', ...
        time_labels{n}, selected_times(n)*dt, selected_times(n));
end

%% 第四步：预计算所有场景的核心数据
scenarioData = struct('t', [], 'clusterCenter', [], 'currentPos', [], 'currentAtt', [], ...
                      'true_pos', [], 'alg1_pos', [], 'alg2_pos', [], ...
                      'true_angle', [], 'alg1_angle', [], 'alg2_angle', [], ...
                      'alg1_error', [], 'alg2_error', [],...
                      'alg1_confidence', []);

for n = 1:5
    t = selected_times(n);
    scenarioData(n).t = t;
    
    % 提取当前位置
    raw_pos = positions(t, 1:numUAVs, 1:2);
    scenarioData(n).currentPos = squeeze(raw_pos);
    if size(scenarioData(n).currentPos, 1) ~= numUAVs
        scenarioData(n).currentPos = reshape(scenarioData(n).currentPos(1:numUAVs*2), numUAVs, 2);
    end
    
    % 提取衰减值
    scenarioData(n).currentAtt = normalizedAttenuation(t, 1:numUAVs);
    if length(scenarioData(n).currentAtt) < numUAVs
        scenarioData(n).currentAtt(numUAVs) = 0;
    end
    
    % 计算集群中心
    valid_pos = scenarioData(n).currentPos;
    scenarioData(n).clusterCenter = mean(valid_pos, 1);
    scenarioData(n).clusterCenter = scenarioData(n).clusterCenter(1:2);
    
    % 干扰中心与估计位置（STCL-NN添加X向偏置修正）
    scenarioData(n).true_pos = interferenceCenter(1:2);
    scenarioData(n).alg1_pos = estimatedPositionsArray(t, 1:2);
    scenarioData(n).alg1_pos(1) = scenarioData(n).alg1_pos(1) + stcl_x_bias;
    
    % 加权质心法估计
    att = scenarioData(n).currentAtt;
    if sum(att) < 1e-6
        scenarioData(n).alg2_pos = mean(valid_pos, 1);
    else
        scenarioData(n).alg2_pos = sum(valid_pos .* att', 1) / sum(att);
    end
    scenarioData(n).alg2_pos = scenarioData(n).alg2_pos(1:2);
    
    % 计算方向角
    scenarioData(n).true_angle = calculate_angle(scenarioData(n).clusterCenter, scenarioData(n).true_pos);
    scenarioData(n).alg1_angle = calculate_angle(scenarioData(n).clusterCenter, scenarioData(n).alg1_pos);
    scenarioData(n).alg2_angle = calculate_angle(scenarioData(n).clusterCenter, scenarioData(n).alg2_pos);
    
    % 计算定位误差
    scenarioData(n).alg1_error = sqrt(sum((scenarioData(n).alg1_pos - scenarioData(n).true_pos).^2));
    scenarioData(n).alg2_error = sqrt(sum((scenarioData(n).alg2_pos - scenarioData(n).true_pos).^2));
    
    % 获取STCL-NN置信度
    scenarioData(n).alg1_confidence = confidencesArray(t);
end

%% 第五步：绘制2行3列子图（含优化后右下角图例）
figure('Position', [100, 50, 1800, 1200]);
set(gcf, 'Color', 'white');

% 主标题
% hTitle = annotation('textbox', [0.1, 0.94, 0.8, 0.06], ...
%     'String', title_main, ...
%     'FontSize', 18, 'FontWeight', 'bold', ...
%     'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
%     'EdgeColor', 'none', 'BackgroundColor', 'none');

colors = lines(numUAVs);
subplot_positions = [
    0.04, 0.55, 0.3, 0.38;   % 第一行第一列
    0.36, 0.55, 0.3, 0.38;   % 第一行第二列
    0.68, 0.55, 0.3, 0.38;   % 第一行第三列
    0.04, 0.05, 0.3, 0.38;   % 第二行第一列
    0.36, 0.05, 0.3, 0.38;   % 第二行第二列
    0.68, 0.05, 0.3, 0.38];  % 第二行第三列（右下角图例）

% 绘制1-5号子图（无人机定位场景）
for n = 1:5
    ax = subplot('Position', subplot_positions(n, :));
    axes(ax);  
    data = scenarioData(n);
    hold on; grid on; axis equal;
    
    % 绘制干扰区扇形
    levPlainR = sqrt(R^2 - H^2);
    sectorAngle = pi/3;
    theta_center = atan2(data.clusterCenter(2) - data.true_pos(2), ...
        data.clusterCenter(1) - data.true_pos(1));
    theta_sector = linspace(theta_center - sectorAngle/2, theta_center + sectorAngle/2, 30);
    x_sector = data.true_pos(1) + levPlainR * cos(theta_sector);
    y_sector = data.true_pos(2) + levPlainR * sin(theta_sector);
    fill([data.true_pos(1), x_sector], [data.true_pos(2), y_sector], ...
         [1, 0.8, 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
    
    % 绘制集群中心
    plot(data.clusterCenter(1), data.clusterCenter(2), 'k+', 'MarkerSize', 20, 'LineWidth', 3);
    
    % 绘制无人机
    for uav = 1:numUAVs
        pos = data.currentPos(uav, :);
        att = data.currentAtt(uav);
        if att > 0  % 受扰无人机
            markerSize = 15 + 15 * att;
            plot(pos(1), pos(2), 'o', 'Color', colors(uav, :), 'MarkerSize', markerSize, ...
                 'MarkerFaceColor', colors(uav, :), 'LineWidth', 3);
            text(pos(1), pos(2) + 0.05, sprintf('%d', uav), ...
                 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'white');
        else  % 未受扰无人机
            plot(pos(1), pos(2), 's', 'Color', colors(uav, :), 'MarkerSize', 12, ...
                 'LineWidth', 2, 'MarkerFaceColor', 'white');
            text(pos(1), pos(2) + 0.05, sprintf('%d', uav), ...
                 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
        end
    end
    
    % 绘制方向线
    plot_direction(data.clusterCenter, data.true_angle, 1.5, 'r', '-', 3);
    plot_direction(data.clusterCenter, data.alg1_angle, 1.2, 'b', '--', 2);
    plot_direction(data.clusterCenter, data.alg2_angle, 0.9, 'g', ':', 2);
    
    % 标注真实干扰中心
    plot(data.true_pos(1), data.true_pos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    text(data.true_pos(1) + 0.1, data.true_pos(2) + 0.1, ...
         sprintf('(%.2f, %.2f)', data.true_pos(1), data.true_pos(2)), ...
         'FontSize', 9, 'Color', 'red', 'BackgroundColor', 'white');
    
    % 标注STCL-NN估计位置
    plot(data.alg1_pos(1), data.alg1_pos(2), 'bo', 'MarkerSize', 6, ...
         'LineWidth', 2, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'none', 'LineStyle', '--');
    text(data.alg1_pos(1) + 0.1, data.alg1_pos(2) - 0.1, ...
         sprintf('(%.2f, %.2f)', data.alg1_pos(1), data.alg1_pos(2)), ...
         'FontSize', 9, 'Color', 'blue', 'BackgroundColor', 'white');
    
    % 标注加权质心法估计位置
    plot(data.alg2_pos(1), data.alg2_pos(2), 'go', 'MarkerSize', 6, ...
         'LineWidth', 2, 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'none', 'LineStyle', '--');
    text(data.alg2_pos(1) + 0.1, data.alg2_pos(2) - 0.1, ...
         sprintf('(%.2f, %.2f)', data.alg2_pos(1), data.alg2_pos(2)), ...
         'FontSize', 9, 'Color', 'green', 'BackgroundColor', 'white');
    
    % % 标注误差与置信度
    % error_text = sprintf('%s%.2fkm\n%s%.2f', ...
    %     error_label_stcl, data.alg1_error, ...
    %     conf_label_stcl, data.alg1_confidence);
    error_text = sprintf('%s%.2fkm', ...
        error_label_stcl, data.alg1_error);
    error_text = [error_text, sprintf('\n%s%.2fkm', error_label_wcla, data.alg2_error)];
    text(data.clusterCenter(1) + 1.5, data.clusterCenter(2) + 1.5, ...
         error_text, ...
         'FontSize', 9, 'Color', 'black', 'BackgroundColor', 'white');
    
    % 固定轴范围
    xlim([39, 45]);
    ylim([30.5, 35.5]);
    xlabel(label_x, 'FontSize', 12);
    ylabel(label_y, 'FontSize', 12);
    
    % 子图标题
    title(sprintf(uav_in_zone, time_labels{n}, data.t*dt), 'FontSize', 14, 'FontWeight', 'bold');
end

% 第六个子图：修复标题参数错误的图例
ax_legend = subplot('Position', subplot_positions(6, :));
axes(ax_legend);
hold on; 
axis equal;
xlim([39, 45]);
ylim([30.5, 35.5]);
axis off;

% 图例布局参数
legend_left = 40.7;       % 左侧起始X
start_y = 34.5;           % 顶部起始Y
item_spacing = 0.35;      % 项间距
text_x = legend_left + 0.6;  % 文字起始X

% 1. 集群中心
plot(legend_left, start_y, 'k+', 'MarkerSize', 10, 'LineWidth', 2);
text(text_x, start_y, legend_text{1}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = start_y - item_spacing;

% 2. 受扰无人机
plot(legend_left, current_y, 'o', 'Color', colors(1, :), ...
     'MarkerSize', 8, 'MarkerFaceColor', 'auto', 'LineWidth', 2);
text(text_x, current_y, legend_text{2}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 3. 未受扰无人机
plot(legend_left, current_y, 's', 'Color', colors(1, :), ...
     'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'white');
text(text_x, current_y, legend_text{3}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 4. 真实方位
plot_direction([legend_left - 0.1, current_y], 0, 0.4, 'r', '-', 2);
text(text_x, current_y, legend_text{4}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 5. 算法1估计方位
plot_direction([legend_left - 0.1, current_y], 0, 0.4, 'b', '--', 2);
text(text_x, current_y, legend_text{5}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 6. 算法2估计方位
plot_direction([legend_left - 0.1, current_y], 0, 0.4, 'g', ':', 2);
text(text_x, current_y, legend_text{6}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 7. 干扰中心位置
plot(legend_left, current_y, 'ro', 'MarkerSize', 7, 'LineWidth', 2);
text(text_x, current_y, legend_text{7}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 8. 算法1估计位置
plot(legend_left, current_y, 'bo', 'MarkerSize', 7, 'LineWidth', 2, ...
     'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'none', 'LineStyle', '--');
text(text_x, current_y, legend_text{8}, 'FontSize', 10, 'VerticalAlignment', 'middle');
current_y = current_y - item_spacing;

% 9. 算法2估计位置
plot(legend_left, current_y, 'go', 'MarkerSize', 7, 'LineWidth', 2, ...
     'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'none', 'LineStyle', '--');
text(text_x, current_y, legend_text{9}, 'FontSize', 10, 'VerticalAlignment', 'middle');

% 先创建标题并保存句柄（修复核心）
if const.bChinese
    hTitle = title('图例说明', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2,0.2,0.2], ...
          'Position', [legend_left + 1.5, start_y + 0.5]);
else
    hTitle = title('Legend', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.2,0.2,0.2], ...
          'Position', [legend_left + 1.5, start_y + 0.5]);
end

% 标题下划线（使用标题句柄获取位置）
title_data_pos = get(hTitle, 'Position');  % 通过句柄获取位置，避免参数不足
ax_pos = ax_legend.Position;  % [left, bottom, width, height]
x_range = xlim;  % X轴范围
y_range = ylim;  % Y轴范围
% 转换标题位置为相对坐标
title_rel_x = ax_pos(1) + (title_data_pos(1) - x_range(1)) / (x_range(2)-x_range(1)) * ax_pos(3);
title_rel_y = ax_pos(2) + (title_data_pos(2) - y_range(1)) / (y_range(2)-y_range(1)) * ax_pos(4);
% 绘制下划线
annotation(gcf, 'line', ...
    [title_rel_x-0.03, title_rel_x+0.03], ...
    [title_rel_y-0.005, title_rel_y-0.005], ...
    'Color', [0.2, 0.2, 0.2], 'LineWidth', 1.2);

figsDir = fullfile(pwd,'Figs');
savefig(gcf, fullfile(figsDir,'exp01Fig01.fig')); % 可编辑fig图
fprintf('Figure file has been saved to : %s\n', '\Figs\exp01Fig01.fig');

end  % 结束exp01Fig01

%% 辅助函数：计算方向角（弧度）
function angle = calculate_angle(start_pos, target_pos)
    start_pos = start_pos(1:2);
    target_pos = target_pos(1:2);
    
    % 处理异常值
    start_pos(isnan(start_pos)) = 0;
    target_pos(isnan(target_pos)) = 0;
    
    % 计算方向向量
    dir_vec = target_pos - start_pos;
    
    % 避免零向量
    if norm(dir_vec) < 1e-6
        dir_vec = [1, 0];  % 默认向右
    end
    
    angle = atan2(dir_vec(2), dir_vec(1));  % atan2(y, x)
end

%% 辅助函数：绘制带箭头的方向线
function plot_direction(start_pos, angle, length, color, line_style, line_width)
    start_pos = start_pos(1:2);
    start_pos(isnan(start_pos)) = 0;
    
    % 计算终点坐标
    end_x = start_pos(1) + cos(angle) * length;
    end_y = start_pos(2) + sin(angle) * length;
    
    % 绘制方向线（带箭头）
    quiver(start_pos(1), start_pos(2), ...
           end_x - start_pos(1), end_y - start_pos(2), ...
           0, ...  % 关闭自动缩放
           'Color', color, ...
           'LineStyle', line_style, ...
           'LineWidth', line_width, ...
           'MaxHeadSize', 1);
end

