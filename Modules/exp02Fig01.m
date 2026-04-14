% % ***在ResilenceSim中设置：***
% % simParams.CtrlMode = 0;
% % simParams.ConfAnaFlg = false;   % 清置信度分析标志
% % 
% % 生成数据文件：ResilenceCtrlSimData_C0.mat
% % 将其更名为ResilenceCtrlSimExp02.mat
% % 运行：exp02Fig01('ResilenceCtrlSimExp02.mat');

function exp02Fig01(fileName)
    % ===================== 1. 参数配置（RESS规范） =====================
    const = Constants();
    journal_cfg = struct();
    journal_cfg.dpi = 1200;
    journal_cfg.line_width = 2.0;
    journal_cfg.sub_line_width = 1.5;
    journal_cfg.aux_marker_size = 0.5; % 干扰曲线点大小（放大以显噪声）
    journal_cfg.aux_line_width = 0.3;% 干扰曲线线宽（细但清晰）
    journal_cfg.marker_size = 6;
    journal_cfg.axis_label_fontsize = 10;
    journal_cfg.title_fontsize = 12;
    journal_cfg.legend_fontsize = 8;
    journal_cfg.text_fontsize = 8;
    journal_cfg.grid_line_width = 0.6;
    journal_cfg.figure_width = 17;
    journal_cfg.figure_height = 10;
    journal_cfg.font_eng = 'Times New Roman';
    journal_cfg.font_chn = 'SimHei';
    journal_cfg.color_damage = [0.2, 0.5, 0.8];
    journal_cfg.color_confidence = [0.8, 0.2, 0.2];
    % 【新增：五架无人机干扰强度曲线配色（区分明显且不刺眼）】
    journal_cfg.color_uav_intensity = [
        0.2, 0.7, 0.2;    % 无人机1：深绿
        0.8, 0.4, 0.1;    % 无人机2：深橙
        0.5, 0.2, 0.7;    % 无人机3：深紫
        0.1, 0.6, 0.8;    % 无人机4：深青
        0.7, 0.2, 0.2];   % 无人机5：深红
    journal_cfg.color_resilience = [0.7, 0.4, 0.1];
    journal_cfg.color_payload = [0.5, 0.2, 0.7];
    journal_cfg.color_grid = [0.8, 0.8, 0.8];

    % ===================== 2. 数据加载与严格校验 =====================
    if ~exist(fileName, 'file')
        error('数据文件不存在：%s，请检查路径', fileName);
    end

    data = load(fileName);
    if isfield(data, 'researchData')
        researchData = data.researchData;
    else
        researchData = data;
    end
    
    requiredFields = {'timeArray', 'confidencesArray', 'damageFactorsArray', ...
                     'totalEffectivePayloadArray', 'RArray', 'simParams', 'attenuationArray'};
    for f = 1:length(requiredFields)
        if ~isfield(researchData, requiredFields{f})
            error('数据缺少必要字段：%s', requiredFields{f});
        end
    end
    
    time = researchData.timeArray;
    confidences = researchData.confidencesArray;
    damageFactors = researchData.damageFactorsArray;
    totalPayload = researchData.totalEffectivePayloadArray;
    resilienceIndex = researchData.RArray;
    simParams = researchData.simParams;
    dt = simParams.dt;
    % 【关键：提取五架无人机的干扰强度数据（确保是2D数组：时间×无人机数）】
    attenuation = researchData.attenuationArray;
    if size(attenuation, 2) > size(attenuation, 1)
        attenuation = attenuation'; % 转置为：时间步长 × 无人机数量（如1000×5）
    end
    numUAV = size(attenuation, 2); % 获取无人机数量（应为5）
    if numUAV ~= 5
        warning('当前数据无人机数量为%d，非预期5架，将按实际数量绘制', numUAV);
    end

    if isfield(researchData, 'positionsArray')
        uavPositions = researchData.positionsArray;
        if ndims(uavPositions) == 3
            uavX = squeeze(uavPositions(:, :, 1));
            uavY = squeeze(uavPositions(:, :, 2));
        else
            uavX = repmat(uavPositions(:, 1)', length(time), 1);
            uavY = repmat(uavPositions(:, 2)', length(time), 1);
        end
    else
        uavX = repmat(linspace(0, 100, numUAV), length(time), 1);
        uavY = repmat(50 + randn(1, numUAV)*10, length(time), 1);
        warning('数据中未找到无人机位置信息，使用模拟位置数据');
    end
    
    if isfield(researchData, 'interferenceSource') && isfield(researchData.interferenceSource, 'center')
        interfX = researchData.interferenceSource.center(1);
        interfY = researchData.interferenceSource.center(2);
    else
        interfX = 50;
        interfY = 50;
        warning('数据中未找到干扰源位置，使用默认位置(50,50)');
    end
    
    meanDamage = mean(damageFactors, 2);
    stdDamage = std(damageFactors, 0, 2);
    Y_max = max(totalPayload);
    R_post = (trapz(time, totalPayload) / (Y_max * time(end))) * (min(totalPayload) / Y_max);
    
    T_windows = [5, 10, 15];
    R_multi = cell(length(T_windows), 1);
    for i = 1:length(T_windows)
        R_multi{i} = calculate_resilience(time, damageFactors, totalPayload, ...
                                          T_windows(i), dt, Y_max);
    end
    
    [~, peakIdx] = min(totalPayload);
    if isempty(peakIdx)
        peakIdx = round(length(time)/2);
    end
    
    peakTime = time(peakIdx);
    entryTime = 1027.4;
    exitTime = 1172.9;
    entryIdx = find(time >= entryTime, 1);
    exitIdx = find(time >= exitTime, 1);
    if isempty(entryIdx)
        entryIdx = round(length(time)*0.3);
    end
    if isempty(exitIdx)
        exitIdx = round(length(time)*0.7);
    end


    % ===================== 3. 中英文标签配置（兼容输出） =====================
    if const.bChinese
        fontName = journal_cfg.font_chn;
        % 【新增：五架无人机干扰强度图例标签】
        label_uav_intensity = {...
            '无人机1 干扰强度', ...
            '无人机2 干扰强度', ...
            '无人机3 干扰强度', ...
            '无人机4 干扰强度', ...
            '无人机5 干扰强度'...
        };
    else
        fontName = journal_cfg.font_eng;
        label_uav_intensity = {...
            'UAV 1 Interference',...
            'UAV 2 Interference',...
            'UAV 3 Interference',...
            'UAV 4 Interference',...
            'UAV 5 Interference'...
        };
    end
    if const.bChinese
        label_damage = '损伤因子';
        label_damage_mean = '损伤因子均值';
        label_damage_std = '损伤因子标准差';
        label_confidence = '干扰置信度';
        label_intensity = '干扰强度';
        label_time = '时间 (s)';
        label_resilience = '动态韧性值';
        label_resilience_traditional = '传统事后韧性';
        label_payload = '总有效载荷';
        label_x = 'X坐标 (m)';
        label_y = 'Y坐标 (m)';
        
        title_fig1 = '损伤动力学模型：时空-多变量耦合验证';
        title_fig2 = '动态韧性指标：多窗口-多基准对比验证';
        title_sub1 = sprintf('t=%.1fs 损伤空间分布', peakTime);
        title_sub2 = '不同滑动窗口韧性对比';
        
        text_entry = '进入干扰区';
        text_peak = sprintf('干扰最强点 (t=%.1fs)', peakTime);
        text_exit = '离开干扰区';
        text_peak_vals1 = sprintf('干扰最强点: 置信度=%.2f, 损伤=%.2f±%.2f', ...
            confidences(peakIdx), meanDamage(peakIdx), stdDamage(peakIdx));
        text_peak_vals2 = sprintf('载荷最低谷: 韧性=%.2f, 载荷=%.1f', ...
            resilienceIndex(peakIdx), totalPayload(peakIdx));
        % 【更新图例：包含五架无人机干扰曲线】
        legend_fig1 = {
            label_damage_mean, label_damage_std, label_confidence, ...
            label_uav_intensity{1:numUAV}
        };
       % 【新增：补充中文模式下的legend_fig2定义】
        legend_fig2 = {sprintf('动态韧性 (T_w=10s)'), label_resilience_traditional, label_payload};
    else
        label_damage = 'Damage Factor';
        label_damage_mean = 'Mean Damage Factor';
        label_damage_std = 'Damage Std Dev';
        label_confidence = 'Confidence';
        label_intensity = 'Intensity';
        label_time = 'Time (s)';
        label_resilience = 'Dynamic Resilience';
        label_resilience_traditional = 'Traditional Post-hoc Resilience';
        label_payload = 'Total Effective Payload';
        label_x = 'X Coordinate (m)';
        label_y = 'Y Coordinate (m)';
        
        title_fig1 = 'Damage Dynamics Model: Spatiotemporal-Multivariable Verification';
        title_fig2 = 'Dynamic Resilience Index: Multi-window & Multi-benchmark Verification';
        title_sub1 = sprintf('Damage Spatial Distribution at t=%.1fs', peakTime);
        title_sub2 = 'Resilience Comparison with Different Window Sizes';
        
        text_entry = 'Enter Interference Zone';
        text_peak = sprintf('Peak Interference (t=%.1fs)', peakTime);
        text_exit = 'Exit Interference Zone';
        text_peak_vals1 = sprintf('Peak: C=%.2f, Damage=%.2f±%.2f', ...
            confidences(peakIdx), meanDamage(peakIdx), stdDamage(peakIdx));
        text_peak_vals2 = sprintf('Trough: R=%.2f, Payload=%.1f', ...
            resilienceIndex(peakIdx), totalPayload(peakIdx));
        legend_fig1 = {
            label_damage_mean, label_damage_std, label_confidence, ...
            label_uav_intensity{1:numUAV}
        };
        legend_fig2 = {sprintf('Dynamic Resilience (T_w=10s)'), label_resilience_traditional, label_payload};
    end
    
    text_corr = '';
    if size(uavX, 1) >= peakIdx && size(uavX, 2) == size(damageFactors, 2)
        distances = sqrt((uavX(peakIdx,:)-interfX).^2 + (uavY(peakIdx,:)-interfY).^2);
        corrData = corrcoef(distances, damageFactors(peakIdx,:));
        spatialCorrVal = corrData(1,2);
        if const.bChinese
            text_corr = sprintf('空间相关性: %.2f', spatialCorrVal);
        else
            text_corr = sprintf('Spatial Correlation: %.2f', spatialCorrVal);
        end
    else
        warning('无法计算空间相关性，数据维度不匹配');
    end
    
    [confidenceGridX, confidenceGridY] = meshgrid(linspace(min(uavX(:)), max(uavX(:)), 100), ...
                                                  linspace(min(uavY(:)), max(uavY(:)), 100));
    confidenceGrid = zeros(size(confidenceGridX));
    maxDist = max(sqrt((uavX(:)-interfX).^2 + (uavY(:)-interfY).^2));
    for i = 1:size(confidenceGridX,1)
        for j = 1:size(confidenceGridX,2)
            dist = sqrt((confidenceGridX(i,j)-interfX)^2 + (confidenceGridY(i,j)-interfY)^2);
            if maxDist > 0
                confidenceGrid(i,j) = max(0, 1 - dist/maxDist);
            else
                confidenceGrid(i,j) = 1;
            end
        end
    end


    % ===================== 4. 图1：损伤动力学模型 =====================
    fig1 = figure('Name', title_fig1, 'Color', 'white');
    set(fig1, 'Position', [100, 200, ...
        journal_cfg.figure_width*37.8, journal_cfg.figure_height*37.8]);
    set(fig1, 'Renderer', 'Painters');
    hold on; grid on; grid minor; box on;
    set(gca, 'GridColor', journal_cfg.color_grid, 'GridLineWidth', journal_cfg.grid_line_width);
    set(gca, 'MinorGridColor', journal_cfg.color_grid, 'MinorGridLineWidth', journal_cfg.grid_line_width/2);

    % 左Y轴：损伤因子（主曲线+误差填充）
    yyaxis left;
    % 绘制损伤因子标准差填充
    h_fill = fill([time; flipud(time)], [meanDamage+stdDamage; flipud(meanDamage-stdDamage)], ...
         journal_cfg.color_damage, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', label_damage_std);
    % 强制将填充对象移至最底层
    uistack(h_fill, 'bottom');
    
    % 绘制损伤因子均值曲线（上层）
    plot(time, meanDamage, 'Color', journal_cfg.color_damage, 'LineWidth', journal_cfg.line_width, ...
         'DisplayName', label_damage_mean);
    ylabel(label_damage, 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    ylim([0, max(meanDamage+stdDamage)*1.15]);

    % 右Y轴：干扰置信度 + 五架无人机干扰强度（分色+噪声）
    yyaxis right;
    % 1. 干扰置信度（主曲线）
    plot(time, confidences, 'Color', journal_cfg.color_confidence, 'LineWidth', journal_cfg.line_width, ...
         'DisplayName', label_confidence);
    % 2. 【核心优化：循环绘制五架无人机干扰强度曲线（分色+散点显噪声）】
    for uavIdx = 1:numUAV
        % 用scatter逐点绘制，点大小2（显噪声），线宽0.3（不抢焦），每架颜色不同
        scatter(time, attenuation(:, uavIdx), journal_cfg.aux_marker_size, ...
                journal_cfg.color_uav_intensity(uavIdx, :), 'filled', 'MarkerEdgeColor', 'none', ...
                'LineWidth', journal_cfg.aux_line_width, 'MarkerFaceAlpha', 0.4,'DisplayName', label_uav_intensity{uavIdx});
    end
    ylabel([label_confidence ' / ' label_intensity], 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    ylim([0, 1.15]);

    xlabel(label_time, 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    title(title_fig1, 'FontName', fontName, 'FontSize', journal_cfg.title_fontsize, 'FontWeight', 'bold');

    % 时间窗口：800-1300s（突出干扰区）
    targetTimeRange = [800, 1300];
    xlim(targetTimeRange);
    
    % 调整关键时间点标注（不超出窗口）
    validEntryIdx = find(time >= targetTimeRange(1), 1);
    validExitIdx = find(time <= targetTimeRange(2), 1, 'last');
    entryIdx = max(entryIdx, validEntryIdx);
    exitIdx = min(exitIdx, validExitIdx);

    plot([time(entryIdx), time(entryIdx)], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
         'LineWidth', journal_cfg.sub_line_width);
    text(time(entryIdx), 1.08, text_entry, 'VerticalAlignment', 'top', 'FontName', fontName, ...
         'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', [1,1,1,0.6]);

    plot([peakTime, peakTime], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
         'LineWidth', journal_cfg.sub_line_width);
    text(peakTime, 1.08, text_peak, 'VerticalAlignment', 'top', 'FontName', fontName, ...
         'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', [1,1,1,0.6]);

    plot([time(exitIdx), time(exitIdx)], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
         'LineWidth', journal_cfg.sub_line_width);
    text(time(exitIdx), 1.08, text_exit, 'VerticalAlignment', 'top', 'FontName', fontName, ...
         'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', [1,1,1,0.6]);

    % 图例：调整位置至右下角
    legend(legend_fig1{:}, ...
           'Location', 'southeast', ...
           'FontName', fontName, ...
           'FontSize', journal_cfg.legend_fontsize, ...
           'Box', 'on', ...
           'EdgeColor', [0.7,0.7,0.7], ...
           'Position', [0.7, 0.1, 0.25, 0.35], ...  % 增加高度以容纳多行
           'NumColumns', 1);  % 强制单列显示

    % 损伤空间分布热力图（高度增加，显示完整）
    ax1 = axes('Parent', fig1, 'Position', [0.18, 0.15, 0.25, 0.35]);
    hold on; box on; grid on;
    set(ax1, 'GridColor', journal_cfg.color_grid, 'GridLineWidth', journal_cfg.grid_line_width/2);
    scatter(ax1, uavX(peakIdx,:), uavY(peakIdx,:), journal_cfg.marker_size*4, ...
            damageFactors(peakIdx,:), 'filled', 'MarkerEdgeColor', [0,0,0], 'MarkerEdgeAlpha', 0.8);
    plot(ax1, interfX, interfY, 'pentagram', 'MarkerSize', journal_cfg.marker_size+2, ...
         'Color', [0,0,0], 'LineWidth', journal_cfg.sub_line_width);
    contourf(ax1, confidenceGridX, confidenceGridY, confidenceGrid, 10, 'LineStyle', 'none');
    h = findobj(ax1, 'Type', 'patch');
    set(h, 'FaceAlpha', 0.3);
    colormap(ax1, parula);
    colorbar(ax1, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize);
    xlabel(ax1, label_x, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize);
    ylabel(ax1, label_y, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize);
    title(ax1, title_sub1, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize+1, 'FontWeight', 'bold');
    
    if ~isempty(text_corr)
        text(ax1, 0.05, 0.95, text_corr, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
             'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', 'none');
    end

    text(0.02, 0.02, text_peak_vals1, 'Units', 'normalized', 'VerticalAlignment', 'bottom', ...
         'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'Color', [0,0,0], 'BackgroundColor', 'none');

    hold off;


    % % ===================== 5. 图2：动态韧性指标 =====================
    % fig2 = figure('Name', title_fig2, 'Color', 'white');
    % set(fig2, 'Position', [200, 200, ...
    %     journal_cfg.figure_width*37.8, journal_cfg.figure_height*37.8]);
    % set(fig2, 'Renderer', 'Painters');
    % hold on; grid on; grid minor; box on;
    % set(gca, 'GridColor', journal_cfg.color_grid, 'GridLineWidth', journal_cfg.grid_line_width);
    % set(gca, 'MinorGridColor', journal_cfg.color_grid, 'MinorGridLineWidth', journal_cfg.grid_line_width/2);
    % 
    % yyaxis left;
    % plot(time, resilienceIndex, 'Color', journal_cfg.color_resilience, 'LineWidth', journal_cfg.line_width, ...
    %      'DisplayName', sprintf('动态韧性 (T_w=10s)'));
    % plot(time, R_post*ones(size(time)), 'Color', journal_cfg.color_resilience, 'LineStyle', '--', ...
    %      'LineWidth', journal_cfg.sub_line_width, 'DisplayName', label_resilience_traditional);
    % ylabel(label_resilience, 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    % ylim([0, max(resilienceIndex)*1.15]);
    % 
    % yyaxis right;
    % plot(time, totalPayload, 'Color', journal_cfg.color_payload, 'LineWidth', journal_cfg.line_width, ...
    %      'DisplayName', label_payload);
    % ylabel(label_payload, 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    % ylim([0, max(totalPayload)*1.15]);
    % 
    % xlabel(label_time, 'FontName', fontName, 'FontSize', journal_cfg.axis_label_fontsize);
    % title(title_fig2, 'FontName', fontName, 'FontSize', journal_cfg.title_fontsize, 'FontWeight', 'bold');
    % 
    % % 同步时间窗口：800-1300s
    % xlim(targetTimeRange);
    % 
    % plot([time(entryIdx), time(entryIdx)], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
    %      'LineWidth', journal_cfg.sub_line_width);
    % text(time(entryIdx), max(totalPayload)*1.08, text_entry, 'VerticalAlignment', 'bottom', ...
    %      'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', 'none');
    % 
    % plot([peakTime, peakTime], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
    %      'LineWidth', journal_cfg.sub_line_width);
    % text(peakTime, max(totalPayload)*1.08, text_peak, 'VerticalAlignment', 'bottom', ...
    %      'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', 'none');
    % 
    % plot([time(exitIdx), time(exitIdx)], ylim, 'Color', [0.5,0.5,0.5], 'LineStyle', ':', ...
    %      'LineWidth', journal_cfg.sub_line_width);
    % text(time(exitIdx), max(totalPayload)*1.08, text_exit, 'VerticalAlignment', 'bottom', ...
    %      'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'BackgroundColor', 'none');
    % 
    % legend(legend_fig2, 'Location', 'northeast', 'FontName', fontName, 'FontSize', journal_cfg.legend_fontsize, ...
    %        'Box', 'on', 'EdgeColor', [0.7,0.7,0.7], 'Position', [0.65, 0.65, 0.25, 0.25]);
    % 
    % ax2 = axes('Parent', fig2, 'Position', [0.62, 0.22, 0.28, 0.28]);
    % hold on; box on; grid on;
    % set(ax2, 'GridColor', journal_cfg.color_grid, 'GridLineWidth', journal_cfg.grid_line_width/2);
    % 
    % boxplot(ax2, [R_multi{1}, R_multi{2}, R_multi{3}], ...
    %         'Labels', arrayfun(@(x) sprintf('%ds',x), T_windows, 'UniformOutput', false), ...
    %         'BoxStyle', 'filled');
    % 
    % hLines = findobj(ax2, 'Type', 'line');
    % set(hLines, 'LineWidth', journal_cfg.sub_line_width);
    % 
    % % 【只设置箱子(patch对象)的填充颜色和透明度】
    % hBox = findobj(ax2, 'Tag', 'Box', 'Type', 'patch');
    % for i = 1:length(hBox)
    %     set(hBox(i), 'FaceColor', journal_cfg.color_resilience, 'FaceAlpha', 0.6);
    % end
    % 
    % ylabel(ax2, label_resilience, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize);
    % title(ax2, title_sub2, 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize+1, 'FontWeight', 'bold');
    % 
    % for i = 1:length(T_windows)
    %     stdVal = std(R_multi{i});
    %     text(ax2, i, max(R_multi{i})*1.08, sprintf('σ=%.2f', stdVal), ...
    %          'HorizontalAlignment', 'center', 'FontName', fontName, 'FontSize', journal_cfg.text_fontsize);
    % end
    % 
    % text(0.02, 0.02, text_peak_vals2, 'Units', 'normalized', 'VerticalAlignment', 'bottom', ...
    %      'FontName', fontName, 'FontSize', journal_cfg.text_fontsize, 'Color', [0,0,0], 'BackgroundColor', 'none');
    % 
    % hold off;


    % ===================== 6. 输出高清图像（RESS规范） =====================
    figsDir = fullfile(pwd,'Figs');
    savefig(fig1, fullfile(figsDir,'exp02Fig01.fig'));

    fprintf('\n================== 图表生成完成 ==================\n');
end

function R = calculate_resilience(time, damageFactors, payload, windowSize, dt, Y_max)
    numSteps = length(time);
    windowSteps = round(windowSize / dt);
    R = zeros(numSteps, 1);
    meanDamage = mean(damageFactors, 2);
    
    for t = 1:numSteps
        startIdx = max(1, t - windowSteps + 1);
        windowIdx = startIdx:t;
        
        if length(windowIdx) < windowSteps/2
            R(t) = 0;
            continue;
        end
        
        avgPerformance = round(mean(payload(windowIdx)) / Y_max, 4);
        damageResistance = round(1 - mean(meanDamage(windowIdx)), 4);
        recoveryRate = 1;
        if t > 1
            recoveryRate = round(max(0, payload(t) - payload(t-1)) / Y_max / dt, 4);
        end
        
        R(t) = avgPerformance * damageResistance * (1 + recoveryRate);
    end
end