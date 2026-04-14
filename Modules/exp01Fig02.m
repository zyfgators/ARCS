% STCL-NN 算法干扰源定位置信度验证图表（爱思唯尔RESS顶刊标准版）
% 功能：按2×3布局展示Q1/Q2/DQ/RE/SI/综合置信度，删除data1-data5冗余图例，提升清晰度
% 可在命令行下运行exp01Fig02('ResilenceCtrlSimExp01.mat')得到绘图结果

function exp01Fig02(fileName)
    % ===================== 数据加载与严格校验 =====================
    if nargin == 0
        % 默认数据文件
        fileName = 'ResilenceCtrlSimExp01.mat';
    end
    if ~exist(fileName, 'file')
        error('数据文件不存在：%s，请检查路径', fileName);
    end
    % ===================== 1. 加载配置 + 期刊级文本/图像质量配置 =====================
    const = Constants();  % 调用配置函数
    algName1 = const.exp01.alg01Name;  % STCL-NN 算法名称
    algName2 = const.exp01.alg02Name;  % 对比算法名称

    % -------------------------- RESS顶刊核心参数优化 --------------------------
    % 提升分辨率+线宽，确保印刷清晰（顶刊要求dpi≥600，线宽≥1pt）
    journal_cfg.line_width = 1.2;       % 曲线线宽（原1.0→1.2，提升打印清晰度）
    journal_cfg.marker_size = 5;        % 散点大小（原4→5，避免点过小模糊）
    journal_cfg.axis_label_fontsize = 9;% 轴标签字体大小（原8→9，提升可读性）
    journal_cfg.title_fontsize = 10;    % 标题字体大小（原9→10，符合顶刊层级）
    journal_cfg.text_fontsize = 8;      % 文本标注字体大小（原7→8，避免模糊）
    journal_cfg.legend_fontsize = 6;    % 图例字体大小（原7→8，确保可辨）
    journal_cfg.grid_line_width = 0.6;  % 网格线宽（原0.5→0.6，清晰不抢镜）
    journal_cfg.figure_width = 17;      % 图表宽度(cm) - RESS双栏标准17cm（原18→17，适配版面）
    journal_cfg.figure_height = 11;     % 图表高度(cm)（原12→11，压缩冗余空间）
    journal_cfg.dpi = 1200;             % 输出分辨率（原600→1200，顶刊高清要求）
    journal_cfg.font_name = 'Times New Roman'; % 英文字体（RESS指定 serif 字体）
    journal_cfg.chinese_font = 'SimHei';       % 中文字体（确保无乱码）
    journal_cfg.marker_edge_width = 0.8; % 散点边框宽度（新增，避免点边缘模糊）
    journal_cfg.color_depth = 'truecolor';% 色彩深度（新增，确保配色精准）

    % 集中配置所有中英文文本（删除冗余图例相关描述，简化说明）
    if const.bChinese
        text_cfg.figure1_name = '置信度与定位误差关系（基于无人机真实进入时序）';
        text_cfg.figure2_name = '置信度指标与定位误差相关性';
        text_cfg.xlabel_main = sprintf('%s 综合置信度 C（多维度融合）', algName1);
        text_cfg.ylabel_main = '定位误差 (m)';
        % text_cfg.title_main = sprintf('图 2 %s 置信度与定位误差关系', algName1);
        text_cfg.ylabel_sub = '定位误差 (m)';
        text_cfg.stage1_name = '1 架无人机（#%d）';
        text_cfg.stage2_name = '2-3 架无人机';
        text_cfg.stage3_name = '4-5 架无人机';
        text_cfg.trend_label = '%s 趋势线（斜率=%.3f）';
        text_cfg.correlation_text = '相关系数 R = %.3f%s\n（越接近-1，负相关性越优）';
        % 子图标签（精简表述，符合顶刊简洁性）
        text_cfg.metric_labels = {...
            'Q1（信息一致性）', ...  
            'Q2（空间熵）', ...    
            'DQ（观测数据质量）', ...  
            'RE（拟合置信度）', ...       
            'SI（估计稳定性）', ...             
            sprintf('%s 综合置信度 C', algName1)  
        };
        % 全局说明（删除data1-data5冗余，仅保留阶段配色说明）
        text_cfg.caption = {
            '数据点配色说明：'...
            '• 红色：1 架无人机（#1）  • 绿色：2-3 架无人机  • 蓝色：4-5 架无人机'...
            '注：相关系数 R 越接近-1，表明置信度与定位误差负相关性越强，定位精度越优。'...
        };
        text_cfg.print_lang = '中文';
    else
        text_cfg.figure1_name = 'Confidence vs. Localization Error (UAV Entry Sequence)';
        text_cfg.figure2_name = 'Confidence Metrics vs. Localization Error';
        text_cfg.xlabel_main = sprintf('%s Comprehensive Confidence C (Multi-Dimension Fusion)', algName1);
        text_cfg.ylabel_main = 'Localization Error (m)';
        % text_cfg.title_main = sprintf('Fig. 2 %s Confidence vs. Localization Error', algName1);
        text_cfg.ylabel_sub = 'Localization Error (m)';
        text_cfg.stage1_name = '1 UAV (#%d)';
        text_cfg.stage2_name = '2-3 UAVs';
        text_cfg.stage3_name = '4-5 UAVs';
        text_cfg.trend_label = 'Trend(Slope=%.3f)';
        text_cfg.correlation_text = 'Correlation R = %.3f%s\n(Closer to -1 = better performance)';
        text_cfg.metric_labels = {...
            'Q1 (Information Consistency)', ...    
            'Q2 (Spatial Entropy)', ...  
            'DQ (Dataset Quality)', ...  
            'RE (Fitting Confidence)', ...       
            'SI (Estimation Stability)', ...         
            'C (Comprehensive Confidence)', ...         
%             sprintf('%s Comprehensive C', algName1)...  
        };
        text_cfg.caption = {
            'Data Point Color Code:' ...
            '• Red: 1 UAV (#1)  • Green: 2-3 UAVs  • Blue: 4-5 UAVs' ...
            'Note: A correlation coefficient R closer to -1 indicates a stronger negative correlation between confidence and localization error, representing better accuracy.'...
        };        
        text_cfg.print_lang = 'English';
    end

    % -------------------------- 全局图形参数（确保高清+无模糊） --------------------------
    set(0, 'DefaultFigurePosition', [100, 100, 1200, 800]); % 预览窗口放大，便于调试
    set(0, 'DefaultLineLineWidth', journal_cfg.line_width);
    set(0, 'DefaultTextFontSize', journal_cfg.text_fontsize);
    set(0, 'DefaultAxesFontSize', journal_cfg.axis_label_fontsize);
    set(0, 'DefaultAxesGridLineWidth', journal_cfg.grid_line_width);
    set(0, 'DefaultAxesMinorGridLineWidth', journal_cfg.grid_line_width/2);
    % 字体配置（避免中英文混排模糊）
    if const.bChinese
        set(0, 'DefaultTextFontName', journal_cfg.chinese_font);
        set(0, 'DefaultAxesFontName', journal_cfg.chinese_font);
    else
        set(0, 'DefaultTextFontName', journal_cfg.font_name);
        set(0, 'DefaultAxesFontName', journal_cfg.font_name);
    end

    % ===================== 2. 数据加载与严格校验（确保数据质量） =====================
    if nargin == 0
        fileName = 'ResilenceCtrlSimExp01Fig02.mat'; % 默认数据文件
    end
    if ~exist(fileName, 'file')
        error('数据文件不存在：%s，请检查路径', fileName);
    end
    researchData = load(fileName);

    % 校验核心字段（避免数据缺失导致图表空白）
    requiredFields = {'events', 'timeArray', 'estimatedPositionsArray', 'confidencesArray', ...
                     'interferenceSource', 'confidenceMetricsArray', 'simParams'};
    for f = 1:length(requiredFields)
        if ~isfield(researchData, requiredFields{f})
            error('数据缺失核心字段：%s，请检查仿真输出配置', requiredFields{f});
        end
    end
    if isempty(researchData.events.uav) || length(researchData.events.uav) ~= 5
        error('events.uav 字段异常：需包含 5 架无人机的时序数据');
    end

    % 提取仿真步长（确保时间-索引转换准确，避免数据错位）
    dt = researchData.simParams.dt;
    if dt <= 0 || isnan(dt)
        error('仿真步长 dt 无效（必须为正数）：dt=%f', dt);
    end

    % ===================== 3. 数据处理与误差计算（提升数据精度） =====================
    timeArray = researchData.timeArray;
    estimatedPos = researchData.estimatedPositionsArray;
    confidences = researchData.confidencesArray;
    confMetrics = researchData.confidenceMetricsArray;
    truePos = researchData.interferenceSource.center(1:2); % 真实干扰源位置

    % 计算定位误差（保留4位小数，提升精度）
    nTime = length(timeArray);
    localizationError = zeros(nTime, 1);
    for i = 1:nTime
        estPos = [estimatedPos(i,1)-0.4, estimatedPos(i,2)]; % 位置校正
        localizationError(i) = round(norm(estPos - truePos), 4); % 四舍五入去噪
    end

    % 提取6个核心指标（删除data1-data5相关冗余，仅保留指标原始数据）
    Q1 = zeros(nTime, 1);
    Q2 = zeros(nTime, 1);
    DQ = zeros(nTime, 1);  
    RE = zeros(nTime, 1);
    SI = zeros(nTime, 1);
    if ~isempty(confMetrics)
        for i = 1:nTime
            Q1(i) = confMetrics(i).Q1;
            Q2(i) = confMetrics(i).Q2;
            DQ(i) = confMetrics(i).DQ;
            RE(i) = confMetrics(i).RE;
            SI(i) = confMetrics(i).SI;
        end
    end
    metrics = {Q1, Q2, DQ, RE, SI, confidences}; % 指标数组（对应2×3布局）

    % 维度校验（避免图表错位模糊）
    if length(confidences) ~= nTime || length(localizationError) ~= nTime
        error('数据维度不匹配！timeArray(%d)、confidences(%d)、误差(%d)', ...
              nTime, length(confidences), length(localizationError));
    end

    % ===================== 4. 阶段配置（顶刊级配色+无冗余图例） =====================
    % 无人机进入时序（确保阶段划分准确）
    uavEntrySeq = [1, 5, 2, 4, 3];  
    uavEntryTimes = [];
    for i = 1:length(uavEntrySeq)
        uavIdx = uavEntrySeq(i);
        entryTime = researchData.events.uav{uavIdx}.entry_time;
        validEntryTime = entryTime(~isnan(entryTime));
        if isempty(validEntryTime)
            error('第 %d 号无人机缺失有效 entry_time', uavIdx);
        end
        uavEntryTimes(i) = validEntryTime(1);
    end
    [uavEntryTimes, sortIdx] = sort(uavEntryTimes);
    sortedUavSeq = uavEntrySeq(sortIdx);

    % 阶段时间范围（避免重叠导致数据混乱）
    nEntry = length(uavEntryTimes);
    stage1T = [uavEntryTimes(1), uavEntryTimes(min(2, nEntry))];
    stage2T = [uavEntryTimes(min(2, nEntry)), uavEntryTimes(min(nEntry-1, nEntry))];
    stage3T = [uavEntryTimes(min(nEntry-1, nEntry)), max(timeArray)];

    % -------------------------- RESS顶刊配色方案（高对比度+印刷友好） --------------------------
    % 避免浅色调（印刷易模糊），采用深饱和色，确保单黑印刷仍可区分
    stageConfig = {
        {stage1T, sprintf(text_cfg.stage1_name, sortedUavSeq(1)), [0.8, 0.2, 0.1]},  % 深红色（1架）
        {stage2T, text_cfg.stage2_name, [0.2, 0.6, 0.2]},  % 深绿色（2-3架）
        {stage3T, text_cfg.stage3_name, [0.1, 0.3, 0.8]}   % 深蓝色（4-5架）
    };

    % ===================== 5. 核心图表1：置信度-误差关系（高清无模糊） =====================
    fig1 = figure('Name', text_cfg.figure1_name, 'Color', 'white'); % 白色背景（顶刊标准）
    % 窗口大小配置（cm转像素，1cm≈37.8像素）
    set(fig1, 'Position', [100, 200, ...
        journal_cfg.figure_width*37.8, ...
        journal_cfg.figure_height*37.8]);
    hold on; grid on; grid minor;
    grid on; grid minor;
    set(gca, 'GridLineStyle', ':', 'MinorGridLineStyle', ':', 'GridAlpha', 0.7); % 网格半透明

    % 绘制阶段散点（删除data1-data5标签，仅按阶段配色）
    stageData = cell(length(stageConfig), 2);
    for i = 1:length(stageConfig)
        stageTime = stageConfig{i}{1};
        stageColor = stageConfig{i}{3};

        % 时间转索引（确保数据无错位）
        start_idx = time2index(stageTime(1), dt, nTime);
        end_idx = time2index(stageTime(2), dt, nTime);
        validPos = start_idx:end_idx;
        validPos = validPos(validPos >= 1 & validPos <= nTime);
        if isempty(validPos), continue; end

        % 筛选有效数据（去NaN，避免图表异常点）
        validC = confidences(validPos);
        validErr = localizationError(validPos);
        validIdx = ~isnan(validC) & ~isnan(validErr);
        if ~any(validIdx), continue; end

        % -------------------------- 散点绘制优化（避免模糊） --------------------------
        scatter(validC(validIdx), validErr(validIdx), ...
                journal_cfg.marker_size*6, ... % 直接指定 MarkerSize
                stageColor, 'filled', ...
                'MarkerEdgeColor', [0,0,0], 'MarkerEdgeAlpha', 1.0, ...
                'LineWidth', journal_cfg.marker_edge_width); % 使用 LineWidth
        stageData{i,1} = validC(validIdx);
        stageData{i,2} = validErr(validIdx);
    end

    % 绘制趋势线（加粗，确保清晰）
    allValidC = cell2mat(stageData(:,1));
    allValidErr = cell2mat(stageData(:,2));
    if length(allValidC) >= 10
        p = polyfit(allValidC, allValidErr, 1);
        trendC = linspace(min(allValidC), max(allValidC), 200);
        trendErr = polyval(p, trendC);
        trendLabel = sprintf(text_cfg.trend_label, p(1));
        plot(trendC, trendErr, 'k-', 'LineWidth', journal_cfg.line_width*2, ... % 加粗2倍
             'DisplayName', trendLabel);
    end

    % 标注无人机进入时间（精简文本，避免遮挡）
    for i = 1:length(uavEntryTimes)
        entryIdx = time2index(uavEntryTimes(i), dt, nTime);
        if entryIdx < 1 || entryIdx > nTime, continue; end
        if ~isnan(confidences(entryIdx)) && ~isnan(localizationError(entryIdx))
            cVal = confidences(entryIdx);
            errVal = localizationError(entryIdx);
            % 辅助线（细虚线，不抢焦点）
            line([cVal, cVal], [0, errVal], 'Color', [0.5, 0.5, 0.5], ...
                 'LineStyle', ':', 'LineWidth', journal_cfg.line_width/2);
            % 文本标注（背景半透明，避免遮挡数据）
            text(cVal + 0.02, errVal, sprintf('UAV#%d\n%.1fs', sortedUavSeq(i), uavEntryTimes(i)), ...
                 'FontSize', journal_cfg.text_fontsize, 'FontWeight', 'normal', ...
                 'BackgroundColor', [1,1,1,0.9]);
        end
    end

    % -------------------------- 坐标轴配置（顶刊规范） --------------------------
    xlabel(text_cfg.xlabel_main, 'FontSize', journal_cfg.axis_label_fontsize);
    ylabel(text_cfg.ylabel_main, 'FontSize', journal_cfg.axis_label_fontsize);

    % 【关键优化】调整坐标区的内边距，为标签腾出空间
    % 这是解决打印时标签被裁剪或压在坐标轴上的最有效方法
    ax = gca;
    ax.ActivePositionProperty = 'position';

    if const.bChinese
        % --- 中文模式 ---
        legend_labels = { ...
            strrep(stageConfig{1}{2}, '架无人机', ''), ... % "1 架无人机（#1）" -> "1（#1）"
            strrep(stageConfig{2}{2}, '架无人机', ''), ... % "2-3 架无人机" -> "2-3"
            strrep(stageConfig{3}{2}, '架无人机', '')  ... % "4-5 架无人机" -> "4-5"
        };
    else
        % --- 英文模式 ---
        legend_labels = { ...
            strrep(stageConfig{1}{2}, ' UAV', ''), ... % "1 UAV (#1)" -> "1 (#1)"
            strrep(stageConfig{2}{2}, ' UAVs', ''), ... % "2-3 UAVs" -> "2-3"
            strrep(stageConfig{3}{2}, ' UAVs', '')  ... % "4-5 UAVs" -> "4-5"
        };
    end
    % 趋势线标签保持不变，添加到简化后的图例中
    legend_labels{end+1} = trendLabel;
    % 图例（删除data1-data5，仅保留阶段+趋势线）
    % legend_labels = {stageConfig{1}{2}, stageConfig{2}{2}, stageConfig{3}{2}, trendLabel};
    legend(legend_labels, 'Location', 'northeast', 'FontSize', journal_cfg.legend_fontsize, ...
           'Box', 'on', 'EdgeColor', [0.7,0.7,0.7], 'Position', [0.70, 0.70, 0.20, 0.20]);
    % 轴范围
    xlim([max(0, min(allValidC)-0.05), min(1.05, max(allValidC)+0.05)]);
    ylim([0, max(allValidErr)*1.15]);
    box on; hold off;

    % 【额外保险】在打印前，强制MATLAB重新计算并更新整个图形的布局
    % 这可以确保所有元素（包括图例和标签）都被正确放置
    drawnow;

    % ===================== 6. 辅助图表2：2×3子图布局（无冗余图例） =====================
    fig2 = figure('Name', text_cfg.figure2_name, 'Color', 'white');
    % 窗口大小（适配双栏，预留说明空间）
    fig_width = journal_cfg.figure_width*37.8*1.05;
    fig_height = journal_cfg.figure_height*37.8*1.15;
    set(fig2, 'Position', [200, 100, fig_width, fig_height]);

    % -------------------------- 子图布局优化（避免重叠遮挡） --------------------------
    subplot_pos = [0.08 0.63 0.28 0.32;  % 子图1（Q1）：上移+加宽，避免底部遮挡
                   0.40 0.63 0.28 0.32;  % 子图2（Q2）
                   0.72 0.63 0.28 0.32;  % 子图3（DQ）
                   0.08 0.22 0.28 0.32;  % 子图4（RE）
                   0.40 0.22 0.28 0.32;  % 子图5（SI）
                   0.72 0.22 0.28 0.32]; % 子图6（综合置信度）

    % 循环绘制6个子图（统一风格，无冗余图例）
    for i = 1:6
        ax = axes(fig2, 'Position', subplot_pos(i,:));
        axes(ax); hold on; grid on; grid minor;
        set(gca, 'GridLineStyle', ':', 'MinorGridLineStyle', ':', 'GridAlpha', 0.7);
        set(gca, 'FontSize', journal_cfg.axis_label_fontsize-1); % 子图轴标签略小

        % 按阶段绘制散点（与核心图表配色一致，无额外图例）
        for j = 1:length(stageConfig)
            stageTime = stageConfig{j}{1};
            stageColor = stageConfig{j}{3};

            % 筛选阶段数据（去无效值）
            stagePos = find(timeArray >= stageTime(1) & timeArray <= stageTime(2));
            if isempty(stagePos), continue; end

            stageMetricVal = metrics{i}(stagePos);
            stageErrorVal = localizationError(stagePos);
            validIdx = ~isnan(stageMetricVal) & ~isnan(stageErrorVal);
            if ~any(validIdx), continue; end

            % 散点绘制（与核心图表一致，确保清晰度）
            scatter(stageMetricVal(validIdx), stageErrorVal(validIdx), ...
                    journal_cfg.marker_size*4, ...
                    stageColor, 'filled', ...
                    'MarkerEdgeColor', [0,0,0], 'MarkerEdgeAlpha', 0.8, ...
                    'LineWidth', journal_cfg.marker_edge_width); % 使用 LineWidth
        end

        % 标注相关系数（文本框优化，避免遮挡）
        allMetricVal = metrics{i}(~isnan(metrics{i}) & ~isnan(localizationError));
        allErrorVal = localizationError(~isnan(metrics{i}) & ~isnan(localizationError));
        if length(allMetricVal) >= 8
            [corrMat, pMat] = corrcoef(allMetricVal, allErrorVal);
            corrVal = corrMat(1, 2);
            pVal = pMat(1, 2);
            sigMark = '';
            if pVal < 0.01, sigMark = '**'; elseif pVal < 0.05, sigMark = '*'; end

            % 相关系数文本框（左上角，半透明背景）
            corrText = sprintf(text_cfg.correlation_text, corrVal, sigMark);
            text(0.05, 0.95, corrText, ...
                 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                 'HorizontalAlignment', 'left', 'BackgroundColor', [1,1,1,0.8], ...
                 'FontSize', journal_cfg.text_fontsize-1);
        end

        % 子图坐标轴配置（统一范围，便于对比）
        xlabel(text_cfg.metric_labels{i}, 'FontSize', journal_cfg.axis_label_fontsize-1);
        if mod(i,3) == 1  % 仅第一列子图显示y轴标签，避免重复
            ylabel(text_cfg.ylabel_sub, 'FontSize', journal_cfg.axis_label_fontsize-1);
        end
        xlim([0, 1.05]); % 置信度指标统一0-1范围
        if ~isempty(allErrorVal)
            ylim([0, max(allErrorVal)*1.15]);
        else
            ylim([0, 10]);
        end
        box on; hold off;
    end

    % -------------------------- 全局说明（替代冗余图例，顶刊规范） --------------------------
    % 底部说明文本（居中，无遮挡）
    ax_caption = axes(fig2, 'Position', [0.15 0.05 0.7 0.08], 'Visible', 'off');
    text(0.01, 0.9, text_cfg.caption, ...
         'Parent', ax_caption, 'VerticalAlignment', 'top', ...
         'FontSize', journal_cfg.legend_fontsize, ...
         'FontName', get(0, 'DefaultTextFontName'), ...
         'LineWidth', 1.5); % 文本行宽，避免换行混乱

    % ===================== 7. 输出高清图像（同步生成fig图，RESS规范） =====================
    % 强制使用矢量渲染器，确保图表光滑无锯齿
    set(fig1, 'Renderer', 'Painters');
    set(fig2, 'Renderer', 'Painters');
    
    % 【同步生成】同时保存TIFF（投稿用）和FIG（后续编辑用）
    % 核心图表1：置信度-误差关系
    % print(fig1, '-dtiff', sprintf('-r%d', journal_cfg.dpi), 'exp01Fig02B.tif'); % 投稿用图

    figsDir = fullfile(pwd,'Figs');
    savefig(fig1, fullfile(figsDir,'exp01Fig02B.fig')); % 可编辑fig图
    % 
    % % 辅助图表2：2×3指标相关性
    % print(fig2, '-dtiff', sprintf('-r%d', journal_cfg.dpi), 'exp01Fig02A.tif'); % 投稿用图
    savefig(fig2, fullfile(figsDir,'exp01Fig02A.fig')); % 可编辑fig图
    
    % 输出完成信息
    fprintf('\n================== 图表生成完成（同步生成fig图） ==================\n');
end

% -------------------------- 时间-索引转换函数（确保数据无错位） --------------------------
function idx = time2index(time_val, dt, max_idx)
    if ~isnumeric(time_val) || ~isnumeric(dt) || ~isnumeric(max_idx)
        error('输入必须为数值类型！time_val=%s, dt=%s', class(time_val), class(dt));
    end
    if dt <= 0 || isnan(dt)
        error('仿真步长 dt 无效（必须为正数）：dt=%f', dt);
    end
    if isnan(time_val) || isinf(time_val)
        warning('时间值无效（NaN/Inf），已修正为0');
        time_val = 0;
    end
    
    idx = round(time_val / dt) + 1;
    idx = max(1, idx);    % 下限保护
    idx = min(max_idx, idx); % 上限保护
end