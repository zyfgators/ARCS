%% 自动缩放函数（最终版）
function autoZoom(fig, fileName, timeRange)
    % 对由 plotUavTrajectory 生成的图形进行自动缩放。
    %
    % 输入参数:
    %   hWnd        - plotUavTrajectory 函数返回的图形句柄。
    %   fileName    - 数据文件路径。
    %   timeRange   - 一个包含两个元素的数组 [startTime, endTime]。

    % 1. 加载数据
    data = load(fileName);
    
    % 2. 计算指定时间窗口内的坐标范围
    startTime = timeRange(1);
    endTime = min(timeRange(2), data.timeArray(end));
    timeMask = (data.timeArray >= startTime) & (data.timeArray <= endTime);
    
    validX = data.positionsArray(timeMask, :, 1);
    validY = data.positionsArray(timeMask, :, 2);

    focus_x_min = floor(min(validX(:)) - 2);
    focus_x_max = ceil(max(validX(:)) + 2);
    focus_y_min = floor(min(validY(:)) - 2);
    focus_y_max = ceil(max(validY(:)) + 2);

    % 3. 应用缩放
    ax = gca(fig); % 使用传入的句柄获取坐标轴
    xlim(ax, [focus_x_min, focus_x_max]);
    ylim(ax, [focus_y_min, focus_y_max]);

    % 4. (可选) 更新标题以反映当前视图
    const = Constants(); % 假设您有此配置函数
    title_text = get(ax, 'Title');
    original_title_string = title_text.String;
    if const.bChinese
        new_title_string = [original_title_string, sprintf(' (观察窗口: %.1fs 至 %.1fs)', startTime, endTime)];
    else
        new_title_string = [original_title_string, sprintf(' (View: %.1fs to %.1fs)', startTime, endTime)];
    end
    set(title_text, 'String', new_title_string);

    [~, name, ~] = fileparts(fileName);
    num = str2double(name(end)) + 1;
    newFileName = sprintf('exp03Fig%02dA', num);
    % 保存图形
    figsDir = fullfile(pwd,'Figs');
    savefig(fig, fullfile(figsDir, newFileName));

    fprintf('自动缩放完成！\n');
end