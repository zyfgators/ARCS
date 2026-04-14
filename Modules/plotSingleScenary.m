%% 目前使用的数据文件如下：
%    无控制模式：       ResilenceCtrlSimData_C0.mat       结果：exp03Fig01A/B/C.fig
%    APF控制模式：      ResilenceCtrlSimData_C1.mat       结果：exp03Fig02A/B/C.fig
%    R(t)韧性控制模式： ResilenceCtrlSimData_C6.mat       结果：exp03Fig03A/B/C.fig
%    ∂(t)韧性控制模式： ResilenceCtrlSimData_C4.mat       结果：exp03Fig04A/B/C.fig

%fileName = 'ResilenceCtrlSimData_C6.mat';
function plotSingleScenary(fileName)

    hWnd = plotUavTrajectory(fileName,'full');
    % 
    % % 2. 调用自动缩放函数，传入句柄、文件名和时间范围
    autoZoom(hWnd, fileName, [920, 1400]);
    
    % plotResilenceIdxSimp(fileName);
    
    plotResilenceIdx(fileName);
end