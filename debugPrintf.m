% ==================== 核心：调试打印控制函数 ====================
function debugPrintf(msg, bCtrlPrint)
    % debugPrintf - 基于模式和控制参数的打印控制函数
    % 输入：
    %   msg         - 待打印的消息字符串
    %   bCtrlPrint  - 逻辑值（默认false），控制打印行为：
    %          - 调试模式（isDebugMode=true）下：bCtrlPrint=true → 跳过打印；false → 打印
    %          - 非调试模式（isDebugMode=false）下：bCtrlPrint=true → 强制打印；false → 不打印
    % 切换模式：修改下方isDebugMode变量
    
    isDebugMode = true;  % 调试模式开关（true=调试模式，false=非调试模式）
    
    % 打印逻辑实现
    if (isDebugMode && ~bCtrlPrint) || (~isDebugMode && bCtrlPrint)
        fprintf('%s\n', msg);
    end
end