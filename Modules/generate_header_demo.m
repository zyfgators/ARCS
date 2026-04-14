% =========================================================================
% Script: generate_header_demo.m
% Description: ARCS README 首页动图 V3.3 (全员完赛修正版)
% Changelog V3.3:
%   - 【修复】中间组(s=2) 无法到达终点的问题：大幅提升其速度，并增加指向目标的权重。
%   - 微调帧数至 150，给绕行组更多时间余量。
% =========================================================================

clear; clc; close all;

% --- 1. 基础设置 ---
T_final = 150;           % 【修正】微调帧数，给绕行组留足时间
n_agents = 45;           % 粒子数量
map_res = 200;           % 渲染分辨率
color_bg = [0.1 0.1 0.12];   % 深色背景
color_agent = [0.4 0.95 1];  % 荧光蓝
color_trail = [0.2 0.8 1];   % 拖尾色

% --- 2. 场景定义 ---
start_center = [8, 8];        % 起点
target_pos = [95, 95];        % 终点
center = [50, 50];            % 干扰中心

% 创建干扰场
[X, Y] = meshgrid(linspace(0,100,map_res), linspace(0,100,map_res));
sigma = 12; 
Field = exp(-((X-center(1)).^2 + (Y-center(2)).^2) / (2*sigma^2));

% 初始化窗口
hFig = figure('Color', color_bg, 'Position', [100, 100, 1200, 380], ...
    'InvertHardcopy', 'off', 'MenuBar', 'none', 'ToolBar', 'none');

% --- 3. 运动学初始化 ---
start_pos = [start_center(1) + 5*randn(n_agents,1), start_center(2) + 5*randn(n_agents,1)];
history_len = 12; 
pos_history = zeros(n_agents, 2, 3, history_len); 
for h=1:history_len, pos_history(:,:,:,h) = repmat(start_pos, [1, 1, 3]); end

pos = repmat(start_pos, [1, 1, 3]); 
health = ones(n_agents, 3);
finished = false(n_agents, 3);
gifFilename = 'header_demo.gif';

fprintf('正在渲染 V3.3 (中间组修复版)...\n');

% --- 4. 动画主循环 ---
for t = 1:T_final
    % 动态显影
    fade_in = min(1, max(0, (t - 10) / 40)); 
    
    % 更新历史
    pos_history(:,:,:,2:end) = pos_history(:,:,:,1:end-1);
    pos_history(:,:,:,1) = pos;
    
    % === 运动逻辑更新 ===
    for s = 1:3
        for i = 1:n_agents
            % 到达锁定
            if finished(i,s) 
                pos(i,:,s) = target_pos + 0.5*randn(1,2);
                continue; 
            end
            % 死亡锁定
            if health(i,s) <= 0, continue; end
            
            p = squeeze(pos(i, :, s));
            dist = norm(p - center);
            dir_target = (target_pos - p) / (norm(target_pos - p)+0.01);
            
            speed = 1.6; % 基础速度
            vel = [0,0];
            
            if s == 1 % 【策略A: 直穿】
                vel = dir_target * speed * 1.8; 
                
            elseif s == 2 % 【策略B: 绕行】 (核心修正区域)
                detect_r = 30; % 稍微加大探测半径，提早变向
                if dist < detect_r
                    dir_rep = (p - center) / dist;
                    tangent = [-dir_rep(2), dir_rep(1)];
                    
                    % 【修正点1】: 提高 dir_target 权重 (0.1 -> 0.4)，让它更想去终点
                    % 降低 tangent 权重 (1.5 -> 1.2)，避免绕得太远
                    vel = (tangent * 1.2 + dir_rep * 0.3 + dir_target * 0.4); 
                    
                    % 【修正点2】: 大幅提速 (1.6 -> 2.0)，补偿路程损耗
                    vel = vel / norm(vel) * speed * 2.0; 
                else
                    % 正常飞行时也提速，确保能追上进度
                    vel = dir_target * speed * 2.0;
                end
                
            elseif s == 3 % 【策略C: ARCS】
                if dist < 18 
                    dir_rep = (p - center) / dist;
                    tangent = [-dir_rep(2), dir_rep(1)];
                    vel = (tangent * 0.6 + dir_target * 0.8);
                    vel = vel / norm(vel) * speed * 1.8; 
                else
                    vel = dir_target * speed * 1.8;
                end
            end
            
            % 更新位置
            pos(i, :, s) = p + vel + 0.1*randn(1,2);
            
            % === 损伤逻辑 (保持不变) ===
            dmg = 0;
            if dist < 45
                field_val = Field(min(map_res, max(1, round(p(2)/100*map_res))), ...
                                  min(map_res, max(1, round(p(1)/100*map_res))));
                dmg = 0.8 * field_val;
            end
            
            if s==1 
                if dist < 15, dmg = 0.2; 
                elseif dist < 30, dmg = 0.02; 
                else, dmg = 0; end
            end 
            
            if s==3, dmg = dmg * 0.1; end 
            
            health(i,s) = health(i,s) - dmg;
            
            % 判定到达 (放宽一点判定范围，确保容易吸入终点)
            if p(1) > 92 && p(2) > 92
                finished(i,s) = true; 
            end
        end
    end
    
    % === 绘图渲染 ===
    clf;
    titles = {'Naive Penetration', 'Conservative Avoidance', 'ARCS (Optimal)'};
    
    for s = 1:3
        ax = axes('Position', [(s-1)/3, 0.15, 0.33, 0.85]);
        hold on;
        set(ax, 'Color', color_bg, 'XColor', 'none', 'YColor', 'none', ...
            'XLim', [0 100], 'YLim', [0 100], 'DataAspectRatio', [1 1 1]);
        
        % 场
        hImg = imagesc([0 100], [0 100], Field);
        colormap(ax, 'hot'); caxis([0 1.2]); 
        set(hImg, 'AlphaData', Field * 0.7 * fade_in); 
        
        % 拖尾
        for k = history_len:-1:1
            alpha = (1 - k/history_len) * 0.5;
            px = squeeze(pos_history(:,1,s,k));
            py = squeeze(pos_history(:,2,s,k));
            alive = health(:,s) > 0;
            if any(alive)
                plot(px(alive), py(alive), '.', 'Color', [color_trail alpha], 'MarkerSize', 5 + alpha*5);
            end
        end
        
        % 智能体
        alive = health(:,s) > 0;
        if any(alive)
            plot(pos(alive,1,s), pos(alive,2,s), 'o', ...
                'MarkerFaceColor', 'w', 'MarkerEdgeColor', color_agent, 'MarkerSize', 4, 'LineWidth', 1);
        end
        
        % 标题
        text(50, 95, titles{s}, 'Color', [0.8 0.8 0.8], 'FontName', 'Helvetica', ...
            'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        
        % 动态字幕
        res_str = ''; res_col = 'w';
        if s==1 
            if sum(health(:,s)>0) == 0, res_str = 'DESTROYED'; res_col = [1 0.3 0.3]; end
        elseif s==2 
            % 中间组到达后显示
            if all(finished(:,s)), res_str = 'SAFE BUT SLOW'; res_col = [1 0.8 0.2]; end
        elseif s==3 
            if all(finished(:,s)), res_str = 'OPTIMAL'; res_col = [0.4 1 0.4]; end
        end
        
        if ~isempty(res_str)
            text(50, 50, res_str, 'Color', res_col, 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
        
        % 血条
        avg_h = max(0, mean(health(:,s)));
        if avg_h > 0
            rectangle('Position', [20, 5, 60, 2], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
            bar_col = [1-avg_h, avg_h, 0.2];
            rectangle('Position', [20, 5, 60*avg_h, 2], 'FaceColor', bar_col, 'EdgeColor', 'none');
        end
    end
    
    % === 抓取帧 ===
    drawnow; frame = getframe(hFig); im = frame2im(frame); [imind, cm] = rgb2ind(im, 256);
    if t == 1, imwrite(imind, cm, gifFilename, 'gif', 'LoopCount', inf, 'DelayTime', 0.08);
    else, imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08); end
end
fprintf('V3.3 (修复版) 生成完毕: %s\n', gifFilename);