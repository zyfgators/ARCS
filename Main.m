function Main(experiment_id, bRunSim,fGammaRecovery)
% MAIN Entry point for the Active Resilience Control simulation framework.
%
% This is the primary script to reproduce all experimental results presented
% in the paper "Active Resilience Control for UAV Swarms: A Closed-Loop 
% Framework Integrating Collaborative Perception and Dynamic Metrics".
% It manages paths, executes simulations, and generates figures automatically.
%
% Usage:
%   Main(experiment_id, bRunSim)
%
% Inputs:
%   experiment_id (String/Char): Semantic identifier for the experiment of the "Active Resilience Control for UAV Swarm (ARCS)" paper.
%       -- Collaborative Perception --
%       'exp01Fig01' / 'Localization'  -> Experiment 1: Interference Source Localization (Corresponds to Fig. 3 of the ARCS paper)
%       'exp01Fig02' / 'Confidence'    -> Experiment 1: Confidence Analysis (Corresponds to Figs. 4-5 of the ARCS paper)
%
%       -- Damage Dynamics --
%       'exp02' / 'Dynamics_Resilience' -> Experiment 2: Damage Dynamics Resilience Measurement (Corresponds to Figs. 6-9)
%
%       -- Control Benchmarks --
%       'exp03Fig01' / 'BL_NoControl'  -> Experiment 3: Uncontrolled Strategy (Corresponds to Fig. 10)
%       'exp03Fig02' / 'BL_APF'        -> Experiment 3: APF Control (Corresponds to Fig. 11)
%       'exp03Fig03' / 'AROC_R'        -> Experiment 3: Active Resilience Optimal Control Based on R(t) for Swarm(Corresponds to Fig. 12)
%       'exp03Fig04' / 'AROC_Sigma'    -> Experiment 3: Active Resilience Optimal Control Based on d(t)-based for Swarm(Corresponds to Fig. 13)
%
%   bRunSim (Logical): Simulation control flag.
%       false (Default) -> Visualization Mode. Loads existing data from 'Results/'
%                          and plots figures directly. Skips simulation.
%       true            -> Reproduction Mode. Forces execution of the simulation
%                          to generate new data before plotting.
%   fGammaRecovery：self-recovery coefficients of Interference Damage Dynamics Equition
%       0.0(Default)  ->   Models permanent degradation, resulting in monotonic decay.
%       > 0           ->   Models temporary suppression, enabling the simulation of “V-shaped” recovery trajectories(in reversible scenarios)
%     
% Examples:
%   % Example 1: Plot Fig. 11 using existing data (Visualization only)
%   >> Main('Fig11'); 
%   
%   % Example 2: Run simulation for the proposed method and plot Fig. 13
%   >> Main('Fig13', true);
%
% Author: Yifan Zeng, Xuebin Zhuang et al.
% Journal: Reliability Engineering & System Safety (2025)

     % 默认参数处理
    if nargin < 2
        bRunSim = false;
    end

    if nargin < 3
        fGammaRecovery = 0.0;
    end

    % 1. 环境初始化
    clearvars -except experiment_id bRunSim fGammaRecovery; close all; clc;
    
    % 获取根目录并加载模块
    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'Modules')); 
    addpath(fullfile(rootDir, 'Results'));
    
    % 确保 Results 目录存在
    resultDir = fullfile(rootDir, 'Results');
    figsDir = fullfile(pwd,'Figs');
    % 检查文件夹是否存在，如果不存在则创建
    if ~exist(figsDir, 'dir')
        mkdir(figsDir);
        fprintf('已创建文件夹: %s\n', figsDir);
    end

    if ~exist(resultDir, 'dir')
        mkdir(resultDir);
    end

    fprintf('======================================================\n');
    fprintf('   ARCS Framework: Active Resilience Control Simulation\n');
    fprintf('   Root Dir: %s\n', rootDir);
    fprintf('======================================================\n');

    experiment_id = lower(experiment_id);
    % 2. 实验调度
    switch experiment_id
        % --- Group 1: Perception ---
        case {'localization', 'exp01fig01'} % 支持双名：既支持语义，也兼容旧习惯
            disp('>> [Task] Running Experiment 1A: Interference Localization...');
            fileName = 'ResilenceCtrlSimExp01.mat';
            if(bRunSim) 
                ResilenceSim(0, true,fGammaRecovery); 
                moveData('ResilenceCtrlSimData_C0.mat', fileName, resultDir);
            end
            checkData(fileName, resultDir);
            exp01Fig01; 
            
        case {'confidence', 'exp01fig02'}
            disp('>> [Task] Running Experiment 1B: Confidence Analysis...');
            fileName = 'ResilenceCtrlSimExp01.mat'; 
            checkData(fileName, resultDir);
            exp01Fig02(fullfile(resultDir, fileName));
            
        % --- Group 2: Dynamics ---
        case {'dynamics_resilience', 'exp02'}
            disp('>> [Task] Running Experiment 2: Damage Dynamics Resilience Measurement...');
            fileName = 'ResilenceCtrlSimExp02.mat';
            if(bRunSim) 
                ResilenceSim(0, false,fGammaRecovery);
                moveData('ResilenceCtrlSimData_C0.mat', fileName, resultDir);
            end
            fileName = fullfile(resultDir, fileName);
            exp02Fig01(fileName); 
            fprintf('>> Plotting dynamic resilience process...\n');

            exp02Fig02fun(fileName, 1, 0);
            exp02Fig02fun(fileName, 2, 45);
            exp02Fig02fun(fileName, 3, 0);
            % --- Group 3: Control Strategy Comparison ---
        case {'bl_nocontrol', 'exp03fig01'}
            % --- 1. Mode 0: Uncontrolled (无控制) ---
            fprintf('>> [1/4] Simulating Mode 0: Uncontrolled (Baseline 1)...\n');
            
            if(bRunSim)
                runSimAndMove(0, 'ResilenceCtrlSimData_C0.mat', resultDir,fGammaRecovery);
            end
            plotSingleScenary(fullfile(resultDir, 'ResilenceCtrlSimData_C0.mat'));
            
            fprintf('>> [Done] Mode 0 finished. Check figures.\n');
            fprintf('>> Press SPACE (or any key) to continue to APF Mode...\n');

        case {'bl_apf', 'exp03fig02'}
            % --- 2. Mode 1: APF (人工势场法) ---
            fprintf('\n------------------------------------------------------\n');
            fprintf('>> [2/4] Simulating Mode 1: APF Autonomous Avoidance (Baseline 2)...\n');
            if(bRunSim)
                runSimAndMove(1, 'ResilenceCtrlSimData_C1.mat', resultDir,fGammaRecovery); 
            end
            plotSingleScenary(fullfile(resultDir, 'ResilenceCtrlSimData_C1.mat'));
            
            fprintf('>> [Done] Mode 1 finished. Check figures.\n');
            fprintf('>> Press SPACE (or any key) to continue to R-Control Mode...\n');

        case {'aroc_r', 'exp03fig03'}
            % --- 3. Mode 2: R(t) Resilience Control (基于R的韧性控制) ---
            fprintf('\n------------------------------------------------------\n');
            fprintf('>> [3/4] Simulating Mode 2: R(t)-based Resilience Control (Baseline 3)...\n');
            if(bRunSim)
                runSimAndMove(2, 'ResilenceCtrlSimData_C2.mat', resultDir,fGammaRecovery); 
            end
            plotSingleScenary(fullfile(resultDir, 'ResilenceCtrlSimData_C2.mat'));
            
            fprintf('>> [Done] Mode 2 finished. Check figures.\n');
            fprintf('>> Press SPACE (or any key) to continue to Sigma-Control Mode...\n');

        case {'aroc_sigma', 'exp03fig04'}  
            % --- 4. Mode 3: Sigma(t) Resilience Control (Proposed) ---
            fprintf('\n------------------------------------------------------\n');
            fprintf('>> [4/4] Simulating Mode 3: Sigma(t)-based Active Resilience (Proposed)...\n');
            if(bRunSim)
                runSimAndMove(3, 'ResilenceCtrlSimData_C3.mat', resultDir,fGammaRecovery); 
            end
            plotSingleScenary(fullfile(resultDir, 'ResilenceCtrlSimData_C3.mat'));
            
            fprintf('>> [Done] All control strategies simulated and plotted.\n')
            
        otherwise
            error('Invalid ID. Usage: Main(''1A''), Main(''1B''), Main(''2''), or Main(''3'')');
    end
    
    fprintf('\n>> [Success] Experiment %s completed.\n', experiment_id);
end

% --- Helper Functions ---

function moveData(srcName, destName, resultDir)
    if isfile(srcName)
        destPath = fullfile(resultDir, destName);
        movefile(srcName, destPath);
        fprintf('   [IO] Archived: %s -> Results/%s\n', srcName, destName);
    else
        warning('File %s not found. Simulation may have failed.', srcName);
    end
end

function checkData(fileName, resultDir)
    if ~isfile(fullfile(resultDir, fileName))
        error('Data %s missing in Results/. Run previous step first.', fileName);
    end
end

function runSimAndMove(mode, fileName, resultDir,fGammaRecovery)
    fprintf('   [Sim] Running Mode %d...\n', mode);
    ResilenceSim(mode, false,fGammaRecovery);
    if isfile(fileName)
        movefile(fileName, fullfile(resultDir, fileName));
    end
end