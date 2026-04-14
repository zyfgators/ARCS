% % ***在ResilenceSim中设置：***
% % simParams.CtrlMode = 0;
% % simParams.ConfAnaFlg = false;   % 清置信度分析标志
% % 
% 生成数据文件：ResilenceCtrlSimData_C0.mat
% 将其更名为ResilenceCtrlSimExp02.mat
% 运行：exp02Fig02; 
% exp02Fig02fun的第2个参数是场景参数，第3个参数是角度调整参数
%   第2个参数：表示场景类型，1=T0+10s, 2=T1, 3=离开前10s。T0表示刚无人机集群进入干扰区时刻, T1为峰值时间，或给定时刻
%   第3个参数：表示角度调整参数，左转为正，右转为负，单位为度。
%  论文做了如下几组实验：
%      exp02Fig02A:  场景1，Tk = T0 + 10, Tw为默认50s, psi = 0
%      exp02Fig02B:  场景2，Tk = T1, Tw为默认50s, psi = 30
%      exp02Fig02C:  场景3，Tk = Texit - 10, Tw为离开时刻 - Tk, psi = 0
function exp02Fig02(iChoice)
    % clc;clear all;
    fileName = 'ResilenceCtrlSimExp02.mat';
    if (iChoice == 2)
       exp02Fig02fun(fileName,2,30);      % exp02Fig02B实验程序
    else
        % exp02Fig02fun(fileName,1);           % exp02Fig02A实验程序
        % exp02Fig02fun(fileName,3);         % exp02Fig02C实验程序
        exp02Fig02fun(fileName,iChoice);         % exp02Fig02A或C实验程序
    end
end