%% main.m - A* + DWA 路径规划仿真主入口
% 作者: [11k]
% 日期: 2026-01-20
% 描述: 一键运行路径规划仿真，支持多种模式选择

clear; clc; close all;

%% 添加路径
addpath('src');
addpath('utils');

%% 选择仿真模式
fprintf('========================================\n');
fprintf('   A* + DWA 路径规划仿真系统\n');
fprintf('========================================\n\n');
fprintf('请选择仿真模式:\n');
fprintf('  [1] 单机器人仿真（可切换传统/改进DWA）\n');
fprintf('  [2] 对比仿真（传统DWA vs 改进DWA 同时运行）\n');
fprintf('  [0] 退出\n\n');

mode = input('请输入选项 (0-2): ');

switch mode
    case 1
        fprintf('\n启动单机器人仿真...\n');
        fprintf('提示: 在 Main_AStar_DWA_Simulation.m 第167行修改 use_adaptive\n');
        fprintf('      true=改进DWA, false=传统DWA\n\n');
        Main_AStar_DWA_Simulation;
        
    case 2
        fprintf('\n启动对比仿真（左:传统DWA 右:改进DWA）...\n');
        Compare_DWA;
        
    case 0
        fprintf('\n已退出。\n');
        return;
        
    otherwise
        fprintf('\n无效选项。\n');
        return;
end

fprintf('\n仿真完成！\n');
