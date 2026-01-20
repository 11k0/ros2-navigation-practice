function [target_point, target_idx, reached_goal] = Lookahead_Point(current_pos, global_path, lookahead_dist, last_idx)
% Lookahead_Point - 在全局路径上寻找前瞻目标点（胶水函数）
%
% 功能:
%   这是A*和DWA之间的"胶水"函数，告诉DWA在全局路径上应该追踪哪个目标点。
%   使用前瞻距离（lookahead distance）在路径上寻找合适的目标点。
%
% 输入:
%   current_pos    - 当前位置 [x, y]
%   global_path    - 全局路径 [N×2] 矩阵
%   lookahead_dist - 前瞻距离（米）
%   last_idx       - 上一次的路径索引（用于避免回退）
%
% 输出:
%   target_point   - 前瞻目标点 [x, y]
%   target_idx     - 目标点在路径中的索引
%   reached_goal   - 是否到达终点 (true/false)
%
% 算法:
%   1. 从last_idx开始向前搜索
%   2. 找到第一个距离>=lookahead_dist的点
%   3. 如果没找到，返回路径终点
%
% 作者: 为A*+DWA融合设计
% 日期: 2026-01-15

%% 参数检查
if nargin < 4
    last_idx = 1;  % 默认从路径起点开始
end

if isempty(global_path)
    error('全局路径为空！');
end

%% 初始化
n_points = size(global_path, 1);
current_x = current_pos(1);
current_y = current_pos(2);

% 确保last_idx在有效范围内
last_idx = max(1, min(last_idx, n_points));

%% 寻找最近的路径点（避免偏离路径太远）
min_dist = inf;
nearest_idx = last_idx;

for i = last_idx:n_points
    dist = sqrt((global_path(i, 1) - current_x)^2 + (global_path(i, 2) - current_y)^2);
    if dist < min_dist
        min_dist = dist;
        nearest_idx = i;
    end
    
    % 如果距离开始增大，说明已经过了最近点
    if dist > min_dist + 1.0
        break;
    end
end

%% 从最近点开始寻找前瞻点
target_idx = nearest_idx;
found_lookahead = false;

for i = nearest_idx:n_points
    dist = sqrt((global_path(i, 1) - current_x)^2 + (global_path(i, 2) - current_y)^2);
    
    if dist >= lookahead_dist
        target_idx = i;
        found_lookahead = true;
        break;
    end
end

% 如果没找到足够远的点，使用路径终点
if ~found_lookahead
    target_idx = n_points;
end

target_point = global_path(target_idx, :);

%% 判断是否到达终点
goal_point = global_path(end, :);
dist_to_goal = sqrt((current_x - goal_point(1))^2 + (current_y - goal_point(2))^2);

% 到达终点的阈值
goal_threshold = 0.5;  % 0.5米

if dist_to_goal < goal_threshold
    reached_goal = true;
    target_point = goal_point;
    target_idx = n_points;
else
    reached_goal = false;
end

%% 可选：自适应前瞻距离调整
% 根据路径曲率动态调整前瞻距离
% 这里暂时不实现，保持简单

end
