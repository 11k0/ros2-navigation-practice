function smooth_path = Path_Smoother(raw_path, method, params)
% Path_Smoother - 路径平滑处理函数
%
% 输入:
%   raw_path - 原始路径 [N×2] 矩阵，每行为 [x, y]
%   method   - 平滑方法 ('spline', 'pchip', 'linear', 'bezier')
%   params   - 参数结构体
%       .num_points - 插值点数量 (默认: 原路径长度的5倍)
%       .smooth_factor - 平滑因子 (0-1, 仅用于某些方法)
%
% 输出:
%   smooth_path - 平滑后的路径 [M×2] 矩阵
%
% 说明:
%   DWA需要平滑的曲线路径，而A*生成的是折线路径。
%   本函数通过插值方法将折线转换为平滑曲线。
%
% 作者: 为A*+DWA融合设计
% 日期: 2026-01-15

%% 参数检查
if nargin < 2
    method = 'pchip';  % 默认使用PCHIP插值（保形）
end

if nargin < 3
    params = struct();
end

if ~isfield(params, 'num_points')
    params.num_points = size(raw_path, 1) * 5;  % 默认5倍密度
end

if ~isfield(params, 'smooth_factor')
    params.smooth_factor = 0.5;
end

%% 检查路径有效性
if size(raw_path, 1) < 2
    warning('路径点数量不足，无法平滑');
    smooth_path = raw_path;
    return;
end

%% 计算累积路径长度（用于参数化）
n = size(raw_path, 1);
t = zeros(n, 1);  % 参数化变量

for i = 2:n
    dist = sqrt((raw_path(i, 1) - raw_path(i-1, 1))^2 + (raw_path(i, 2) - raw_path(i-1, 2))^2);
    t(i) = t(i-1) + dist;
end

% 归一化到[0, 1]
if t(end) > 0
    t = t / t(end);
else
    smooth_path = raw_path;
    return;
end

%% 生成插值参数
t_interp = linspace(0, 1, params.num_points)';

%% 根据方法进行插值
switch lower(method)
    case 'spline'
        % 三次样条插值（最平滑，但可能产生振荡）
        x_interp = spline(t, raw_path(:, 1), t_interp);
        y_interp = spline(t, raw_path(:, 2), t_interp);
        
    case 'pchip'
        % 保形分段三次Hermite插值（推荐，平滑且不振荡）
        x_interp = pchip(t, raw_path(:, 1), t_interp);
        y_interp = pchip(t, raw_path(:, 2), t_interp);
        
    case 'linear'
        % 线性插值（最简单，不够平滑）
        x_interp = interp1(t, raw_path(:, 1), t_interp, 'linear');
        y_interp = interp1(t, raw_path(:, 2), t_interp, 'linear');
        
    case 'bezier'
        % Bezier曲线（非常平滑，但计算复杂）
        smooth_path = bezier_smooth(raw_path, params.num_points);
        return;
        
    otherwise
        error('未知的平滑方法: %s', method);
end

%% 组合结果
smooth_path = [x_interp, y_interp];

%% 移除重复点（距离过近的点）
min_dist = 0.1;  % 最小点间距
i = 1;
while i < size(smooth_path, 1)
    dist = sqrt((smooth_path(i+1, 1) - smooth_path(i, 1))^2 + ...
                (smooth_path(i+1, 2) - smooth_path(i, 2))^2);
    if dist < min_dist
        smooth_path(i+1, :) = [];  % 删除过近的点
    else
        i = i + 1;
    end
end

end

%% 辅助函数：Bezier曲线平滑
function smooth_path = bezier_smooth(raw_path, num_points)
    % 使用三次Bezier曲线段连接路径点
    n = size(raw_path, 1);
    
    if n < 2
        smooth_path = raw_path;
        return;
    end
    
    % 每段路径使用Bezier曲线
    points_per_segment = ceil(num_points / (n - 1));
    smooth_path = [];
    
    for i = 1:(n - 1)
        P0 = raw_path(i, :);
        P3 = raw_path(i + 1, :);
        
        % 计算控制点（简化版本：沿直线1/3和2/3处）
        P1 = P0 + (P3 - P0) / 3;
        P2 = P0 + 2 * (P3 - P0) / 3;
        
        % 生成Bezier曲线点
        t = linspace(0, 1, points_per_segment)';
        
        for j = 1:length(t)
            tj = t(j);
            B = (1 - tj)^3 * P0 + 3 * (1 - tj)^2 * tj * P1 + ...
                3 * (1 - tj) * tj^2 * P2 + tj^3 * P3;
            smooth_path = [smooth_path; B];
        end
    end
    
    % 移除重复的最后一个点
    smooth_path = unique(smooth_path, 'rows', 'stable');
end
