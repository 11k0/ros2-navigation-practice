function line_nodes = LINE(p1, p2)
% LINE - 计算两点之间连线上的所有栅格节点
%
% 输入:
%   p1 - 起点坐标 [x1, y1]
%   p2 - 终点坐标 [x2, y2]
%
% 输出:
%   line_nodes - 连线上的所有栅格节点 [N×2] 矩阵
%
% 算法: Bresenham直线算法的变体，生成两点间所有经过的栅格
%
% 作者: A*路径优化辅助函数
% 日期: 2026-01-20

x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);

% 计算差值
dx = abs(x2 - x1);
dy = abs(y2 - y1);

% 确定步进方向
if x1 < x2
    sx = 1;
else
    sx = -1;
end

if y1 < y2
    sy = 1;
else
    sy = -1;
end

% 初始化
line_nodes = [];
x = x1;
y = y1;

% Bresenham算法
if dx >= dy
    % 以x为主轴
    err = dx / 2;
    while x ~= x2
        line_nodes = [line_nodes; x, y];
        err = err - dy;
        if err < 0
            y = y + sy;
            err = err + dx;
        end
        x = x + sx;
    end
else
    % 以y为主轴
    err = dy / 2;
    while y ~= y2
        line_nodes = [line_nodes; x, y];
        err = err - dx;
        if err < 0
            x = x + sx;
            err = err + dy;
        end
        y = y + sy;
    end
end

% 添加终点
line_nodes = [line_nodes; x2, y2];

% 确保至少有一个点
if isempty(line_nodes)
    line_nodes = [x1, y1; x2, y2];
end

end
