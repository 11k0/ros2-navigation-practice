function [global_path, path_info] = AStar_Planner(MAP, start_pos, goal_pos, CLOSED)
% AStar_Planner - 封装的改进A*全局路径规划器
%
% 输入:
%   MAP        - 地图矩阵 (0=可通行, 1=障碍物)
%   start_pos  - 起始位置 [x, y]
%   goal_pos   - 目标位置 [x, y]
%   CLOSED     - 障碍物列表 [x, y] (可选，如果为空则从MAP提取)
%
% 输出:
%   global_path - 优化后的全局路径 [N×2] 矩阵，每行为 [x, y]
%   path_info   - 路径信息结构体
%       .length        - 路径长度
%       .time          - 规划时间
%       .nodes_visited - 遍历节点数
%       .turn_angle    - 转折角度总和
%       .turn_count    - 转折次数
%
% 改进点:
%   1. 8方向搜索 + 障碍物顶点避让
%   2. 三次折线平滑优化
%   3. 自适应评价函数 f(n)=g(n)+(1-log(P))*h(n)
%
% 作者: 基于A_07.m改进封装
% 日期: 2026-01-15

%% 参数提取
xStart = start_pos(1);
yStart = start_pos(2);
xTarget = goal_pos(1);
yTarget = goal_pos(2);

MAX_X = size(MAP, 2);
MAX_Y = size(MAP, 1);

%% 如果CLOSED为空，从MAP提取障碍物
% 注意：MAP(row, col) = MAP(Y, X)
% CLOSED存储为[X, Y]格式，与起点终点坐标一致
if isempty(CLOSED)
    k = 1;
    CLOSED = [];
    for row = 1:MAX_Y
        for col = 1:MAX_X
            if MAP(row, col) == 1
                CLOSED(k, 1) = col;  % X = col
                CLOSED(k, 2) = row;  % Y = row
                k = k + 1;
            end
        end
    end
end

%% 初始化
tic;  % 开始计时

OPEN = [];
CLOSED_COUNT = size(CLOSED, 1);
Nobs = CLOSED_COUNT;

xNode = xStart;
yNode = yStart;
OPEN_COUNT = 1;
path_cost = 0;
goal_distance = distance(xNode, yNode, xTarget, yTarget);

% 插入起始节点到OPEN列表
OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
OPEN(OPEN_COUNT, 1) = 0;

% 将起始点加入CLOSED
CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED(CLOSED_COUNT, 1) = xNode;
CLOSED(CLOSED_COUNT, 2) = yNode;

NoPath = 1;

% 计算障碍率
P_obsNT = Obs_array(xStart, yStart, xTarget, yTarget, CLOSED, Nobs);
P = log(P_obsNT);

%% A*搜索主循环
while ((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    % 扩展当前节点
    exp_array = expand_array_Obs8(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, Nobs, P);
    exp_count = size(exp_array, 1);
    
    % 更新OPEN列表
    for i = 1:exp_count
        flag = 0;
        for j = 1:OPEN_COUNT
            if (exp_array(i, 1) == OPEN(j, 2) && exp_array(i, 2) == OPEN(j, 3))
                OPEN(j, 8) = min(OPEN(j, 8), exp_array(i, 5));
                if OPEN(j, 8) == exp_array(i, 5)
                    OPEN(j, 4) = xNode;
                    OPEN(j, 5) = yNode;
                    OPEN(j, 6) = exp_array(i, 3);
                    OPEN(j, 7) = exp_array(i, 4);
                end
                flag = 1;
            end
        end
        
        if flag == 0
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT, :) = insert_open(exp_array(i, 1), exp_array(i, 2), xNode, yNode, exp_array(i, 3), exp_array(i, 4), exp_array(i, 5));
        end
    end
    
    % 选择fn最小的节点
    index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
    
    if (index_min_node ~= -1)
        xNode = OPEN(index_min_node, 2);
        yNode = OPEN(index_min_node, 3);
        path_cost = OPEN(index_min_node, 6);
        
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1) = xNode;
        CLOSED(CLOSED_COUNT, 2) = yNode;
        OPEN(index_min_node, 1) = 0;
    else
        NoPath = 0;
    end
end

%% 路径回溯
i = size(CLOSED, 1);
Optimal_path = [];
xval = CLOSED(i, 1);
yval = CLOSED(i, 2);
i = 1;
Optimal_path(i, 1) = xval;
Optimal_path(i, 2) = yval;

if ((xval == xTarget) && (yval == yTarget))
    % 回溯路径
    Target_ind = node_index(OPEN, xval, yval);
    parent_x = OPEN(Target_ind, 4);
    parent_y = OPEN(Target_ind, 5);
    Optimal_path(i, 3) = parent_x;
    Optimal_path(i, 4) = parent_y;
    
    while (parent_x ~= xStart || parent_y ~= yStart)
        i = i + 1;
        Optimal_path(i, 1) = parent_x;
        Optimal_path(i, 2) = parent_y;
        
        inode = node_index(OPEN, parent_x, parent_y);
        Optimal_path(i, 3) = OPEN(inode, 4);
        Optimal_path(i, 4) = OPEN(inode, 5);
        parent_x = Optimal_path(i, 3);
        parent_y = Optimal_path(i, 4);
    end
    
    Num_Opt = size(Optimal_path, 1);
    
    %% 第一次折线优化
    Optimal_path_one = Line_OPEN_ST(Optimal_path, CLOSED, Nobs, Num_Opt);
    
    % 提取路径
    Optimal_path_try = [1 1 1 1];
    Optimal_path = [1 1];
    i = 1;
    q = 1;
    x_g = Optimal_path_one(Num_Opt, 3);
    y_g = Optimal_path_one(Num_Opt, 4);
    
    Optimal_path_try(i, 1) = Optimal_path_one(q, 1);
    Optimal_path_try(i, 2) = Optimal_path_one(q, 2);
    Optimal_path_try(i, 3) = Optimal_path_one(q, 3);
    Optimal_path_try(i, 4) = Optimal_path_one(q, 4);
    
    while (Optimal_path_try(i, 3) ~= x_g || Optimal_path_try(i, 4) ~= y_g)
        i = i + 1;
        q = Optimal_index(Optimal_path_one, Optimal_path_one(q, 3), Optimal_path_one(q, 4));
        
        Optimal_path_try(i, 1) = Optimal_path_one(q, 1);
        Optimal_path_try(i, 2) = Optimal_path_one(q, 2);
        Optimal_path_try(i, 3) = Optimal_path_one(q, 3);
        Optimal_path_try(i, 4) = Optimal_path_one(q, 4);
    end
    
    % 反向排列
    n = size(Optimal_path_try, 1);
    for i = 1:1:n
        Optimal_path(i, 1) = Optimal_path_try(n, 3);
        Optimal_path(i, 2) = Optimal_path_try(n, 4);
        n = n - 1;
    end
    num_op = size(Optimal_path, 1) + 1;
    Optimal_path(num_op, 1) = Optimal_path_try(1, 1);
    Optimal_path(num_op, 2) = Optimal_path_try(1, 2);
    
    %% 第二次折线优化
    Optimal_path_two = Line_OPEN_STtwo(Optimal_path, CLOSED, Nobs, num_op);
    num_optwo = size(Optimal_path_two, 1) + 1;
    Optimal_path_two(num_optwo, 1) = xStart;
    Optimal_path_two(num_optwo, 2) = yStart;
    
    %% 第三次折线优化
    j = num_optwo;
    Optimal_path_two2 = [xStart yStart];
    for i = 1:1:num_optwo
        Optimal_path_two2(i, 1) = Optimal_path_two(j, 1);
        Optimal_path_two2(i, 2) = Optimal_path_two(j, 2);
        j = j - 1;
    end
    
    Optimal_path_three = Line_OPEN_STtwo(Optimal_path_two2, CLOSED, Nobs, num_optwo);
    num_opthree = size(Optimal_path_three, 1) + 1;
    Optimal_path_three(num_opthree, 1) = xTarget;
    Optimal_path_three(num_opthree, 2) = yTarget;
    
    global_path = Optimal_path_three;
    
    %% 计算路径信息
    planning_time = toc;
    
    j = size(global_path, 1);
    
    % 计算转折角度
    angle_du = 0;
    for i = 1:1:(j - 2)
        du = angle6(global_path(i, 1), global_path(i, 2), global_path(i + 1, 1), global_path(i + 1, 2), global_path(i + 2, 1), global_path(i + 2, 2));
        angle_du = angle_du + du;
    end
    
    % 计算路径长度
    S = 0;
    for i = 1:1:(j - 1)
        Dist = sqrt((global_path(i, 1) - global_path(i + 1, 1))^2 + (global_path(i, 2) - global_path(i + 1, 2))^2);
        S = S + Dist;
    end
    
    % 输出路径信息
    path_info.length = S;
    path_info.time = planning_time;
    path_info.nodes_visited = size(OPEN, 1);
    path_info.turn_angle = angle_du;
    path_info.turn_count = j - 2;
    
    % 显示信息
    fprintf('改进A*算法规划完成\n');
    fprintf('  规划时间: %.4f 秒\n', planning_time);
    fprintf('  路径长度: %.2f\n', S);
    fprintf('  转折角度: %.2f 度\n', angle_du);
    fprintf('  转折次数: %d\n', j - 2);
    fprintf('  遍历节点: %d\n', size(OPEN, 1));
    
else
    % 无路径
    global_path = [];
    path_info.length = inf;
    path_info.time = toc;
    path_info.nodes_visited = 0;
    path_info.turn_angle = 0;
    path_info.turn_count = 0;
    
    warning('未找到从起点到终点的路径！');
end

end
