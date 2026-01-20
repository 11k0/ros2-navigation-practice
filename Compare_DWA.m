%% Compare_DWA.m - 传统DWA vs 改进DWA 对比仿真
% 功能：在同一地图上同时运行两种算法，对比轨迹和性能
% 作者：A*+DWA融合项目
% 日期：2026-01-20

clc; clear; close all;

% 添加子目录到路径
addpath('src');
addpath('utils');

fprintf('========== 传统DWA vs 改进DWA 对比仿真 ==========\n\n');

%% 1. 环境初始化
disp('正在初始化地图与全局路径...');

% 加载狭窄通道地图
MAP_Grid = MAP(1); 
[MAX_Y, MAX_X] = size(MAP_Grid);

% 设置固定的起点和终点
start_pos = [19.0, 4.0];   % 起点（右下）
goal_pos  = [2.0, 19.0];   % 终点（左上）

% 构建CLOSED列表
CLOSED = [];
for row = 1:MAX_Y
    for col = 1:MAX_X
        if MAP_Grid(row, col) == 1
            CLOSED = [CLOSED; col, row];
        end
    end
end
if isempty(CLOSED)
    CLOSED = [0, 0];
end

% A*规划（两者共用同一条全局路径）
[global_path, ~] = AStar_Planner(MAP_Grid, start_pos, goal_pos, CLOSED);
if isempty(global_path)
    error('A* 规划失败！');
end

% 路径平滑
smooth_params.num_points = size(global_path, 1) * 5;
smooth_path = Path_Smoother(global_path, 'pchip', smooth_params);

% 路径重采样
diffs = diff(smooth_path);
dists = sqrt(sum(diffs.^2, 2));
cumDist = [0; cumsum(dists)];
finalDist = cumDist(end);
if finalDist > 0
    interpDist = 0:0.1:finalDist;
    smooth_path = interp1(cumDist, smooth_path, interpDist, 'linear');
end

fprintf('A*规划完成，路径点数: %d\n', size(smooth_path, 1));

%% 2. DWA参数设置（两种算法共用基础参数）
base_params.max_speed = 0.5;
base_params.min_speed = 0.0;
base_params.max_yaw_rate = 1.0;
base_params.max_accel = 2.0;
base_params.max_dyaw_rate = 1.5;
base_params.v_resolution = 0.05;
base_params.yaw_rate_resolution = 0.2;
base_params.dt = 0.1;
base_params.predict_time = 1.0;
base_params.robot_radius = 0.3;
base_params.obstacle_margin = 0.1;
base_params.heading_weight = 4.0;
base_params.dist_weight = 4.0;
base_params.velocity_weight = 0.05;
base_params.current_v = 0;
base_params.current_w = 0;

% 传统DWA参数
params_traditional = base_params;
params_traditional.use_adaptive = false;

% 改进DWA参数
params_improved = base_params;
params_improved.use_adaptive = true;

%% 3. 仿真参数
dt = 0.1;
lookahead_dist = 2.0;
goal_tolerance = 0.5;
max_simulation_time = 200;

% 初始位姿
initial_theta = atan2(goal_pos(2) - start_pos(2), goal_pos(1) - start_pos(1));

% 传统DWA状态
robot1_pose = [start_pos(1), start_pos(2), initial_theta];
trajectory1 = robot1_pose;
last_idx1 = 1;
reached1 = false;
collision1 = false;

% 改进DWA状态
robot2_pose = [start_pos(1), start_pos(2), initial_theta];
trajectory2 = robot2_pose;
last_idx2 = 1;
reached2 = false;
collision2 = false;

%% 4. 主仿真循环
fprintf('\n开始对比仿真...\n');
fig = figure('Name', '传统DWA vs 改进DWA', 'Position', [50, 50, 1400, 600]);

simulation_time = 0;
step_count = 0;

while simulation_time < max_simulation_time
    step_count = step_count + 1;
    
    %% 传统DWA更新
    if ~reached1 && ~collision1
        dist1 = sqrt((robot1_pose(1) - goal_pos(1))^2 + (robot1_pose(2) - goal_pos(2))^2);
        if dist1 < goal_tolerance
            reached1 = true;
            fprintf('[传统DWA] 到达目标！时间: %.1f s\n', simulation_time);
        else
            [local_goal1, idx1, ~] = Lookahead_Point(robot1_pose(1:2), smooth_path, lookahead_dist, last_idx1);
            last_idx1 = idx1;
            [v1, w1, ~] = Adaptive_DWA(robot1_pose, local_goal1, MAP_Grid, params_traditional, []);
            
            % 转弯减速
            v1 = v1 * (1 - 0.8 * abs(w1) / params_traditional.max_yaw_rate);
            v1 = max(v1, 0);
            params_traditional.current_v = v1;
            params_traditional.current_w = w1;
            
            % 更新位置
            next_x1 = robot1_pose(1) + v1 * cos(robot1_pose(3)) * dt;
            next_y1 = robot1_pose(2) + v1 * sin(robot1_pose(3)) * dt;
            
            % 碰撞检测
            gx1 = round(next_x1); gy1 = round(next_y1);
            if gx1 >= 1 && gx1 <= MAX_X && gy1 >= 1 && gy1 <= MAX_Y
                if MAP_Grid(gy1, gx1) == 1
                    collision1 = true;
                    fprintf('[传统DWA] 碰撞！时间: %.1f s\n', simulation_time);
                end
            end
            
            if ~collision1
                robot1_pose(1) = next_x1;
                robot1_pose(2) = next_y1;
                robot1_pose(3) = robot1_pose(3) + w1 * dt;
                robot1_pose(3) = atan2(sin(robot1_pose(3)), cos(robot1_pose(3)));
                trajectory1 = [trajectory1; robot1_pose];
            end
        end
    end
    
    %% 改进DWA更新
    if ~reached2 && ~collision2
        dist2 = sqrt((robot2_pose(1) - goal_pos(1))^2 + (robot2_pose(2) - goal_pos(2))^2);
        if dist2 < goal_tolerance
            reached2 = true;
            fprintf('[改进DWA] 到达目标！时间: %.1f s\n', simulation_time);
        else
            [local_goal2, idx2, ~] = Lookahead_Point(robot2_pose(1:2), smooth_path, lookahead_dist, last_idx2);
            last_idx2 = idx2;
            [v2, w2, ~] = Adaptive_DWA(robot2_pose, local_goal2, MAP_Grid, params_improved, []);
            
            % 转弯减速
            v2 = v2 * (1 - 0.8 * abs(w2) / params_improved.max_yaw_rate);
            v2 = max(v2, 0);
            params_improved.current_v = v2;
            params_improved.current_w = w2;
            
            % 更新位置
            next_x2 = robot2_pose(1) + v2 * cos(robot2_pose(3)) * dt;
            next_y2 = robot2_pose(2) + v2 * sin(robot2_pose(3)) * dt;
            
            % 碰撞检测
            gx2 = round(next_x2); gy2 = round(next_y2);
            if gx2 >= 1 && gx2 <= MAX_X && gy2 >= 1 && gy2 <= MAX_Y
                if MAP_Grid(gy2, gx2) == 1
                    collision2 = true;
                    fprintf('[改进DWA] 碰撞！时间: %.1f s\n', simulation_time);
                end
            end
            
            if ~collision2
                robot2_pose(1) = next_x2;
                robot2_pose(2) = next_y2;
                robot2_pose(3) = robot2_pose(3) + w2 * dt;
                robot2_pose(3) = atan2(sin(robot2_pose(3)), cos(robot2_pose(3)));
                trajectory2 = [trajectory2; robot2_pose];
            end
        end
    end
    
    %% 可视化（每5步更新）
    if mod(step_count, 5) == 0
        % 左图：传统DWA
        subplot(1, 2, 1);
        cla; hold on;
        axis equal; xlim([1 MAX_X+1]); ylim([1 MAX_Y+1]); grid on;
        title(sprintf('传统DWA | 时间: %.1f s', simulation_time));
        
        % 绘制障碍物
        for row = 1:MAX_Y
            for col = 1:MAX_X
                if MAP_Grid(row, col) == 1
                    fill([col, col+1, col+1, col], [row, row, row+1, row+1], [0.3 0.3 0.3]);
                end
            end
        end
        
        % 绘制路径
        plot(global_path(:,1)+0.5, global_path(:,2)+0.5, 'k--', 'LineWidth', 1.5);
        plot(trajectory1(:,1)+0.5, trajectory1(:,2)+0.5, 'r-', 'LineWidth', 2.5);
        plot(start_pos(1)+0.5, start_pos(2)+0.5, 'g^', 'MarkerSize', 12, 'LineWidth', 2);
        plot(goal_pos(1)+0.5, goal_pos(2)+0.5, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
        
        % 绘制机器人
        theta1 = robot1_pose(3);
        rectangle('Position', [robot1_pose(1)-0.3+0.5, robot1_pose(2)-0.3+0.5, 0.6, 0.6], ...
                  'Curvature', [1,1], 'FaceColor', [1 0 0 0.5], 'EdgeColor', 'r');
        quiver(robot1_pose(1)+0.5, robot1_pose(2)+0.5, 0.5*cos(theta1), 0.5*sin(theta1), 'r', 'LineWidth', 2);
        
        if collision1
            text(10, 19, '碰撞!', 'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold');
        elseif reached1
            text(10, 19, '到达!', 'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % 右图：改进DWA
        subplot(1, 2, 2);
        cla; hold on;
        axis equal; xlim([1 MAX_X+1]); ylim([1 MAX_Y+1]); grid on;
        title(sprintf('改进DWA (自适应权重) | 时间: %.1f s', simulation_time));
        
        % 绘制障碍物
        for row = 1:MAX_Y
            for col = 1:MAX_X
                if MAP_Grid(row, col) == 1
                    fill([col, col+1, col+1, col], [row, row, row+1, row+1], [0.3 0.3 0.3]);
                end
            end
        end
        
        % 绘制路径
        plot(global_path(:,1)+0.5, global_path(:,2)+0.5, 'k--', 'LineWidth', 1.5);
        plot(trajectory2(:,1)+0.5, trajectory2(:,2)+0.5, 'b-', 'LineWidth', 2.5);
        plot(start_pos(1)+0.5, start_pos(2)+0.5, 'g^', 'MarkerSize', 12, 'LineWidth', 2);
        plot(goal_pos(1)+0.5, goal_pos(2)+0.5, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
        
        % 绘制机器人
        theta2 = robot2_pose(3);
        rectangle('Position', [robot2_pose(1)-0.3+0.5, robot2_pose(2)-0.3+0.5, 0.6, 0.6], ...
                  'Curvature', [1,1], 'FaceColor', [0 0 1 0.5], 'EdgeColor', 'b');
        quiver(robot2_pose(1)+0.5, robot2_pose(2)+0.5, 0.5*cos(theta2), 0.5*sin(theta2), 'b', 'LineWidth', 2);
        
        if collision2
            text(10, 19, '碰撞!', 'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold');
        elseif reached2
            text(10, 19, '到达!', 'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        drawnow;
    end
    
    % 两者都结束则退出
    if (reached1 || collision1) && (reached2 || collision2)
        break;
    end
    
    simulation_time = simulation_time + dt;
end

%% 5. 结果统计
fprintf('\n========== 对比结果 ==========\n');
fprintf('传统DWA:\n');
fprintf('  状态: %s\n', get_status(reached1, collision1));
fprintf('  轨迹长度: %.2f m\n', calc_length(trajectory1));
fprintf('  步数: %d\n', size(trajectory1, 1));

fprintf('\n改进DWA:\n');
fprintf('  状态: %s\n', get_status(reached2, collision2));
fprintf('  轨迹长度: %.2f m\n', calc_length(trajectory2));
fprintf('  步数: %d\n', size(trajectory2, 1));

%% 辅助函数
function s = get_status(reached, collision)
    if reached
        s = '成功到达';
    elseif collision
        s = '碰撞';
    else
        s = '超时';
    end
end

function len = calc_length(traj)
    len = 0;
    for i = 2:size(traj, 1)
        len = len + sqrt((traj(i,1)-traj(i-1,1))^2 + (traj(i,2)-traj(i-1,2))^2);
    end
end
