%% Main_AStar_DWA_Simulation - A*+DWA融合仿真主程序
%
% 功能：
%   1. 使用改进A*进行全局路径规划（只运行一次）
%   2. 对路径进行平滑处理
%   3. 使用DWA进行局部路径规划（循环调用）
%   4. 支持动态障碍物避让
%   5. 实时可视化仿真过程
%
% 架构：
%   全局规划（A*） -> 路径平滑 -> 局部规划（DWA循环）
%
% 作者: A*+DWA融合项目
% 日期: 2026-01-15

clear; clc; close all;

% 添加子目录到路径
addpath('src');
addpath('utils');
addpath('data');

fprintf('========== A* + DWA 融合仿真 ==========\n\n');

%% ========== 第1步: 环境初始化 ==========
fprintf('---------- 步骤1: 环境初始化 ----------\n');

% 加载地图（0=空地图，1=狭窄通道，2=U型陷阱）
map_choice = 1;  % 使用狭窄通道地图
MAP = MAP(map_choice);
% MAP = rot90(MAP, 3);  % 取消旋转测试

MAX_X = size(MAP, 2);
MAX_Y = size(MAP, 1);

fprintf('地图尺寸: %d x %d (地图类型: %d)\n', MAX_X, MAX_Y, map_choice);

% 地图膨胀（关闭以便机器人能通过狭窄通道）
robot_radius_grid = 0;  % 关闭膨胀
if robot_radius_grid > 0
    MAP_inflated = imdilate(MAP, strel('disk', robot_radius_grid));
else
    MAP_inflated = MAP;  % 不膨胀
end
fprintf('地图膨胀: 半径 %d 个栅格\n', robot_radius_grid);

% 提取障碍物列表（使用膨胀地图）
% 注意：MAP(row, col) = MAP(Y, X)
% CLOSED存储为[X, Y]格式
k = 1;
CLOSED = zeros(0, 2);  % 初始化为空的2列矩阵
for row = 1:MAX_Y
    for col = 1:MAX_X
        if MAP_inflated(row, col) == 1
            CLOSED(k, 1) = col;  % X = col
            CLOSED(k, 2) = row;  % Y = row
            k = k + 1;
        end
    end
end

% 如果没有障碍物，添加一个边界外的虚拟障碍物（避免A*报错）
if isempty(CLOSED)
    CLOSED = [0, 0];  % 添加一个不影响规划的虚拟障碍物
end

fprintf('膨胀后障碍物: %d 个\n', size(CLOSED, 1));

%% ========== 第2步: 设置固定起点和终点 ==========
fprintf('\n---------- 步骤2: 设置固定起点和终点 ----------\n');

% 策略二：对角线穿梭 (S-Turn) - 测试切角问题
% 起点在右下，终点在左上，连接两点的直线切过障碍物内角
start_pos = [19.0, 4.0];   % 起点（右下）- 与Compare_Simulation.m一致
goal_pos  = [2.0, 19.0];   % 终点（左上）- 与Compare_Simulation.m一致

fprintf('起点: [%.1f, %.1f]\n', start_pos(1), start_pos(2));
fprintf('终点: [%.1f, %.1f]\n', goal_pos(1), goal_pos(2));

% 检查起点是否在障碍物区域内
if MAP(round(start_pos(2)), round(start_pos(1))) == 1
    warning('起点在障碍物区域内！');
end

% 检查终点是否在障碍物区域内
if MAP(round(goal_pos(2)), round(goal_pos(1))) == 1
    warning('终点在障碍物区域内！');
end

%% ========== 第3步: A*全局路径规划（只运行一次）==========
fprintf('\n---------- 步骤3: A*全局路径规划 ----------\n');

[global_path, path_info] = AStar_Planner(MAP, start_pos, goal_pos, CLOSED);

if isempty(global_path)
    error('A*规划失败！');
end

fprintf('A*规划成功！\n');
fprintf('  路径点数: %d\n', size(global_path, 1));
fprintf('  规划时间: %.4f 秒\n', path_info.time);
fprintf('  路径长度: %.2f 米\n', path_info.length);

%% ========== 第4步: 路径平滑 ==========
fprintf('\n---------- 步骤4: 路径平滑处理 ----------\n');

% 使用PCHIP插值平滑路径
smooth_params.num_points = size(global_path, 1) * 5;
smooth_path = Path_Smoother(global_path, 'pchip', smooth_params);

fprintf('路径平滑完成: %d 点 -> %d 点\n', size(global_path, 1), size(smooth_path, 1));

% 对路径进行均匀重采样（每0.1米一个点）
diffs = diff(smooth_path);
dists = sqrt(sum(diffs.^2, 2));
cumDist = [0; cumsum(dists)];
finalDist = cumDist(end);
if finalDist > 0
    interpDist = 0:0.1:finalDist;
    smooth_path = interp1(cumDist, smooth_path, interpDist, 'linear');
    fprintf('路径重采样: 每0.1米一个点，共 %d 点\n', size(smooth_path, 1));
end

%% ========== 第5步: 初始化动态障碍物 ==========
fprintf('\n---------- 步骤5: 初始化动态障碍物 ----------\n');

% 动态障碍物（可选）
use_dynamic_obstacle = false;  % 关闭动态障碍物（狭窄通道测试用）

if use_dynamic_obstacle
    dyn_obs(1).pos = [12.0, 8.0];   % 初始位置（通道出口附近，制造"遭遇战"）
    dyn_obs(1).vel = [0, 0.6];      % 速度向量（向Y轴正方向移动，切断机器人的路）
    dyn_obs(1).radius = 0.8;        % 障碍物半径（设大一点，逼机器人绕大弯）
    dyn_obs(1).y_range = [8.0, 14.0]; % 移动范围（在Y=8到Y=14之间往复巡逻）
    
    fprintf('动态障碍物: 1 个（巡逻模式）\n');
    fprintf('  初始位置: [%.1f, %.1f]\n', dyn_obs(1).pos(1), dyn_obs(1).pos(2));
    fprintf('  速度: [%.2f, %.2f] m/s\n', dyn_obs(1).vel(1), dyn_obs(1).vel(2));
    fprintf('  巡逻范围: Y=[%.1f, %.1f]\n', dyn_obs(1).y_range(1), dyn_obs(1).y_range(2));
else
    dyn_obs = [];
    fprintf('无动态障碍物\n');
end

%% ========== 第6步: DWA参数设置 ==========
fprintf('\n---------- 步骤6: DWA参数设置 ----------\n');

% DWA参数
dwa_params.max_speed = 0.5;              % 降低最大速度 m/s（让规划时间更长）
dwa_params.min_speed = 0.0;
dwa_params.max_yaw_rate = 1.0;           % 最大角速度 1.0 rad/s (约57度/秒)，更真实
dwa_params.max_accel = 2.0;              % 增大加速度，让动态窗口更大
dwa_params.max_dyaw_rate = 1.5;          % 最大角加速度 1.5 rad/s^2
dwa_params.v_resolution = 0.05;          % 减小分辨率，增加速度采样
dwa_params.yaw_rate_resolution = 0.2;    % 约11度，减少角速度采样
dwa_params.dt = 0.1;                     % 时间步长
dwa_params.predict_time = 1.0;           % 减小预测时间
dwa_params.robot_radius = 0.3;           % 机器人半径（缩小）
dwa_params.obstacle_margin = 0.1;        % 安全裕度（total=0.4，不检查相邻栅格）

% 评价函数权重（传统DWA：制造死锁效果）
dwa_params.heading_weight = 4.0;   % 引力极大：死命要去终点
dwa_params.dist_weight = 4.0;      % 斥力极大：死命怕撞墙
dwa_params.velocity_weight = 0.05; % 动力极小：没有向前惯性

% === 传统DWA开关 ===
dwa_params.use_adaptive = false;   % true=改进DWA（自适应权重），false=传统DWA（固定权重）

% 当前速度（初始为0）
dwa_params.current_v = 0;
dwa_params.current_w = 0;

fprintf('DWA参数设置完成\n');

%% ========== 第7步: 仿真参数设置 ==========
fprintf('\n---------- 步骤7: 仿真参数设置 ----------\n');

% 机器人初始位姿 [x, y, theta]
% 根据起点和终点位置自动设置车头朝向
initial_theta = atan2(goal_pos(2) - start_pos(2), goal_pos(1) - start_pos(1));
robot_pose = [start_pos(1), start_pos(2), initial_theta];
fprintf('初始朝向: %.1f 度 (朝向终点)\n', initial_theta * 180 / pi);

% 仿真参数
dt = 0.1;                    % 时间步长（秒）
lookahead_dist = 2.0;        % 前瞻距离（米）
goal_tolerance = 0.5;        % 到达阈值（米）
max_simulation_time = 200;   % 最大仿真时间（秒）- 增加时间以观察路径差异

% 记录轨迹
trajectory = robot_pose;
velocity_history = [];  % 记录速度历史 [v, w]
last_path_idx = 1;

% === 论文图片模式设置 ===
% 设置为true时，仿真结束后生成带机器人残影的论文图片
generate_paper_figure = true;
ghost_step_interval = 2;     % 每隔多少步绘制一个残影（更密集）
stagnation_threshold = 0.45; % 速度低于此值视为减速（红色）- 接近max_speed的90%

fprintf('仿真参数:\n');
fprintf('  时间步长: %.2f 秒\n', dt);
fprintf('  前瞻距离: %.1f 米\n', lookahead_dist);
fprintf('  到达阈值: %.1f 米\n', goal_tolerance);

%% ========== 第8步: 主仿真循环 ==========
fprintf('\n---------- 步骤8: 开始仿真 ----------\n');
fprintf('按 Ctrl+C 停止仿真\n\n');

% 创建图形窗口
fig = figure('Name', 'A* + DWA 实时仿真', 'Position', [100, 100, 1000, 800]);

simulation_time = 0;
step_count = 0;
is_collision = false;      % 碰撞标志
collision_pos = [];         % 碰撞位置

while simulation_time < max_simulation_time
    step_count = step_count + 1;
    
    %% 1. 检查是否到达目标
    dist_to_goal = sqrt((robot_pose(1) - goal_pos(1))^2 + (robot_pose(2) - goal_pos(2))^2);
    
    if dist_to_goal < goal_tolerance
        fprintf('\n成功到达目标！\n');
        fprintf('  仿真时间: %.2f 秒\n', simulation_time);
        fprintf('  仿真步数: %d\n', step_count);
        break;
    end
    
    %% 2. 更新动态障碍物（巡逻模式：往复移动）
    if ~isempty(dyn_obs)
        for i = 1:length(dyn_obs)
            dyn_obs(i).pos = dyn_obs(i).pos + dyn_obs(i).vel * dt;
            
            % 巡逻模式：碰到边界就反弹
            if isfield(dyn_obs(i), 'y_range')
                if dyn_obs(i).pos(2) > dyn_obs(i).y_range(2)
                    dyn_obs(i).pos(2) = dyn_obs(i).y_range(2);
                    dyn_obs(i).vel = -dyn_obs(i).vel;  % 速度反向
                elseif dyn_obs(i).pos(2) < dyn_obs(i).y_range(1)
                    dyn_obs(i).pos(2) = dyn_obs(i).y_range(1);
                    dyn_obs(i).vel = -dyn_obs(i).vel;  % 速度反向
                end
            else
                % 无巡逻范围时的边界限制
                if dyn_obs(i).pos(2) >= MAX_Y
                    dyn_obs(i).pos(2) = MAX_Y;
                    dyn_obs(i).vel = [0, 0];
                end
            end
            
            % X方向边界限制
            if dyn_obs(i).pos(1) < 1
                dyn_obs(i).pos(1) = 1;
            elseif dyn_obs(i).pos(1) > MAX_X
                dyn_obs(i).pos(1) = MAX_X;
            end
            if dyn_obs(i).pos(2) < 1
                dyn_obs(i).pos(2) = 1;
            end
        end
    end
    
    %% 3. 在A*路径上找局部目标（Lookahead）
    [local_goal, target_idx, reached_goal] = Lookahead_Point(robot_pose(1:2), smooth_path, lookahead_dist, last_path_idx);
    last_path_idx = target_idx;
    
    %% 4. 运行DWA获取控制指令（使用原始地图，不用膨胀地图）
    [v, w, best_traj] = Adaptive_DWA(robot_pose, local_goal, MAP, dwa_params, dyn_obs);
    
    % 调试输出
    if v == 0 && w == 0
        fprintf('[警告] 步骤 %d: DWA返回速度为0！位置: [%.2f, %.2f]\n', step_count, robot_pose(1), robot_pose(2));
    end
    
    %% 4.1 物理约束：转弯必减速（模拟真实车辆动力学）
    % 角速度越大，线速度越受限制
    % v_limited = v * (1 - k * |w| / max_yaw_rate)
    % k = 0.8 表示最大角速度时，线速度降到20%
    k_turn_penalty = 0.8;  % 转弯惩罚系数
    max_yaw_rate = dwa_params.max_yaw_rate;
    turn_ratio = abs(w) / max_yaw_rate;  % 0~1
    v_original = v;
    v = v * (1 - k_turn_penalty * turn_ratio);
    v = max(v, 0);  % 确保速度非负
    
    % 更新DWA参数中的当前速度
    dwa_params.current_v = v;
    dwa_params.current_w = w;
    
    %% 4.5 碰撞检测（在更新状态前检查，考虑机器人半径）
    theta = robot_pose(3);
    next_x = robot_pose(1) + v * cos(theta) * dt;
    next_y = robot_pose(2) + v * sin(theta) * dt;
    
    % 机器人半径和安全裕度（与DWA保持一致）
    robot_radius = dwa_params.robot_radius;
    margin = dwa_params.obstacle_margin;
    total_radius = robot_radius + margin;  % 总检测半径
    check_radius = ceil(total_radius);     % 需要检查的栅格范围
    
    % 将坐标转为地图栅格索引
    grid_x = round(next_x);
    grid_y = round(next_y);
    
    % 检查是否越界（只检查中心点是否在地图内）
    if next_x < 1 || next_x > MAX_X || next_y < 1 || next_y > MAX_Y
        is_collision = true;
        collision_pos = [next_x, next_y, theta];
        fprintf('\n[!!!] 检测到越界碰撞！位置: [%.2f, %.2f]\n', next_x, next_y);
    else
        % 检查机器人半径+安全裕度范围内的所有栅格
        for dx = -check_radius:check_radius
            for dy = -check_radius:check_radius
                % 计算该栅格中心到机器人中心的距离
                % 栅格中心相对于机器人中心的偏移
                cell_center_dist = sqrt(dx^2 + dy^2);
                
                % 如果栅格在机器人占用范围内（考虑栅格大小0.5）
                if cell_center_dist <= total_radius + 0.5
                    check_x = grid_x + dx;
                    check_y = grid_y + dy;
                    
                    % 确保在地图范围内
                    if check_x >= 1 && check_x <= MAX_X && check_y >= 1 && check_y <= MAX_Y
                        if MAP(check_y, check_x) == 1  % 注意: MAP(Y, X)
                            is_collision = true;
                            collision_pos = [next_x, next_y, theta];
                            fprintf('\n[!!!] 检测到障碍物碰撞！位置: [%.2f, %.2f], 碰撞栅格: [%d, %d]\n', ...
                                    next_x, next_y, check_x, check_y);
                            break;
                        end
                    end
                end
            end
            if is_collision, break; end
        end
    end
    
    % 如果碰撞，记录并退出
    if is_collision
        trajectory = [trajectory; collision_pos];
        velocity_history = [velocity_history; v, w];
        fprintf('机器人停止。仿真时间: %.2f s\n', simulation_time);
        break;
    end
    
    %% 5. 运动学更新（差分驱动模型）
    robot_pose(1) = next_x;
    robot_pose(2) = next_y;
    robot_pose(3) = robot_pose(3) + w * dt;
    
    % 角度归一化
    robot_pose(3) = atan2(sin(robot_pose(3)), cos(robot_pose(3)));
    
    % 记录轨迹和速度
    trajectory = [trajectory; robot_pose];
    velocity_history = [velocity_history; v, w];
    
    %% 6. 可视化（每5步更新一次）
    if mod(step_count, 5) == 0
        clf;
        hold on;
        
        % 设置坐标轴
        axis equal;
        xlim([1 MAX_X+1]);
        ylim([1 MAX_Y+1]);
        set(gca, 'xtick', 1:2:MAX_X+1, 'ytick', 1:2:MAX_Y+1);
        grid on;
        
        % 绘制障碍物（只绘制一个用于图例）
        % 注意：MAP(row, col) = MAP(Y, X)
        h_obs = [];
        for row = 1:MAX_Y
            for col = 1:MAX_X
                if MAP(row, col) == 1
                    h = fill([col, col+1, col+1, col], [row, row, row+1, row+1], [0.3 0.3 0.3]);
                    if isempty(h_obs)
                        h_obs = h;  % 保存第一个障碍物句柄用于图例
                    end
                end
            end
        end
        
        % 绘制起点和终点
        h_start = plot(start_pos(1) + 0.5, start_pos(2) + 0.5, 'g^', 'MarkerSize', 12, 'LineWidth', 2);
        h_goal = plot(goal_pos(1) + 0.5, goal_pos(2) + 0.5, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
        
        % 绘制A*全局路径（黑色虚线，理想参考路径）
        h_astar = plot(global_path(:, 1) + 0.5, global_path(:, 2) + 0.5, 'k--', 'LineWidth', 1.5);
        
        % 绘制DWA预测轨迹
        h_dwa = [];
        if ~isempty(best_traj)
            h_dwa = plot(best_traj(:, 1) + 0.5, best_traj(:, 2) + 0.5, 'm-', 'LineWidth', 1);
        end
        
        % 绘制DWA实际轨迹（蓝色粗实线，实际走过的路径）- 放在最后确保可见
        if size(trajectory, 1) > 1
            h_traj = plot(trajectory(:, 1) + 0.5, trajectory(:, 2) + 0.5, 'b-', 'LineWidth', 2.5);
        else
            h_traj = plot(trajectory(1, 1) + 0.5, trajectory(1, 2) + 0.5, 'bo', 'MarkerSize', 8);
        end
        
        % 绘制局部目标
        h_local = plot(local_goal(1) + 0.5, local_goal(2) + 0.5, 'c*', 'MarkerSize', 10);
        
        % 绘制机器人
        h_robot = plot_robot(robot_pose, dwa_params.robot_radius);
        
        % 绘制动态障碍物（红色圆圈+速度箭头）
        h_dyn = [];
        if ~isempty(dyn_obs)
            for i = 1:length(dyn_obs)
                % 绘制红色半透明圆圈
                h = rectangle('Position', [dyn_obs(i).pos(1) - dyn_obs(i).radius + 0.5, ...
                                       dyn_obs(i).pos(2) - dyn_obs(i).radius + 0.5, ...
                                       2*dyn_obs(i).radius, 2*dyn_obs(i).radius], ...
                          'Curvature', [1, 1], 'FaceColor', [1 0 0 0.3], 'EdgeColor', 'r', 'LineWidth', 2);
                if isempty(h_dyn)
                    h_dyn = h;
                end
                
                % 绘制速度箭头（显示运动方向）
                quiver(dyn_obs(i).pos(1) + 0.5, dyn_obs(i).pos(2) + 0.5, ...
                       dyn_obs(i).vel(1), dyn_obs(i).vel(2), 1.5, ...
                       'k', 'LineWidth', 2, 'MaxHeadSize', 0.8);
            end
        end
        
        title(sprintf('A* + DWA 仿真 | 时间: %.1f s | 距离目标: %.2f m', simulation_time, dist_to_goal));
        xlabel('X (m)');
        ylabel('Y (m)');
        
        % 创建图例（简化版本，避免句柄问题）
        try
            legend_handles = [h_obs, h_astar, h_start, h_goal, h_local, h_traj];
            legend_labels = {'障碍物', 'A*全局路径', '起点', '终点', '局部目标', 'DWA实际轨迹'};
            if ~isempty(h_dwa)
                legend_handles = [legend_handles, h_dwa];
                legend_labels = [legend_labels, {'DWA预测'}];
            end
            legend(legend_handles, legend_labels, 'Location', 'best');
        catch
            % 如果legend失败，跳过
        end
        
        drawnow limitrate;
    end
    
    %% 7. 更新时间
    simulation_time = simulation_time + dt;
    
    % 每1秒输出一次状态
    if mod(step_count, 10) == 0
        fprintf('时间: %.1f s | 位置: [%.2f, %.2f] | 速度: %.2f m/s | 距离: %.2f m\n', ...
            simulation_time, robot_pose(1), robot_pose(2), v, dist_to_goal);
    end
end

%% ========== 仿真结束 ==========
fprintf('\n========== 仿真结束 ==========\n');
fprintf('总仿真时间: %.2f 秒\n', simulation_time);
fprintf('总步数: %d\n', step_count);
fprintf('实际轨迹长度: %.2f 米\n', calculate_path_length(trajectory));

%% ========== 生成论文图片（机器人残影可视化） ==========
if generate_paper_figure
    fprintf('\n---------- 生成论文图片 ----------\n');
    
    % ===== 图1: 机器人残影可视化（主图） =====
    fig_paper = figure('Name', '论文图片 - 震荡与停滞可视化', 'Position', [100, 100, 1400, 500]);
    
    % --- 子图1: 机器人残影 ---
    subplot(1, 2, 1);
    hold on;
    axis equal;
    xlim([1 MAX_X+1]);
    ylim([1 MAX_Y+1]);
    set(gca, 'xtick', 1:2:MAX_X+1, 'ytick', 1:2:MAX_Y+1);
    grid on;
    set(gca, 'FontSize', 11);
    
    % 绘制障碍物
    for row = 1:MAX_Y
        for col = 1:MAX_X
            if MAP(row, col) == 1
                fill([col, col+1, col+1, col], [row, row, row+1, row+1], [0.3 0.3 0.3], 'EdgeColor', 'none');
            end
        end
    end
    
    % 绘制A*全局路径（黑色虚线）
    plot(global_path(:, 1) + 0.5, global_path(:, 2) + 0.5, 'k--', 'LineWidth', 2);
    
    % ===== 核心：绘制机器人残影 =====
    num_poses = size(trajectory, 1);
    
    for i = 1 : ghost_step_interval : num_poses
        x = trajectory(i, 1) + 0.5;
        y = trajectory(i, 2) + 0.5;
        th = trajectory(i, 3);
        
        % 获取当前速度
        if i > 1 && i-1 <= size(velocity_history, 1)
            current_v = velocity_history(i-1, 1);
        else
            current_v = 0.5;
        end
        
        % 根据速度选择颜色和透明度
        % max_speed = 0.5, stagnation_threshold = 0.45
        if current_v < 0.1
            % 极慢/停滞 -> 红色
            face_color = [1, 0.2, 0.2];
            alpha_val = 0.8;
            edge_color = 'r';
        elseif current_v < stagnation_threshold
            % 减速 -> 橙色（0.1 <= v < 0.45）
            face_color = [1, 0.6, 0.2];
            alpha_val = 0.6;
            edge_color = [0.8, 0.4, 0];
        else
            % 正常/全速 -> 蓝色（v >= 0.45）
            face_color = [0.2, 0.4, 0.8];
            alpha_val = 0.25;
            edge_color = 'b';
        end
        
        draw_robot_ghost(x, y, th, dwa_params.robot_radius, face_color, alpha_val, edge_color);
    end
    
    % 绘制轨迹线
    plot(trajectory(:, 1) + 0.5, trajectory(:, 2) + 0.5, 'k-', 'LineWidth', 1.2);
    
    % 绘制起点和终点
    plot(start_pos(1) + 0.5, start_pos(2) + 0.5, 'g^', 'MarkerSize', 14, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    plot(goal_pos(1) + 0.5, goal_pos(2) + 0.5, 'rp', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', 'r');
    
    % [关键] 标记碰撞点（如果发生碰撞）
    if is_collision && ~isempty(collision_pos)
        plot(collision_pos(1) + 0.5, collision_pos(2) + 0.5, 'rx', 'MarkerSize', 25, 'LineWidth', 4);
        text(collision_pos(1) + 0.8, collision_pos(2) + 0.5, 'Collision!', ...
             'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    % 图例
    h_legend = zeros(5, 1);
    h_legend(1) = plot(NaN, NaN, 'k--', 'LineWidth', 2);
    h_legend(2) = fill(NaN, NaN, [1, 0.2, 0.2], 'FaceAlpha', 0.7, 'EdgeColor', 'r');
    h_legend(3) = fill(NaN, NaN, [1, 0.6, 0.2], 'FaceAlpha', 0.5, 'EdgeColor', [0.8, 0.4, 0]);
    h_legend(4) = fill(NaN, NaN, [0.2, 0.4, 0.8], 'FaceAlpha', 0.25, 'EdgeColor', 'b');
    h_legend(5) = plot(NaN, NaN, 'k-', 'LineWidth', 1.2);
    
    % 根据是否碰撞调整图例
    if is_collision
        h_legend(6) = plot(NaN, NaN, 'rx', 'MarkerSize', 15, 'LineWidth', 3);
        legend(h_legend, {'A*路径', '停滞(v<0.1)', sprintf('减速(v<%.2f)', stagnation_threshold), '正常', '轨迹', '碰撞点'}, ...
               'Location', 'northeast', 'FontSize', 9);
    else
        legend(h_legend, {'A*路径', '停滞(v<0.1)', sprintf('减速(v<%.2f)', stagnation_threshold), '正常', '轨迹'}, ...
               'Location', 'northeast', 'FontSize', 9);
    end
    
    % 根据结果设置标题
    if is_collision
        title('(a) 传统DWA：碰撞失败案例', 'FontSize', 13, 'Color', 'r');
    else
        title('(a) 机器人运动轨迹与残影', 'FontSize', 13);
    end
    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);
    
    % --- 子图2: 速度-时间曲线 ---
    subplot(1, 2, 2);
    hold on;
    grid on;
    set(gca, 'FontSize', 11);
    
    time_vec = (0:size(velocity_history, 1)-1) * dt;
    
    % 绘制线速度
    plot(time_vec, velocity_history(:, 1), 'b-', 'LineWidth', 1.5);
    
    % 绘制角速度（转换为度/秒）
    plot(time_vec, velocity_history(:, 2) * 180 / pi, 'r-', 'LineWidth', 1.5);
    
    % 标记停滞区域（速度低于阈值）
    stagnation_mask = velocity_history(:, 1) < stagnation_threshold;
    if any(stagnation_mask)
        stag_times = time_vec(stagnation_mask);
        stag_v = velocity_history(stagnation_mask, 1);
        scatter(stag_times, stag_v, 20, 'r', 'filled', 'MarkerFaceAlpha', 0.5);
    end
    
    % 标记震荡区域（角速度频繁变化）
    w_diff = abs(diff(velocity_history(:, 2)));
    oscillation_mask = [false; w_diff > 0.3];  % 角速度变化大于0.3 rad/s
    if any(oscillation_mask)
        osc_times = time_vec(oscillation_mask);
        osc_w = velocity_history(oscillation_mask, 2) * 180 / pi;
        scatter(osc_times, osc_w, 25, 'm', 'filled', 'MarkerFaceAlpha', 0.5);
    end
    
    % 添加停滞阈值线
    yline(stagnation_threshold, 'r--', sprintf('停滞阈值=%.2f', stagnation_threshold), ...
          'LineWidth', 1, 'LabelHorizontalAlignment', 'left', 'FontSize', 9);
    
    xlabel('时间 (s)', 'FontSize', 11);
    ylabel('速度', 'FontSize', 11);
    legend({'线速度 v (m/s)', '角速度 ω (°/s)', '停滞点', '震荡点'}, ...
           'Location', 'best', 'FontSize', 9);
    title('(b) 速度-时间曲线', 'FontSize', 13);
    xlim([0, max(time_vec) + 1]);
    ylim([-100, 100]);  % 限制Y轴范围，角速度约±57°/s，留余量显示
    
    % 添加总标题
    sgtitle(sprintf('DWA局部规划分析 | 总时间: %.1fs | 路径: %.2fm', ...
            simulation_time, calculate_path_length(trajectory)), 'FontSize', 14, 'FontWeight', 'bold');
    
    % 保存图片
    saveas(fig_paper, 'paper_figure_oscillation.png');
    saveas(fig_paper, 'paper_figure_oscillation.fig');
    fprintf('论文图片已保存: paper_figure_oscillation.png/fig\n');
    
    % ===== 图2: 时间密度热力图（可选） =====
    fig_heatmap = figure('Name', '时间密度热力图', 'Position', [150, 150, 800, 700]);
    hold on;
    
    % 创建网格统计每个格子的停留时间
    time_density = zeros(MAX_Y, MAX_X);
    for i = 1:size(trajectory, 1)
        gx = max(1, min(MAX_X, floor(trajectory(i, 1))));
        gy = max(1, min(MAX_Y, floor(trajectory(i, 2))));
        time_density(gy, gx) = time_density(gy, gx) + dt;
    end
    
    % 绘制热力图
    imagesc([1.5, MAX_X+0.5], [1.5, MAX_Y+0.5], time_density);
    colormap(hot);
    colorbar('Label', '停留时间 (s)');
    
    % 叠加障碍物
    for row = 1:MAX_Y
        for col = 1:MAX_X
            if MAP(row, col) == 1
                fill([col, col+1, col+1, col], [row, row, row+1, row+1], ...
                     [0.2 0.2 0.2], 'EdgeColor', 'w', 'LineWidth', 0.5);
            end
        end
    end
    
    % 绘制轨迹
    plot(trajectory(:, 1) + 0.5, trajectory(:, 2) + 0.5, 'c-', 'LineWidth', 1.5);
    plot(start_pos(1) + 0.5, start_pos(2) + 0.5, 'g^', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    plot(goal_pos(1) + 0.5, goal_pos(2) + 0.5, 'wp', 'MarkerSize', 14, 'LineWidth', 2, 'MarkerFaceColor', 'w');
    
    axis equal;
    xlim([1 MAX_X+1]);
    ylim([1 MAX_Y+1]);
    set(gca, 'YDir', 'normal', 'FontSize', 11);
    grid on;
    title('时间密度热力图 - 亮色区域表示停滞', 'FontSize', 13);
    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);
    
    saveas(fig_heatmap, 'paper_figure_heatmap.png');
    saveas(fig_heatmap, 'paper_figure_heatmap.fig');
    fprintf('热力图已保存: paper_figure_heatmap.png/fig\n');
    
    % 统计震荡和停滞信息
    stagnation_time = sum(velocity_history(:, 1) < stagnation_threshold) * dt;
    oscillation_count = sum(abs(diff(velocity_history(:, 2))) > 0.3);
    fprintf('\n=== 震荡与停滞统计 ===\n');
    fprintf('停滞时间: %.2f s (占比 %.1f%%)\n', stagnation_time, stagnation_time/simulation_time*100);
    fprintf('震荡次数: %d 次\n', oscillation_count);
    fprintf('平均速度: %.3f m/s\n', mean(velocity_history(:, 1)));
end

%% 辅助函数

function h = plot_robot(pose, radius)
    % 绘制机器人（圆形+方向箭头）
    x = pose(1) + 0.5;
    y = pose(2) + 0.5;
    theta = pose(3);
    
    % 绘制圆形
    h = rectangle('Position', [x - radius, y - radius, 2*radius, 2*radius], ...
              'Curvature', [1, 1], 'FaceColor', [0 0 1 0.3], 'EdgeColor', 'b', 'LineWidth', 2);
    
    % 绘制方向箭头
    arrow_length = radius * 1.5;
    quiver(x, y, arrow_length * cos(theta), arrow_length * sin(theta), ...
           'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

function length = calculate_path_length(path)
    % 计算路径长度
    length = 0;
    for i = 1:(size(path, 1) - 1)
        length = length + sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    end
end

function draw_robot_ghost(x, y, theta, radius, face_color, alpha_val, edge_color)
    % 绘制机器人残影（矩形轮廓+方向指示）
    % 用于论文图片中展示震荡和停滞效果
    %
    % 输入:
    %   x, y       - 机器人中心位置
    %   theta      - 机器人朝向（弧度）
    %   radius     - 机器人半径
    %   face_color - 填充颜色 [R, G, B]
    %   alpha_val  - 透明度 (0-1)
    %   edge_color - 边框颜色
    
    % 机器人尺寸（矩形，长宽比1.5:1）
    robot_length = radius * 2;
    robot_width = radius * 1.2;
    
    % 计算矩形四个角点（相对于中心）
    corners = [
        -robot_length/2, -robot_width/2;
        robot_length/2, -robot_width/2;
        robot_length/2, robot_width/2;
        -robot_length/2, robot_width/2
    ];
    
    % 旋转矩阵
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % 旋转并平移
    rotated_corners = (R * corners')';
    rotated_corners(:, 1) = rotated_corners(:, 1) + x;
    rotated_corners(:, 2) = rotated_corners(:, 2) + y;
    
    % 绘制填充矩形
    fill(rotated_corners(:, 1), rotated_corners(:, 2), face_color, ...
         'FaceAlpha', alpha_val, 'EdgeColor', edge_color, 'LineWidth', 0.5);
    
    % 绘制方向指示（小三角形箭头）
    arrow_length = radius * 0.8;
    arrow_tip = [x + arrow_length * cos(theta), y + arrow_length * sin(theta)];
    arrow_base1 = [x + arrow_length * 0.3 * cos(theta + 2.5), y + arrow_length * 0.3 * sin(theta + 2.5)];
    arrow_base2 = [x + arrow_length * 0.3 * cos(theta - 2.5), y + arrow_length * 0.3 * sin(theta - 2.5)];
    
    fill([arrow_tip(1), arrow_base1(1), arrow_base2(1)], ...
         [arrow_tip(2), arrow_base1(2), arrow_base2(2)], ...
         edge_color, 'FaceAlpha', alpha_val + 0.2, 'EdgeColor', 'none');
end
