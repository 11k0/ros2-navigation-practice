function [v, w, best_traj] = Adaptive_DWA(robot_pose, local_goal, MAP, params, dyn_obs)
% Adaptive_DWA - 自适应动态窗口法局部路径规划器
%
% 输入:
%   robot_pose  - 机器人位姿 [x, y, theta] (theta单位:弧度)
%   local_goal  - 局部目标点 [x, y] (来自Lookahead)
%   MAP         - 地图矩阵 (0=可通行, 1=障碍物)
%   params      - DWA参数结构体
%   dyn_obs     - 动态障碍物信息 (可选)
%       .pos    - 位置 [x, y]
%       .vel    - 速度 [vx, vy]
%       .radius - 半径
%
% 输出:
%   v           - 线速度 [vx, vy] (全向移动) 或 v (差分驱动)
%   w           - 角速度
%   best_traj   - 最优轨迹 [N×3] 矩阵 [x, y, theta]
%
% 特点:
%   1. 自适应评价函数权重
%   2. 动态障碍物避让
%   3. 距离场加速碰撞检测
%
% 作者: A*+DWA融合项目
% 日期: 2026-01-15

%% 参数设置
if nargin < 4
    params = get_default_params();
end

if nargin < 5
    dyn_obs = [];
end

% 提取参数
max_speed = params.max_speed;           % 最大线速度 (m/s)
min_speed = params.min_speed;           % 最小线速度
max_yaw_rate = params.max_yaw_rate;     % 最大角速度 (rad/s)
max_accel = params.max_accel;           % 最大线加速度 (m/s^2)
max_dyaw_rate = params.max_dyaw_rate;   % 最大角加速度 (rad/s^2)
v_resolution = params.v_resolution;     % 速度分辨率
yaw_rate_resolution = params.yaw_rate_resolution; % 角速度分辨率
dt = params.dt;                         % 时间步长
predict_time = params.predict_time;     % 预测时间
robot_radius = params.robot_radius;     % 机器人半径
obstacle_margin = params.obstacle_margin; % 障碍物安全距离

% 评价函数权重（自适应）
heading_weight = params.heading_weight;
dist_weight = params.dist_weight;
velocity_weight = params.velocity_weight;

%% 提取当前状态
x = robot_pose(1);
y = robot_pose(2);
theta = robot_pose(3);
current_v = params.current_v;  % 当前速度
current_w = params.current_w;  % 当前角速度

%% 计算动态窗口
% 根据加速度限制计算速度范围
v_min = max(min_speed, current_v - max_accel * dt);
v_max = min(max_speed, current_v + max_accel * dt);
w_min = max(-max_yaw_rate, current_w - max_dyaw_rate * dt);
w_max = min(max_yaw_rate, current_w + max_dyaw_rate * dt);

%% 生成速度采样
v_samples = v_min:v_resolution:v_max;
w_samples = w_min:yaw_rate_resolution:w_max;

if isempty(v_samples)
    v_samples = current_v;
end
if isempty(w_samples)
    w_samples = current_w;
end

%% 预生成膨胀地图（用于视线检测，只生成一次）
if isfield(params, 'use_adaptive') && params.use_adaptive == true
    [MAX_Y_map, MAX_X_map] = size(MAP);
    MAP_inflated_los = MAP;
    inflate_radius = 2;  % 膨胀半径增大到2格（5×5范围）
    for ii = 1:MAX_Y_map
        for jj = 1:MAX_X_map
            if MAP(ii,jj) == 1
                % 膨胀范围：周围2格
                for di = -inflate_radius:inflate_radius
                    for dj = -inflate_radius:inflate_radius
                        ni = ii + di;
                        nj = jj + dj;
                        if ni >= 1 && ni <= MAX_Y_map && nj >= 1 && nj <= MAX_X_map
                            MAP_inflated_los(ni, nj) = 1;
                        end
                    end
                end
            end
        end
    end
else
    MAP_inflated_los = MAP;  % 传统DWA不使用膨胀地图
end

%% 评估所有速度组合
best_score = -inf;
best_v = 0;
best_w = 0;
best_traj = [];
collision_count = 0;
total_count = 0;

for v_candidate = v_samples
    for w_candidate = w_samples
        total_count = total_count + 1;
        
        % 生成预测轨迹
        traj = generate_trajectory(x, y, theta, v_candidate, w_candidate, dt, predict_time);
        
        % 碰撞检测
        if check_collision(traj, MAP, robot_radius, obstacle_margin, dyn_obs)
            collision_count = collision_count + 1;
            continue;  % 跳过碰撞轨迹
        end
        
        % 计算评价函数
        % 1. 航向得分（朝向目标）
        end_pose = traj(end, :);
        angle_to_goal = atan2(local_goal(2) - end_pose(2), local_goal(1) - end_pose(1));
        heading_error = abs(angle_diff(angle_to_goal, end_pose(3)));
        heading_score = (pi - heading_error) / pi;  % 归一化到[0,1]
        
        % 2. 障碍物距离得分（远离障碍物）
        % 计算轨迹上所有点到最近障碍物的最小距离
        min_obs_dist = inf;
        [MAX_Y_map, MAX_X_map] = size(MAP);
        for ti = 1:size(traj, 1)
            tx = traj(ti, 1);
            ty = traj(ti, 2);
            % 检查周围3格范围内的障碍物
            for dx = -3:3
                for dy = -3:3
                    gx = round(tx) + dx;
                    gy = round(ty) + dy;
                    if gx >= 1 && gx <= MAX_X_map && gy >= 1 && gy <= MAX_Y_map
                        if MAP(gy, gx) == 1
                            obs_dist = sqrt((tx - gx)^2 + (ty - gy)^2);
                            min_obs_dist = min(min_obs_dist, obs_dist);
                        end
                    end
                end
            end
        end
        % 归一化障碍物距离得分：距离越远得分越高（温和惩罚）
        safe_dist = 1.0;  % 安全距离阈值（适中）
        if min_obs_dist == inf
            dist_score = 1.0;  % 周围没有障碍物
        elseif min_obs_dist < robot_radius + obstacle_margin
            % 碰撞区域，惩罚
            dist_score = -2.0;
        elseif min_obs_dist < safe_dist
            % 警戒区域，线性渐变
            dist_score = (min_obs_dist - robot_radius - obstacle_margin) / (safe_dist - robot_radius - obstacle_margin);
        else
            % 距离足够安全
            dist_score = 1.0;
        end
        
        % 3. 速度得分（鼓励高速，极度惩罚停止 - 不许停车等待！）
        if v_candidate < 0.01
            velocity_score = -100.0;  % 极度惩罚停止（从-10增强到-100，强制绕行）
        elseif v_candidate < 0.1
            velocity_score = -5.0;  % 低速也惩罚
        else
            velocity_score = v_candidate / max_speed;
        end
        
        % 4. 动态障碍物评分（预测避障）- 检查整条轨迹 + 方向偏好
        dyn_score = 1.0;  % 默认满分
        if ~isempty(dyn_obs)
            for di = 1:length(dyn_obs)
                % 安全距离 = 机器人半径 + 障碍物半径 + 余量（大幅增大余量，提前绕行）
                safe_dist_dyn = robot_radius + dyn_obs(di).radius + 2.0;  % 余量从1.0增到2.0
                
                % 获取障碍物速度方向（用于计算绕行方向偏好）
                obs_vel = dyn_obs(di).vel;
                obs_speed = sqrt(obs_vel(1)^2 + obs_vel(2)^2);
                
                % 检查轨迹上每个点到动态障碍物的距离
                for ti = 1:size(traj, 1)
                    % 预测障碍物在该时刻的位置
                    t_elapsed = ti * dt;
                    obs_future_pos = dyn_obs(di).pos + obs_vel * t_elapsed;
                    
                    % 计算轨迹点到障碍物的距离
                    traj_point = traj(ti, 1:2);
                    dist_to_dyn = sqrt((traj_point(1) - obs_future_pos(1))^2 + (traj_point(2) - obs_future_pos(2))^2);
                    
                    if dist_to_dyn < safe_dist_dyn
                        % 太近了，强烈惩罚
                        dyn_score = -10.0;
                        break;
                    else
                        % 距离越远越好（归一化到0-1）
                        score_i = min((dist_to_dyn - safe_dist_dyn) / 3.0, 1.0);
                        
                        % 方向偏好：向障碍物来的方向绕行（即障碍物速度的反方向）
                        % 障碍物向下移动(vel_y<0) → 机器人向上绕行(rel_y>0) → 奖励正
                        % 障碍物向上移动(vel_y>0) → 机器人向下绕行(rel_y<0) → 奖励正
                        % 公式：dir_bonus = -vel_y * rel_y > 0 时奖励
                        if obs_speed > 0.01
                            rel_pos = traj_point - dyn_obs(di).pos;
                            % 直接用速度Y分量和相对位置Y分量的乘积
                            % vel_y<0且rel_y>0 → 乘积<0 → 取负后>0 → 奖励
                            % vel_y>0且rel_y<0 → 乘积<0 → 取负后>0 → 奖励
                            dir_bonus = -obs_vel(2) * rel_pos(2) * 0.5;  % 增强方向奖励
                            score_i = score_i + dir_bonus;
                            
                            % 强制方向偏好：如果机器人选择了错误的绕行方向，直接拒绝该轨迹
                            % 障碍物向下移动时，机器人不能向下绕行
                            if obs_vel(2) < -0.01 && rel_pos(2) < -0.5
                                score_i = -100.0;  % 直接拒绝错误方向的轨迹
                            elseif obs_vel(2) > 0.01 && rel_pos(2) > 0.5
                                score_i = -100.0;  % 直接拒绝错误方向的轨迹
                            end
                        end
                        
                        dyn_score = min(dyn_score, score_i);
                    end
                end
                if dyn_score < 0
                    break;  % 已经检测到碰撞，跳出外层循环
                end
            end
        end
        
        % 5. 自适应权重调整
        % 检查是否开启自适应模式
        if isfield(params, 'use_adaptive') && params.use_adaptive == true
            % === 改进DWA：基于视线检测的自适应权重 ===
            % 论文公式(6): η = η_min + (η_max - η_min) * (1 - flag)
            % flag=1(阻挡): η=η_min, flag=0(无阻挡): η=η_max
            
            % 检测机器人到局部目标的视线是否被遮挡（使用预生成的膨胀地图）
            line_blocked = check_line_of_sight(x, y, local_goal(1), local_goal(2), MAP_inflated_los);
            flag = double(line_blocked);  % 1=阻挡, 0=无阻挡
            
            % 定义权重范围（强化切角惩罚）
            eta_min = 0.01;  % 阻挡时航向权重降到1%，几乎完全不走直线
            eta_max = 1.0;   % 无阻挡时航向权重正常
            
            % 应用公式(6)
            eta = eta_min + (eta_max - eta_min) * (1 - flag);
            
            % 自适应航向权重
            adaptive_heading_weight = heading_weight * eta;
            
            % 障碍物权重：始终保持较高的障碍物权重，防止靠近墙面
            if line_blocked
                adaptive_dist_weight = dist_weight * 6.0;  % 阻挡时极度怕撞墙
                adaptive_velocity_weight = velocity_weight * 0.1;  % 阻挡时大幅减速
            else
                % 通畅模式下也必须保持对墙的高度敬畏（关键修复）
                adaptive_dist_weight = dist_weight * 4.0;  % 从2.0提升到4.0
                adaptive_velocity_weight = velocity_weight * 0.6;  % 稍微减速
            end
            
            % 额外检查：如果距离最近障碍物很近，适度增大dist_weight
            if min_obs_dist < 0.8 && min_obs_dist ~= inf
                % 近墙区域：适度增强避障权重
                adaptive_dist_weight = adaptive_dist_weight * 2.0;
                adaptive_heading_weight = adaptive_heading_weight * 0.5;
            end
            
            % 接近目标时进一步调整
            dist_to_final_goal = sqrt((x - local_goal(1))^2 + (y - local_goal(2))^2);
            if dist_to_final_goal < 2.0
                adaptive_heading_weight = adaptive_heading_weight * 1.3;
                adaptive_velocity_weight = adaptive_velocity_weight * 0.5;
            end
        else
            % === 传统DWA：固定权重 ===
            adaptive_heading_weight = heading_weight;
            adaptive_dist_weight = dist_weight;
            adaptive_velocity_weight = velocity_weight;
        end
        
        % 综合得分（加入动态障碍物评分）
        dyn_weight = 2.0;  % 动态障碍物权重
        total_score = adaptive_heading_weight * heading_score + ...
                      adaptive_dist_weight * dist_score + ...
                      adaptive_velocity_weight * velocity_score + ...
                      dyn_weight * dyn_score;
        
        % 更新最优解
        if total_score > best_score
            best_score = total_score;
            best_v = v_candidate;
            best_w = w_candidate;
            best_traj = traj;
        end
    end
end

%% 输出结果
v = best_v;
w = best_w;

% 调试：如果速度接近0，输出信息
if best_v < 0.05
    fprintf('[DWA调试] best_v=0! 位置: [%.2f, %.2f], 朝向: %.1f°\n', x, y, theta*180/pi);
    fprintf('[DWA调试] 碰撞统计: %d/%d 轨迹被拒绝\n', collision_count, total_count);
    fprintf('[DWA调试] 局部目标: [%.2f, %.2f], 距离: %.2f\n', local_goal(1), local_goal(2), ...
        sqrt((x-local_goal(1))^2 + (y-local_goal(2))^2));
    % 检查周围障碍物
    for dx = -2:2
        for dy = -2:2
            gx = round(x) + dx;
            gy = round(y) + dy;
            if gx >= 1 && gx <= size(MAP,2) && gy >= 1 && gy <= size(MAP,1)
                if MAP(gy, gx) == 1
                    fprintf('[DWA调试] 障碍物: [%d, %d]\n', gx, gy);
                end
            end
        end
    end
end

% 如果没有找到可行轨迹，停止
if isempty(best_traj)
    v = 0;
    w = 0;
    best_traj = [x, y, theta];
    
    % 调试信息：检查为什么所有轨迹都失败
    fprintf('[DWA调试] 位置: [%.2f, %.2f], 朝向: %.1f°\n', x, y, theta*180/pi);
    fprintf('[DWA调试] 检查周围障碍物...\n');
    for dx = -2:2
        for dy = -2:2
            gx = round(x) + dx;
            gy = round(y) + dy;
            if gx >= 1 && gx <= size(MAP,2) && gy >= 1 && gy <= size(MAP,1)
                if MAP(gy, gx) == 1
                    fprintf('[DWA调试] 障碍物在: [%d, %d]\n', gx, gy);
                end
            end
        end
    end
    fprintf('[DWA调试] 局部目标: [%.2f, %.2f]\n', local_goal(1), local_goal(2));
    warning('DWA: 未找到可行轨迹，机器人停止');
end

end

%% 辅助函数

function traj = generate_trajectory(x, y, theta, v, w, dt, predict_time)
    % 生成预测轨迹
    n_steps = ceil(predict_time / dt);
    traj = zeros(n_steps, 3);
    
    for i = 1:n_steps
        traj(i, :) = [x, y, theta];
        
        % 运动学更新（差分驱动模型）
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + w * dt;
        
        % 角度归一化
        theta = atan2(sin(theta), cos(theta));
    end
end

function collision = check_collision(traj, MAP, robot_radius, margin, dyn_obs)
    % 碰撞检测（改进版：考虑机器人半径）
    collision = false;
    
    MAX_X = size(MAP, 2);
    MAX_Y = size(MAP, 1);
    
    % 总检测半径 = 机器人半径 + 安全裕度
    total_radius = robot_radius + margin;
    % 需要检查的栅格范围（至少检查1格）
    check_radius = max(1, ceil(total_radius));
    
    for i = 1:size(traj, 1)
        x = traj(i, 1);
        y = traj(i, 2);
        
        % 检查地图边界（只检查中心点是否在地图内）
        % 放宽边界检测，允许机器人靠近边缘
        if x < 1 || x > MAX_X || y < 1 || y > MAX_Y
            collision = true;
            return;
        end
        
        % 检查静态障碍物（检查机器人占据的所有栅格）
        grid_x = round(x);
        grid_y = round(y);
        
        % 检查机器人周围的栅格
        for dx = -check_radius:check_radius
            for dy = -check_radius:check_radius
                % 计算栅格中心到机器人中心的距离
                cell_dist = sqrt(dx^2 + dy^2);
                
                % 如果栅格在机器人占用范围内（考虑栅格大小0.5）
                % 栅格边缘到机器人中心的最近距离 = cell_dist - 0.5
                if cell_dist <= total_radius + 0.5
                    check_x = grid_x + dx;
                    check_y = grid_y + dy;
                    
                    % 确保在地图范围内
                    if check_x >= 1 && check_x <= MAX_X && check_y >= 1 && check_y <= MAX_Y
                        if MAP(check_y, check_x) == 1
                            collision = true;
                            return;
                        end
                    end
                end
            end
        end
        
        % 检查动态障碍物
        if ~isempty(dyn_obs)
            for j = 1:length(dyn_obs)
                obs_pos = dyn_obs(j).pos;
                obs_radius = dyn_obs(j).radius;
                
                dist = sqrt((x - obs_pos(1))^2 + (y - obs_pos(2))^2);
                if dist < (robot_radius + obs_radius + margin)
                    collision = true;
                    return;
                end
            end
        end
    end
end

function diff = angle_diff(angle1, angle2)
    % 计算两个角度的差值（归一化到[-pi, pi]）
    diff = angle1 - angle2;
    diff = atan2(sin(diff), cos(diff));
end

function blocked = check_line_of_sight(x1, y1, x2, y2, MAP)
    % 检测从(x1,y1)到(x2,y2)的"安全通道"是否被障碍物遮挡
    % 使用"带宽度的射线检测"(Fat Raycast)方法
    % 不仅检测中心线，还检测线周围一定范围内是否有障碍物
    %
    % 输入:
    %   x1, y1 - 起点坐标（机器人位置）
    %   x2, y2 - 终点坐标（局部目标）
    %   MAP    - 地图矩阵 (0=可通行, 1=障碍物)
    %
    % 输出:
    %   blocked - true表示视线被遮挡，false表示视线通畅
    
    blocked = false;
    
    % 安全通道宽度 = 机器人半径 + 安全余量（增大以避免切角）
    safe_radius = 1.0;  % 增大到1.0m，让机器人更早感知角落
    check_range = ceil(safe_radius);  % 需要检查的栅格范围
    
    % 计算线段长度
    dist = sqrt((x2 - x1)^2 + (y2 - y1)^2);
    
    if dist < 0.1
        return;  % 距离太近，不检测
    end
    
    % 采样步长（每0.3米检测一次）
    step_size = 0.3;
    n_samples = ceil(dist / step_size);
    
    [MAX_Y, MAX_X] = size(MAP);
    
    for i = 1:n_samples
        % 计算采样点
        t = i / n_samples;
        sample_x = x1 + t * (x2 - x1);
        sample_y = y1 + t * (y2 - y1);
        
        % 转换为栅格坐标
        grid_x = round(sample_x);
        grid_y = round(sample_y);
        
        % 边界检查
        if grid_x < 1 || grid_x > MAX_X || grid_y < 1 || grid_y > MAX_Y
            blocked = true;
            return;
        end
        
        % === 带宽度的检测：检查采样点周围是否有障碍物 ===
        for dx = -check_range:check_range
            for dy = -check_range:check_range
                % 计算该栅格到采样点的距离
                cell_dist = sqrt(dx^2 + dy^2);
                
                % 如果在安全通道范围内
                if cell_dist <= safe_radius + 0.5
                    check_x = grid_x + dx;
                    check_y = grid_y + dy;
                    
                    % 边界检查
                    if check_x >= 1 && check_x <= MAX_X && check_y >= 1 && check_y <= MAX_Y
                        if MAP(check_y, check_x) == 1
                            blocked = true;
                            return;
                        end
                    end
                end
            end
        end
    end
end

function params = get_default_params()
    % 默认DWA参数（与Main_AStar_DWA_Simulation.m保持一致）
    params.max_speed = 0.5;              % m/s
    params.min_speed = 0.0;
    params.max_yaw_rate = 1.0;           % rad/s (约57度/秒)
    params.max_accel = 2.0;              % m/s^2
    params.max_dyaw_rate = 1.5;          % rad/s^2
    params.v_resolution = 0.05;          % m/s
    params.yaw_rate_resolution = 0.2;    % rad/s
    params.dt = 0.1;                     % s
    params.predict_time = 1.0;           % s
    params.robot_radius = 0.3;           % m
    params.obstacle_margin = 0.1;        % m
    
    % 评价函数权重
    params.heading_weight = 4.0;
    params.dist_weight = 4.0;
    params.velocity_weight = 0.05;
    
    % 当前状态
    params.current_v = 0;
    params.current_w = 0;
    
    % 自适应模式开关
    params.use_adaptive = false;
end
