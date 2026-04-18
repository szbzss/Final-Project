%% 1. 画地图与环境初始化 (保持原格式)
m = 20; n = 20;   
start_node = [1, 1]; target_node = [20, 20];
G=[0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0; 
   0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 0 0 0 0 0; 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 0 0 0 0 0; 
   0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0; 
   0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0; 0 1 1 1 0 0 1 1 1 0 1 1 1 1 0 0 0 0 0 0; 
   0 1 1 1 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0; 
   0 0 0 0 0 0 0 1 1 0 1 1 1 1 0 0 0 0 0 0; 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0; 
   0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1 1 1 0; 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1 1 1 0; 
   1 1 1 1 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0; 1 1 1 1 0 0 1 1 0 0 1 1 1 0 0 0 0 0 0 0; 
   0 0 0 0 0 0 1 1 0 1 1 1 1 0 0 0 0 1 1 0; 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0; 
   0 0 1 1 0 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;];
G=rot90(G,3);

figure(1); hold on;
for i = 1:m
    for j = 1:n
        if G(i,j) == 1
            fill([i-1,i,i,i-1],[j-1,j-1,j,j],'k');
        else
            plot([i-1,i,i,i-1,i-1],[j-1,j-1,j,j,j-1],'k');
        end
    end
end
plot(start_node(1)-0.5, start_node(2)-0.5, 'ro', 'MarkerSize', 8);
plot(target_node(1)-0.5, target_node(2)-0.5, 'gx', 'MarkerSize', 8);

%% 2. A* 全局规划
open_list = [start_node(1), start_node(2), 0, norm(start_node-target_node), norm(start_node-target_node), 0, 0];
closed_list = []; path_found = false;
dirs = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
while ~isempty(open_list)
    [~, min_idx] = min(open_list(:, 5)); current = open_list(min_idx, :);
    if current(1) == target_node(1) && current(2) == target_node(2)
        closed_list = [closed_list; current]; path_found = true; break;
    end
    open_list(min_idx, :) = []; closed_list = [closed_list; current];
    for i = 1:8
        neighbor = current(1:2) + dirs(i, :);
        if neighbor(1)<1 || neighbor(1)>m || neighbor(2)<1 || neighbor(2)>n || G(neighbor(1), neighbor(2))==1, continue; end
        if any(closed_list(:,1)==neighbor(1) & closed_list(:,2)==neighbor(2)), continue; end
        g_n = current(3) + norm(dirs(i, :)); h_n = norm(neighbor - target_node);
        idx = find(open_list(:,1)==neighbor(1) & open_list(:,2)==neighbor(2));
        if isempty(idx), open_list = [open_list; neighbor, g_n, h_n, g_n+h_n, current(1), current(2)];
        elseif g_n < open_list(idx, 3), open_list(idx, 3:7) = [g_n, h_n, g_n+h_n, current(1), current(2)]; end
    end
end
curr = closed_list(end, :); global_path = [];
while curr(6) ~= 0
    global_path = [curr(1:2); global_path];
    parent_idx = find(closed_list(:,1)==curr(6) & closed_list(:,2)==curr(7), 1, 'last');
    curr = closed_list(parent_idx, :);
end
global_path = [start_node; global_path] - 0.5;

%% 3. DWA 局部规划 (借鉴 Python 逻辑并修正局部最优)
% 机器人运动参数
max_speed = 1.0; max_yawrate = 60.0 * pi / 180.0;
max_accel = 0.5; max_dyawrate = 60.0 * pi / 180.0;
v_reso = 0.05; yawrate_reso = 3.0 * pi / 180.0;
dt = 0.1; predict_time = 2.5;

% 评价增益
gain_goal = 2.0; gain_speed = 0.5; gain_obs = 1.2;

x = [start_node(1)-0.5; start_node(2)-0.5; pi/4; 0; 0]; 
[obs_r, obs_c] = find(G == 1);
ob = [obs_r - 0.5, obs_c - 0.5]; 
dwa_final_path = x(1:2)';

max_iter = 1000;
for iter = 1:max_iter
    if norm(x(1:2) - (target_node' - 0.5)) < 0.3, break; end
    Vs = [0, max_speed, -max_yawrate, max_yawrate];
    Vd = [x(4)-max_accel*dt, x(4)+max_accel*dt, x(5)-max_dyawrate*dt, x(5)+max_dyawrate*dt];
    dw = [max(Vs(1), Vd(1)), min(Vs(2), Vd(2)), max(Vs(3), Vd(3)), min(Vs(4), Vd(4))];
    dists = sqrt(sum((global_path - x(1:2)').^2, 2));
    [~, nearest_idx] = min(dists);
    l_goal = global_path(min(nearest_idx + 4, size(global_path, 1)), :);
    best_u = [0; 0]; min_cost = inf;
    for v = dw(1):v_reso:dw(2)
        for w = dw(3):yawrate_reso:dw(4)
            temp_x = x;
            for t = 0:dt:predict_time
                temp_x(1) = temp_x(1) + v * cos(temp_x(3)) * dt;
                temp_x(2) = temp_x(2) + v * sin(temp_x(3)) * dt;
                temp_x(3) = temp_x(3) + w * dt;
            end
            traj_end = temp_x(1:2)';
            cost_goal = gain_goal * norm(traj_end - l_goal);
            cost_speed = gain_speed * (max_speed - v);
            d_to_obs = min(sqrt(sum((ob - traj_end).^2, 2)));
            if d_to_obs < 0.4, cost_obs = inf; else, cost_obs = gain_obs * (1.0 / d_to_obs); end
            total_cost = cost_goal + cost_speed + cost_obs;
            if total_cost < min_cost, min_cost = total_cost; best_u = [v; w]; end
        end
    end
    x(3) = x(3) + best_u(2) * dt;
    x(1) = x(1) + best_u(1) * cos(x(3)) * dt;
    x(2) = x(2) + best_u(1) * sin(x(3)) * dt;
    x(4) = best_u(1); x(5) = best_u(2);
    dwa_final_path = [dwa_final_path; x(1:2)'];
end

%% 4. 绘制两种路径
% A* 路径：蓝色实线
plot(global_path(:, 1), global_path(:, 2), 'b', 'LineWidth', 2);
% DWA 路径：粉色实线
plot(dwa_final_path(:, 1), dwa_final_path(:, 2), 'm', 'LineWidth', 2.5);

title('A* (Blue) and DWA (Pink) Path Planning');
legend('Obstacles', 'Grid', 'Start', 'Target', 'A* Path', 'DWA Path');