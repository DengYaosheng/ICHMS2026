clear;
clc;
close all;

%% === Parameter Settings ===
timestep = 0.1; 
timeend = 30;
ubound = [10, 10, 10];
lbound = [-10, -10, -10];

n_p = 4; 
n_e = 2; 

% === CBF Safety & Connectivity Parameters ===
D_MIN = 1.0;   
D_MAX = 4.0;   
GAMMA = 5.0;   

% === Capture Threshold (Volume) ===
VOL_CAPTURE_THRESHOLD = 5.0; 

pursuers = cell(1, n_p);
evaders = cell(1, n_e);

% Initialize agents
for i=1:n_p
    pos = rand(1,3) .* (ubound - lbound) + lbound;
    pursuers{i} = Pursuer(pos);
end

for i=1:n_e
    pos = rand(1,3) .* (ubound - lbound) + lbound;
    evaders{i} = Evader(pos);
end

bound = Polyhedron('ub', ubound, 'lb', lbound); 

% === Data Recording Setup ===
num_steps = ceil(timeend / timestep) + 1;
history_dist = zeros(num_steps, n_p); 
history_vol  = nan(num_steps, n_e); 
history_time = zeros(num_steps, 1);
step_count = 0;

% === GUI Setup ===
hFig = figure(1);
set(hFig, 'Name', 'Voronoi Pursuit (Strategy: 3 vs 1)', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

hPauseBtn = uicontrol('Style', 'togglebutton', 'String', 'Pause', ...
                      'FontSize', 12, 'Position', [20 20 100 30], 'Callback', 'drawnow');

pos_pursuers = zeros(n_p, 3);

%% === Main Loop ===
for t=0:timestep:timeend
    step_count = step_count + 1;
    history_time(step_count) = t;
    
    % --- 0. Pause Logic ---
    while ishghandle(hPauseBtn) && get(hPauseBtn, 'Value') == 1
        set(hPauseBtn, 'String', 'RESUME'); set(hPauseBtn, 'BackgroundColor', [1 0.6 0.6]); 
%         title(['PAUSED at T = ' num2str(t) '. Capture snapshot now.']);
        drawnow; 
    end
    if ishghandle(hPauseBtn)
        set(hPauseBtn, 'String', 'Pause'); set(hPauseBtn, 'BackgroundColor', [0.94 0.94 0.94]);
    end

    % --- 1. Data Preparation ---
    n_e_alive = 0;
    evaders_alive_index = [];
    pos_evaders = [];
    
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            n_e_alive = n_e_alive + 1;
            evaders_alive_index = [evaders_alive_index, i];
            pos_evaders = [pos_evaders; evaders{i}.position];
        end
    end
    
    if n_e_alive == 0
        disp('All evaders captured!');
        break;
    end

    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    % --- 2. Voronoi Calculation & Capture Check ---
    pos_all = [pos_evaders; pos_pursuers];
    [v, p] = mpt_voronoi(pos_all', 'bound', bound); 
    
    capture_event = false;
    for k = n_e_alive:-1:1
        real_id = evaders_alive_index(k);
        vol = p(k).volume();
        history_vol(step_count, real_id) = vol;
        if vol < VOL_CAPTURE_THRESHOLD
            fprintf('Bird %d CAPTURED! Volume (%.2f) < Threshold. T=%.2fs\n', real_id, vol, t);
            evaders{real_id}.isDead = true;
            capture_event = true;
        end
    end
    
    if capture_event
        pause(0.01); continue; 
    end

    % --- 3. Update Cells ---
    for k = 1:n_e_alive
        real_id = evaders_alive_index(k);
        if ~evaders{real_id}.isDead
             evaders{real_id} = evaders{real_id}.setVoronoiCell(p(k));
             evaders{real_id} = evaders{real_id}.calculateVelocity();
        end
    end
    for k = 1:n_p
        pursuers{k} = pursuers{k}.setVoronoiCell(p(n_e_alive + k));
    end
    
    adjacencyMatrix = getVoronoiAdjacency(p);

    % --- 4. Pursuer Decision (Strategy: 3 vs 1 then Swarm) ---
    if isempty(pos_evaders)
        continue;
    end
    
    % === [策略定义] ===
    % UAV 1, 2, 3 -> 优先抓 Bird 1
    % UAV 4       -> 优先抓 Bird 2
    preferred_targets = [1, 1, 1, 2]; 
    
    for i = 1:n_p
        target_real_id = preferred_targets(i);
        
        % [逻辑判断] 优先目标还活着吗？
        if evaders{target_real_id}.isDead
            % 如果死了，切换到另一个 (假设只有两个目标：1和2)
            % 简单的切换逻辑：3 - 当前ID (即 3-1=2, 3-2=1)
            target_real_id = 3 - target_real_id;
            
            % 如果另一个也死了，那就停下
            if evaders{target_real_id}.isDead
                pursuers{i}.velocity = [0, 0, 0];
                continue;
            end
        end
        
        % === 执行追捕逻辑 ===
        pursuers{i}.target = evaders{target_real_id};
        
        % 记录距离
        dist_to_target = norm(pursuers{i}.position - evaders{target_real_id}.position);
        history_dist(step_count, i) = dist_to_target;
        
        % 查找 Voronoi 邻接关系
        % 需要找到 target 在当前 p 数组中的索引
        voronoi_target_idx = find(evaders_alive_index == target_real_id);
        pursuer_voronoi_idx = n_e_alive + i;
        
        if ~isempty(voronoi_target_idx) && adjacencyMatrix(pursuer_voronoi_idx, voronoi_target_idx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        % 计算控制
        try
            pursuers{i} = pursuers{i}.calculateVelocity();
        catch
            pursuers{i}.velocity = [0,0,0]; continue;
        end
        
        % CBF 修正
        u_safe = solve_cbf_range_3d(pursuers{i}.velocity, ...
            pursuers{i}.position, pursuers{i}.target.position, pursuers{i}.target.velocity, ...
            D_MIN, D_MAX, GAMMA);
        pursuers{i}.velocity = u_safe;
    end

    % --- 5. Move Agents ---
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
            evaders{i} = evaders{i}.move(timestep);
        end
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep);
    end
    
    % --- 6. Collision Capture ---
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
             evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end

    % --- 7. Visualization ---
    cla; hold on;
    
    % Draw Voronoi
    if n_e_alive > 0
        valid_p_indices = [];
        for k=1:n_e_alive
            if ~evaders{evaders_alive_index(k)}.isDead
                valid_p_indices = [valid_p_indices, k];
            end
        end
        if ~isempty(valid_p_indices)
            plot(p(valid_p_indices), 'Color', [0 1 1], 'alpha', 0.15, 'EdgeColor', [0 0.5 1], 'EdgeAlpha', 0.5);
        end
    end
    
    % Draw Birds
    for i=1:n_e
        if ~evaders{i}.isDead
            p_e = evaders{i}.position;
            scatter3(p_e(1), p_e(2), p_e(3), 80, 'r', 'filled', '^', 'MarkerEdgeColor', 'k');
            [sx, sy, sz] = sphere(12);
            surf(sx*D_MIN + p_e(1), sy*D_MIN + p_e(2), sz*D_MIN + p_e(3), 'FaceColor', 'none', 'EdgeColor', 'r', 'EdgeAlpha', 0.3);
            surf(sx*D_MAX + p_e(1), sy*D_MAX + p_e(2), sz*D_MAX + p_e(3), 'FaceColor', 'none', 'EdgeColor', 'g', 'EdgeAlpha', 0.3);
        end
    end
    
    % Draw UAVs
    for k = 1:n_p
        draw_drone(pursuers{k}.position, 0.5);
        % 画出锁定连线，方便观察 3 vs 1 战术
        if ~pursuers{k}.target.isDead
             t_pos = pursuers{k}.target.position;
             u_pos = pursuers{k}.position;
             % UAV 4 用不同颜色的线，方便区分
             if k == 4
                 l_col = 'k:'; % 洋红色虚线
             else
                 l_col = 'k:';  % 黑色点线
             end
             plot3([u_pos(1), t_pos(1)], [u_pos(2), t_pos(2)], [u_pos(3), t_pos(3)], l_col, 'LineWidth', 0.5);
        end
    end
    
    % Legend & Setup
    h_legend_bird = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_legend_uav  = plot3(NaN, NaN, NaN, 'k+', 'LineWidth', 2, 'MarkerSize', 10);
    h_legend_cell = plot3(NaN, NaN, NaN, 's', 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.5 1], 'MarkerSize', 10);
    legend([h_legend_uav, h_legend_bird, h_legend_cell], {'UAV Swarm', 'Bird Target', 'Bird Voronoi Cell'}, 'Location', 'northeast', 'AutoUpdate', 'off');
    
%     title(['UAV Pursuit (Strategy: 3 vs 1) | T = ' num2str(t, '%.1f') 's']);
    axis([lbound(1) ubound(1) lbound(2) ubound(2) lbound(3) ubound(3)]);
    daspect([1 1 1]); pbaspect([1 1 1]); view(3); grid on; box on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    hold off; drawnow;
end

%% === 8. Post-Processing ===
valid_idx = 1:step_count;
valid_time = history_time(valid_idx);
valid_dist = history_dist(valid_idx, :);
valid_vol  = history_vol(valid_idx, :);

figure('Name', 'Analysis 1: Safety Constraints', 'Position', [100, 100, 600, 400], 'Color', 'w');
hold on; grid on; colors = lines(n_p);
for i = 1:n_p
    plot(valid_time, valid_dist(:, i), 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', ['UAV ' num2str(i)]);
end
yline(D_MIN, 'r--', 'LineWidth', 2); yline(D_MAX, 'g--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Distance to Target (m)'); title('Safety & Connectivity Verification'); legend;

figure('Name', 'Analysis 2: Containment Performance', 'Position', [720, 100, 600, 400], 'Color', 'w');
hold on; grid on; colors_e = autumn(n_e); 
for i = 1:n_e
    plot(valid_time, valid_vol(:, i), 'LineWidth', 2, 'Color', colors_e(i,:), 'DisplayName', ['Bird ' num2str(i) ' Volume']);
end
yline(VOL_CAPTURE_THRESHOLD, 'k--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Voronoi Cell Volume (m^3)'); title('Bird Containment Performance'); legend;

%% === Helper Functions ===
function draw_drone(pos, r)
    x = pos(1); y = pos(2); z = pos(3);
    plot3([x-r, x+r], [y-r, y+r], [z, z], 'k-', 'LineWidth', 2);
    plot3([x-r, x+r], [y+r, y-r], [z, z], 'k-', 'LineWidth', 2);
    plot3(x, y, z, 'k.', 'MarkerSize', 15);
    prop_r = r * 0.4; theta = 0:0.5:2*pi; xc = prop_r * cos(theta); yc = prop_r * sin(theta);
    offsets = [-r, -r; r, r; -r, r; r, -r];
    for i = 1:4
        prop_x = x + offsets(i, 1) + xc; prop_y = y + offsets(i, 2) + yc; prop_z = z * ones(size(xc));
        patch(prop_x, prop_y, prop_z, 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'b');
    end
end

function u_safe = solve_cbf_range_3d(u_nom, p_p, p_e, v_e, d_min, d_max, gamma)
    u_nom = u_nom(:); p_p = p_p(:); p_e = p_e(:); v_e = v_e(:);
    rel_p = p_p - p_e; dist_sq = sum(rel_p.^2);
    H = eye(3); f = -u_nom;
    h1 = dist_sq - d_min^2; A_min = -2 * rel_p'; b_min = gamma * h1 + 2 * rel_p' * v_e;
    h2 = d_max^2 - dist_sq; A_max = 2 * rel_p'; b_max = gamma * h2 + 2 * rel_p' * v_e;
    A = [A_min; A_max]; b = [b_min; b_max];
    v_max = 5.0; lb = [-v_max; -v_max; -v_max]; ub = [v_max; v_max; v_max];
    options = optimoptions('quadprog', 'Display', 'off');
    [u_safe, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);
    if exitflag < 0
        dist = sqrt(dist_sq); dir = rel_p / dist;
        if dist < d_min, u_safe = dir * v_max;
        elseif dist > d_max, u_safe = -dir * v_max;
        else, u_safe = [0;0;0]; end
    end
    u_safe = u_safe';
end