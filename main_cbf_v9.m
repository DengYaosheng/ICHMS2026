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
VOL_CAPTURE_THRESHOLD = 5.0; % 如果 Voronoi 体积小于 5，判定捕获

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
set(hFig, 'Name', 'Voronoi Pursuit-Evasion with CBF', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

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
        title(['PAUSED at T = ' num2str(t) '. Capture snapshot now.']); drawnow; 
    end
    if ishghandle(hPauseBtn)
        set(hPauseBtn, 'String', 'Pause'); set(hPauseBtn, 'BackgroundColor', [0.94 0.94 0.94]);
    end

    % --- 1. Data Preparation (Filter Alive Evaders) ---
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
        disp('All evaders captured based on Volume or Collision!');
        break;
    end

    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    % --- 2. Voronoi Calculation ---
    pos_all = [pos_evaders; pos_pursuers];
    % 注意：mpt_voronoi 计算可能会比较耗时，确保安装了 MPT3
    [v, p] = mpt_voronoi(pos_all', 'bound', bound); 
    
    % [重要] Volume Check & Capture Logic
    % 倒序遍历，因为我们可能会标记 bird 为 dead，这会影响下一帧的索引，但不影响当前帧
    for k = n_e_alive:-1:1
        real_id = evaders_alive_index(k);
        
        % 计算体积
        vol = p(k).volume();
        history_vol(step_count, real_id) = vol;
        
        % === 捕获判定：体积小于阈值 ===
        if vol < VOL_CAPTURE_THRESHOLD
            fprintf('Bird %d CAPTURED! Volume (%.2f) < Threshold (%.2f) at T=%.2fs\n', ...
                real_id, vol, VOL_CAPTURE_THRESHOLD, t);
            
            % 标记为死亡 (Captured)
            evaders{real_id}.isDead = true;
            
            % 可选：记录捕获时的体积为 0，以便在图表中显示落地
            % history_vol(step_count:end, real_id) = 0; 
        end
    end
    
    % 如果这一帧刚刚有鸟被捕获，我们需要重新检查循环条件
    % 为了简单起见，我们允许它在这一帧继续显示，下一帧循环开始时 Data Prep 会把它过滤掉
    
    % --- 3. Update Evader Cells ---
    % 注意：如果上面刚刚标记了 isDead，这里的 p(k) 可能就不再准确对应下一帧的逻辑
    % 但在当前帧，p 数组还是完整的，所以我们只更新还活着的
    
    current_p_idx = 1;
    for k = 1:n_e_alive
        real_id = evaders_alive_index(k);
        % 只更新还没死的（虽然上面刚刚可能标记了死，但那是下一帧的事）
        if ~evaders{real_id}.isDead
             evaders{real_id} = evaders{real_id}.setVoronoiCell(p(k));
             evaders{real_id} = evaders{real_id}.calculateVelocity();
        end
    end
    
    % Update Pursuer Cells
    % p 数组中，Evader 之后就是 Pursuer
    for k = 1:n_p
        pursuers{k} = pursuers{k}.setVoronoiCell(p(n_e_alive + k));
    end

    adjacencyMatrix = getVoronoiAdjacency(p);
    
    % --- 4. Pursuer Decision & CBF ---
    for i = 1:length(pursuers)
        % 重新计算距离（只针对活着的鸟）
        % 如果所有鸟都刚死，这里可能会空，需要保护
        if isempty(pos_evaders)
            continue; 
        end
        
        dists = pdist2(pursuers{i}.position, pos_evaders);
        [min_dist, nearestEvaderIdx] = min(dists);
        history_dist(step_count, i) = min_dist; 
        
        target_idx_global = evaders_alive_index(nearestEvaderIdx);
        
        % 如果目标刚刚被判定捕获，追捕者应该换目标或停止
        % 简单处理：如果目标死了，追捕者这帧继续按惯性或原地不动
        if evaders{target_idx_global}.isDead
             pursuers{i}.velocity = [0, 0, 0];
             continue;
        end
        
        pursuers{i}.target = evaders{target_idx_global};
        
        if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        pursuers{i} = pursuers{i}.calculateVelocity();
        
        % CBF Apply
        u_nom = pursuers{i}.velocity; 
        p_pursuer = pursuers{i}.position;
        p_evader_target = evaders{target_idx_global}.position;
        v_evader_target = evaders{target_idx_global}.velocity; 
        
        u_safe = solve_cbf_range_3d(u_nom, p_pursuer, p_evader_target, v_evader_target, D_MIN, D_MAX, GAMMA);
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

    % --- 6. Collision Capture Check (Original Logic) ---
    % 保留原有的基于距离的捕获逻辑作为双重保险
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
             evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end

    % --- 7. Visualization ---
    cla; hold on;
    
    % A. Draw Voronoi Cells (Only for Alive Birds)
    if n_e_alive > 0
        % 筛选出活着的 bird 对应的 polyhedron
        % 注意：p 包含了这一帧所有刚开始活着的 bird
        % 我们只画那些现在还活着的
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
    
    % B. Draw Birds
    % 重新遍历画图，只画活着的
    for i=1:n_e
        if ~evaders{i}.isDead
            p_e = evaders{i}.position;
            scatter3(p_e(1), p_e(2), p_e(3), 80, 'r', 'filled', '^', 'MarkerEdgeColor', 'k');
            
            [sx, sy, sz] = sphere(12);
            surf(sx*D_MIN + p_e(1), sy*D_MIN + p_e(2), sz*D_MIN + p_e(3), ...
                'FaceColor', 'none', 'EdgeColor', 'r', 'EdgeAlpha', 0.3);
            surf(sx*D_MAX + p_e(1), sy*D_MAX + p_e(2), sz*D_MAX + p_e(3), ...
                'FaceColor', 'none', 'EdgeColor', 'g', 'EdgeAlpha', 0.3);
        end
    end
    
    % C. Draw UAVs
    for k = 1:n_p
        draw_drone(pursuers{k}.position, 0.5);
    end
    
    % D. Legend & Title
    h_legend_bird = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_legend_uav  = plot3(NaN, NaN, NaN, 'k+', 'LineWidth', 2, 'MarkerSize', 10);
    h_legend_cell = plot3(NaN, NaN, NaN, 's', 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.5 1], 'MarkerSize', 10);
    legend([h_legend_uav, h_legend_bird, h_legend_cell], {'UAV Swarm', 'Bird Target', 'Bird Voronoi Cell'}, 'Location', 'northeast', 'AutoUpdate', 'off');
    
    title(['UAV Pursuit Simulation | T = ' num2str(t, '%.1f') 's']);
    
    % Axes Settings
    axis([lbound(1) ubound(1) lbound(2) ubound(2) lbound(3) ubound(3)]);
    daspect([1 1 1]); pbaspect([1 1 1]);
    view(3); grid on; box on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    hold off;
    drawnow;
end

%% === 8. Post-Processing: Data Analysis ===
valid_idx = 1:step_count;
valid_time = history_time(valid_idx);
valid_dist = history_dist(valid_idx, :);
valid_vol  = history_vol(valid_idx, :);

% === Figure 2: Distance Analysis ===
figure('Name', 'Analysis 1: Safety Constraints', 'Position', [100, 100, 600, 400], 'Color', 'w');
hold on; grid on;
colors = lines(n_p);
for i = 1:n_p
    plot(valid_time, valid_dist(:, i), 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', ['UAV ' num2str(i)]);
end
yline(D_MIN, 'r--', 'LineWidth', 2, 'DisplayName', 'Min Safe Dist');
yline(D_MAX, 'g--', 'LineWidth', 2, 'DisplayName', 'Max Conn Dist');
xlabel('Time (s)'); ylabel('Distance to Target (m)');
title('Safety & Connectivity Verification');
legend('Location', 'bestoutside');

% === Figure 3: Volume Analysis ===
figure('Name', 'Analysis 2: Containment Performance', 'Position', [720, 100, 600, 400], 'Color', 'w');
hold on; grid on;
colors_e = autumn(n_e); 
for i = 1:n_e
    plot(valid_time, valid_vol(:, i), 'LineWidth', 2, 'Color', colors_e(i,:), ...
        'DisplayName', ['Bird ' num2str(i) ' Volume']);
end

% 画出阈值线
yline(VOL_CAPTURE_THRESHOLD, 'k--', 'LineWidth', 2, 'DisplayName', 'Capture Threshold (5.0)');

xlabel('Time (s)');
ylabel('Voronoi Cell Volume (m^3)');
title('Bird Containment Performance (Voronoi Volume)');
legend('Location', 'best');
subtitle('Bird captured when volume drops below 5.0');

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