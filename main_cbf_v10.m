clear;
clc;
close all;

%% === Parameter Settings ===
timestep = 0.1; 
timeend = 40;
ubound = [10, 10, 10];
lbound = [-10, -10, -10];

n_p = 4; 
n_e = 2; 

% === CBF Safety & Connectivity Parameters ===
D_MIN = 1.0;   
D_MAX = 4.0;   
GAMMA = 5.0;   

% === Capture Threshold (Volume) ===
VOL_CAPTURE_THRESHOLD = 10.0; 

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
    
    capture_event_this_frame = false; % [Flag] 标记本帧是否发生捕获
    
    for k = n_e_alive:-1:1
        real_id = evaders_alive_index(k);
        vol = p(k).volume();
        history_vol(step_count, real_id) = vol;
        
        if vol < VOL_CAPTURE_THRESHOLD
            fprintf('Bird %d CAPTURED! Volume (%.2f) < Threshold. T=%.2fs\n', real_id, vol, t);
            evaders{real_id}.isDead = true;
            capture_event_this_frame = true; % 标记事件
        end
    end
    
    % --- 3. Update Cells (Skip if capture happened to allow topology reset) ---
    % 如果这一帧发生了捕获，跳过更新和计算，让下一帧的数据准备阶段把死鸟移除
    if capture_event_this_frame
        disp('Capture event detected. Skipping control update for stability...');
        % 保持当前位置不动，或者继续惯性飞行(这里选不动)
        pause(0.01); 
        continue; 
    end

    % Update Evaders
    for k = 1:n_e_alive
        real_id = evaders_alive_index(k);
        evaders{real_id} = evaders{real_id}.setVoronoiCell(p(k));
        evaders{real_id} = evaders{real_id}.calculateVelocity();
    end
    
    % Update Pursuers
    for k = 1:n_p
        pursuers{k} = pursuers{k}.setVoronoiCell(p(n_e_alive + k));
    end
    
    adjacencyMatrix = getVoronoiAdjacency(p);

    % --- 4. Pursuer Decision (Global Allocation) & CBF ---
    if isempty(pos_evaders)
        continue;
    end
    
    % Cost Matrix Calculation
    cost_matrix = pdist2(pos_pursuers, pos_evaders);
    num_repeats = ceil(n_p / n_e_alive);
    expanded_cost = repmat(cost_matrix, 1, num_repeats);
    if size(expanded_cost, 2) > n_p
        expanded_cost = expanded_cost(:, 1:n_p);
    end
    
    assignments = matchpairs(expanded_cost, 1000); 
    
    for i = 1:n_p
        row_idx = find(assignments(:,1) == i);
        if isempty(row_idx)
            [~, target_local_idx] = min(cost_matrix(i,:));
        else
            col_idx = assignments(row_idx, 2);
            target_local_idx = mod(col_idx - 1, n_e_alive) + 1;
        end
        
        target_idx_global = evaders_alive_index(target_local_idx);
        
        % [Double Check] 如果目标已死，停止追击
        if evaders{target_idx_global}.isDead
             pursuers{i}.velocity = [0, 0, 0];
             continue;
        end
        
        dist_to_target = norm(pursuers{i}.position - evaders{target_idx_global}.position);
        history_dist(step_count, i) = dist_to_target;
        
        pursuers{i}.target = evaders{target_idx_global};
        
        if adjacencyMatrix(i + n_e_alive, target_local_idx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        % === [关键修复] Try-Catch 保护 ===
        try
            pursuers{i} = pursuers{i}.calculateVelocity();
        catch ME
            % 如果 calculateVelocity 内部报错（例如维度不一致），则紧急悬停
            % warning(['Pursuer ' num2str(i) ' calculation failed: ' ME.message '. Hovering.']);
            pursuers{i}.velocity = [0, 0, 0];
            % 跳过后续 CBF，直接进入下一循环
            continue; 
        end
        
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
    
    % --- 6. Collision Capture (Backup) ---
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
        if ~isempty(pos_evaders) && ~pursuers{k}.target.isDead
             t_pos = pursuers{k}.target.position;
             u_pos = pursuers{k}.position;
             plot3([u_pos(1), t_pos(1)], [u_pos(2), t_pos(2)], [u_pos(3), t_pos(3)], 'k:', 'LineWidth', 0.5);
        end
    end
    
    % Legend & Setup
    h_legend_bird = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_legend_uav  = plot3(NaN, NaN, NaN, 'k+', 'LineWidth', 2, 'MarkerSize', 10);
    h_legend_cell = plot3(NaN, NaN, NaN, 's', 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.5 1], 'MarkerSize', 10);
    legend([h_legend_uav, h_legend_bird, h_legend_cell], {'UAV Swarm', 'Bird Target', 'Bird Voronoi Cell'}, 'Location', 'northeast', 'AutoUpdate', 'off');
    
    title(['UAV Pursuit Simulation | T = ' num2str(t, '%.1f') 's']);
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

figure( 'Position', [100, 100, 600, 400], 'Color', 'w');
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