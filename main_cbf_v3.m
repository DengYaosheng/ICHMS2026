clear;
clc;
close all;

%% === Parameter Settings ===
timestep = 0.1; 
timeend = 200;
ubound = [10, 10, 10];
lbound = [-10, -10, -10];

n_p = 4; 
n_e = 2; 

% === CBF Safety & Connectivity Parameters ===
D_MIN = 1.0;   
D_MAX = 4.0;   
GAMMA = 5.0;   

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

% === GUI Setup (Pause Button) ===
hFig = figure(1);
set(hFig, 'Name', 'Voronoi Pursuit-Evasion with CBF', 'NumberTitle', 'off');

% 创建一个切换按钮 (Toggle Button)
% Position: [left, bottom, width, height]
hPauseBtn = uicontrol('Style', 'togglebutton', ...
                      'String', 'Pause', ...
                      'FontSize', 12, ...
                      'Position', [20 20 100 30], ...
                      'Callback', 'drawnow'); % Callback ensures immediate update

pos_pursuers = zeros(n_p, 3);

%% === Main Loop ===
for t=0:timestep:timeend
    
    % --- 0. Pause Logic (Check Button State) ---
    % 如果按钮被按下 (Value == 1)，进入等待循环
    while ishghandle(hPauseBtn) && get(hPauseBtn, 'Value') == 1
        set(hPauseBtn, 'String', 'RESUME'); % 更改文字提示
        set(hPauseBtn, 'BackgroundColor', [1 0.6 0.6]); % 变红提醒
        title(['PAUSED at T = ' num2str(t) '. Capture your snapshot now.']);
        drawnow; % 保持界面响应，等待用户点击按钮恢复
    end
    
    % 恢复按钮样式
    if ishghandle(hPauseBtn)
        set(hPauseBtn, 'String', 'Pause');
        set(hPauseBtn, 'BackgroundColor', [0.94 0.94 0.94]);
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

    % Get Pursuer positions
    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    % --- 2. MPT Voronoi Calculation ---
    pos_all = [pos_evaders; pos_pursuers];
    [v, p] = mpt_voronoi(pos_all', 'bound', bound); 

    % --- 3. Update Evader and Calculate Velocity ---
    for i = 1:length(p)
        if i <= n_e_alive
            e_idx = evaders_alive_index(i);
            evaders{e_idx} = evaders{e_idx}.setVoronoiCell(p(i));
            evaders{e_idx} = evaders{e_idx}.calculateVelocity();
        else
            p_idx = i - n_e_alive;
            pursuers{p_idx} = pursuers{p_idx}.setVoronoiCell(p(i));
        end
    end

    adjacencyMatrix = getVoronoiAdjacency(p);
    
    % --- 4. Pursuer Decision & CBF Correction ---
    for i = 1:length(pursuers)
        dists = pdist2(pursuers{i}.position, pos_evaders);
        [min_dist, nearestEvaderIdx] = min(dists);
        target_idx_global = evaders_alive_index(nearestEvaderIdx);
        
        pursuers{i}.target = evaders{target_idx_global};
        
        if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        pursuers{i} = pursuers{i}.calculateVelocity();
        
        % === CBF Apply ===
        u_nom = pursuers{i}.velocity; 
        p_pursuer = pursuers{i}.position;
        p_evader_target = evaders{target_idx_global}.position;
        v_evader_target = evaders{target_idx_global}.velocity; 
        
        u_safe = solve_cbf_range_3d(u_nom, p_pursuer, p_evader_target, v_evader_target, D_MIN, D_MAX, GAMMA);
        pursuers{i}.velocity = u_safe;
    end

    % --- 5. Move Agents ---
    for i = 1:length(evaders)
        evaders{i} = evaders{i}.move(timestep);
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep);
    end

    % --- 6. Capture Check ---
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
             evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end

    disp(['Time Step: ', num2str(t)]);

    % --- 7. Visualization ---
    % [重要修改] 使用 cla 而不是 clf，以防止按钮被删除
    cla; 
    hold on
    view(3); grid on; axis equal;
    axis([lbound(1) ubound(1) lbound(2) ubound(2) lbound(3) ubound(3)]);
    
    % Plot Voronoi
    plot(p, 'alpha', 0.1); 
    
    % Plot Agents
    if ~isempty(pos_evaders)
        scatter3(pos_evaders(:,1), pos_evaders(:,2), pos_evaders(:,3), 50, 'r', 'filled', 'd');
        [sx, sy, sz] = sphere(15);
        for k=1:size(pos_evaders,1)
            % Min Dist Sphere (Red)
            surf(sx*D_MIN + pos_evaders(k,1), sy*D_MIN + pos_evaders(k,2), sz*D_MIN + pos_evaders(k,3), ...
                'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
            % Max Dist Sphere (Green)
            surf(sx*D_MAX + pos_evaders(k,1), sy*D_MAX + pos_evaders(k,2), sz*D_MAX + pos_evaders(k,3), ...
                'FaceColor', 'g', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
        end
    end
    scatter3(pos_pursuers(:,1), pos_pursuers(:,2), pos_pursuers(:,3), 50, 'k', 'filled', 'd');
    
    title(['T = ' num2str(t)]);
    hold off;
    drawnow;
end

%% === CBF 3D Range Solver Function ===
function u_safe = solve_cbf_range_3d(u_nom, p_p, p_e, v_e, d_min, d_max, gamma)
    % CBF Solver (Same as before)
    u_nom = u_nom(:); p_p = p_p(:); p_e = p_e(:); v_e = v_e(:);
    rel_p = p_p - p_e; dist_sq = sum(rel_p.^2);
    
    H = eye(3); f = -u_nom;
    
    h1 = dist_sq - d_min^2;
    A_min = -2 * rel_p'; b_min = gamma * h1 + 2 * rel_p' * v_e;
    
    h2 = d_max^2 - dist_sq;
    A_max = 2 * rel_p'; b_max = gamma * h2 + 2 * rel_p' * v_e;
    
    A = [A_min; A_max]; b = [b_min; b_max];
    
    v_max_limit = 5.0; 
    lb = [-v_max_limit; -v_max_limit; -v_max_limit];
    ub = [ v_max_limit;  v_max_limit;  v_max_limit];
    
    options = optimoptions('quadprog', 'Display', 'off');
    [u_safe, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);
    
    if exitflag < 0
        dist = sqrt(dist_sq); dir = rel_p / dist;
        if dist < d_min, u_safe = dir * v_max_limit;
        elseif dist > d_max, u_safe = -dir * v_max_limit;
        else, u_safe = [0;0;0]; end
    end
    u_safe = u_safe';
end