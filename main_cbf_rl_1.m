clear; clc; close all;

%% === 1. Experimental Settings ===
TIMESTEP = 0.1; 
TIME_END = 30; 
UBOUND = [10, 10, 10];
LBOUND = [-10, -10, -10];

N_P = 4; % Number of Pursuers
N_E = 2; % Number of Evaders

% Constraints
D_MIN = 1.0; 
D_MAX = 4.0; 
V_MAX = 2.0; 

% Boundary for MPT
bound = Polyhedron('ub', UBOUND, 'lb', LBOUND); 

%% === 2. Q-Learning Training (Simplified) ===
fprintf('=== Stage 1: Training Q-Learning Agent ===\n');

% Init Q-Table (18 States x 6 Actions)
Q_table = zeros(18, 6); 
Action_Space = [
     V_MAX, 0, 0; -V_MAX, 0, 0;
     0, V_MAX, 0;  0,-V_MAX, 0;
     0, 0, V_MAX;  0, 0,-V_MAX
];

% Quick Training Loop
TRAIN_EPISODES = 500;
ALPHA = 0.1; GAMMA = 0.9; EPSILON = 0.1;

wb = waitbar(0, 'Training RL Agent...');
for epi = 1:TRAIN_EPISODES
    p_train = [0, 0, 0];
    e_train = (rand(1,3)-0.5) * 10;
    for step = 1:50 
        state_idx = get_state_index(p_train, e_train, D_MIN, D_MAX);
        if rand < EPSILON, act_idx = randi(6);
        else, [~, act_idx] = max(Q_table(state_idx, :)); end
        
        vel = Action_Space(act_idx, :);
        p_next = p_train + vel * TIMESTEP;
        
        dist = norm(p_next - e_train);
        if dist < D_MIN, reward = -100;
        elseif dist > D_MAX, reward = -1;
        else, reward = 10; end
        
        state_next_idx = get_state_index(p_next, e_train, D_MIN, D_MAX);
        q_predict = Q_table(state_idx, act_idx);
        q_target = reward + GAMMA * max(Q_table(state_next_idx, :));
        Q_table(state_idx, act_idx) = q_predict + ALPHA * (q_target - q_predict);
        
        p_train = p_next;
        e_train = e_train + (rand(1,3)-0.5)*TIMESTEP; 
    end
    waitbar(epi/TRAIN_EPISODES, wb);
end
close(wb);

%% === 3. Simulation & Visualization (Unified Style) ===
disp('=== Stage 2: Simulation with Unified Visualization ===');

% Init Positions
pos_p = (rand(N_P, 3) .* (UBOUND - LBOUND) + LBOUND) * 0.6;
pos_e = (rand(N_E, 3) .* (UBOUND - LBOUND) + LBOUND) * 0.5;

% Data Recording Setup
num_steps = ceil(TIME_END / TIMESTEP) + 1;
history_dist = zeros(num_steps, N_P); 
history_vol  = nan(num_steps, N_E); 
history_time = zeros(num_steps, 1);
step_count = 0;

% GUI Setup
hFig = figure('Name', 'RL Baseline Simulation', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hPauseBtn = uicontrol('Style', 'togglebutton', 'String', 'Pause', ...
                      'FontSize', 12, 'Position', [20 20 100 30], 'Callback', 'drawnow');

for t = 0:TIMESTEP:TIME_END
    step_count = step_count + 1;
    history_time(step_count) = t;

    % --- Pause Logic ---
    while ishghandle(hPauseBtn) && get(hPauseBtn, 'Value') == 1
        set(hPauseBtn, 'String', 'RESUME'); set(hPauseBtn, 'BackgroundColor', [1 0.6 0.6]);
        drawnow;
    end
    if ishghandle(hPauseBtn)
        set(hPauseBtn, 'String', 'Pause'); set(hPauseBtn, 'BackgroundColor', [0.94 0.94 0.94]);
    end
    
    % --- RL Logic (Decisions) ---
    for i = 1:N_P
        % Find Nearest Evader
        dists = zeros(1, N_E);
        for j = 1:N_E, dists(j) = norm(pos_p(i,:) - pos_e(j,:)); end
        [min_dist, target_idx] = min(dists);
        
        % Record Distance
        history_dist(step_count, i) = min_dist;

        % Choose Action
        state_idx = get_state_index(pos_p(i,:), pos_e(target_idx, :), D_MIN, D_MAX);
        [~, act_idx] = max(Q_table(state_idx, :));
        vel = Action_Space(act_idx, :);
        
        % Update Position
        pos_p(i,:) = pos_p(i,:) + vel * TIMESTEP;
    end
    
    % --- Evader Logic (Simple Movement) ---
    for j = 1:N_E
        vel_e = [-pos_e(j,2), pos_e(j,1), 0] * 0.5 + (rand(1,3)-0.5)*0.5; 
        vel_e = vel_e / norm(vel_e) * (V_MAX * 0.5); 
        pos_e(j,:) = pos_e(j,:) + vel_e * TIMESTEP;
    end
    
    % --- MPT Voronoi Calculation (For Visualization & Volume) ---
    pos_all = [pos_e; pos_p];
    try
        [~, p_cells] = mpt_voronoi(pos_all', 'bound', bound);
        
        % Record Volume
        if length(p_cells) >= N_E
            for k = 1:N_E
                history_vol(step_count, k) = p_cells(k).volume();
            end
        end
    catch
        p_cells = []; % Handle potential errors
    end
    
    % --- Visualization (Strictly Matching Previous Style) ---
    cla; hold on;
    
    % A. Draw Voronoi Cells (Cyan for Birds, Gray for UAVs)
    if ~isempty(p_cells)
        % UAV Cells (Gray Background)
%         if length(p_cells) > N_E
%             plot(p_cells(N_E+1:end), 'Color', [0.95 0.95 0.95], 'alpha', 0.05, ...
%                 'EdgeColor', [0.8 0.8 0.8], 'EdgeAlpha', 0.1); 
%         end
        % Bird Cells (Cyan Highlight)
        if length(p_cells) >= N_E
            plot(p_cells(1:N_E), 'Color', [0 1 1], 'alpha', 0.15, ...
                'EdgeColor', [0 0.5 1], 'EdgeAlpha', 0.5);
        end
    end
    
    % B. Draw Evaders (Birds) - Red Triangles + Spheres
    scatter3(pos_e(:,1), pos_e(:,2), pos_e(:,3), 80, 'r', 'filled', '^', 'MarkerEdgeColor', 'k');
    [sx, sy, sz] = sphere(12);
    for j = 1:N_E
        surf(sx*D_MIN + pos_e(j,1), sy*D_MIN + pos_e(j,2), sz*D_MIN + pos_e(j,3), ...
            'FaceColor', 'none', 'EdgeColor', 'r', 'EdgeAlpha', 0.3);
        surf(sx*D_MAX + pos_e(j,1), sy*D_MAX + pos_e(j,2), sz*D_MAX + pos_e(j,3), ...
            'FaceColor', 'none', 'EdgeColor', 'g', 'EdgeAlpha', 0.3);
    end
    
    % C. Draw Pursuers (UAVs) - Using draw_drone
    drone_size = 0.5;
    for k = 1:N_P
        draw_drone(pos_p(k, :), drone_size);
    end
    
    % D. Legend
    h_legend_bird = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_legend_uav  = plot3(NaN, NaN, NaN, 'k+', 'LineWidth', 2, 'MarkerSize', 10);
    h_legend_cell = plot3(NaN, NaN, NaN, 's', 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.5 1], 'MarkerSize', 10);
    legend([h_legend_uav, h_legend_bird, h_legend_cell], ...
           {'UAV Swarm', 'Bird Target', 'Bird Voronoi Cell'}, ...
           'Location', 'northeast', 'AutoUpdate', 'off');
       
%     title(['RL Baseline Simulation | T = ' num2str(t, '%.1f') 's']);
    
    % E. Axis Locking (Critical for Style Match)
    axis([LBOUND(1) UBOUND(1) LBOUND(2) UBOUND(2) LBOUND(3) UBOUND(3)]);
    daspect([1 1 1]); pbaspect([1 1 1]);
    view(3); grid on; box on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    hold off;
    drawnow;
end

%% === 4. Post-Processing: Data Analysis (Same as Formal Code) ===
valid_idx = 1:step_count;
valid_time = history_time(valid_idx);
valid_dist = history_dist(valid_idx, :);
valid_vol  = history_vol(valid_idx, :);

% Figure 2: Distance Analysis
figure('Name', 'RL Baseline: Safety Constraints', 'Position', [100, 100, 600, 400], 'Color', 'w');
hold on; grid on;
colors = lines(N_P);
for i = 1:N_P
    plot(valid_time, valid_dist(:, i), 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', ['UAV ' num2str(i)]);
end
yline(D_MIN, 'r--', 'LineWidth', 2, 'DisplayName', 'Min Safe Dist');
yline(D_MAX, 'g--', 'LineWidth', 2, 'DisplayName', 'Max Conn Dist');
xlabel('Time (s)'); ylabel('Distance to Target (m)');
title('RL Baseline: Safety & Connectivity');
legend('Location', 'bestoutside');
ylim([0, max(D_MAX + 2, max(valid_dist(:)))]);

% Figure 3: Volume Analysis
figure('Name', 'RL Baseline: Containment Performance', 'Position', [720, 100, 600, 400], 'Color', 'w');
hold on; grid on;
colors_e = autumn(N_E);
for i = 1:N_E
    plot(valid_time, valid_vol(:, i), 'LineWidth', 2, 'Color', colors_e(i,:), ...
        'DisplayName', ['Bird ' num2str(i) ' Volume']);
end
xlabel('Time (s)'); ylabel('Voronoi Cell Volume (m^3)');
title('RL Baseline: Containment Performance');
legend('Location', 'best');

%% === Helper Functions ===
function idx = get_state_index(p_pos, e_pos, d_min, d_max)
    rel = e_pos - p_pos;
    dist = norm(rel);
    if dist < d_min, d_state = 1;
    elseif dist > d_max, d_state = 3;
    else, d_state = 2; end
    
    [~, max_dim] = max(abs(rel));
    val = rel(max_dim);
    if max_dim == 1, if val > 0, a_state = 1; else, a_state = 2; end
    elseif max_dim == 2, if val > 0, a_state = 3; else, a_state = 4; end
    else, if val > 0, a_state = 5; else, a_state = 6; end
    end
    idx = (d_state - 1) * 6 + a_state;
end

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