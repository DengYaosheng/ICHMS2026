clear; clc; close all;

%% === 1. 实验参数设置 ===
TIMESTEP = 0.1; 
TIME_END = 30; 
UBOUND = [10, 10, 10];
LBOUND = [-10, -10, -10];

N_P = 4; % 4个追捕者
N_E = 2; % 2个逃跑者

% 约束参数
D_MIN = 1.0; 
D_MAX = 4.0; 
V_MAX = 2.0; 

% 定义边界 (用于 MPT 绘图)
bound = Polyhedron('ub', UBOUND, 'lb', LBOUND); 

%% === 2. Q-Learning 训练阶段 (保持不变) ===
fprintf('=== Stage 1: Training Q-Learning Agent ===\n');

% 初始化 Q 表
Q_table = zeros(18, 6); 
Action_Space = [
     V_MAX, 0, 0; -V_MAX, 0, 0;
     0, V_MAX, 0;  0,-V_MAX, 0;
     0, 0, V_MAX;  0, 0,-V_MAX
];

% 简化的训练过程 (为了快速演示，实际论文中可用预训练好的数据)
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

%% === 3. 仿真验证阶段 (MPT 风格绘图) ===
disp('=== Stage 2: Simulation (Visual Style: MPT Voronoi) ===');

% 初始化位置
pos_p = (rand(N_P, 3) .* (UBOUND - LBOUND) + LBOUND) * 0.6;
pos_e = (rand(N_E, 3) .* (UBOUND - LBOUND) + LBOUND) * 0.3;

% 创建图形界面 (带暂停按钮)
figure('Name', 'RL Baseline (MPT Style)', 'Color', 'w');
hPauseBtn = uicontrol('Style', 'togglebutton', 'String', 'Pause', ...
                      'Position', [20 20 100 30], 'Callback', 'drawnow');

% 设置视角和坐标轴
view(3); grid on; axis equal;
axis([LBOUND(1) UBOUND(1) LBOUND(2) UBOUND(2) LBOUND(3) UBOUND(3)]);
xlabel('X'); ylabel('Y'); zlabel('Z');

for t = 0:TIMESTEP:TIME_END
    
    % --- 0. 暂停逻辑 ---
    while ishghandle(hPauseBtn) && get(hPauseBtn, 'Value') == 1
        set(hPauseBtn, 'String', 'RESUME', 'BackgroundColor', [1 0.6 0.6]);
        drawnow;
    end
    if ishghandle(hPauseBtn)
        set(hPauseBtn, 'String', 'Pause', 'BackgroundColor', [0.94 0.94 0.94]);
    end
    
    current_violations = 0;
    
    % --- 1. RL 决策与移动 (逻辑不变) ---
    for i = 1:N_P
        % 寻找最近 Evader
        dists = zeros(1, N_E);
        for j = 1:N_E, dists(j) = norm(pos_p(i,:) - pos_e(j,:)); end
        [min_dist, target_idx] = min(dists);
        
        % RL 决策
        state_idx = get_state_index(pos_p(i,:), pos_e(target_idx, :), D_MIN, D_MAX);
        [~, act_idx] = max(Q_table(state_idx, :));
        vel = Action_Space(act_idx, :);
        
        % 更新位置
        pos_p(i,:) = pos_p(i,:) + vel * TIMESTEP;
        
        % 统计违规
        if min_dist < D_MIN || min_dist > D_MAX
            current_violations = current_violations + 1;
        end
    end
    
    % Evader 移动
    for j = 1:N_E
        vel_e = [-pos_e(j,2), pos_e(j,1), 0] * 0.5 + (rand(1,3)-0.5)*0.5; 
        vel_e = vel_e / norm(vel_e) * (V_MAX * 0.5); 
        pos_e(j,:) = pos_e(j,:) + vel_e * TIMESTEP;
    end
    
    % --- 2. 绘图 (核心修改：使用 MPT Voronoi) ---
    cla; hold on;
    
    % [Style Match] 计算 Voronoi 仅用于绘图，不用于控制
    % 这一步让 RL 的图看起来和你的主实验风格完全一致
    pos_all = [pos_e; pos_p];
    try
        [~, p_cells] = mpt_voronoi(pos_all', 'bound', bound);
        % 绘制透明多面体 (Style Consistency)
        plot(p_cells, 'alpha', 0.1); 
    catch
        % 防止共线导致 MPT 报错
    end
    
    % 绘制 Evaders (红色菱形，为了和主实验一致)
    % 假设主实验 Evader 是红色，Pursuer 是黑色
    scatter3(pos_e(:,1), pos_e(:,2), pos_e(:,3), 80, 'r', 'filled', 'd');
    
    % 绘制距离约束球 (Visual Consistency)
    [sx, sy, sz] = sphere(10);
    for j = 1:N_E
        % 最小距离球 (红)
        surf(sx*D_MIN + pos_e(j,1), sy*D_MIN + pos_e(j,2), sz*D_MIN + pos_e(j,3), ...
            'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
        % 最大距离球 (绿)
        surf(sx*D_MAX + pos_e(j,1), sy*D_MAX + pos_e(j,2), sz*D_MAX + pos_e(j,3), ...
            'FaceColor', 'g', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    end
    
    % 绘制 Pursuers (黑色菱形，违规变色)
    for i = 1:N_P
        d = min(pdist2(pos_p(i,:), pos_e));
        if d < D_MIN || d > D_MAX
            % 违规时可以用醒目的颜色 (例如洋红色 Magenta)，或者保持黑色但加粗
            col = 'm'; 
            marker_size = 80;
        else
            col = 'k'; % 正常时为黑色 (与主实验 Pursuer 一致)
            marker_size = 50;
        end
        scatter3(pos_p(i,1), pos_p(i,2), pos_p(i,3), marker_size, col, 'filled', 'd');
    end
    
    axis equal; grid on;
    axis([LBOUND(1) UBOUND(1) LBOUND(2) UBOUND(2) LBOUND(3) UBOUND(3)]);
    view(3);
    
    drawnow;
end

%% === 辅助函数 (必须在文件末尾) ===
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