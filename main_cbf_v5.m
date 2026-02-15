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
    cla; 
    hold on
% --- 7. Visualization ---
    % === 关键修改：强制锁定坐标轴范围和比例 ===
    % 1. 设置坐标轴范围
    xlim([lbound(1) ubound(1)]);
    ylim([lbound(2) ubound(2)]);
    zlim([lbound(3) ubound(3)]);
    
    % 2. 锁定数据比例 (DataAspectRatio)
    % [1 1 1] 表示 x, y, z 轴的一个单位长度在屏幕上显示的像素长度是一样的
    daspect([1 1 1]); 
    
    % 3. (可选) 锁定绘图框比例 (PlotBoxAspectRatio)
    % 确保绘图框是一个正方体盒子
    pbaspect([1 1 1]); 
    
    % 4. 固定视角 (防止视角乱动，可选)
    view(3); 
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    % === A. 绘制 Voronoi (修改颜色逻辑) ===
    
    % 逻辑：p 的顺序对应输入的 [pos_evaders; pos_pursuers]
    % 所以 p(1 : n_e_alive) 是 Bird 的格子
    % p(n_e_alive + 1 : end) 是 UAV 的格子
    
    % 1. (可选) 绘制 UAV 的背景格子 - 统一成极淡的灰色
    % 如果你完全不想看 UAV 的格子，可以把下面这行注释掉
%     if length(p) > n_e_alive
%         plot(p(n_e_alive+1:end), 'Color', [0.95 0.95 0.95], 'alpha', 0.05, ...
%             'EdgeColor', [0.8 0.8 0.8], 'EdgeAlpha', 0.2); 
%     end
    
    % 2. 重点绘制 Bird 的格子 - 统一成青色 (Cyan)
    if n_e_alive > 0
        % Color: 填充颜色, EdgeColor: 边框颜色
        plot(p(1:n_e_alive), 'Color', [0 1 1], 'alpha', 0.15, ...
            'EdgeColor', [0 0.5 1], 'EdgeAlpha', 0.5);
    end
    
    % === B. 绘制 Evaders (Birds) ===
    if ~isempty(pos_evaders)
        scatter3(pos_evaders(:,1), pos_evaders(:,2), pos_evaders(:,3), ...
            50, 'r', 'filled', '^', 'MarkerEdgeColor', 'k');
            
        % 绘制距离保护罩 (线框模式)
        [sx, sy, sz] = sphere(12);
        for k=1:size(pos_evaders,1)
            % Min Dist (红色警戒圈)
            surf(sx*D_MIN + pos_evaders(k,1), sy*D_MIN + pos_evaders(k,2), sz*D_MIN + pos_evaders(k,3), ...
                'FaceColor', 'none', 'EdgeColor', 'r', 'EdgeAlpha', 0.3);
            % Max Dist (绿色连接圈)
            surf(sx*D_MAX + pos_evaders(k,1), sy*D_MAX + pos_evaders(k,2), sz*D_MAX + pos_evaders(k,3), ...
                'FaceColor', 'none', 'EdgeColor', 'g', 'EdgeAlpha', 0.3);
        end
    end
    
    % === C. 绘制 Pursuers (Quadrotors) ===
    drone_size = 0.8; 
    for k = 1:size(pos_pursuers, 1)
        draw_drone(pos_pursuers(k, :), drone_size);
    end
    
    % === D. 添加 Legend ===
    h_legend_bird = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_legend_uav  = plot3(NaN, NaN, NaN, 'k+', 'LineWidth', 2, 'MarkerSize', 10);
    h_legend_cell = plot3(NaN, NaN, NaN, 's', 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.5 1], 'MarkerSize', 10);
    
    legend([h_legend_uav, h_legend_bird, h_legend_cell], ...
           {'UAV Swarm', 'Bird Target', 'Bird Voronoi Cell'}, ...
           'Location', 'northeast', 'AutoUpdate', 'off');
    
    title(['UAV Pursuit Simulation | T = ' num2str(t, '%.1f') 's']);
    hold off;
    drawnow;
end
function draw_drone(pos, r)
    % 绘制简易四旋翼无人机
    % pos: [x, y, z] 中心坐标
    % r: 机臂长度的一半 (尺寸)
    
    x = pos(1); y = pos(2); z = pos(3);
    
    % 1. 绘制机臂 (X 型) - 黑色粗线
    % Arm 1
    plot3([x-r, x+r], [y-r, y+r], [z, z], 'k-', 'LineWidth', 2);
    % Arm 2
    plot3([x-r, x+r], [y+r, y-r], [z, z], 'k-', 'LineWidth', 2);
    
    % 2. 绘制中心机身 - 黑色圆点
    plot3(x, y, z, 'k.', 'MarkerSize', 15);
    
    % 3. 绘制四个旋翼 (用蓝色圆圈表示)
    % 定义旋翼半径
    prop_r = r * 0.4; 
    theta = 0:0.5:2*pi;
    xc = prop_r * cos(theta);
    yc = prop_r * sin(theta);
    
    % 旋翼中心位置偏移
    offsets = [-r, -r; r, r; -r, r; r, -r];
    
    for i = 1:4
        % 计算每个旋翼的位置
        prop_x = x + offsets(i, 1) + xc;
        prop_y = y + offsets(i, 2) + yc;
        prop_z = z * ones(size(xc));
        
        % 画旋翼圈
        patch(prop_x, prop_y, prop_z, 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'b');
    end
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

