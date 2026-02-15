clear;
clc;
close all;

%% === 参数设置 ===
timestep = 0.1; 
timeend = 200;
ubound = [10, 10, 10];
lbound = [-10, -10, -10];

n_p = 4; 
n_e = 2; 

% === CBF 安全约束参数 ===
D_MIN = 2;   % 最小安全距离 (Constraint: dist >= 0.5)
GAMMA = 10.0;  % CBF收敛系数 (越大越允许激进靠近)

pursuers = cell(1, n_p);
evaders = cell(1, n_e);

% initialize agents (假设构造函数不变)
for i=1:n_p
    pos = rand(1,3) .* (ubound - lbound) + lbound;
    pursuers{i} = Pursuer(pos);
end

for i=1:n_e
    pos = rand(1,3) .* (ubound - lbound) + lbound;
    evaders{i} = Evader(pos);
end

bound = Polyhedron('ub', ubound, 'lb', lbound); 

figure(1)

pos_pursuers = zeros(n_p, 3);

%% === 主循环 ===
%% === 主循环 (修复版) ===
for t=0:timestep:timeend
    
    % --- 1. 准备数据 ---
    n_e_alive = 0;
    evaders_alive_index = [];
    pos_evaders = [];
    
    % 仅收集位置，先不计算速度（因为还没算 Voronoi）
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

    % 获取 Pursuer 位置
    for i = 1:length(pursuers)
        pos_pursuers(i,:) = pursuers{i}.position;
    end

    % --- 2. MPT Voronoi 计算 (核心几何) ---
    pos_all = [pos_evaders; pos_pursuers];
    % 计算 Voronoi 图
    [v, p] = mpt_voronoi(pos_all', 'bound', bound); 

    % --- 3. 更新 Evader 并计算其速度 (供下一步 CBF 使用) ---
    % 注意：必须先 setVoronoiCell，才能 calculateVelocity
    for i = 1:length(p)
        if i <= n_e_alive
            % 这是 Evader
            e_idx = evaders_alive_index(i);
            evaders{e_idx} = evaders{e_idx}.setVoronoiCell(p(i));
            % 现在 Cell 已更新，可以安全计算速度了
            evaders{e_idx} = evaders{e_idx}.calculateVelocity();
        else
            % 这是 Pursuer，先更新 Cell，暂不计算速度
            p_idx = i - n_e_alive;
            pursuers{p_idx} = pursuers{p_idx}.setVoronoiCell(p(i));
        end
    end

    adjacencyMatrix = getVoronoiAdjacency(p);
    
    % --- 4. Pursuer 决策与 CBF 修正 ---
    for i = 1:length(pursuers)
        % A. 寻找最近的 Evader
        dists = pdist2(pursuers{i}.position, pos_evaders);
        [min_dist, nearestEvaderIdx] = min(dists);
        target_idx_global = evaders_alive_index(nearestEvaderIdx);
        
        pursuers{i}.target = evaders{target_idx_global};
        
        % 判断邻接
        if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        % B. 计算 Pursuer 的标称速度 (Nominal Velocity)
        % 这是类内部的方法，基于 Voronoi 梯度
        pursuers{i} = pursuers{i}.calculateVelocity();
        
        % === C. 插入 CBF 模块 ===
        % 1. 获取刚刚计算出的标称速度
        u_nom = pursuers{i}.velocity; 
        
        % 2. 获取状态
        p_pursuer = pursuers{i}.position;
        p_evader_target = evaders{target_idx_global}.position;
        
        % 3. 获取 Evader 的速度 (在步骤 3 中已经安全计算出来了)
        v_evader_target = evaders{target_idx_global}.velocity; 
        
        % 4. 调用 QP 求解器修正速度
        u_safe = solve_cbf_3d(u_nom, p_pursuer, p_evader_target, v_evader_target, D_MIN, GAMMA);
        
        % 5. 将安全速度写回对象
        pursuers{i}.velocity = u_safe;
    end

    % --- 5. 执行移动 (Move) ---
    for i = 1:length(evaders)
        evaders{i} = evaders{i}.move(timestep);
    end
    for i = 1:length(pursuers)
        pursuers{i} = pursuers{i}.move(timestep);
    end

    % --- 6. 捕获判定 ---
    for i = 1:length(evaders)
        if ~evaders{i}.isDead
             evaders{i} = evaders{i}.checkIfAlive(pursuers);
        end
    end

    disp(['Time Step: ', num2str(t)]);

    % --- 7. 绘图 ---
    clf;
    hold on
    view(3); grid on; axis equal;
    axis([lbound(1) ubound(1) lbound(2) ubound(2) lbound(3) ubound(3)]);
    
    % 绘制 Voronoi (如果不报错且速度允许)
    plot(p, 'alpha', 0.1); 
    
    % 绘制 Agent 位置
    if ~isempty(pos_evaders)
        scatter3(pos_evaders(:,1), pos_evaders(:,2), pos_evaders(:,3), 50, 'r', 'filled', 'd');
        % 画出保护罩
        [sx, sy, sz] = sphere(10);
        for k=1:size(pos_evaders,1)
            surf(sx*D_MIN + pos_evaders(k,1), sy*D_MIN + pos_evaders(k,2), sz*D_MIN + pos_evaders(k,3), ...
                'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
        end
    end
    scatter3(pos_pursuers(:,1), pos_pursuers(:,2), pos_pursuers(:,3), 50, 'k', 'filled', 'd');
    
    title(['T = ' num2str(t)]);
    hold off;
    drawnow;
end

%% === CBF 3D 求解器函数 ===
function u_safe = solve_cbf_3d(u_nom, p_p, p_e, v_e, d_min, gamma)
    % 3D CBF-QP Solver
    % 约束: ||p_p - p_e|| >= d_min
    
    % 确保输入是列向量
    u_nom = u_nom(:);
    p_p = p_p(:);
    p_e = p_e(:);
    v_e = v_e(:);
    
    % 相对位置和距离
    rel_p = p_p - p_e;
    dist_sq = sum(rel_p.^2);
    
    % Barrier Function h(x) = ||p_p - p_e||^2 - d_min^2
    h = dist_sq - d_min^2;
    
    % 如果已经违反约束 (h < 0)，需要更强的恢复力或容错
    if h < 0
        % warning('Constraint violated! Activating strong repulsion.');
        gamma = gamma * 2; % 临时增加增益
    end
    
    % QP Formulation
    % min ||u - u_nom||^2  =>  min 0.5*u'*H*u + f'*u
    H = eye(3);
    f = -u_nom;
    
    % Constraint: Lfh + Lgh*u >= -gamma * h
    % h_dot = 2 * (p_p - p_e)' * (u_p - v_e)
    % 2*(p-e)'*u_p - 2*(p-e)'*v_e >= -gamma * h
    % => -2*(p-e)' * u_p <= gamma * h + 2*(p-e)'*v_e
    
    A = -2 * rel_p';
    b = gamma * h + 2 * rel_p' * v_e;
    
    % 设置最大速度限制 (假设 max speed = 2.0，根据你的实际 Pursuer 设定调整)
    v_max_limit = 5.0; 
    lb = [-v_max_limit; -v_max_limit; -v_max_limit];
    ub = [ v_max_limit;  v_max_limit;  v_max_limit];
    
    % 求解 QP
    options = optimoptions('quadprog', 'Display', 'off');
    [u_safe, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);
    
    if exitflag < 0
        % 如果无解 (通常因为已经被完全包围或者不得不撞)，全速远离
        dir = rel_p / norm(rel_p);
        u_safe = dir * v_max_limit; % 简单的逃逸策略
    end
    
    u_safe = u_safe'; % 转回原有形状 (假设是行向量)
end