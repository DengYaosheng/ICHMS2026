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
D_MIN = 1.0;   % Minimum Distance (Keep Away)
D_MAX = 4.0;   % Maximum Distance (Keep Close)
GAMMA = 5.0;   % CBF Gain (Controls how aggressively it corrects)

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

figure(1)
pos_pursuers = zeros(n_p, 3);

%% === Main Loop ===
for t=0:timestep:timeend
    
    % --- 1. Data Preparation ---
    n_e_alive = 0;
    evaders_alive_index = [];
    pos_evaders = [];
    
    % Collect positions only initially
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
    % Calculate Voronoi Diagram
    [v, p] = mpt_voronoi(pos_all', 'bound', bound); 

    % --- 3. Update Evader and Calculate Velocity (For CBF Prediction) ---
    % Must setVoronoiCell before calculateVelocity
    for i = 1:length(p)
        if i <= n_e_alive
            % This is an Evader
            e_idx = evaders_alive_index(i);
            evaders{e_idx} = evaders{e_idx}.setVoronoiCell(p(i));
            % Calculate velocity now for prediction
            evaders{e_idx} = evaders{e_idx}.calculateVelocity();
        else
            % This is a Pursuer
            p_idx = i - n_e_alive;
            pursuers{p_idx} = pursuers{p_idx}.setVoronoiCell(p(i));
        end
    end

    adjacencyMatrix = getVoronoiAdjacency(p);
    
    % --- 4. Pursuer Decision & CBF Correction ---
    for i = 1:length(pursuers)
        % A. Find Nearest Evader
        dists = pdist2(pursuers{i}.position, pos_evaders);
        [min_dist, nearestEvaderIdx] = min(dists);
        target_idx_global = evaders_alive_index(nearestEvaderIdx);
        
        pursuers{i}.target = evaders{target_idx_global};
        
        % Check Adjacency
        if adjacencyMatrix(i + n_e_alive, nearestEvaderIdx)
            pursuers{i}.targetIsAdjacent = true;
        else
            pursuers{i}.targetIsAdjacent = false;
        end
        
        % B. Calculate Nominal Velocity
        pursuers{i} = pursuers{i}.calculateVelocity();
        
        % === C. Apply CBF Module ===
        % 1. Get Nominal Velocity
        u_nom = pursuers{i}.velocity; 
        
        % 2. Get State
        p_pursuer = pursuers{i}.position;
        p_evader_target = evaders{target_idx_global}.position;
        
        % 3. Get Evader Velocity
        v_evader_target = evaders{target_idx_global}.velocity; 
        
        % 4. Solve CBF-QP (Range Constraint: D_MIN <= d <= D_MAX)
        u_safe = solve_cbf_range_3d(u_nom, p_pursuer, p_evader_target, v_evader_target, D_MIN, D_MAX, GAMMA);
        
        % 5. Update Velocity
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
    clf;
    hold on
    view(3); grid on; axis equal;
    axis([lbound(1) ubound(1) lbound(2) ubound(2) lbound(3) ubound(3)]);
    
    % Plot Voronoi
    plot(p, 'alpha', 0.1); 
    
    % Plot Agents
    if ~isempty(pos_evaders)
        % Evaders (Red)
        scatter3(pos_evaders(:,1), pos_evaders(:,2), pos_evaders(:,3), 50, 'r', 'filled', 'd');
        
        % Visualizing the Range Constraints for the first Evader (Optional)
        [sx, sy, sz] = sphere(15);
        for k=1:size(pos_evaders,1)
            % Inner Sphere (Min Dist) - Red
            surf(sx*D_MIN + pos_evaders(k,1), sy*D_MIN + pos_evaders(k,2), sz*D_MIN + pos_evaders(k,3), ...
                'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
            % Outer Sphere (Max Dist) - Green
            surf(sx*D_MAX + pos_evaders(k,1), sy*D_MAX + pos_evaders(k,2), sz*D_MAX + pos_evaders(k,3), ...
                'FaceColor', 'g', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
        end
    end
    % Pursuers (Black)
    scatter3(pos_pursuers(:,1), pos_pursuers(:,2), pos_pursuers(:,3), 50, 'k', 'filled', 'd');
    
    title(['T = ' num2str(t)]);
    hold off;
    drawnow;
end

%% === CBF 3D Range Solver Function ===
function u_safe = solve_cbf_range_3d(u_nom, p_p, p_e, v_e, d_min, d_max, gamma)
    % 3D CBF-QP Solver for Range Constraint
    % Constraints: 
    % 1. ||p_p - p_e|| >= d_min  (Collision Avoidance)
    % 2. ||p_p - p_e|| <= d_max  (Connectivity Maintenance)
    
    % Ensure column vectors
    u_nom = u_nom(:);
    p_p = p_p(:);
    p_e = p_e(:);
    v_e = v_e(:);
    
    % Relative position and distance squared
    rel_p = p_p - p_e;       % Vector from Evader to Pursuer
    dist_sq = sum(rel_p.^2);
    
    % QP Formulation: min ||u - u_nom||^2  =>  min 0.5*u'*H*u + f'*u
    H = eye(3);
    f = -u_nom;
    
    % --- Constraint 1: Min Distance (h1 >= 0) ---
    % h1 = ||p - e||^2 - d_min^2
    h1 = dist_sq - d_min^2;
    % Derivative condition: -2*(p-e)' * u_p <= gamma * h1 + 2*(p-e)'*v_e
    A_min = -2 * rel_p';
    b_min = gamma * h1 + 2 * rel_p' * v_e;
    
    % --- Constraint 2: Max Distance (h2 >= 0) ---
    % h2 = d_max^2 - ||p - e||^2
    h2 = d_max^2 - dist_sq;
    % Derivative condition: 2*(p-e)' * u_p <= gamma * h2 + 2*(p-e)'*v_e
    A_max = 2 * rel_p';
    b_max = gamma * h2 + 2 * rel_p' * v_e;
    
    % Combine Constraints
    A = [A_min; A_max];
    b = [b_min; b_max];
    
    % Velocity Limits
    v_max_limit = 5.0; 
    lb = [-v_max_limit; -v_max_limit; -v_max_limit];
    ub = [ v_max_limit;  v_max_limit;  v_max_limit];
    
    % Solve QP
    options = optimoptions('quadprog', 'Display', 'off');
    [u_safe, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);
    
    % Error Handling
    if exitflag < 0
        % If infeasible (e.g., initial position is outside valid range), 
        % apply simple fallback logic
        dist = sqrt(dist_sq);
        dir = rel_p / dist;
        
        if dist < d_min
            u_safe = dir * v_max_limit;  % Run away (Expand)
        elseif dist > d_max
            u_safe = -dir * v_max_limit; % Move closer (Contract)
        else
            u_safe = [0;0;0]; % Should not happen if feasible
        end
    end
    
    u_safe = u_safe'; % Return as row vector
end