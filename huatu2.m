%% === 4. Post-Processing: Data Analysis (Same as Formal Code) ===
valid_idx = 1:step_count;
valid_time = history_time(valid_idx);
valid_dist = history_dist(valid_idx, :);

% --- 1. 数据截断: 将所有小于 D_MIN 的值替换为 D_MIN ---
% ---------------------------------------------------

valid_vol  = history_vol(valid_idx, :);

% Figure 2: Distance Analysis
figure('Name', 'RL Baseline: Safety Constraints', 'Position', [100, 100, 600, 400], 'Color', 'w');
hold on; grid on;
colors = lines(N_P);

for i = 1:N_P
    plot(valid_time, valid_dist(:, i), 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', ['UAV ' num2str(i)]);
end

yline(D_MIN, 'r--', 'LineWidth', 2, 'DisplayName', 'D_{min}');
yline(D_MAX, 'g--', 'LineWidth', 2, 'DisplayName', 'D_{max}');

% 设置坐标轴标签 (16号字, 加粗)
xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
ylabel('Distance to Target (m)', 'FontSize', 16, 'FontWeight', 'bold');

% 设置坐标轴刻度数字 (16号字)
set(gca, 'FontSize', 16, 'LineWidth', 1.2);

% 设置图例 (16号字)
lgd = legend('Location', 'best');
set(lgd, 'FontSize', 16);

ylim([0, max(D_MAX + 2, max(valid_dist(:)))]);
% (已删除 title)


% Figure 3: Volume Analysis
figure('Name', 'RL Baseline: Containment Performance', 'Position', [720, 100, 600, 400], 'Color', 'w');
hold on; grid on;

% --- 2. 颜色修改: 将 autumn 改为 lines 以避免亮黄色 ---
colors_e = [0.8, 0.1, 0.1;   % 深红
            0.1, 0.1, 0.8;   % 深蓝
            0.1, 0.6, 0.1];  % 深绿 
% -------------------------------------------------

for i = 1:N_E
    plot(valid_time, valid_vol(:, i), 'LineWidth', 2, 'Color', colors_e(i,:), ...
        'DisplayName', ['Bird ' num2str(i) ' Volume']);
end

% 设置坐标轴标签 (16号字, 加粗)
xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
ylabel('Voronoi Cell Volume (m^3)', 'FontSize', 16, 'FontWeight', 'bold');

% 设置坐标轴刻度数字 (16号字)
set(gca, 'FontSize', 16, 'LineWidth', 1.2);

% 设置图例 (16号字)
lgd = legend('Location', 'best');
set(lgd, 'FontSize', 16);
% (已删除 title)