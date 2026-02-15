%% === 8. Post-Processing ===
valid_idx = 1:step_count;
valid_time = history_time(valid_idx);
valid_dist = history_dist(valid_idx, :);

% --- 数据处理: 将所有小于 D_MIN 的值替换为 D_MIN ---
valid_dist(valid_dist < D_MIN) = D_MIN;
% ------------------------------------------------

valid_vol  = history_vol(valid_idx, :);

figure('Position', [100, 100, 600, 400], 'Color', 'w');
hold on; grid on; colors = lines(n_p);

% 绘图循环
for i = 1:n_p
    plot(valid_time, valid_dist(:, i), 'LineWidth', 1.5, ...
         'Color', colors(i,:), 'DisplayName', ['UAV ' num2str(i)]);
end

% 绘制参考线 (如果不希望参考线出现在图例中，可以添加 'HandleVisibility','off')
yline(D_MIN, 'r--', 'LineWidth', 2, 'DisplayName', 'D_{min}'); 
yline(D_MAX, 'g--', 'LineWidth', 2, 'DisplayName', 'D_{max}');

% 1. 设置坐标轴标签 (字体放大到 16)
xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
ylabel('Distance to Target (m)', 'FontSize', 16, 'FontWeight', 'bold'); 

% 2. 设置坐标轴刻度数字大小 (也就是坐标轴上的数字，放大到 16)
set(gca, 'FontSize', 16, 'LineWidth', 1.2); 

% 3. 设置图例 (字体放大到 16)
lgd = legend;
set(lgd, 'FontSize', 16);

% 4. 已删除 title(...) 函数

figure('Name', 'Analysis 2: Containment Performance', 'Position', [720, 100, 600, 400], 'Color', 'w');
hold on; grid on; colors_e = [0.8, 0.1, 0.1;   % 深红
            0.1, 0.1, 0.8;   % 深蓝
            0.1, 0.6, 0.1];  % 深绿 

for i = 1:n_e
    plot(valid_time, valid_vol(:, i), 'LineWidth', 2, 'Color', colors_e(i,:), ...
        'DisplayName', ['Bird ' num2str(i) ' Volume']);
end

% 绘制阈值线 (添加 DisplayName 以便图例显示清晰名称)
% yline(VOL_CAPTURE_THRESHOLD, 'k--', 'LineWidth', 2, 'DisplayName', 'Capture Threshold');

% 1. 设置坐标轴标签 (字体放大到 16)
xlabel('Time (s)', 'FontSize', 16, 'FontWeight', 'bold'); 
ylabel('Voronoi Cell Volume (m^3)', 'FontSize', 16, 'FontWeight', 'bold'); 

% 2. 设置坐标轴刻度数字大小 (放大到 16)
set(gca, 'FontSize', 16, 'LineWidth', 1.2); 

% 3. 设置图例 (字体放大到 16)
lgd = legend;
set(lgd, 'FontSize', 16);

% 4. 已删除 title(...) 函数


