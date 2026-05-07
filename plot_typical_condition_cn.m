function plot_typical_condition_cn(out, p, m)
%PLOT_TYPICAL_CONDITION_CN 绘制并保存中文典型工况开机仿真图
%
% 生成一张 2x3 总图：
%   (a) 转速响应；(b) 导叶开度；(c) 过机流量；
%   (d) 有效水头；(e) 蜗壳压力水头；(f) 尾水压力水头。

if nargin < 3 || isempty(m)
    m = calc_startup_metrics(out, p);
    m = complete_startup_metrics(out, p, m);
end

set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

fig = figure('Name', '设计水头典型工况开机过渡过程响应', ...
    'Color', 'w', ...
    'Position', [80 60 1450 820]);

%% (a) 转速响应
subplot(2, 3, 1);
plot(out.t, out.n_rpm, 'LineWidth', 2.0); hold on;
yline(p.nr, '--', '额定转速', 'LineWidth', 1.0);
xline(m.t_start_s, ':', '启动稳定时间', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('转速 / (r/min)'); title('(a) 转速响应');
legend('机组转速', '额定转速', '启动稳定时间', 'Location', 'best');

%% (b) 导叶开度响应
subplot(2, 3, 2);
plot(out.t, out.y_deg, 'LineWidth', 2.0); hold on;
plot(out.t, out.yc_deg, '--', 'LineWidth', 1.5);
xline(p.guide.t3, ':', 'PID接入', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('导叶开度 / deg'); title('(b) 导叶开度响应');
legend('实际导叶开度', '目标导叶开度', 'PID接入', 'Location', 'best');

%% (c) 过机流量响应
subplot(2, 3, 3);
plot(out.t, out.q_m3s, 'LineWidth', 2.0); hold on;
yline(p.Qr, '--', '额定流量', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('流量 / (m^3/s)'); title('(c) 过机流量响应');
legend('过机流量', '额定流量', 'Location', 'best');

%% (d) 有效水头响应
subplot(2, 3, 4);
plot(out.t, out.he_m, 'LineWidth', 2.0); hold on;
yline(p.Hr, '--', '设计水头', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('有效水头 / m'); title('(d) 有效水头响应');
legend('有效水头', '设计水头', 'Location', 'best');

%% (e) 蜗壳压力水头响应
subplot(2, 3, 5);
plot(out.t, out.casing_pressure_head_m, 'LineWidth', 2.0); hold on;
yline(m.hc_max_m, '--', '最大值', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('蜗壳压力水头 / m'); title('(e) 蜗壳压力水头响应');
legend('蜗壳压力水头', '最大值', 'Location', 'best');

%% (f) 尾水压力水头响应
subplot(2, 3, 6);
plot(out.t, out.tail_pressure_head_m, 'LineWidth', 2.0); hold on;
yline(m.ht_min_m, '--', '最小值', 'LineWidth', 1.0);
grid on; box on;
xlabel('时间 / s'); ylabel('尾水压力水头 / m'); title('(f) 尾水压力水头响应');
legend('尾水压力水头', '最小值', 'Location', 'best');

sgtitle('设计水头典型工况开机过渡过程响应');

if isfield(p, 'plot') && isfield(p.plot, 'save_figures') && p.plot.save_figures
    output_dir = p.plot.output_dir;
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    png_path = fullfile(output_dir, 'fig_typical_condition_response.png');
    fig_path = fullfile(output_dir, 'fig_typical_condition_response.fig');

    try
        exportgraphics(fig, png_path, 'Resolution', 300);
    catch
        saveas(fig, png_path);
    end
    savefig(fig, fig_path);
end

end
