clc;
clear;
close all;
%  三种水头工况模型合理性验证
p0 = psu_default_params();
output_dir = fullfile(pwd, 'model_validation_results', datestr(now, 'yyyymmdd_HHMMSS'));
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

head_names = {'低水头工况'; '设计水头典型工况'; '高水头工况'};
head_values = [p0.ref.Hmin; p0.ref.Hr; p0.ref.Hmax];
line_names = {'低水头', '设计水头', '高水头'};

colors = [0.20 0.62 0.28; 0.18 0.18 0.18; 0.00 0.27 0.78];
line_width = [1.8, 2.2, 1.8];

n_case = numel(head_values);
P = cell(n_case, 1);
OUT = cell(n_case, 1);
METRICS = cell(n_case, 1);
X = cell(n_case, 1);

for i = 1:n_case
    p_case0 = set_operating_head(p0, head_values(i));
    p_case0.plot.output_dir = output_dir;

    % 完成一次典型参数仿真并返回响应结果。
    % 三水头验证只看模型响应规律，不计算 J，避免每个水头都归一成 1 造成误导。
    [p_case, out_case, m_case, x_case] = prepare_objective_params(p_case0);

    P{i} = p_case;
    OUT{i} = out_case;
    METRICS{i} = m_case;
    X{i} = x_case;

    fprintf('\n========== %s H = %.3f m ==========%s', head_names{i}, head_values(i), newline);
    print_validation_metrics_cn(m_case, out_case);
end

T_head = make_head_validation_table(head_names, head_values, OUT, METRICS);
fprintf('\n========== 三种水头工况模型适应性验证指标 ==========%s', newline);
disp(T_head);
safe_writetable_local(T_head, output_dir, 'table_three_head_validation.csv');

plot_three_head_response(OUT, P, head_values, line_names, colors, line_width, output_dir);

save(fullfile(output_dir, 'workspace_three_head_validation.mat'), ...
    'p0', 'P', 'OUT', 'METRICS', 'X', 'T_head', 'head_names', 'head_values');

fprintf('\n三种水头工况验证结果已保存到：%s%s', output_dir, newline);

function T = make_head_validation_table(names, heads, outs, metrics)
n = numel(names);
condition = names(:);
H_operating_m = heads(:);
n_max_rpm = zeros(n,1);
overshoot_pu = zeros(n,1);
final_error_pu = zeros(n,1);
speed_iae = zeros(n,1);
q_max_m3s = zeros(n,1);
hc_max_m = zeros(n,1);
ht_range_m = zeros(n,1);
ht_min_m = zeros(n,1);
guide_rate_rms = zeros(n,1);

for k = 1:n
    m = metrics{k};
    out = outs{k};

    n_max_rpm(k) = getv(m, 'n_max_rpm');
    overshoot_pu(k) = getv(m, 'overshoot_base_pu');
    final_error_pu(k) = getv(m, 'final_speed_error_pu');
    speed_iae(k) = getv(m, 'speed_iae');
    q_max_m3s(k) = max(out.q_m3s);
    hc_max_m(k) = getv(m, 'hc_max_m');
    ht_range_m(k) = getv(m, 'ht_range_m');
    ht_min_m(k) = getv(m, 'ht_min_m');
    guide_rate_rms(k) = getv(m, 'guide_rate_rms');
end

T = table(condition, H_operating_m, n_max_rpm, overshoot_pu, ...
    final_error_pu, speed_iae, q_max_m3s, hc_max_m, ht_range_m, ht_min_m, guide_rate_rms, ...
    'VariableNames', {'condition','H_operating_m','n_max_rpm','overshoot_pu', ...
    'final_error_pu','speed_iae','q_max_m3s','hc_max_m','ht_range_m','ht_min_m','guide_rate_rms'});
end

function plot_three_head_response(outs, ps, heads, names, colors, line_width, output_dir)
fields = {'n_rpm','y_deg','q_m3s','he_m','casing_pressure_head_m','tail_pressure_head_m'};
ylabels = {'转速 / (r/min)','导叶开度 / deg','流量 / (m^3/s)','有效水头 / m','蜗壳压力水头 / m','尾水压力水头 / m'};
file_tags = {'speed','guide','flow','effective_head','casing_pressure','tail_pressure'};
fig_names = {'fig_01a_three_head_speed','fig_01b_three_head_guide','fig_01c_three_head_flow', ...
    'fig_01d_three_head_effective_head','fig_01e_three_head_casing_pressure','fig_01f_three_head_tail_pressure'};

for k = 1:numel(fields)
    fig = figure('Name', fig_names{k}, 'Color', 'w', 'Position', [100 100 900 560]);
    for i = 1:numel(outs)
        plot(outs{i}.t, outs{i}.(fields{k}), 'Color', colors(i,:), 'LineWidth', line_width(i));
        hold on;
    end

    if k == 1
        yline(ps{2}.nr, ':', '额定转速', 'LineWidth', 1.0);
    elseif k == 2
        xline(ps{2}.guide.t3, ':', 'PID接入', 'LineWidth', 1.0);
    elseif k == 3
        yline(ps{2}.Qr, ':', '额定流量', 'LineWidth', 1.0);
    elseif k == 4
        for i = 1:numel(heads)
            yline(heads(i), ':', sprintf('H=%.1f m', heads(i)), 'Color', colors(i,:), 'LineWidth', 0.7);
        end
    end

    format_axes('时间 / s', ylabels{k});
    legend(names, 'Location', 'best');
    save_figure(fig, output_dir, [fig_names{k} '.png']);
end
end

function format_axes(xlab, ylab)
grid on;
box on;
xlabel(xlab);
ylabel(ylab);
end

function save_figure(fig, output_dir, filename)
file = fullfile(output_dir, filename);
try
    exportgraphics(fig, file, 'Resolution', 300);
catch
    saveas(fig, file);
end
end

function safe_writetable_local(T, output_dir, filename)
file = fullfile(output_dir, filename);
try
    writetable(T, file);
catch ME
    [~, name, ext] = fileparts(filename);
    alt = [name '_' datestr(now, 'yyyymmdd_HHMMSS') ext];
    warning('写入 %s 失败：%s。已改写为 %s。', filename, ME.message, alt);
    writetable(T, fullfile(output_dir, alt));
end
end


function print_validation_metrics_cn(m, out)
fprintf('最大转速 n_max = %.3f r/min\n', getv(m, 'n_max_rpm'));
fprintf('转速超调量 overshoot = %.6f p.u.\n', getv(m, 'overshoot_base_pu'));
fprintf('终值转速误差 final_error = %.6f p.u.\n', getv(m, 'final_speed_error_pu'));
fprintf('稳定段速度误差积分 IAE = %.6f\n', getv(m, 'speed_iae'));
fprintf('最大过机流量 q_max = %.3f m^3/s\n', max(out.q_m3s));
fprintf('蜗壳最大压力水头 hc_max = %.3f m\n', getv(m, 'hc_max_m'));
fprintf('尾水压力波动范围 ht_range = %.3f m\n', getv(m, 'ht_range_m'));
fprintf('尾水最小压力水头 ht_min = %.3f m\n', getv(m, 'ht_min_m'));
fprintf('导叶速率均方根 guide_rate = %.6f p.u./s\n', getv(m, 'guide_rate_rms'));
end

function v = getv(s, name)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(s.(name))
    v = s.(name);
else
    v = nan;
end
end
