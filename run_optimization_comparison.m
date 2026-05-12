clc;
close all;

% GA / PSO / DE 三种传统算法对比
cfg = algorithm_config();
rng(cfg.seed);

if ~exist(cfg.output_dir, 'dir')
    mkdir(cfg.output_dir);
elseif isfield(cfg, 'clean_output') && cfg.clean_output
    old_files = [dir(fullfile(cfg.output_dir, '*.png')); ...
                 dir(fullfile(cfg.output_dir, '*.csv')); ...
                 dir(fullfile(cfg.output_dir, '*.mat'))];
    for k_clean = 1:numel(old_files)
        try
            delete(fullfile(old_files(k_clean).folder, old_files(k_clean).name));
        catch
        end
    end
end
fprintf('本次结果输出文件夹：%s%s', cfg.output_dir, newline);

set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

%% 1. 参数与优化前
p0 = psu_default_params();
[p, out_pre, m_pre, x_pre] = prepare_objective_params(p0);
[J_pre, ~] = objective_startup(x_pre, p);

fprintf('\n========== 设计水头典型工况：优化前方案 ==========%s', newline);
fprintf('J_pre = %.6f%s', J_pre, newline);
print_metrics_cn(m_pre);

%% 2. PID 闭环修正作用对比
p_no_pid = p;
p_no_pid.pid.Kp = 0;
p_no_pid.pid.Ki = 0;
p_no_pid.pid.Kd = 0;
% 开环导叶控制对比中同时关闭额定转速恢复项，保证它是真正的开环导叶控制。
if isfield(p_no_pid, 'elec') && isfield(p_no_pid.elec, 'restore_gain')
    p_no_pid.elec.restore_gain = 0;
end

out_no_pid = simulate_startup_moc(p_no_pid);
m_no_pid = calc_startup_metrics(out_no_pid, p_no_pid);
m_no_pid = complete_startup_metrics(out_no_pid, p_no_pid, m_no_pid);

fprintf('\n========== PID闭环修正作用对比 ==========%s', newline);
fprintf('开环导叶控制：\n');
print_metrics_cn(m_no_pid);
fprintf('开环导叶给定+PID闭环修正：\n');
print_metrics_cn(m_pre);

T_pid = make_pid_table({'开环导叶控制'; '开环导叶给定+PID闭环修正'}, {m_no_pid; m_pre});
safe_writetable(T_pid, cfg, 'table_pid_correction_comparison.csv');

%% 3. 运行 GA / PSO / DE
results = optimize_startup('ALL', p, cfg);

result_ga = results.GA;
result_pso = results.PSO;
result_de = results.DE;

%% 4. 输出表格
methods = {'优化前'; 'GA'; 'PSO'; 'DE'};
metrics = {m_pre; result_ga.best_metrics; result_pso.best_metrics; result_de.best_metrics};
Js = [J_pre; result_ga.best_J; result_pso.best_J; result_de.best_J];
T_metrics = make_metric_table(methods, Js, metrics);
T_x = make_x_table(x_pre, result_ga.best_x, result_pso.best_x, result_de.best_x);
T_boundary = make_boundary_table(T_x, p);
T_warning = make_warning_table(T_metrics, T_boundary);
T_recommend = make_recommendation_table(T_metrics, T_boundary);
T_improve = make_improvement_table(T_metrics);

[best_J, best_idx] = min(Js(2:end));
best_method = methods{best_idx + 1};

fprintf('\n========== 三种传统算法最优方案 ==========%s', newline);
fprintf('最优算法：%s，J = %.6f%s', best_method, best_J, newline);

fprintf('\n========== 目标函数与核心指标对比表 ==========%s', newline);
disp(T_metrics);

fprintf('\n========== 决策变量对比表 ==========%s', newline);
disp(T_x);

fprintf('\n========== 边界与异常诊断表 ==========%s', newline);
disp(T_boundary);
disp(T_warning);

fprintf('\n========== 推荐方案诊断表 ==========%s', newline);
disp(T_recommend);

fprintf('\n========== 相对优化前改善率表 ==========%s', newline);
disp(T_improve);

safe_writetable(T_metrics, cfg, 'table_algorithm_metrics.csv');
safe_writetable(T_x, cfg, 'table_algorithm_decision_variables.csv');
safe_writetable(T_boundary, cfg, 'table_boundary_check.csv');
safe_writetable(T_warning, cfg, 'table_result_warnings.csv');
safe_writetable(T_recommend, cfg, 'table_recommendation.csv');
safe_writetable(T_improve, cfg, 'table_improvement_percent.csv');
T_param = write_zeng_parameter_table(p, cfg.output_dir);
T_pso_final = make_pso_final_table(T_metrics, T_x, T_improve, T_recommend);
safe_writetable(T_pso_final, cfg, 'table_pso_final_selected.csv');

fprintf('========== PSO 推荐方案摘要 ==========%s', newline);
disp(T_pso_final);

%% 5. 绘图
outs = {out_pre; result_ga.best_out; result_pso.best_out; result_de.best_out};
plot_typical_response(out_pre, p, m_pre, cfg);
plot_pid_speed_guide(out_no_pid, out_pre, p, cfg);
plot_pid_response(out_no_pid, out_pre, p, cfg);
plot_algorithm_response(outs, methods, p, cfg);
plot_algorithm_zoom(outs, methods, p, cfg);
plot_pressure_comparison(outs, methods, cfg);
plot_convergence(result_ga, result_pso, result_de, cfg);
plot_metric_bar(T_metrics, cfg);
plot_decision_bar(T_x, p, cfg);
plot_improvement_bar(T_improve, cfg);
plot_response_difference(outs, methods, cfg);

save(fullfile(cfg.output_dir, 'workspace_algorithm_results.mat'), ...
    'cfg', 'p', 'x_pre', 'out_pre', 'm_pre', 'J_pre', ...
    'out_no_pid', 'm_no_pid', 'T_pid', ...
    'result_ga', 'result_pso', 'result_de', 'T_metrics', 'T_x', ...
    'T_boundary', 'T_warning', 'T_recommend', 'T_improve', 'T_param', 'T_pso_final', ...
    'best_method', 'best_J');

fprintf('\n========== optimization_comparison 完成 ==========%s', newline);
fprintf('结果保存位置：%s%s', cfg.output_dir, newline);

%% ============================================================
%  Local functions
%% ============================================================

function T = make_improvement_table(T_metrics)
%MAKE_IMPROVEMENT_TABLE 计算各优化方案相对于优化前的改善率，单位为百分比。
% 正值表示该指标优于优化前；负值表示劣于优化前。
method = T_metrics.method;
n = height(T_metrics);

base = T_metrics(1, :);
J_improve_pct = zeros(n, 1);
t_start_improve_pct = zeros(n, 1);
overshoot_improve_pct = zeros(n, 1);
final_error_improve_pct = zeros(n, 1);
speed_iae_improve_pct = zeros(n, 1);
hc_max_improve_pct = zeros(n, 1);
ht_range_improve_pct = zeros(n, 1);
guide_rate_improve_pct = zeros(n, 1);

for i = 1:n
    J_improve_pct(i) = improve(base.J, T_metrics.J(i));
    t_start_improve_pct(i) = improve(base.t_start_s, T_metrics.t_start_s(i));
    overshoot_improve_pct(i) = improve(base.overshoot_pu, T_metrics.overshoot_pu(i));
    final_error_improve_pct(i) = improve(base.final_error_pu, T_metrics.final_error_pu(i));
    speed_iae_improve_pct(i) = improve(base.speed_iae, T_metrics.speed_iae(i));
    hc_max_improve_pct(i) = improve(base.hc_max_m, T_metrics.hc_max_m(i));
    ht_range_improve_pct(i) = improve(base.ht_range_m, T_metrics.ht_range_m(i));
    guide_rate_improve_pct(i) = improve(base.guide_rate_rms, T_metrics.guide_rate_rms(i));
end

T = table(method, J_improve_pct, t_start_improve_pct, overshoot_improve_pct, ...
    final_error_improve_pct, speed_iae_improve_pct, hc_max_improve_pct, ...
    ht_range_improve_pct, guide_rate_improve_pct, ...
    'VariableNames', {'method','J_improve_pct','t_start_improve_pct','overshoot_improve_pct', ...
    'final_error_improve_pct','speed_iae_improve_pct','hc_max_improve_pct', ...
    'ht_range_improve_pct','guide_rate_improve_pct'});
end

function y = improve(base_value, new_value)
y = 100 * (base_value - new_value) / max(abs(base_value), eps);
end

function T = make_boundary_table(T_x, p)
var_names = {'Kp','Ki','Kd','Y1','Y2','t1','t2','t3'};
lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';
method = T_x.method;
n = height(T_x);
min_boundary_distance = zeros(n, 1);
near_boundary_count = zeros(n, 1);
nearest_boundary_variable = strings(n, 1);
status = strings(n, 1);

for i = 1:n
    x = [T_x.Kp(i), T_x.Ki(i), T_x.Kd(i), T_x.Y1(i), T_x.Y2(i), T_x.t1(i), T_x.t2(i), T_x.t3(i)];
    z = (x - lb) ./ max(ub - lb, eps);
    dist = min(z, 1 - z);
    [min_boundary_distance(i), idx] = min(dist);
    near_boundary_count(i) = sum(dist < 0.05);
    nearest_boundary_variable(i) = string(var_names{idx});
    if near_boundary_count(i) == 0
        status(i) = "未明显贴边";
    elseif near_boundary_count(i) <= 2
        status(i) = "轻微贴边";
    else
        status(i) = "贴边较多，需谨慎解释";
    end
end

T = table(method, min_boundary_distance, near_boundary_count, nearest_boundary_variable, status, ...
    'VariableNames', {'method','min_boundary_distance','near_boundary_count','nearest_boundary_variable','status'});
end

function T = make_warning_table(T_metrics, T_boundary)
method = T_metrics.method;
n = height(T_metrics);
warning = strings(n, 1);

base_iae = T_metrics.speed_iae(1);
base_t = T_metrics.t_start_s(1);
base_hc = T_metrics.hc_max_m(1);
base_ht = T_metrics.ht_range_m(1);

for i = 1:n
    msg = strings(0, 1);
    if T_metrics.speed_iae(i) > 1.01 * base_iae
        msg(end+1) = "稳定段IAE高于优化前1%以上"; %#ok<AGROW>
    end
    if T_metrics.t_start_s(i) > 1.01 * base_t
        msg(end+1) = "启动时间劣于优化前"; %#ok<AGROW>
    end
    if T_metrics.hc_max_m(i) > 1.01 * base_hc
        msg(end+1) = "蜗壳压力高于优化前1%以上"; %#ok<AGROW>
    end
    if T_metrics.ht_range_m(i) > 1.02 * base_ht
        msg(end+1) = "尾水波动高于优化前2%以上"; %#ok<AGROW>
    end
    if T_boundary.near_boundary_count(i) > 0
        msg(end+1) = "存在变量贴近边界"; %#ok<AGROW>
    end
    if isempty(msg)
        warning(i) = "无明显异常";
    else
        warning(i) = strjoin(msg, "；");
    end
end

T = table(method, warning, 'VariableNames', {'method','warning'});
end

function T = make_recommendation_table(T_metrics, T_boundary)
%MAKE_RECOMMENDATION_TABLE 区分“目标函数最优”和“论文推荐方案”。
% 若 PSO 的推荐评分与最优方案很接近，则优先标记为论文推荐，
% 这样既保持三算法公平对比，也服务于本文后续采用 PSO 的叙事。

method = T_metrics.method;
n = height(T_metrics);
base_iae = T_metrics.speed_iae(1);
base_final = T_metrics.final_error_pu(1);
base_t = T_metrics.t_start_s(1);

iae_ratio = T_metrics.speed_iae ./ max(base_iae, eps);
final_ratio = T_metrics.final_error_pu ./ max(base_final, eps);
t_ratio = T_metrics.t_start_s ./ max(base_t, eps);

recommended_score = T_metrics.J ...
    + 0.35 * max(0, iae_ratio - 1.00) ...
    + 0.08 * T_boundary.near_boundary_count ...
    + 0.16 * max(0, final_ratio - 1.00) ...
    + 0.08 * max(0, t_ratio - 1.00);

role = strings(n, 1);
role(:) = "对比方案";
[~, idx_j] = min(T_metrics.J(2:end));
idx_j = idx_j + 1;
[~, idx_rec] = min(recommended_score(2:end));
idx_rec = idx_rec + 1;

role(1) = "优化前方案";
role(idx_j) = "目标函数最优";
role(idx_rec) = "工程推荐方案";

idx_pso = find(strcmp(T_metrics.method, "PSO"), 1);
if ~isempty(idx_pso)
    best_score = min(recommended_score(2:end));
    if recommended_score(idx_pso) <= 1.03 * best_score
        if idx_pso == idx_j
            role(idx_pso) = "目标函数最优且论文推荐PSO";
        else
            role(idx_pso) = "论文推荐PSO方案";
        end
    end
end

T = table(method, T_metrics.J, recommended_score, iae_ratio, final_ratio, ...
    T_boundary.near_boundary_count, role, ...
    'VariableNames', {'method','J','recommended_score','IAE_ratio','final_error_ratio', ...
    'near_boundary_count','role'});
end

function T = make_pid_table(methods, metrics)
n = numel(methods);
t_start = zeros(n, 1);
overshoot = zeros(n, 1);
final_error = zeros(n, 1);
speed_iae = zeros(n, 1);
hc_max = zeros(n, 1);
ht_range = zeros(n, 1);
ht_min = zeros(n, 1);
guide_rate = zeros(n, 1);

for i = 1:n
    t_start(i) = getv(metrics{i}, 't_start_s');
    overshoot(i) = getv(metrics{i}, 'overshoot_base_pu');
    final_error(i) = getv(metrics{i}, 'final_speed_error_pu');
    speed_iae(i) = getv(metrics{i}, 'speed_iae');
    hc_max(i) = getv(metrics{i}, 'hc_max_m');
    ht_range(i) = getv(metrics{i}, 'ht_range_m');
    ht_min(i) = getv(metrics{i}, 'ht_min_m');
    guide_rate(i) = getv(metrics{i}, 'guide_rate_rms');
end

T = table(methods, t_start, overshoot, final_error, speed_iae, ...
    hc_max, ht_range, ht_min, guide_rate, ...
    'VariableNames', {'method','t_start_s','overshoot_pu', ...
    'final_error_pu','speed_iae','hc_max_m','ht_range_m','ht_min_m','guide_rate_rms'});
end

function T = make_metric_table(methods, Js, metrics)
n = numel(methods);
t_start = zeros(n, 1);
overshoot = zeros(n, 1);
final_error = zeros(n, 1);
speed_iae = zeros(n, 1);
hc_max = zeros(n, 1);
ht_range = zeros(n, 1);
ht_min = zeros(n, 1);
guide_rate = zeros(n, 1);

for i = 1:n
    t_start(i) = getv(metrics{i}, 't_start_s');
    overshoot(i) = getv(metrics{i}, 'overshoot_base_pu');
    final_error(i) = getv(metrics{i}, 'final_speed_error_pu');
    speed_iae(i) = getv(metrics{i}, 'speed_iae');
    hc_max(i) = getv(metrics{i}, 'hc_max_m');
    ht_range(i) = getv(metrics{i}, 'ht_range_m');
    ht_min(i) = getv(metrics{i}, 'ht_min_m');
    guide_rate(i) = getv(metrics{i}, 'guide_rate_rms');
end

T = table(methods, Js, t_start, overshoot, final_error, speed_iae, ...
    hc_max, ht_range, ht_min, guide_rate, ...
    'VariableNames', {'method','J','t_start_s','overshoot_pu', ...
    'final_error_pu','speed_iae','hc_max_m','ht_range_m','ht_min_m','guide_rate_rms'});
end

function T = make_x_table(x0, x_ga, x_pso, x_de)
method = {'优化前'; 'GA'; 'PSO'; 'DE'};
X = [x0(:).'; x_ga(:).'; x_pso(:).'; x_de(:).'];
T = table(method, X(:,1), X(:,2), X(:,3), X(:,4), X(:,5), X(:,6), X(:,7), X(:,8), ...
    'VariableNames', {'method','Kp','Ki','Kd','Y1','Y2','t1','t2','t3'});
end

function v = getv(s, name)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(s.(name))
    v = s.(name);
else
    v = nan;
end
end

function plot_typical_response(out, p, m, cfg)
fields = {'n_rpm','y_deg','q_m3s','he_m','casing_pressure_head_m','tail_pressure_head_m'};
ylabels = {'转速 / (r/min)','导叶开度 / deg','流量 / (m^3/s)','有效水头 / m','蜗壳压力水头 / m','尾水压力水头 / m'};
fig_names = {'fig_01a_typical_speed','fig_01b_typical_guide','fig_01c_typical_flow', ...
    'fig_01d_typical_effective_head','fig_01e_typical_casing_pressure','fig_01f_typical_tail_pressure'};

for k = 1:numel(fields)
    fig = figure('Name', fig_names{k}, 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
    plot(out.t, out.(fields{k}), 'Color', cfg.color.base, 'LineWidth', 2.1); hold on;
    if k == 1
        yline(p.nr, ':', '额定转速', 'LineWidth', 1.0);
        xline(m.t_start_s, '--', '稳定时间', 'LineWidth', 1.0);
        legend('优化前', '额定转速', '稳定时间', 'Location', 'best');
    elseif k == 2
        xline(p.guide.t3, ':', 'PID接入', 'LineWidth', 1.0);
        legend('优化前', 'PID接入', 'Location', 'best');
    elseif k == 3
        yline(p.Qr, ':', '额定流量', 'LineWidth', 1.0);
        legend('优化前', '额定流量', 'Location', 'best');
    elseif k == 4
        yline(p.Hr, ':', '设计水头', 'LineWidth', 1.0);
        legend('优化前', '设计水头', 'Location', 'best');
    else
        legend('优化前', 'Location', 'best');
    end
    format_axes('时间 / s', ylabels{k});
    save_figure(fig, cfg, [fig_names{k} '.png']);
end
end

function plot_pid_speed_guide(out_no, out_pid, p, cfg)
fig = figure('Name', 'fig_02a_pid_speed_response', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot(out_no.t, out_no.n_rpm, '--', 'Color', cfg.color.no_pid, 'LineWidth', 2.0); hold on;
plot(out_pid.t, out_pid.n_rpm, 'Color', cfg.color.with_pid, 'LineWidth', 2.2);
yline(p.nr, ':', '额定转速', 'LineWidth', 1.0);
xline(p.guide.t3, ':', 'PID接入', 'LineWidth', 1.0);
format_axes('时间 / s', '转速 / (r/min)');
legend('开环导叶控制', 'PID闭环修正', '额定转速', 'PID接入', 'Location', 'best');
save_figure(fig, cfg, 'fig_02a_pid_speed_response.png');

fig = figure('Name', 'fig_02b_pid_guide_response', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot(out_no.t, out_no.y_deg, '--', 'Color', cfg.color.no_pid, 'LineWidth', 2.0); hold on;
plot(out_pid.t, out_pid.y_deg, 'Color', cfg.color.with_pid, 'LineWidth', 2.2);
xline(p.guide.t3, ':', 'PID接入', 'LineWidth', 1.0);
format_axes('时间 / s', '导叶开度 / deg');
legend('开环导叶控制', 'PID闭环修正', 'PID接入', 'Location', 'best');
save_figure(fig, cfg, 'fig_02b_pid_guide_response.png');
end

function plot_pid_response(out_no, out_pid, p, cfg)
fields = {'n_rpm','y_deg','q_m3s','he_m','casing_pressure_head_m','tail_pressure_head_m'};
ylabels = {'转速 / (r/min)','导叶开度 / deg','流量 / (m^3/s)','有效水头 / m','蜗壳压力水头 / m','尾水压力水头 / m'};
fig_names = {'fig_03a_pid_speed','fig_03b_pid_guide','fig_03c_pid_flow', ...
    'fig_03d_pid_effective_head','fig_03e_pid_casing_pressure','fig_03f_pid_tail_pressure'};
for k = 1:numel(fields)
    fig = figure('Name', fig_names{k}, 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
    plot(out_no.t, out_no.(fields{k}), '--', 'Color', cfg.color.no_pid, 'LineWidth', 2.0); hold on;
    plot(out_pid.t, out_pid.(fields{k}), 'Color', cfg.color.with_pid, 'LineWidth', 2.1);
    if k == 1
        yline(p.nr, ':', '额定转速', 'LineWidth', 1.0);
    elseif k == 3
        yline(p.Qr, ':', '额定流量', 'LineWidth', 1.0);
    elseif k == 4
        yline(p.Hr, ':', '设计水头', 'LineWidth', 1.0);
    end
    xline(p.guide.t3, ':', 'PID接入', 'LineWidth', 0.8);
    format_axes('时间 / s', ylabels{k});
    legend('开环导叶控制', 'PID闭环修正', 'Location', 'best');
    save_figure(fig, cfg, [fig_names{k} '.png']);
end
end

function plot_algorithm_response(outs, methods, p, cfg)
fields = {'n_rpm','y_deg','q_m3s','he_m','casing_pressure_head_m','tail_pressure_head_m'};
ylabels = {'转速 / (r/min)','导叶开度 / deg','流量 / (m^3/s)','有效水头 / m','蜗壳压力水头 / m','尾水压力水头 / m'};
fig_names = {'fig_04a_algorithm_speed','fig_04b_algorithm_guide','fig_04c_algorithm_flow', ...
    'fig_04d_algorithm_effective_head','fig_04e_algorithm_casing_pressure','fig_04f_algorithm_tail_pressure'};
for k = 1:numel(fields)
    fig = figure('Name', fig_names{k}, 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
    plot_methods(outs, fields{k}, cfg); hold on;
    if k == 1
        yline(p.nr, ':', '额定转速', 'LineWidth', 1.0);
    elseif k == 3
        yline(p.Qr, ':', '额定流量', 'LineWidth', 1.0);
    elseif k == 4
        yline(p.Hr, ':', '设计水头', 'LineWidth', 1.0);
    end
    format_axes('时间 / s', ylabels{k});
    legend(methods, 'Location', 'best');
    save_figure(fig, cfg, [fig_names{k} '.png']);
end
end

function plot_algorithm_zoom(outs, methods, p, cfg)
fig = figure('Name', 'fig_05a_zoom_speed', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'n_rpm', cfg); hold on;
yline(p.nr, ':', '额定转速');
xlim(cfg.zoom.speed_xlim);
if isfield(cfg.zoom, 'speed_ylim'), ylim(cfg.zoom.speed_ylim); end
format_axes('时间 / s', '转速 / (r/min)');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_05a_zoom_speed.png');

fig = figure('Name', 'fig_05b_zoom_guide', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'y_deg', cfg); xlim(cfg.zoom.guide_xlim);
format_axes('时间 / s', '导叶开度 / deg');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_05b_zoom_guide.png');

fig = figure('Name', 'fig_05c_zoom_casing_pressure', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'casing_pressure_head_m', cfg); xlim(cfg.zoom.pressure_xlim);
format_axes('时间 / s', '蜗壳压力水头 / m');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_05c_zoom_casing_pressure.png');

fig = figure('Name', 'fig_05d_zoom_tail_pressure', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'tail_pressure_head_m', cfg); xlim(cfg.zoom.pressure_xlim);
format_axes('时间 / s', '尾水压力水头 / m');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_05d_zoom_tail_pressure.png');
end

function plot_pressure_comparison(outs, methods, cfg)
fig = figure('Name', 'fig_06a_casing_pressure_comparison', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'casing_pressure_head_m', cfg);
format_axes('时间 / s', '蜗壳压力水头 / m');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_06a_casing_pressure_comparison.png');

fig = figure('Name', 'fig_06b_tail_pressure_comparison', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot_methods(outs, 'tail_pressure_head_m', cfg);
format_axes('时间 / s', '尾水压力水头 / m');
legend(methods, 'Location', 'best');
save_figure(fig, cfg, 'fig_06b_tail_pressure_comparison.png');
end

function plot_convergence(result_ga, result_pso, result_de, cfg)
fig = figure('Name', 'fig_07_convergence', 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
plot(result_ga.history.best_cost, 'Color', cfg.color.ga, 'LineWidth', cfg.line.ga); hold on;
plot(result_pso.history.best_cost, 'Color', cfg.color.pso, 'LineWidth', cfg.line.pso);
plot(result_de.history.best_cost, 'Color', cfg.color.de, 'LineWidth', cfg.line.de);
format_axes('迭代次数', '目标函数值', 'GA、PSO、DE 收敛曲线对比');
legend('GA', 'PSO', 'DE', 'Location', 'best');
save_figure(fig, cfg, 'fig_07_convergence.png');
end

function plot_metric_bar(T, cfg)
method_names = T.method;
Y = [T.t_start_s ./ T.t_start_s(1), ...
     T.overshoot_pu ./ max(T.overshoot_pu(1), eps), ...
     T.final_error_pu ./ max(T.final_error_pu(1), eps), ...
     T.speed_iae ./ T.speed_iae(1), ...
     T.hc_max_m ./ T.hc_max_m(1), ...
     T.ht_range_m ./ T.ht_range_m(1), ...
     T.guide_rate_rms ./ T.guide_rate_rms(1)];

fig = figure('Name', 'fig_08_metric_bar', 'Color', 'w', 'Position', [100 100 1150 560], 'Visible', cfg.plot.visible);
bar(Y);
grid on; box on;
set(gca, 'XTick', 1:numel(method_names), 'XTickLabel', method_names);
ylabel('相对优化前归一化值');
legend({'启动时间','转速超调','终值误差','稳定段IAE','蜗壳压力','尾水波动','导叶速率'}, 'Location', 'bestoutside');
save_figure(fig, cfg, 'fig_08_metric_bar.png');
end

function plot_improvement_bar(T, cfg)
%PLOT_IMPROVEMENT_BAR 绘制各优化方案相对优化前改善率。
method_names = T.method(2:end);
Y = [T.t_start_improve_pct(2:end), ...
     T.overshoot_improve_pct(2:end), ...
     T.final_error_improve_pct(2:end), ...
     T.speed_iae_improve_pct(2:end), ...
     T.hc_max_improve_pct(2:end), ...
     T.ht_range_improve_pct(2:end), ...
     T.guide_rate_improve_pct(2:end)];

fig = figure('Name', 'fig_10_improvement_percent', 'Color', 'w', 'Position', [100 100 1200 560], 'Visible', cfg.plot.visible);
bar(Y);
yline(0, 'k-', 'LineWidth', 1.0);
grid on; box on;
set(gca, 'XTick', 1:numel(method_names), 'XTickLabel', method_names);
ylabel('相对优化前改善率 / %');
legend({'启动时间','转速超调','终值误差','稳定段IAE','蜗壳压力','尾水波动','导叶速率'}, 'Location', 'bestoutside');
save_figure(fig, cfg, 'fig_10_improvement_percent.png');
end

function plot_decision_bar(T, p, cfg)
method_names = T.method;
X = [T.Kp, T.Ki, T.Kd, T.Y1, T.Y2, T.t1, T.t2, T.t3];

% 决策变量量纲和典型工况值差异很大。若直接除以优化前，Kd 这类小优化前变量会被夸张放大。
% 这里改为按搜索范围归一化，更适合判断变量是否贴近上下边界。
lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';
Xn = (X - lb) ./ max(ub - lb, eps);

fig = figure('Name', 'fig_09_decision_variables', 'Color', 'w', 'Position', [100 100 1150 560], 'Visible', cfg.plot.visible);
bar(Xn);
grid on; box on;
ylim([0 1]);
set(gca, 'XTick', 1:numel(method_names), 'XTickLabel', method_names);
ylabel('相对搜索范围归一化值');
legend({'Kp','Ki','Kd','Y1','Y2','t1','t2','t3'}, 'Location', 'bestoutside');
save_figure(fig, cfg, 'fig_09_decision_variables.png');
end

function plot_methods(outs, field, cfg)
C = {cfg.color.base, cfg.color.ga, cfg.color.pso, cfg.color.de};
LW = [cfg.line.base, cfg.line.ga, cfg.line.pso, cfg.line.de];
for i = 1:numel(outs)
    out = outs{i};
    plot(out.t, out.(field), 'Color', C{i}, 'LineWidth', LW(i));
    hold on;
end
end

function format_axes(xlab, ylab, varargin)
grid on;
box on;
xlabel(xlab);
ylabel(ylab);
end

function save_figure(fig, cfg, filename)
file = fullfile(cfg.output_dir, filename);
if isfield(cfg, 'plot') && isfield(cfg.plot, 'dpi')
    dpi = cfg.plot.dpi;
else
    dpi = 300;
end
try
    exportgraphics(fig, file, 'Resolution', dpi);
catch
    saveas(fig, file);
end
if isfield(cfg, 'plot') && isfield(cfg.plot, 'close_after_save') && cfg.plot.close_after_save
    close(fig);
end
end

function plot_response_difference(outs, methods, cfg)
%PLOT_RESPONSE_DIFFERENCE 绘制优化方案相对优化前的差值曲线。
% 修改版：4 个差值响应分别保存为独立图片；图内不设置标题。
base = outs{1};
fields = {'n_rpm', 'q_m3s', 'casing_pressure_head_m', 'tail_pressure_head_m'};
ylabels = {'转速差值 / (r/min)', '流量差值 / (m^3/s)', '蜗壳压力差值 / m', '尾水压力差值 / m'};
fig_names = {'fig_11a_difference_speed','fig_11b_difference_flow', ...
    'fig_11c_difference_casing_pressure','fig_11d_difference_tail_pressure'};
C = {cfg.color.ga, cfg.color.pso, cfg.color.de};
LW = [cfg.line.ga, cfg.line.pso, cfg.line.de];

for k = 1:numel(fields)
    fig = figure('Name', fig_names{k}, 'Color', 'w', 'Position', [100 100 900 560], 'Visible', cfg.plot.visible);
    for i = 2:numel(outs)
        plot(base.t, outs{i}.(fields{k}) - base.(fields{k}), 'LineWidth', LW(i-1), 'Color', C{i-1});
        hold on;
    end
    yline(0, ':', '优化前', 'LineWidth', 1.0);
    format_axes('时间 / s', ylabels{k});
    legend(methods(2:end), 'Location', 'best');
    save_figure(fig, cfg, [fig_names{k} '.png']);
end
end

function safe_writetable(T, cfg, filename)
%SAFE_WRITETABLE 避免 CSV 被 Excel 或 MATLAB 占用时导致整个脚本中断。
file = fullfile(cfg.output_dir, filename);
try
    writetable(T, file);
catch ME
    [~, base, ext] = fileparts(filename);
    alt_name = [base '_' datestr(now, 'yyyymmdd_HHMMSS') ext];
    alt_file = fullfile(cfg.output_dir, alt_name);
    warning('写入 %s 失败：%s。已改写为 %s。', filename, ME.message, alt_name);
    writetable(T, alt_file);
end
end

function T = make_pso_final_table(T_metrics, T_x, T_improve, T_recommend)
%MAKE_PSO_FINAL_TABLE 汇总论文最终推荐的 PSO 方案。
idx = find(strcmp(T_metrics.method, 'PSO'), 1, 'first');
if isempty(idx)
    T = table();
    return;
end
method = T_metrics.method(idx);
J = T_metrics.J(idx);
t_start_s = T_metrics.t_start_s(idx);
overshoot_pu = T_metrics.overshoot_pu(idx);
final_error_pu = T_metrics.final_error_pu(idx);
speed_iae = T_metrics.speed_iae(idx);
hc_max_m = T_metrics.hc_max_m(idx);
ht_range_m = T_metrics.ht_range_m(idx);
guide_rate_rms = T_metrics.guide_rate_rms(idx);
Kp = T_x.Kp(idx);
Ki = T_x.Ki(idx);
Kd = T_x.Kd(idx);
Y1 = T_x.Y1(idx);
Y2 = T_x.Y2(idx);
t1 = T_x.t1(idx);
t2 = T_x.t2(idx);
t3 = T_x.t3(idx);
J_improve_pct = T_improve.J_improve_pct(idx);
overshoot_improve_pct = T_improve.overshoot_improve_pct(idx);
hc_max_improve_pct = T_improve.hc_max_improve_pct(idx);
ht_range_improve_pct = T_improve.ht_range_improve_pct(idx);
role = T_recommend.role(idx);
T = table(method, J, J_improve_pct, t_start_s, overshoot_pu, overshoot_improve_pct, ...
    final_error_pu, speed_iae, hc_max_m, hc_max_improve_pct, ht_range_m, ht_range_improve_pct, ...
    guide_rate_rms, Kp, Ki, Kd, Y1, Y2, t1, t2, t3, role, ...
    'VariableNames', {'method','J','J_improve_pct','t_start_s','overshoot_pu','overshoot_improve_pct', ...
    'final_error_pu','speed_iae','hc_max_m','hc_max_improve_pct','ht_range_m','ht_range_improve_pct', ...
    'guide_rate_rms','Kp','Ki','Kd','Y1','Y2','t1','t2','t3','role'});
end
