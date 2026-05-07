clc;
close all;

% 快速调参入口：只运行 PSO，不跑 GA/DE，适合普通电脑反复试参数。
cfg = algorithm_config();
cfg.methods = {'PSO'};
rng(cfg.seed);
if ~exist(cfg.output_dir, 'dir')
    mkdir(cfg.output_dir);
end
fprintf('本次 PSO 快速结果输出文件夹：%s%s', cfg.output_dir, newline);

p0 = psu_default_params();
[p, out_pre, m_pre, x_pre] = prepare_objective_params(p0);
[J_pre, ~] = objective_startup(x_pre, p);

fprintf('\n========== PSO 快速检查：设计水头典型工况优化前方案 ==========%s', newline);
fprintf('J_pre = %.6f%s', J_pre, newline);
print_metrics_cn(m_pre);

result_pso = optimize_startup('PSO', p, cfg);
methods = {'优化前'; 'PSO'};
metrics = {m_pre; result_pso.best_metrics};
Js = [J_pre; result_pso.best_J];
T_metrics = make_metric_table_local(methods, Js, metrics);
T_x = make_x_table_local(x_pre, result_pso.best_x);
writetable(T_metrics, fullfile(cfg.output_dir, 'table_pso_metrics_fast.csv'));
writetable(T_x, fullfile(cfg.output_dir, 'table_pso_decision_variables_fast.csv'));

fprintf('\n========== PSO 快速检查结果 ==========%s', newline);
disp(T_metrics);
disp(T_x);
save(fullfile(cfg.output_dir, 'workspace_pso_fast.mat'), 'cfg', 'p', 'x_pre', 'out_pre', 'm_pre', 'J_pre', 'result_pso', 'T_metrics', 'T_x');

function T = make_metric_table_local(methods, Js, metrics)
n = numel(methods);
t_start = zeros(n, 1); overshoot = zeros(n, 1); final_error = zeros(n, 1);
speed_iae = zeros(n, 1); hc_max = zeros(n, 1); ht_range = zeros(n, 1);
ht_min = zeros(n, 1); guide_rate = zeros(n, 1);
for i = 1:n
    t_start(i) = getv_local(metrics{i}, 't_start_s');
    overshoot(i) = getv_local(metrics{i}, 'overshoot_base_pu');
    final_error(i) = getv_local(metrics{i}, 'final_speed_error_pu');
    speed_iae(i) = getv_local(metrics{i}, 'speed_iae');
    hc_max(i) = getv_local(metrics{i}, 'hc_max_m');
    ht_range(i) = getv_local(metrics{i}, 'ht_range_m');
    ht_min(i) = getv_local(metrics{i}, 'ht_min_m');
    guide_rate(i) = getv_local(metrics{i}, 'guide_rate_rms');
end
T = table(methods, Js, t_start, overshoot, final_error, speed_iae, hc_max, ht_range, ht_min, guide_rate, ...
    'VariableNames', {'method','J','t_start_s','overshoot_pu','final_error_pu','speed_iae','hc_max_m','ht_range_m','ht_min_m','guide_rate_rms'});
end

function T = make_x_table_local(x0, x_pso)
method = {'优化前'; 'PSO'};
X = [x0(:).'; x_pso(:).'];
T = table(method, X(:,1), X(:,2), X(:,3), X(:,4), X(:,5), X(:,6), X(:,7), X(:,8), ...
    'VariableNames', {'method','Kp','Ki','Kd','Y1','Y2','t1','t2','t3'});
end

function v = getv_local(s, name)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(s.(name))
    v = s.(name);
else
    v = nan;
end
end
