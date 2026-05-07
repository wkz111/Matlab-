function [J, metrics] = objective_startup(x, p)
%OBJECTIVE_STARTUP 抽水蓄能机组开机过程综合目标函数
%
% 决策变量：
%   x = [Kp, Ki, Kd, Y1, Y2, t1, t2, t3]
%
% 目标函数由 7 个核心指标归一化加权得到：
%   启动稳定时间、典型工况口径转速超调量、终值转速误差、速度误差积分、
%   蜗壳最大压力水头、尾水压力波动范围、导叶速率均方根。
%
% 计算要点：
%   1. 加强启动时间约束，避免优化后启动时间反而变长；
%   2. 增加工程偏好软约束，避免 Kp、Ki、Y2、t1、t3 等变量贴边；
%   3. 保留压力和导叶动作保护，使优化结果仍满足工程解释性。

if nargin < 2 || isempty(p)
    p = psu_default_params();
    p = prepare_objective_params(p);
end

try
    x = repair_decision_vector(x, p);
    is_baseline_x = false;
    if isfield(p, 'opt') && isfield(p.opt, 'baseline_x')
        is_baseline_x = max(abs(x(:).' - p.opt.baseline_x(:).')) < 1e-10;
    end
    p2 = apply_decision_vector(x, p);

    out = simulate_startup_moc(p2);
    metrics = calc_startup_metrics(out, p2);
    metrics = complete_startup_metrics(out, p2, metrics);

    w = p2.opt.weight;
    ref = p2.opt.ref;

    J_t_start = safe_div(metrics.t_start_s, ref.t_start_s);
    J_overshoot = safe_div(metrics.overshoot_base_pu, ref.overshoot_pu);
    J_final_error = safe_div(metrics.final_speed_error_pu, ref.final_error_pu);
    J_iae = safe_div(metrics.speed_iae, ref.iae_speed);
    J_hc = safe_div(metrics.hc_max_m, ref.hc_max_m);
    J_ht = safe_div(metrics.ht_range_m, ref.ht_range_m);
    J_guide = safe_div(metrics.guide_rate_rms, ref.guide_rate_rms);

    J0 = ...
        w.t_start     * J_t_start + ...
        w.overshoot   * J_overshoot + ...
        w.final_error * J_final_error + ...
        w.iae_speed   * J_iae + ...
        w.hc_max      * J_hc + ...
        w.ht_range    * J_ht + ...
        w.guide_rate  * J_guide;

    penalty_abs = absolute_constraint_penalty(metrics, p2);
    penalty_guard = baseline_guard_penalty(metrics, p2);
    penalty_edge = boundary_penalty(x, p2);
    penalty_pref = engineering_preference_penalty(x, p2);

    if is_baseline_x
        penalty_guard = 0;
        penalty_edge = 0;
        penalty_pref = 0;
    end

    penalty = penalty_abs + penalty_guard + penalty_edge + penalty_pref;
    J = J0 + penalty;

    metrics.x = x;
    metrics.objective = J;
    metrics.objective_raw = J0;
    metrics.penalty = penalty;
    metrics.penalty_abs = penalty_abs;
    metrics.penalty_guard = penalty_guard;
    metrics.penalty_edge = penalty_edge;
    metrics.penalty_pref = penalty_pref;

    metrics.obj_terms = struct( ...
        't_start', J_t_start, ...
        'overshoot', J_overshoot, ...
        'final_error', J_final_error, ...
        'speed_iae', J_iae, ...
        'hc_max', J_hc, ...
        'ht_range', J_ht, ...
        'guide_rate', J_guide);

catch ME
    J = inf;
    metrics = struct();
    metrics.error_message = ME.message;
    metrics.x = x;
end

end

function y = safe_div(a, b)
y = a / max(abs(b), eps);
end

function v = get_metric(s, name, default_value)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isfinite(s.(name))
    v = s.(name);
else
    v = default_value;
end
end

function v = get_field(s, name, default_value)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default_value;
end
end

function penalty = absolute_constraint_penalty(m, p)
%ABSOLUTE_CONSTRAINT_PENALTY 绝对安全约束惩罚
penalty = 0;

if ~isfield(p, 'opt') || ~isfield(p.opt, 'constraint')
    return;
end

c = p.opt.constraint;

final_error = get_metric(m, 'final_speed_error_pu', 0);
excess = max(0, final_error - c.final_speed_band);
penalty = penalty + c.penalty_final * (excess / max(c.final_speed_band, eps))^2;

hc_max = get_metric(m, 'hc_max_m', 0);
excess = max(0, hc_max - c.hc_max_limit_m);
penalty = penalty + c.penalty_hc * (excess / max(1.0, 0.01 * c.hc_max_limit_m))^2;

ht_range = get_metric(m, 'ht_range_m', 0);
excess = max(0, ht_range - c.ht_range_limit_m);
penalty = penalty + c.penalty_ht_range * (excess / max(0.10, 0.01 * c.ht_range_limit_m))^2;

ht_min = get_metric(m, 'ht_min_m', nan);
if ~isnan(ht_min)
    excess = max(0, c.tail_min_limit_m - ht_min);
    penalty = penalty + c.penalty_tail * (excess / max(0.10, 0.01 * c.tail_min_limit_m))^2;
end

guide_rate = get_metric(m, 'guide_rate_rms', 0);
excess = max(0, guide_rate - c.guide_rate_limit);
penalty = penalty + c.penalty_guide_rate * (excess / max(0.001, 0.01 * c.guide_rate_limit))^2;

end

function penalty = baseline_guard_penalty(m, p)
%BASE_REFERENCE_GUARD_PENALTY 典型工况保护项
penalty = 0;

if ~isfield(p.opt, 'constraint') || ~isfield(p.opt.constraint, 'use_baseline_guard') || ...
        ~p.opt.constraint.use_baseline_guard || ~isfield(p.opt, 'baseline')
    return;
end

c = p.opt.constraint;
b = p.opt.baseline;
W = c.baseline_penalty;

% 启动稳定时间原则上不应劣于优化前，避免算法以变慢换取低超调或低压力。
if isfield(c, 'baseline_t_start_margin') && isfield(b, 't_start_s')
    lim = b.t_start_s * (1 + c.baseline_t_start_margin);
    excess = max(0, get_metric(m, 't_start_s', 0) - lim);
    penalty = penalty + 1.5 * W * (excess / max(0.10, 0.01 * lim))^2;
end

% 蜗壳压力允许略高于优化前。
lim = b.hc_max_m * (1 + c.baseline_hc_margin);
excess = max(0, get_metric(m, 'hc_max_m', 0) - lim);
penalty = penalty + W * (excess / max(1.0, 0.005 * lim))^2;

% 尾水压力波动允许略高于优化前。
lim = b.ht_range_m * (1 + c.baseline_ht_margin);
excess = max(0, get_metric(m, 'ht_range_m', 0) - lim);
penalty = penalty + W * (excess / max(0.05, 0.005 * lim))^2;

% 导叶动作允许略高于优化前。
lim = b.guide_rate_rms * (1 + c.baseline_guide_margin);
excess = max(0, get_metric(m, 'guide_rate_rms', 0) - lim);
penalty = penalty + W * (excess / max(0.0005, 0.005 * lim))^2;

% 速度误差积分不应明显劣于优化前。否则算法会用"前期慢、后期稳"换低超调，
% 表格好看但物理解释尴尬，答辩老师通常不负责温柔。
if isfield(c, 'baseline_iae_margin') && isfield(b, 'speed_iae')
    lim = b.speed_iae * (1 + c.baseline_iae_margin);
    excess = max(0, get_metric(m, 'speed_iae', 0) - lim);
    penalty = penalty + 2.00 * W * (excess / max(0.05, 0.01 * lim))^2;
end

end

function penalty = engineering_preference_penalty(x, p)
%ENGINEERING_PREFERENCE_PENALTY 工程偏好软约束，避免变量贴边。
penalty = 0;

if ~isfield(p, 'opt') || ~isfield(p.opt, 'pref') || ...
        ~isfield(p.opt.pref, 'enable') || ~p.opt.pref.enable
    return;
end

pref = p.opt.pref;
Kp = x(1);
Ki = x(2);
Kd = x(3);
Y1 = x(4);
Y2 = x(5);
t1 = x(6);
t2 = x(7);
t3 = x(8);

scale = get_field(pref, 'weight', 0.018);
viol = 0;
viol = viol + (max(0, Kp - get_field(pref, 'Kp_soft_max', inf)) / 0.06)^2;
viol = viol + (max(0, Ki - get_field(pref, 'Ki_soft_max', inf)) / 0.008)^2;
viol = viol + (max(0, get_field(pref, 'Kd_soft_min', -inf) - Kd) / 0.006)^2;
viol = viol + (max(0, Kd - get_field(pref, 'Kd_soft_max', inf)) / 0.008)^2;
viol = viol + (max(0, get_field(pref, 'Y1_soft_min', -inf) - Y1) / 0.015)^2;
viol = viol + (max(0, Y1 - get_field(pref, 'Y1_soft_max', inf)) / 0.015)^2;
viol = viol + (max(0, get_field(pref, 'Y2_soft_min', -inf) - Y2) / 0.012)^2;
viol = viol + (max(0, Y2 - get_field(pref, 'Y2_soft_max', inf)) / 0.018)^2;
viol = viol + (max(0, get_field(pref, 't1_soft_min', -inf) - t1) / 0.30)^2;
viol = viol + (max(0, t1 - get_field(pref, 't1_soft_max', inf)) / 0.30)^2;
viol = viol + (max(0, get_field(pref, 't2_soft_min', -inf) - t2) / 0.35)^2;
viol = viol + (max(0, get_field(pref, 't3_soft_min', -inf) - t3) / 0.35)^2;
viol = viol + (max(0, t3 - get_field(pref, 't3_soft_max', inf)) / 0.45)^2;

penalty = scale * viol;
end

function penalty = boundary_penalty(x, p)
%BOUNDARY_PENALTY 轻微边界惩罚，避免优化结果大量贴边。
if isfield(p, 'opt') && isfield(p.opt, 'boundary') && ...
        isfield(p.opt.boundary, 'enable') && ~p.opt.boundary.enable
    penalty = 0;
    return;
end

lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';
z = (x - lb) ./ max(ub - lb, eps);

margin = 0.10;
weight = 0.18;
if isfield(p, 'opt') && isfield(p.opt, 'boundary')
    if isfield(p.opt.boundary, 'margin'), margin = p.opt.boundary.margin; end
    if isfield(p.opt.boundary, 'weight'), weight = p.opt.boundary.weight; end
end

edge = max(0, margin - z) + max(0, z - (1 - margin));
penalty = weight * mean((edge ./ max(margin, eps)).^2);
end
