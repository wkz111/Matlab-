function [p, out_base, m_base, x_base] = prepare_objective_params(p)
%PREPARE_OBJECTIVE_PARAMS 以典型工况仿真结果设置目标函数归一化参考值
%
% 用法：
%   p = psu_default_params();
%   [p, out_base, m_base, x_base] = prepare_objective_params(p);
%
% 说明：
%   1. 归一化参考值采用当前典型工况值，因此典型工况的 J_raw 应约为 1；
%   2. 绝对约束只作为安全边界，不用于惩罚典型工况本身；
%   3. 压力、尾水波动和导叶动作若明显劣于优化前，由 baseline_guard 负责约束。

if nargin < 1 || isempty(p)
    p = psu_default_params();
end

x_base = typical_decision_vector(p);
p_base = apply_decision_vector(x_base, p);

out_base = simulate_startup_moc(p_base);
m_base = calc_startup_metrics(out_base, p_base);
m_base = complete_startup_metrics(out_base, p_base, m_base);

%% 1. 归一化参考值采用典型工况值，避免目标函数尺度失衡
p.opt.ref.t_start_s = max(m_base.t_start_s, eps);
p.opt.ref.overshoot_pu = max(m_base.overshoot_base_pu, 0.001);
p.opt.ref.final_error_pu = max(m_base.final_speed_error_pu, 0.001);
p.opt.ref.iae_speed = max(m_base.speed_iae, eps);
p.opt.ref.hc_max_m = max(m_base.hc_max_m, eps);
p.opt.ref.ht_range_m = max(m_base.ht_range_m, eps);
p.opt.ref.guide_rate_rms = max(m_base.guide_rate_rms, eps);

%% 2. 典型工况保护项需要的参考指标
p.opt.baseline.t_start_s = m_base.t_start_s;
p.opt.baseline.overshoot_base_pu = m_base.overshoot_base_pu;
p.opt.baseline.final_speed_error_pu = m_base.final_speed_error_pu;
p.opt.baseline.speed_iae = m_base.speed_iae;
p.opt.baseline.hc_max_m = m_base.hc_max_m;
p.opt.baseline.ht_range_m = m_base.ht_range_m;
p.opt.baseline.ht_min_m = m_base.ht_min_m;
p.opt.baseline.guide_rate_rms = m_base.guide_rate_rms;

p.opt.baseline_x = x_base;

%% 3. 自动校准安全约束，使典型工况本身不被硬惩罚
% final_error、hc、ht_range、guide_rate 已经作为目标函数分项参与优化，
% 这里的约束只用于阻止严重越界，不应把典型工况判成不可接受。
if isfield(p.opt, 'constraint')
    p.opt.constraint.final_speed_band = max(p.opt.constraint.final_speed_band, 1.05 * m_base.final_speed_error_pu);
    p.opt.constraint.hc_max_limit_m = max(p.opt.constraint.hc_max_limit_m, 1.02 * m_base.hc_max_m);
    p.opt.constraint.ht_range_limit_m = max(p.opt.constraint.ht_range_limit_m, 1.05 * m_base.ht_range_m);
    p.opt.constraint.guide_rate_limit = max(p.opt.constraint.guide_rate_limit, 1.08 * m_base.guide_rate_rms);
end

end
