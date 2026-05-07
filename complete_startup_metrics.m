function m = complete_startup_metrics(out, p, m)
%补充论文评价指标
%
% 补充：
%   n_max_rpm：最大转速；
%   overshoot_base_pu：典型工况口径转速超调量；
%   speed_dev_pu：PID 接入后稳定调节段最大转速偏差。

if nargin < 3 || isempty(m)
    m = struct();
end

if isfield(out, 'n_rpm') && isfield(p, 'nr') && p.nr > 0
    n_rpm = out.n_rpm(:);
    n_pu = n_rpm ./ p.nr;
    m.n_max_rpm = max(n_rpm);
    m.overshoot_base_pu = max(0, m.n_max_rpm / p.nr - 1.0);
    m.overshoot_pu = m.overshoot_base_pu;
else
    n_pu = [];
    m.n_max_rpm = nan;
    m.overshoot_base_pu = nan;
    if ~isfield(m, 'overshoot_pu')
        m.overshoot_pu = nan;
    end
end

if ~isempty(n_pu) && isfield(out, 't')
    t = out.t(:);

    if isfield(p, 'guide') && isfield(p.guide, 't3') && ~isempty(p.guide.t3)
        t_pid = p.guide.t3;
    else
        t_pid = 0;
    end

    idx = t >= t_pid;
    if ~any(idx)
        idx = true(size(t));
    end

    m.speed_dev_pu = max(abs(n_pu(idx) - 1));
    m.post_pid_max_speed_error_pu = m.speed_dev_pu;
else
    m.speed_dev_pu = nan;
    m.post_pid_max_speed_error_pu = nan;
end

end
