function metrics = calc_startup_metrics(out, p)
%CALC_STARTUP_METRICS 计算开机过程动态品质指标
%
% 重要口径修正：
%   对开机过程而言，0~t3 阶段目标不是“立刻达到额定转速”，因此若从 0 s
%   起对 |n-1| 积分，会把正常升速过程错误地当成控制误差。 将 speed_iae
%   定义为 PID 接入后稳定调节段速度误差积分；同时保留 speed_iae_all 作为
%   全过程诊断量。

t = out.t(:);
n = out.n_pu(:);
hc = out.casing_pressure_head_m(:);
ht = out.tail_pressure_head_m(:);
y = out.y_pu(:);

%% 1. 转速稳定完成时间
[t_speed_settle, speed_not_settle] = local_speed_settle_finish_time( ...
    t, n, 1.0, p.sim.settle_band, p.sim.settle_hold, p.guide.t3);

if isnan(t_speed_settle)
    t_speed_settle = p.sim.t_end;
    speed_not_settle = true;
end

%% 2. 启动稳定时间
t_start = t_speed_settle;

%% 3. 转速指标
overshoot_pu = max(n - 1.0);
overshoot_pu = max(overshoot_pu, 0);

final_speed_error_pu = abs(n(end) - 1.0);

idx_pid = t >= p.guide.t3;
if ~any(idx_pid)
    idx_pid = true(size(t));
end

speed_iae_all = trapz(t, abs(n - 1.0));
speed_iae_post_pid = trapz(t(idx_pid), abs(n(idx_pid) - 1.0));
speed_iae = speed_iae_post_pid;

%% 4. 压力指标
hc_max_m = max(hc);
hc_min_m = min(hc);
hc_range_m = hc_max_m - hc_min_m;

ht_max_m = max(ht);
ht_min_m = min(ht);
ht_range_m = ht_max_m - ht_min_m;

hc_mean = mean(hc);
hc_fluct_int = trapz(t, abs(hc - hc_mean)) / max(t(end) - t(1), eps);

%% 5. 导叶动作指标
dt = mean(diff(t));
dy = [0; diff(y)] / max(dt, eps);
guide_rate_rms = sqrt(mean(dy.^2));

%% 6. 基本惩罚项，只用于提示
penalty = 0;

if speed_not_settle
    penalty = penalty + 1e3;
end

if final_speed_error_pu > p.opt.constraint.final_speed_band
    penalty = penalty + p.opt.constraint.penalty_final * ...
        (final_speed_error_pu - p.opt.constraint.final_speed_band)^2;
end

if ht_min_m < p.opt.constraint.tail_min_limit_m
    penalty = penalty + p.opt.constraint.penalty_tail * ...
        (p.opt.constraint.tail_min_limit_m - ht_min_m)^2;
end

if guide_rate_rms > p.opt.constraint.guide_rate_limit
    penalty = penalty + p.opt.constraint.penalty_guide_rate * ...
        (guide_rate_rms - p.opt.constraint.guide_rate_limit)^2;
end

%% 7. 输出结构体
metrics = struct();

metrics.t_speed_settle_s = t_speed_settle;
metrics.t_start_s = t_start;
metrics.is_speed_settled = ~speed_not_settle;

metrics.overshoot_pu = overshoot_pu;
metrics.final_speed_error_pu = final_speed_error_pu;
metrics.speed_iae = speed_iae;
metrics.speed_iae_post_pid = speed_iae_post_pid;
metrics.speed_iae_all = speed_iae_all;

metrics.hc_max_m = hc_max_m;
metrics.hc_min_m = hc_min_m;
metrics.hc_range_m = hc_range_m;
metrics.hc_fluct_int = hc_fluct_int;

metrics.ht_max_m = ht_max_m;
metrics.ht_min_m = ht_min_m;
metrics.ht_range_m = ht_range_m;

metrics.guide_rate_rms = guide_rate_rms;
metrics.penalty = penalty;

end

function [t_finish, not_settle] = local_speed_settle_finish_time(t, y, ref, band, hold_time, start_time)
%LOCAL_SPEED_SETTLE_FINISH_TIME 从 start_time 后判断稳定，并返回保持结束时刻

mask = abs(y - ref) <= band;
t_finish = NaN;
not_settle = true;

idx0 = find(t >= start_time, 1, 'first');
if isempty(idx0)
    return;
end

for i = idx0:numel(t)
    if ~mask(i)
        continue;
    end

    idx_end = find(t >= t(i) + hold_time, 1, 'first');

    if isempty(idx_end)
        break;
    end

    if all(mask(i:idx_end))
        t_finish = t(idx_end);
        not_settle = false;
        return;
    end
end

end
