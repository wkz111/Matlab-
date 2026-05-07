function print_metrics_cn(m)
%PRINT_METRICS_CN 简洁输出开机仿真核心指标

fprintf('启动稳定时间 t_start = %.3f s\n', getv(m, 't_start_s'));
fprintf('最大转速 n_max = %.3f r/min\n', getv(m, 'n_max_rpm'));
fprintf('转速超调量 overshoot = %.6f p.u.\n', getv(m, 'overshoot_base_pu'));
fprintf('稳定段最大转速偏差 speed_dev = %.6f p.u.\n', getv(m, 'speed_dev_pu'));
fprintf('终值转速误差 final_error = %.6f p.u.\n', getv(m, 'final_speed_error_pu'));
fprintf('稳定段速度误差积分 IAE_post = %.4f\n', getv(m, 'speed_iae'));
if isfield(m, 'speed_iae_all')
    fprintf('全过程速度误差积分 IAE_all = %.4f（诊断项，不参与目标函数）\n', getv(m, 'speed_iae_all'));
end
fprintf('蜗壳最大压力水头 hc_max = %.3f m\n', getv(m, 'hc_max_m'));
fprintf('尾水压力波动范围 ht_range = %.3f m\n', getv(m, 'ht_range_m'));
fprintf('尾水最小压力水头 ht_min = %.3f m\n', getv(m, 'ht_min_m'));
fprintf('导叶速率均方根 guide_rate = %.6f p.u./s\n', getv(m, 'guide_rate_rms'));

if isfield(m, 'is_speed_settled')
    if m.is_speed_settled
        fprintf('稳定判据状态：已满足 2%% 稳定带连续保持要求\n');
    else
        fprintf('稳定判据状态：未满足，t_start 被置为仿真结束时刻\n');
    end
end

if isfield(m, 'penalty')
    fprintf('指标提示项 metrics_penalty = %.6f（仅用于诊断，不等同于目标函数惩罚）\n', m.penalty);
end
end

function v = getv(s, name)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = nan;
end
end
