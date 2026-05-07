function x = repair_decision_vector(x, p)
%REPAIR_DECISION_VECTOR 修复 8 维决策变量的边界和基本工程约束
%
% 约束：
%   1. 所有变量限制在 p.bounds.lb 与 p.bounds.ub 内；
%   2. t1 < t2 < t3，并保持最小时间间隔；
%   3. Y1 >= Y2，避免导叶开度规律异常。

x = x(:).';

lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';

x = max(x, lb);
x = min(x, ub);

%% 导叶开度关系
min_dy = 0.02;
if x(4) < x(5) + min_dy
    mid = 0.5 * (x(4) + x(5));
    x(4) = mid + 0.5 * min_dy;
    x(5) = mid - 0.5 * min_dy;
end
x(4:5) = max(x(4:5), lb(4:5));
x(4:5) = min(x(4:5), ub(4:5));

%% 时间顺序关系
t = sort(x(6:8));
min_dt = 0.5;

t(1) = max(t(1), lb(6));
t(2) = max(t(2), t(1) + min_dt);
t(3) = max(t(3), t(2) + min_dt);

% 如果超过上界，整体向前平移。
if t(3) > ub(8)
    shift = t(3) - ub(8);
    t = t - shift;
end

% 再检查下界和间隔。
t(1) = max(t(1), lb(6));
t(2) = max(t(2), t(1) + min_dt);
t(3) = max(t(3), t(2) + min_dt);

% 分别限制在各自边界。
t(1) = min(max(t(1), lb(6)), ub(6));
t(2) = min(max(t(2), lb(7)), ub(7));
t(3) = min(max(t(3), lb(8)), ub(8));

x(6:8) = t;

x = max(x, lb);
x = min(x, ub);

end
