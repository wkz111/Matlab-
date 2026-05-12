function yc = guide_law_piecewise(t, guide)
% 三阶段导叶开机给定规律
%
% 0 ~ t1   ：导叶由 0 线性开启到 Y1
% t1 ~ t2  ：导叶保持在 Y1
% t2 ~ t3  ：导叶由 Y1 线性回落到 Y2
% t >= t3  ：导叶保持在 Y2，后续由 PID 在此基础上微调

Y1 = guide.Y1;
Y2 = guide.Y2;
t1 = guide.t1;
t2 = guide.t2;
t3 = guide.t3;

if t <= 0
    yc = 0.0;
elseif t < t1
    yc = Y1 * t / max(t1, eps);
elseif t < t2
    yc = Y1;
elseif t < t3
    yc = Y1 + (Y2 - Y1) * (t - t2) / max(t3 - t2, eps);
else
    yc = Y2;
end

yc = max(min(yc, 1.0), 0.0);

end
