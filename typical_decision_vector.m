function x = typical_decision_vector(p)
%TYPICAL_DECISION_VECTOR 从参数结构体中提取典型工况 8 维决策变量
%
% x = [Kp, Ki, Kd, Y1, Y2, t1, t2, t3]

x = [p.pid.Kp, p.pid.Ki, p.pid.Kd, ...
     p.guide.Y1, p.guide.Y2, p.guide.t1, p.guide.t2, p.guide.t3];

x = repair_decision_vector(x, p);

end
