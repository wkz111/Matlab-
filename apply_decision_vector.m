function p2 = apply_decision_vector(x, p)
%APPLY_DECISION_VECTOR 将 8 维决策变量写入参数结构体
%
% x = [Kp, Ki, Kd, Y1, Y2, t1, t2, t3]

p2 = p;
x = x(:).';

p2.pid.Kp = x(1);
p2.pid.Ki = x(2);
p2.pid.Kd = x(3);

p2.guide.Y1 = x(4);
p2.guide.Y2 = x(5);
p2.guide.t1 = x(6);
p2.guide.t2 = x(7);
p2.guide.t3 = x(8);

end
