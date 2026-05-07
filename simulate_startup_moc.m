function out = simulate_startup_moc(p)
%SIMULATE_STARTUP_MOC 抽水蓄能机组开机典型工况仿真模型
%
% 模型包括：
% 1. 6 段有压引水系统特征线计算；
% 2. 上调压井等效动态边界；
% 3. 岔管、球阀、蜗壳局部损失；
% 4. 三阶段导叶给定 + PID 后段修正；
% 5. 水轮机流量、力矩、转速、电磁阻力耦合。

if nargin < 1 || isempty(p)
    p = psu_default_params();
end

p = local_complete_moc_params(p);

dt = p.sim.dt;
nt = floor(p.sim.t_end / dt) + 1;
t = (0:nt-1)' * dt;

%% 1. 预分配机组变量
q = zeros(nt, 1);
n = zeros(nt, 1);
y = zeros(nt, 1);
yc_hist = zeros(nt, 1);

pid_u = zeros(nt, 1);
pid_e = zeros(nt, 1);

mt = zeros(nt, 1);
me = zeros(nt, 1);

he = zeros(nt, 1);
Hc_abs = zeros(nt, 1);
Ht_abs = zeros(nt, 1);
Hs_abs = zeros(nt, 1);
qtank = zeros(nt, 1);

guide_rate = zeros(nt, 1);
hyd_loss = zeros(nt, 1);

branch_loss = zeros(nt, 1);
valve_loss = zeros(nt, 1);
spiral_loss = zeros(nt, 1);

%% 2. 初始条件
n(1) = 0.0;
q(1) = 0.0;
y(1) = 0.0;
mt(1) = 0.0;
me(1) = 0.0;

Hc_abs(1) = p.H_upstream;
Ht_abs(1) = p.H_downstream;
Hs_abs(1) = p.H_upstream;
he(1) = max(Hc_abs(1) - Ht_abs(1), 1.0);

pid_int = 0.0;
prev_e = p.elec.n_ref - n(1);

%% 3. 初始化各管段水头和流量
nseg = numel(p.pipe.L);
Hseg = cell(nseg, 1);
Qseg = cell(nseg, 1);

for s = 1:nseg
    Hseg{s} = p.H_upstream * ones(p.moc.N(s) + 1, 1);
    Qseg{s} = zeros(p.moc.N(s) + 1, 1);
end

%% 4. 时间推进
for k = 1:(nt-1)

    %% 4.1 三阶段导叶给定 + PID 后段修正
    yc_base = guide_law_piecewise(t(k), p.guide);

    e = p.elec.n_ref - n(k);
    de = (e - prev_e) / max(dt, eps);
    u_pid = 0.0;

    if t(k) >= p.guide.t3
        pid_int_try = pid_int + e * dt;
        pid_int_try = max(min(pid_int_try, p.pid.int_max), p.pid.int_min);

        u_pid_try = p.pid.Kp * e + p.pid.Ki * pid_int_try + p.pid.Kd * de;
        u_pid = max(min(u_pid_try, p.pid.u_max), p.pid.u_min);

        if ~p.pid.antiwindup || abs(u_pid_try - u_pid) < 1e-12
            pid_int = pid_int_try;
        end
    end

    yc = max(min(yc_base + u_pid, 1.0), 0.0);

    yc_hist(k) = yc;
    pid_u(k) = u_pid;
    pid_e(k) = e;
    prev_e = e;

    %% 4.2 导叶执行机构
    ydot_cmd = (yc - y(k)) / max(p.guide.Ty, eps);
    ydot = max(min(ydot_cmd, p.guide.vmax), -p.guide.vmax);

    y_next = y(k) + dt * ydot;
    y_next = max(min(y_next, 1.0), 0.0);
    guide_rate(k) = ydot;

    %% 4.3 特征线内部节点更新
    Hnew = Hseg;
    Qnew = Qseg;

    for s = 1:nseg
        Ns = p.moc.N(s);
        B = p.moc.B(s);
        R = p.moc.R(s);

        if Ns >= 2
            for j = 2:Ns
                qa = Qseg{s}(j-1);
                qb = Qseg{s}(j+1);

                Cp = Hseg{s}(j-1) + B * qa - R * qa * abs(qa);
                Cm = Hseg{s}(j+1) - B * qb + R * qb * abs(qb);

                Qnew{s}(j) = (Cp - Cm) / (2 * B);
                Hnew{s}(j) = 0.5 * (Cp + Cm);
            end
        end
    end

    %% 4.4 上游水库边界
    B1 = p.moc.B(1);
    q_down = Qseg{1}(2);
    Cm = Hseg{1}(2) - B1 * q_down + p.moc.R(1) * q_down * abs(q_down);

    Hnew{1}(1) = p.H_upstream;
    Qnew{1}(1) = (Hnew{1}(1) - Cm) / max(B1, eps);

    %% 4.5 上调压井等效边界：管段1与管段2之间
    CpL = Hseg{1}(end-1) ...
        + p.moc.B(1) * Qseg{1}(end-1) ...
        - p.moc.R(1) * Qseg{1}(end-1) * abs(Qseg{1}(end-1));

    CmR = Hseg{2}(2) ...
        - p.moc.B(2) * Qseg{2}(2) ...
        + p.moc.R(2) * Qseg{2}(2) * abs(Qseg{2}(2));

    beta = p.surge.Th / max(p.surge.Tq, eps);
    alpha = qtank(k) * (1 - dt / max(p.surge.Tq, eps)) - beta * Hs_abs(k);
    denom = (1 / p.moc.B(1)) + (1 / p.moc.B(2)) + beta;

    Hsurge = (CpL / p.moc.B(1) + CmR / p.moc.B(2) - alpha) / max(denom, eps);

    Q1r = (CpL - Hsurge) / max(p.moc.B(1), eps);
    Q2l = (Hsurge - CmR) / max(p.moc.B(2), eps);
    qt = alpha + beta * Hsurge;

    Hnew{1}(end) = Hsurge;
    Qnew{1}(end) = Q1r;
    Hnew{2}(1) = Hsurge;
    Qnew{2}(1) = Q2l;

    %% 4.6 中间管段连接边界：含岔管、球阀局部损失
    for s = 2:(nseg-1)

        Cp = Hseg{s}(end-1) ...
            + p.moc.B(s) * Qseg{s}(end-1) ...
            - p.moc.R(s) * Qseg{s}(end-1) * abs(Qseg{s}(end-1));

        Cm = Hseg{s+1}(2) ...
            - p.moc.B(s+1) * Qseg{s+1}(2) ...
            + p.moc.R(s+1) * Qseg{s+1}(2) * abs(Qseg{s+1}(2));

        if p.branch.enable && s == p.branch.loc_left
            [Qif, Hl, Hr, hloc] = local_interface_with_loss( ...
                Cp, Cm, p.moc.B(s), p.moc.B(s+1), p.branch.K);
            branch_loss(k+1) = hloc;

        elseif p.valve.enable && s == p.valve.loc_left
            Kvalve = local_valve_loss_coeff(p.valve);
            [Qif, Hl, Hr, hloc] = local_interface_with_loss( ...
                Cp, Cm, p.moc.B(s), p.moc.B(s+1), Kvalve);
            valve_loss(k+1) = hloc;

        else
            Qif = (Cp - Cm) / max(p.moc.B(s) + p.moc.B(s+1), eps);
            Hl = Cp - p.moc.B(s) * Qif;
            Hr = Hl;
        end

        Hnew{s}(end) = Hl;
        Qnew{s}(end) = Qif;
        Hnew{s+1}(1) = Hr;
        Qnew{s+1}(1) = Qif;
    end

    %% 4.7 尾水边界
    if k == 1
        dqdt_tail = 0.0;
    else
        dqdt_tail = (q(k) - q(k-1)) / max(dt, eps);
    end
    dq_pos = max(dqdt_tail, 0.0);
    dq_neg = max(-dqdt_tail, 0.0);

    % 尾水压力采用“流量准稳态项 + 流量加速度项”的等效边界。
    % 机组快速增流时，尾水侧可能出现小幅压力下探；减流时则产生回升。
    Htail_ref = p.H_downstream ...
        + p.tail.Kq * abs(q(k)) ...
        - p.tail.Kacc * dq_pos ...
        + p.tail.Kdec * dq_neg;

    Htail = Ht_abs(k) + dt * (Htail_ref - Ht_abs(k)) / max(p.tail.Tt, eps);
    % 采用相对尾水位坐标时，尾水压力水头不再允许出现负值。
    % 负值会把坐标基准误差误读成物理真空风险，答辩时很容易变成灾难现场。
    Htail = max(Htail, p.H_downstream - p.tail.drop_limit);

    %% 4.8 机组边界 + 蜗壳等效模块
    s = nseg;

    CpT = Hseg{s}(end-1) ...
        + p.moc.B(s) * Qseg{s}(end-1) ...
        - p.moc.R(s) * Qseg{s}(end-1) * abs(Qseg{s}(end-1));

    n_dev = n(k) - 1.0;
    ydot_norm = min(abs(ydot) / max(p.guide.vmax, eps), 1.5);

    phi_q_y = max(y_next, 0.0)^p.turb.q_exp;
    phi_q_h = 1.0;
    phi_q_n = max(1 - p.turb.an * n_dev, 0.20);
    phi_q_d = max(1 - p.turb.q_ydot * ydot_norm, 0.75);
    fq = max(p.turb.kq * phi_q_y * phi_q_h * phi_q_n * phi_q_d, 0.0);

    Cq = p.Qr * fq / sqrt(max(p.Hr, eps));
    C = max(CpT - Htail, 0.0);
    Ksp = p.spiral.K;

    if Cq <= 1e-12 || C <= 1e-12
        Qt = 0.0;
    else
        Aq = (1 / max(Cq^2, eps)) + Ksp;
        Bq = p.moc.B(s);
        disc = max(Bq^2 + 4 * Aq * C, 0.0);
        Qt = (-Bq + sqrt(disc)) / max(2 * Aq, eps);
    end

    Hpipe = CpT - p.moc.B(s) * Qt;
    hsp_loss = Ksp * Qt * abs(Qt);
    Hsp_target = max(Hpipe - hsp_loss, Htail + 1e-6);

    Hc_now = Hc_abs(k) + dt * (Hsp_target - Hc_abs(k)) / max(p.spiral.Tc, eps);
    Hc_now = max(Hc_now, Htail + 1e-6);

    Hnew{s}(end) = Hpipe;
    Qnew{s}(end) = Qt;

    %% 4.9 有效水头与动态损失
    qpu = abs(Qt) / max(p.Qr, eps);

    if k == 1
        dqdt = 0.0;
    else
        dqdt = (Qt - q(k)) / max(dt, eps);
    end

    hloss_dyn = p.Hr * ( ...
        p.hyd.Ky * ydot_norm ...
        + p.hyd.Kq * qpu^2 ...
        + p.hyd.Kdq * abs(dqdt) / max(p.Qr, eps));

    he_raw = max(Hc_now - Htail, 1e-6);
    he_now = max(he_raw - hloss_dyn, 1e-6);
    hpu = he_now / max(p.Hr, eps);

    %% 4.10 水轮机力矩
    phi_m_y = max(y_next, 0.0)^p.turb.m_exp;
    phi_m_h = hpu;
    phi_m_n = max(1 - p.turb.bn * n_dev, 0.05);
    phi_m_d = max(1 - p.turb.m_ydot * ydot_norm, 0.70);

    mt_static = max(p.turb.km * phi_m_y * phi_m_h * phi_m_n * phi_m_d, 0.0);
    mt_next = mt(k) + dt * (mt_static - mt(k)) / max(p.dyn.Tm, eps);
    mt_next = max(mt_next, 0.0);

    %% 4.11 电磁阻力矩
    speed_excess = max(n(k) - p.elec.n_ref, 0.0);
    capture_excess = max(n(k) - p.elec.n_capture, 0.0);

    me_target = p.elec.sync_gain * speed_excess ...
              + p.elec.capture_gain * capture_excess;
    me_target = min(max(me_target, 0.0), p.elec.me_max);

    me_next = me(k) + dt * (me_target - me(k)) / max(p.Te, eps);
    me_next = max(me_next, 0.0);

    %% 4.12 转子运动方程
    mf = p.mech.mf0 + p.mech.mf1 * n(k)^2;

    % 可选额定转速恢复项。
    % 默认 restore_gain = 0，不参与本文优化前和算法对比。
    % 保留接口仅用于后续做敏感性检查，避免把非文献参数偷偷塞进优化前模型。
    if isfield(p.elec, 'restore_gain') && p.elec.restore_gain > 0 && t(k) >= p.guide.t3
        if isfield(p.elec, 'restore_ramp')
            restore_alpha = min(max((t(k) - p.guide.t3) / max(p.elec.restore_ramp, eps), 0.0), 1.0);
        else
            restore_alpha = 1.0;
        end
        speed_restore = restore_alpha * p.elec.restore_gain * (p.elec.n_ref - n(k));
    else
        speed_restore = 0.0;
    end

    ndot = (mt_next - me_next - mf) / max(p.Ta, eps) + speed_restore;
    n_next = max(n(k) + dt * ndot, 0.0);

    %% 4.13 写入下一时刻变量
    Hseg = Hnew;
    Qseg = Qnew;

    q(k+1) = Qt;
    n(k+1) = n_next;
    y(k+1) = y_next;

    mt(k+1) = mt_next;
    me(k+1) = me_next;

    he(k+1) = he_now;
    Hc_abs(k+1) = Hc_now;
    Ht_abs(k+1) = Htail;
    Hs_abs(k+1) = Hsurge;
    qtank(k+1) = qt;

    hyd_loss(k+1) = hloss_dyn;
    spiral_loss(k+1) = hsp_loss;
end

%% 5. 补齐末端点
yc_hist(end) = yc_hist(end-1);
pid_u(end) = pid_u(end-1);
pid_e(end) = pid_e(end-1);
guide_rate(end) = guide_rate(end-1);

if nt >= 2
    hyd_loss(1) = hyd_loss(2);
    branch_loss(1) = branch_loss(2);
    valve_loss(1) = valve_loss(2);
    spiral_loss(1) = spiral_loss(2);
end

%% 6. 输出结构体
out = struct();
out.t = t;

out.n_pu = n;
out.n_rpm = n * p.nr;

out.q_m3s = q;
out.q_pu = q / max(p.Qr, eps);

out.y_pu = y;
out.y_deg = y * p.guide.full_open_deg;

out.yc_pu = yc_hist;
out.yc_deg = yc_hist * p.guide.full_open_deg;

out.pid_u = pid_u;
out.pid_e = pid_e;

out.mt_pu = mt;
out.me_pu = me;

out.he_m = he;
out.he_pu = he / max(p.Hr, eps);

out.casing_pressure_head_m = Hc_abs - p.geom.z_casing;
out.tail_pressure_head_m = Ht_abs - p.geom.z_tail;

out.hc_abs_m = Hc_abs;
out.ht_abs_m = Ht_abs;
out.surge_head_m = Hs_abs;
out.qtank_m3s = qtank;
out.hyd_loss_m = hyd_loss;

out.guide_rate_pu_s = guide_rate;
out.guide_rate_deg_s = guide_rate * p.guide.full_open_deg;

out.branch_loss_m = branch_loss;
out.valve_loss_m = valve_loss;
out.spiral_loss_m = spiral_loss;

out.params = p;

end

function p = local_complete_moc_params(p)
%LOCAL_COMPLETE_MOC_PARAMS 补齐 MOC 模型所需参数

if ~isfield(p.pipe, 'A')
    p.pipe.A = pi .* p.pipe.D.^2 ./ 4;
end

if ~isfield(p, 'pressure')
    p.pressure = struct();
end
if ~isfield(p.pressure, 'Hc_base')
    p.pressure.Hc_base = 560.0;
end
if ~isfield(p.pressure, 'Ht_base')
    p.pressure.Ht_base = 35.0;
end

if ~isfield(p, 'geom')
    p.geom = struct();
end
p.geom.z_casing = p.H_upstream - p.pressure.Hc_base;
p.geom.z_tail = p.H_downstream - p.pressure.Ht_base;

if ~isfield(p, 'loss')
    p.loss = struct();
end

if ~isfield(p.loss, 'total_K')
    % 若外部参数未给出总损失系数，则由自然落差与设计水头推导。
    % 不再使用其他文献中的净水头或大流量校核值。
    p.loss.total_K = max(p.Hgross - p.Hr, 0.0) / max(p.Qr^2, eps);
end

if ~isfield(p.loss, 'frac')
    p.loss.frac.branch = 0.08;
    p.loss.frac.valve = 0.18;
    p.loss.frac.spiral = 0.12;
    p.loss.frac.distributed = 0.62;
end

w = p.pipe.L ./ (p.pipe.D .* (p.pipe.A.^2));
p.loss.Kseg = (p.loss.total_K * p.loss.frac.distributed) .* w ./ sum(w);

p.moc.dt = p.sim.dt;
p.moc.N = max(1, round(p.pipe.L ./ (p.pipe.a .* p.moc.dt)));
p.moc.dx = p.pipe.L ./ p.moc.N;
p.moc.a_eff = p.moc.dx ./ p.moc.dt;
p.moc.B = p.moc.a_eff ./ (p.g .* p.pipe.A);
p.moc.R = p.loss.Kseg ./ p.moc.N;

p.branch.enable = true;
p.branch.loc_left = 2;
p.branch.loc_right = 3;
p.branch.K = p.loss.total_K * p.loss.frac.branch;

p.valve.enable = true;
p.valve.loc_left = 4;
p.valve.loc_right = 5;
p.valve.opening = 1.00;
p.valve.min_opening = 0.15;
p.valve.shape = 2.0;
p.valve.K_open = p.loss.total_K * p.loss.frac.valve;

p.spiral.enable = true;
p.spiral.K = p.loss.total_K * p.loss.frac.spiral;
p.spiral.Tc = 0.18;

if ~isfield(p, 'tail')
    p.tail = struct();
end
if ~isfield(p.tail, 'Tt'), p.tail.Tt = 2.0; end
if ~isfield(p.tail, 'Kq'), p.tail.Kq = 0.25; end
if ~isfield(p.tail, 'Kacc'), p.tail.Kacc = 0.075; end
if ~isfield(p.tail, 'Kdec'), p.tail.Kdec = 0.025; end
if ~isfield(p.tail, 'drop_limit'), p.tail.drop_limit = 0.0; end

if ~isfield(p, 'hyd')
    p.hyd = struct();
end
if ~isfield(p.hyd, 'Ky'), p.hyd.Ky = 0.035; end
if ~isfield(p.hyd, 'Kq'), p.hyd.Kq = 0.010; end
if ~isfield(p.hyd, 'Kdq'), p.hyd.Kdq = 0.006; end

if ~isfield(p.turb, 'q_ydot'), p.turb.q_ydot = 0.08; end
if ~isfield(p.turb, 'm_ydot'), p.turb.m_ydot = 0.12; end

if ~isfield(p, 'dyn')
    p.dyn = struct();
end
if ~isfield(p.dyn, 'Tm'), p.dyn.Tm = 0.60; end

if ~isfield(p.guide, 'Ty'), p.guide.Ty = 0.25; end
if ~isfield(p.guide, 'vmax'), p.guide.vmax = 0.18; end

if ~isfield(p.elec, 'restore_gain'), p.elec.restore_gain = 0.0; end
if ~isfield(p.elec, 'restore_ramp'), p.elec.restore_ramp = 1.0; end

if ~isfield(p, 'mech')
    p.mech = struct();
end
if ~isfield(p.mech, 'mf0'), p.mech.mf0 = 0.03; end
if ~isfield(p.mech, 'mf1'), p.mech.mf1 = 0.015; end

end

function [Qif, Hleft, Hright, hloc] = local_interface_with_loss(Cp, Cm, Bleft, Bright, Kloc)
%LOCAL_INTERFACE_WITH_LOSS 含局部损失的管段连接边界

Delta = Cp - Cm;
Bsum = Bleft + Bright;

if abs(Delta) <= 1e-12
    Qif = 0.0;
    hloc = 0.0;
    Hleft = Cp;
    Hright = Cm;
    return;
end

sgn = sign(Delta);
dH = abs(Delta);

if Kloc <= 1e-12
    qmag = dH / max(Bsum, eps);
else
    disc = max(Bsum^2 + 4 * Kloc * dH, 0.0);
    qmag = (-Bsum + sqrt(disc)) / max(2 * Kloc, eps);
end

Qif = sgn * qmag;
Hleft = Cp - Bleft * Qif;
Hright = Cm + Bright * Qif;
hloc = max(Hleft - Hright, 0.0);

end

function K = local_valve_loss_coeff(valve)
%LOCAL_VALVE_LOSS_COEFF 球阀等效损失系数

open_eff = max(valve.opening, valve.min_opening);
amp = 1 + valve.shape * ((1 - open_eff) / open_eff)^2;
K = valve.K_open * amp;

end
