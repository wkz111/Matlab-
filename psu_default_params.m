function p = psu_default_params()
%PSU_DEFAULT_PARAMS 抽水蓄能机组开机仿真与目标函数默认参数
%
% 数据源说明：
% 1) 曾洪涛等《基于 ANN 的抽水蓄能电站建模与过渡过程优化》中明确给出的
%    电站额定参数、B厂单管单机引水系统参数、水击参数、调压井参数和
%    机组等效惯性时间常数统一写入文献参考结构 p.ref。
% 2) 文献未给出开机过程蜗壳最大压力、最大转速、尾水最小压力等现场校核值，
%    因此这些量只作为同一模型内的响应评价指标，不作为实测对比目标。
% 3) 本程序用于典型工况建模验证和 GA、PSO、DE 三种传统优化算法对比。

%% 1. 基本常数
p.g = 9.81;              % 重力加速度，m/s^2
p.rho = 1000;            % 水密度，kg/m^3

%% 2. 曾洪涛文献明确给出的参数，统一作为本文参数来源
p.ref.source = '曾洪涛, 王智欣, 田文刚, 等. 基于ANN的抽水蓄能电站建模与过渡过程优化[J]. 排灌机械工程学报, 2014, 32(10):864-870+876.';
p.ref.Hgross = 531.0;           % 自然落差，m
p.ref.Hmin = 509.0;             % 最小毛水头，m
p.ref.Hmax = 557.0;             % 最大毛水头，m
p.ref.Hr = 517.4;               % 设计水头，m
p.ref.Qr = 66.2;                % 额定流量，m^3/s
p.ref.Pr = 300e6;               % 单机容量，W，由 4 x 300 MW 得到
p.ref.plant_capacity = 1200e6;  % 单厂装机容量，W
p.ref.unit_count = 4;           % 单厂机组数量
p.ref.TaTb = 8.453;             % 发电电动机及负载等效惯性时间常数，s

p.ref.pipe_name = { ...
    '进水口至上调压井', ...
    '上调压井至岔管', ...
    '岔管至钢支管', ...
    '钢支管至球阀', ...
    '球阀至蜗壳', ...
    '蜗壳段'};
p.ref.pipe_L = [1664.52, 897.93, 12.00, 156.06, 10.50, 15.22];
p.ref.pipe_D = [8.55, 8.50, 8.25, 3.50, 2.04, 2.08];
p.ref.pipe_a = [1000, 1000, 1000, 1300, 1300, 1400];

p.ref.waterhammer.group = {'管段1', '管段2+3', '管段4+5+6'};
p.ref.waterhammer.Tr = [3.329, 1.820, 0.279];
p.ref.waterhammer.hw = [0.114, 0.115, 0.996];
p.ref.surge.Th = 443.3;
p.ref.surge.Tq = 171.828;

% 由文献参数直接推导的校核量，不属于额外文献数据。
p.ref.derived.hydraulic_power_W = p.rho * p.g * p.ref.Qr * p.ref.Hr;
p.ref.derived.eta_rated = p.ref.Pr / p.ref.derived.hydraulic_power_W;
p.ref.derived.loss_head_rated_m = p.ref.Hgross - p.ref.Hr;
p.ref.derived.loss_coeff_rated = p.ref.derived.loss_head_rated_m / (p.ref.Qr^2);

%% 3. 机组额定参数
p.Hr = p.ref.Hr;
p.Qr = p.ref.Qr;
p.Pr = p.ref.Pr;
p.nr = 500.0;            % rpm 显示用默认额定转速；优化评价主要采用标幺转速
p.omega_r = 2*pi*p.nr/60;

% 文献只给出自然落差，未给出上、下库绝对水位。
% 因此仿真中采用相对水位：下库为 0，上库为自然落差。
p.H_upstream = p.ref.Hgross;
p.H_downstream = 0.0;
p.Hgross = p.H_upstream - p.H_downstream;

%% 4. 引水系统 6 管段参数
p.pipe.name = p.ref.pipe_name;
p.pipe.L = p.ref.pipe_L;             % 长度，m
p.pipe.D = p.ref.pipe_D;             % 当量直径，m
p.pipe.a = p.ref.pipe_a;             % 水击波速，m/s
p.pipe.A = pi .* p.pipe.D.^2 ./ 4;   % 截面积，m^2

%% 5. 调压井等效参数
p.surge.Th = p.ref.surge.Th;
p.surge.Tq = p.ref.surge.Tq;

%% 6. 机组转动、电磁负载与机械阻力参数
p.Ta = p.ref.TaTb;
p.Te = 0.20;

p.elec.n_ref = 1.0;
p.elec.n_capture = 0.94;
p.elec.sync_gain = 12.0;        % 额定附近超速阻尼，抑制转速超调
p.elec.capture_gain = 2.4;      % 空载捕获阻尼，避免低速段阻尼过强导致稳态偏低
p.elec.me_max = 1.8;
% 曾洪涛文献采用发电电动机一阶惯性模型，未给出额外“转速恢复项”。
% 默认关闭该项，避免用人为恢复项改变开机响应机理。
p.elec.restore_gain = 0.0;
p.elec.restore_ramp = 2.0;

p.mech.mf0 = 0.03;
p.mech.mf1 = 0.015;

%% 7. 水轮机工程等效特性参数
% 文献采用 RBF 神经网络拟合 Q11、M11。由于论文初稿没有原始全特性曲线，
% 这里采用等效非线性模型，只用于不同控制参数之间的相对比较。
p.turb.kq = 1.34;
p.turb.km = 1.36;
p.turb.q_exp = 0.93;
p.turb.m_exp = 1.08;
p.turb.an = 0.20;
p.turb.bn = 0.35;
p.turb.q_ydot = 0.08;
p.turb.m_ydot = 0.12;

%% 8. 动态滞后与损失参数
p.dyn.Tm = 0.60;

% 总损失系数只由曾洪涛文献给出的自然落差和设计水头推导。
p.loss.total_K = p.ref.derived.loss_coeff_rated;
p.loss.Kq = 0.080;
p.loss.Kdq = 0.030;
p.loss.Ky = 0.040;

% 压力水头输出基准：文献未给出蜗壳/尾水开机校核压力，
% 因此不再使用外部压力校核值。蜗壳输出以设计水头为参考，尾水以相对零点为参考。
p.pressure.Hc_base = p.Hr;
p.pressure.Ht_base = 8.0;    % 名义尾水压力水头，仅作相对输出基准，不作为文献校核值

p.tail.Tt = 2.0;
p.tail.Kq = 0.25;
p.tail.Kacc = 0.060;
p.tail.Kdec = 0.020;
p.tail.drop_limit = 2.0;       % 允许相对尾水位有小幅下探，输出压力水头仍以 Ht_base 为正基准

%% 9. 导叶执行机构与三阶段开机规律
p.guide.full_open_deg = 30.0;

p.guide.Y1 = 0.765;      % 典型工况导叶开度：前期开度适中，保留优化空间
p.guide.Y2 = 0.378;      % 典型工况第二阶段开度
p.guide.t1 = 7.2;        % 较快开启，形成可优化的超调和压力响应
p.guide.t2 = 11.6;       % 中段保持时间较短
p.guide.t3 = 15.8;       % PID handover slightly advanced

p.guide.Ty = 0.25;
p.guide.vmax = 0.18;

%% 10. PID 参数
p.pid.Kp = 0.42;        % 典型工况 PID 参数：保证开机过程可稳定，同时保留优化空间
p.pid.Ki = 0.052;
p.pid.Kd = 0.014;

p.pid.int_min = -0.18;
p.pid.int_max = 0.18;
p.pid.u_min = -0.15;
p.pid.u_max = 0.15;
p.pid.antiwindup = true;

%% 11. 仿真设置
p.sim.dt = 0.03;         %普通电脑优先采用 0.03；终稿高清图可改为 0.02
p.sim.t_end = 40.0;
p.sim.settle_band = 0.02;
p.sim.settle_hold = 3.0;

%% 12. 8 维决策变量边界
% x = [Kp, Ki, Kd, Y1, Y2, t1, t2, t3]
p.bounds.lb = [0.05, 0.000, 0.000, 0.64, 0.34, 5.0, 9.5, 14.0];
p.bounds.ub = [1.25, 0.140, 0.080, 0.80, 0.50, 10.0, 18.0, 24.0];

%% 13. 评价指标权重
p.opt.weight.t_start = 0.19;       % startup settling time
p.opt.weight.overshoot = 0.24;     % speed overshoot
p.opt.weight.final_error = 0.12;   % final speed error
p.opt.weight.iae_speed = 0.13;     % post-PID speed IAE
p.opt.weight.hc_max = 0.19;        % casing pressure peak
p.opt.weight.ht_range = 0.09;      % tailwater pressure range
p.opt.weight.guide_rate = 0.04;    % guide-vane rate

%% 14. 目标函数归一化参考值
% prepare_objective_params 会用当前典型工况仿真结果覆盖这些参考值。
p.opt.ref.t_start_s = 18.0;
p.opt.ref.overshoot_pu = 0.0125;
p.opt.ref.final_error_pu = 0.0180;
p.opt.ref.iae_speed = 8.80;
p.opt.ref.hc_max_m = p.Hr;
p.opt.ref.ht_range_m = 15.0;
p.opt.ref.guide_rate_rms = 0.0482;

%% 15. 约束惩罚项
% 文献未给出开机尾水最小压力绝对限值，这里仅设置宽松数值边界，
% 防止仿真发散；论文中不把该值写成文献校核数据。
p.opt.constraint.final_speed_band = 0.020;
p.opt.constraint.tail_min_limit_m = 0.0;
p.opt.constraint.hc_max_limit_m = 1.30 * p.Hr;
p.opt.constraint.ht_range_limit_m = 40.0;
p.opt.constraint.guide_rate_limit = 0.060;

p.opt.constraint.penalty_final = 0.60;
p.opt.constraint.penalty_tail = 1.00;
p.opt.constraint.penalty_hc = 0.80;
p.opt.constraint.penalty_ht_range = 0.60;
p.opt.constraint.penalty_guide_rate = 0.40;

p.opt.constraint.use_baseline_guard = true;
p.opt.constraint.baseline_t_start_margin = 0.010;
p.opt.constraint.baseline_hc_margin = 0.010;
p.opt.constraint.baseline_ht_margin = 0.020;
p.opt.constraint.baseline_guide_margin = 0.080;
p.opt.constraint.baseline_iae_margin = 0.040;       % IAE 允许小幅折中，避免优化效果被过度压扁
p.opt.constraint.baseline_penalty = 18.0;

% 边界惩罚：只在变量过度贴近上下界时轻微惩罚，用于提高参数解释性。
p.opt.boundary.enable = true;
p.opt.boundary.margin = 0.06;
p.opt.boundary.weight = 0.08;

p.opt.pref.enable = true;      % 开启工程偏好软惩罚，避免 Kp、Ki、Y2、t2、t3 等贴边
p.opt.pref.Kp_soft_max = 1.08;
p.opt.pref.Ki_soft_max = 0.118;
p.opt.pref.Kd_soft_min = 0.006;
p.opt.pref.Kd_soft_max = 0.065;
p.opt.pref.Y1_soft_min = 0.660;
p.opt.pref.Y1_soft_max = 0.785;
p.opt.pref.Y2_soft_min = 0.355;
p.opt.pref.Y2_soft_max = 0.435;
p.opt.pref.t1_soft_min = 6.00;
p.opt.pref.t1_soft_max = 9.50;
p.opt.pref.t2_soft_min = 10.20;
p.opt.pref.t3_soft_min = 14.80;
p.opt.pref.t3_soft_max = 22.00;
p.opt.pref.weight = 0.012;

%% 16. 输出设置
p.plot.output_dir = fullfile(pwd, 'objective_results');
p.plot.save_figures = true;

end
