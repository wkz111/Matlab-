function cfg = algorithm_config()
% GA / PSO / DE 三种传统优化算法统一配置
cfg.seed = 1;
cfg.output_root = fullfile(pwd, 'optimization_results');
cfg.run_tag = datestr(now, 'yyyymmdd_HHMMSS');
cfg.output_dir = fullfile(cfg.output_root, ['run_' cfg.run_tag]);
cfg.clean_output = false;

cfg.methods = {'GA', 'PSO', 'DE'};

cfg.common.nPop = 20;
cfg.common.maxIter = 35;
cfg.common.nRun = 1;

cfg.common.paper_nPop_suggested = 28;
cfg.common.paper_maxIter_suggested = 50;
cfg.common.paper_nRun_suggested = 2;
cfg.verbose = false;

% PSO 参数。
cfg.pso.w_max = 0.88;
cfg.pso.w_min = 0.32;
cfg.pso.c1 = 1.45;
cfg.pso.c2 = 1.85;
cfg.pso.vmax_ratio = 0.25;

% GA 参数。
cfg.ga.pc = 0.85;
cfg.ga.pm = 0.16;
cfg.ga.elite_num = 2;
cfg.ga.tournament_k = 3;
cfg.ga.mutation_scale = 0.08;

% DE 参数：DE/rand/1/bin。
cfg.de.F = 0.60;
cfg.de.CR = 0.85;

% 绘图颜色：PSO 使用高辨识度蓝色并加粗；GA/DE 作为对比方案。
cfg.color.base = [0.18 0.18 0.18];      % 优化前：深灰
cfg.color.ga = [0.90 0.45 0.08];        % GA：橙色
cfg.color.pso = [0.00 0.27 0.78];       % PSO：主推蓝
cfg.color.de = [0.20 0.62 0.28];        % DE：绿色
cfg.color.no_pid = [0.45 0.45 0.45];
cfg.color.with_pid = cfg.color.pso;

cfg.line.base = 2.0;
cfg.line.ga = 1.7;
cfg.line.pso = 3.0;
cfg.line.de = 1.8;

% 默认不弹出图窗，直接保存 PNG。
cfg.plot.visible = 'off';
cfg.plot.close_after_save = true;
cfg.plot.dpi = 300;

% 局部放大图范围。
cfg.zoom.speed_xlim = [12 22];
cfg.zoom.speed_ylim = [500 525];
cfg.zoom.guide_xlim = [5 20];
cfg.zoom.pressure_xlim = [8 22];

end
