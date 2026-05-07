function result = optimize_startup(method, p, cfg)
%OPTIMIZE_STARTUP 统一优化接口：GA / PSO / DE / ALL
%
% 决策变量：
%   x = [Kp, Ki, Kd, Y1, Y2, t1, t2, t3]
%
% 用法：
%   result_pso = optimize_startup('PSO', p, cfg);
%   result_de  = optimize_startup('DE',  p, cfg);
%   results    = optimize_startup('ALL', p, cfg);

if nargin < 3 || isempty(cfg)
    cfg = algorithm_config();
end
if nargin < 2 || isempty(p)
    p = psu_default_params();
    p = prepare_objective_params(p);
end

method = normalize_method(method);

if strcmp(method, 'ALL')
    result = struct();
    for k = 1:numel(cfg.methods)
        name = normalize_method(cfg.methods{k});
        result.(name) = optimize_startup(name, p, cfg);
    end
    return;
end

fprintf('\n========== %s 优化开始 ==========', method);
fprintf('\n统一设置：nPop = %d, maxIter = %d, nRun = %d\n', ...
    cfg.common.nPop, cfg.common.maxIter, cfg.common.nRun);

best_result = [];
best_J = inf;
all_best_J = zeros(cfg.common.nRun, 1);

for run_id = 1:cfg.common.nRun
    rng(cfg.seed + method_seed_offset(method) + run_id);

    switch method
        case 'GA'
            result_now = run_ga(p, cfg);
        case 'PSO'
            result_now = run_pso(p, cfg);
        case 'DE'
            result_now = run_de(p, cfg);
        otherwise
            error('不支持的算法：%s', method);
    end

    all_best_J(run_id) = result_now.best_J;
    fprintf('%s 第 %d/%d 次运行：Best J = %.6f\n', ...
        method, run_id, cfg.common.nRun, result_now.best_J);

    if result_now.best_J < best_J
        best_J = result_now.best_J;
        best_result = result_now;
    end
end

result = best_result;
result.name = method;
result.all_run_best_J = all_best_J;
result.best_run_id = find(all_best_J == min(all_best_J), 1, 'first');

fprintf('========== %s 优化结束：Best J = %.6f ==========%s', method, result.best_J, newline);

end

%% ============================================================
%  PSO
%% ============================================================
function result = run_pso(p, cfg)

nVar = 8;
nPop = cfg.common.nPop;
maxIter = cfg.common.maxIter;

a = cfg.pso;
lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';
range = ub - lb;
vmax = a.vmax_ratio .* range;
vmin = -vmax;

pos = initialize_population(nPop, nVar, lb, ub, p);
vel = zeros(nPop, nVar);

cost = inf(nPop, 1);
metric_cell = cell(nPop, 1);
for i = 1:nPop
    [cost(i), metric_cell{i}] = objective_startup(pos(i, :), p);
end

pbest_pos = pos;
pbest_cost = cost;
pbest_metric = metric_cell;

[gbest_cost, gbest_index] = min(pbest_cost);
gbest_pos = pbest_pos(gbest_index, :);
gbest_metric = pbest_metric{gbest_index};

history = init_history(maxIter, nVar);

for iter = 1:maxIter
    w = a.w_max - (a.w_max - a.w_min) * (iter - 1) / max(maxIter - 1, 1);

    for i = 1:nPop
        r1 = rand(1, nVar);
        r2 = rand(1, nVar);

        vel(i, :) = ...
            w * vel(i, :) + ...
            a.c1 * r1 .* (pbest_pos(i, :) - pos(i, :)) + ...
            a.c2 * r2 .* (gbest_pos - pos(i, :));

        vel(i, :) = min(max(vel(i, :), vmin), vmax);
        pos(i, :) = pos(i, :) + vel(i, :);
        pos(i, :) = repair_decision_vector(pos(i, :), p);

        [cost(i), metric_cell{i}] = objective_startup(pos(i, :), p);

        if cost(i) < pbest_cost(i)
            pbest_pos(i, :) = pos(i, :);
            pbest_cost(i) = cost(i);
            pbest_metric{i} = metric_cell{i};
        end
    end

    [local_best_cost, local_best_index] = min(pbest_cost);
    if local_best_cost < gbest_cost
        gbest_cost = local_best_cost;
        gbest_pos = pbest_pos(local_best_index, :);
        gbest_metric = pbest_metric{local_best_index};
    end

    history = record_history(history, iter, gbest_cost, cost, gbest_pos);
    if cfg.verbose
        fprintf('PSO  第 %3d/%3d 代：Best J = %.6f, Mean J = %.6f\n', ...
            iter, maxIter, history.best_cost(iter), history.mean_cost(iter));
    end
end

best_x = repair_decision_vector(gbest_pos, p);
[p_best, out_best, m_best, best_J] = final_evaluate(best_x, p);
result = pack_result(best_x, best_J, m_best, out_best, p_best, history, gbest_metric);

end

%% ============================================================
%  GA
%% ============================================================
function result = run_ga(p, cfg)

nVar = 8;
nPop = cfg.common.nPop;
maxIter = cfg.common.maxIter;
a = cfg.ga;

lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';
range = ub - lb;

pop = initialize_population(nPop, nVar, lb, ub, p);
cost = inf(nPop, 1);
metric_cell = cell(nPop, 1);

for i = 1:nPop
    [cost(i), metric_cell{i}] = objective_startup(pop(i, :), p);
end

history = init_history(maxIter, nVar);

for iter = 1:maxIter
    [cost, order] = sort(cost, 'ascend');
    pop = pop(order, :);
    metric_cell = metric_cell(order);

    history = record_history(history, iter, cost(1), cost, pop(1, :));
    if cfg.verbose
        fprintf('GA   第 %3d/%3d 代：Best J = %.6f, Mean J = %.6f\n', ...
            iter, maxIter, history.best_cost(iter), history.mean_cost(iter));
    end

    new_pop = zeros(nPop, nVar);
    elite_num = min(a.elite_num, nPop);
    new_pop(1:elite_num, :) = pop(1:elite_num, :);

    fill_id = elite_num + 1;
    while fill_id <= nPop
        parent1 = pop(tournament_select(cost, a.tournament_k), :);
        parent2 = pop(tournament_select(cost, a.tournament_k), :);

        if rand < a.pc
            [child1, child2] = blend_crossover(parent1, parent2, lb, ub);
        else
            child1 = parent1;
            child2 = parent2;
        end

        child1 = gaussian_mutation(child1, lb, ub, a.pm, a.mutation_scale, range);
        child2 = gaussian_mutation(child2, lb, ub, a.pm, a.mutation_scale, range);

        new_pop(fill_id, :) = repair_decision_vector(child1, p);
        fill_id = fill_id + 1;

        if fill_id <= nPop
            new_pop(fill_id, :) = repair_decision_vector(child2, p);
            fill_id = fill_id + 1;
        end
    end

    pop = new_pop;
    for i = 1:nPop
        [cost(i), metric_cell{i}] = objective_startup(pop(i, :), p);
    end
end

[~, order] = sort(cost, 'ascend');
pop = pop(order, :);
metric_cell = metric_cell(order);

best_x = repair_decision_vector(pop(1, :), p);
[p_best, out_best, m_best, best_J] = final_evaluate(best_x, p);
result = pack_result(best_x, best_J, m_best, out_best, p_best, history, metric_cell{1});

end

function idx = tournament_select(cost, k)
nPop = numel(cost);
ids = randi(nPop, [k, 1]);
[~, best_local] = min(cost(ids));
idx = ids(best_local);
end

function [c1, c2] = blend_crossover(p1, p2, lb, ub)
alpha = 0.35;
gamma = (1 + 2 * alpha) * rand(size(p1)) - alpha;
c1 = gamma .* p1 + (1 - gamma) .* p2;
c2 = gamma .* p2 + (1 - gamma) .* p1;
c1 = min(max(c1, lb), ub);
c2 = min(max(c2, lb), ub);
end

function child = gaussian_mutation(child, lb, ub, pm, scale, range)
mask = rand(size(child)) < pm;
child(mask) = child(mask) + scale .* range(mask) .* randn(size(child(mask)));
child = min(max(child, lb), ub);
end

%% ============================================================
%  DE/rand/1/bin
%% ============================================================
function result = run_de(p, cfg)

nVar = 8;
nPop = cfg.common.nPop;
maxIter = cfg.common.maxIter;
a = cfg.de;

lb = p.bounds.lb(:).';
ub = p.bounds.ub(:).';

pop = initialize_population(nPop, nVar, lb, ub, p);
cost = inf(nPop, 1);
metric_cell = cell(nPop, 1);

for i = 1:nPop
    [cost(i), metric_cell{i}] = objective_startup(pop(i, :), p);
end

[best_cost, best_idx] = min(cost);
best_pos = pop(best_idx, :);
best_metric = metric_cell{best_idx};

history = init_history(maxIter, nVar);

for iter = 1:maxIter
    for i = 1:nPop
        ids = randperm(nPop, 3);
        while any(ids == i)
            ids = randperm(nPop, 3);
        end

        mutant = pop(ids(1), :) + a.F * (pop(ids(2), :) - pop(ids(3), :));
        mutant = min(max(mutant, lb), ub);

        trial = pop(i, :);
        j_rand = randi(nVar);
        for j = 1:nVar
            if rand <= a.CR || j == j_rand
                trial(j) = mutant(j);
            end
        end

        trial = repair_decision_vector(trial, p);
        [trial_cost, trial_metric] = objective_startup(trial, p);

        if trial_cost < cost(i)
            pop(i, :) = trial;
            cost(i) = trial_cost;
            metric_cell{i} = trial_metric;

            if trial_cost < best_cost
                best_cost = trial_cost;
                best_pos = trial;
                best_metric = trial_metric;
            end
        end
    end

    history = record_history(history, iter, best_cost, cost, best_pos);
    if cfg.verbose
        fprintf('DE   第 %3d/%3d 代：Best J = %.6f, Mean J = %.6f\n', ...
            iter, maxIter, history.best_cost(iter), history.mean_cost(iter));
    end
end

best_x = repair_decision_vector(best_pos, p);
[p_best, out_best, m_best, best_J] = final_evaluate(best_x, p);
result = pack_result(best_x, best_J, m_best, out_best, p_best, history, best_metric);

end

%% ============================================================
%  Common helpers
%% ============================================================
function pop = initialize_population(nPop, nVar, lb, ub, p)

pop = zeros(nPop, nVar);
base_x = typical_decision_vector(p);
pop(1, :) = repair_decision_vector(base_x, p);

for i = 2:nPop
    pop(i, :) = lb + rand(1, nVar) .* (ub - lb);
    pop(i, :) = repair_decision_vector(pop(i, :), p);
end

end

function history = init_history(maxIter, nVar)
history.best_cost = zeros(maxIter, 1);
history.mean_cost = zeros(maxIter, 1);
history.best_position = zeros(maxIter, nVar);
end

function history = record_history(history, iter, best_cost, cost_list, best_pos)
history.best_cost(iter) = best_cost;
finite_cost = cost_list(isfinite(cost_list));
if isempty(finite_cost)
    history.mean_cost(iter) = inf;
else
    history.mean_cost(iter) = mean(finite_cost);
end
history.best_position(iter, :) = best_pos;
end

function [p_best, out_best, m_best, best_J] = final_evaluate(best_x, p)
p_best = apply_decision_vector(best_x, p);
out_best = simulate_startup_moc(p_best);
m_best = calc_startup_metrics(out_best, p_best);
m_best = complete_startup_metrics(out_best, p_best, m_best);
[best_J, raw_metrics] = objective_startup(best_x, p);
m_best.objective = best_J;
if isfield(raw_metrics, 'objective_raw')
    m_best.objective_raw = raw_metrics.objective_raw;
end
if isfield(raw_metrics, 'penalty')
    m_best.penalty = raw_metrics.penalty;
end
end

function result = pack_result(best_x, best_J, best_metrics, best_out, best_params, history, raw_metrics)
result = struct();
result.best_x = best_x;
result.best_J = best_J;
result.best_metrics = best_metrics;
result.best_out = best_out;
result.best_params = best_params;
result.history = history;
result.raw_metrics = raw_metrics;
end

function method = normalize_method(method)
if isstring(method)
    method = char(method);
end
method = upper(strtrim(method));
method = strrep(method, '-', '');
method = strrep(method, '_', '');
end

function s = method_seed_offset(method)
switch method
    case 'GA'
        s = 1000;
    case 'PSO'
        s = 2000;
    case 'DE'
        s = 3000;
    otherwise
        s = 0;
end
end
