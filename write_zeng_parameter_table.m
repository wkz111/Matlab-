function T = write_zeng_parameter_table(p, output_dir)
%WRITE_ZENG_PARAMETER_TABLE 输出与曾洪涛文献一致的参数来源表
%
% 用法：
%   p = psu_default_params();
%   T = write_zeng_parameter_table(p, 'algorithm_results');
%
% 说明：
%   本表只记录曾洪涛文献明确给出的参数，以及由这些参数直接推导的量。
%   不再包含其他文献中的蜗壳最大压力、最大转速、尾水最小压力等数据。

if nargin < 1 || isempty(p)
    p = psu_default_params();
end
if nargin < 2 || isempty(output_dir)
    output_dir = fullfile(pwd, 'algorithm_results');
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

name = {};
value = [];
unit = {};
note = {};

add('自然落差 Hgross', p.Hgross, 'm', '曾洪涛文献给出');
add('最小毛水头 Hmin', p.ref.Hmin, 'm', '仅作为文献参数记录');
add('最大毛水头 Hmax', p.ref.Hmax, 'm', '仅作为文献参数记录');
add('设计水头 Hr', p.Hr, 'm', '曾洪涛文献给出');
add('额定流量 Qr', p.Qr, 'm^3/s', '曾洪涛文献给出');
add('单机容量 Pr', p.Pr/1e6, 'MW', '由 4 x 300 MW 得到');
add('单厂装机容量', p.ref.plant_capacity/1e6, 'MW', '曾洪涛文献给出');
add('机组数量', p.ref.unit_count, '台', '曾洪涛文献给出');
add('等效惯性时间常数 Ta+Tb', p.Ta, 's', '曾洪涛文献给出');
add('额定水力功率', p.ref.derived.hydraulic_power_W/1e6, 'MW', 'rho*g*Qr*Hr 推导');
add('反推额定效率', p.ref.derived.eta_rated, '-', 'Pr/(rho*g*Qr*Hr) 推导');
add('额定总损失水头', p.ref.derived.loss_head_rated_m, 'm', 'Hgross-Hr 推导');
add('总损失系数', p.ref.derived.loss_coeff_rated, 'm/(m^3/s)^2', '(Hgross-Hr)/Qr^2 推导');

for i = 1:numel(p.pipe.L)
    add(sprintf('管段%d长度', i), p.pipe.L(i), 'm', p.pipe.name{i});
    add(sprintf('管段%d直径', i), p.pipe.D(i), 'm', p.pipe.name{i});
    add(sprintf('管段%d波速', i), p.pipe.a(i), 'm/s', p.pipe.name{i});
end

for i = 1:numel(p.ref.waterhammer.Tr)
    add([p.ref.waterhammer.group{i} ' 水击相长 Tr'], p.ref.waterhammer.Tr(i), 's', '曾洪涛文献给出');
    add([p.ref.waterhammer.group{i} ' 管道特性系数 hw'], p.ref.waterhammer.hw(i), '-', '曾洪涛文献给出');
end
add('上调压井水头时间常数 Th', p.surge.Th, 's', '曾洪涛文献给出');
add('上调压井流量时间常数 Tq', p.surge.Tq, 's', '曾洪涛文献给出');

T = table(name(:), value(:), unit(:), note(:), ...
    'VariableNames', {'parameter','value','unit','note'});

writetable(T, fullfile(output_dir, 'table_zeng_parameter_consistency.csv'));
disp(T);

    function add(a, b, c, d)
        name{end+1,1} = a; %#ok<AGROW>
        value(end+1,1) = b; %#ok<AGROW>
        unit{end+1,1} = c; %#ok<AGROW>
        note{end+1,1} = d; %#ok<AGROW>
    end
end
