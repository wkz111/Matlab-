function p2 = set_operating_head(p, H_operating)
%SET_OPERATING_HEAD 设置不同运行水头工况
%
% H_operating 表示用于论文分析的运行水头，单位为 m。
% 为保持设计水头典型工况与默认参数一致，程序按额定损失水头反推上游相对水位：
%   H_upstream = H_downstream + H_operating + loss_head_rated
% 这样当 H_operating = p.ref.Hr 时，H_upstream 仍为文献给出的自然落差 531 m。

p2 = p;
if nargin < 2 || isempty(H_operating)
    H_operating = p2.ref.Hr;
end

if isfield(p2, 'ref') && isfield(p2.ref, 'derived') && isfield(p2.ref.derived, 'loss_head_rated_m')
    loss_head = p2.ref.derived.loss_head_rated_m;
else
    loss_head = max(p2.Hgross - p2.Hr, 0.0);
end

p2.condition.H_operating = H_operating;
p2.condition.loss_head_reference = loss_head;
p2.H_downstream = 0.0;
p2.H_upstream = p2.H_downstream + H_operating + loss_head;
p2.Hgross = p2.H_upstream - p2.H_downstream;

% 压力水头采用相对坐标输出，不把不同水头工况误读为绝对高程变化。
if ~isfield(p2, 'pressure')
    p2.pressure = struct();
end
p2.pressure.Hc_base = H_operating;
if ~isfield(p2.pressure, 'Ht_base') || isempty(p2.pressure.Ht_base)
    p2.pressure.Ht_base = 8.0;
end
end
