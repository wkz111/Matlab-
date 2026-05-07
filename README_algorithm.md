# startup_code_typical_condition

本文件夹用于论文中“模型合理性验证”和“GA / PSO / DE 三种传统算法对比”部分。

## 推荐运行顺序

### 1. 设计水头典型工况检查

```matlab
clear; clc; close all;
run_typical_condition_check
```

该脚本用于检查设计水头典型工况下的开机响应、目标函数分项和核心指标。

### 2. 三种水头工况模型合理性验证

```matlab
clear; clc; close all;
run_model_validation
```

该脚本用于生成低水头、设计水头、高水头三种工况的响应对比图和模型适应性验证指标表。第三章建议使用这部分结果验证模型响应规律和工况适应性；该表不输出 J 和启动稳定时间，避免把模型验证误写成控制性能评价。

### 3. 典型工况优化算法对比

```matlab
clear; clc; close all;
run_optimization_comparison
```

该脚本用于在设计水头典型工况下比较优化前方案、GA、PSO、DE 四类结果。第四章建议使用这部分结果说明三种传统算法的优化效果，并推荐 PSO 方案。

### 4. PSO 快速检查

```matlab
clear; clc; close all;
run_pso_only_check
```

该脚本只运行 PSO，适合普通电脑快速调参或复查推荐方案。

## 输出文件夹

```text
typical_condition_results/     设计水头典型工况检查结果
model_validation_results/      三种水头工况验证结果
optimization_results/          GA / PSO / DE 优化对比结果
```

各输出文件夹默认带时间戳，避免 CSV 或图片被 Excel、MATLAB 占用后写入失败。

## 主要输出

典型工况检查：

```text
table_typical_condition_check.csv
fig_typical_condition_response.png
workspace_typical_condition.mat
```

三种水头工况验证：

```text
table_three_head_validation.csv
fig_01_three_head_response.png
workspace_three_head_validation.mat
```

优化对比：

```text
table_algorithm_metrics.csv
table_algorithm_decision_variables.csv
table_improvement_percent.csv
table_boundary_check.csv
table_recommendation.csv
table_pso_final_selected.csv
fig_01_typical_condition_response.png
fig_02_pid_correction_speed_guide.png
fig_03_pid_correction_response.png
fig_04_algorithm_response.png
fig_05_algorithm_zoom.png
fig_06_pressure_comparison.png
fig_07_convergence.png
fig_08_metric_bar.png
fig_09_decision_variables.png
fig_10_improvement_percent.png
fig_11_response_difference.png
```

## 计算规模

默认计算规模偏向普通电脑：

```matlab
cfg.common.nPop = 20;
cfg.common.maxIter = 35;
cfg.common.nRun = 1;
p.sim.dt = 0.03;
```

终稿需要更稳定的统计结果时，可在 `algorithm_config.m` 和 `psu_default_params.m` 中改为：

```matlab
cfg.common.nPop = 28;
cfg.common.maxIter = 50;
cfg.common.nRun = 2;
p.sim.dt = 0.02;
```

## 指标口径

`speed_iae` 表示 PID 接入后的稳定调节段速度误差积分。`speed_iae_all` 为全过程速度误差积分，仅用于诊断，不进入目标函数。

文献未给出开机过程中的蜗壳最大压力、最大转速、尾水最小压力等现场校核值，因此这些量只作为同一模型内的响应评价指标，不作为实测对比目标。

## 论文叙事建议

第三章可写：在设计水头典型工况下，模型输出的导叶开度、流量、转速、有效水头、蜗壳压力水头和尾水压力水头之间具有明确物理因果关系；进一步通过低水头、设计水头和高水头三种工况验证模型对运行水头变化的响应能力。三水头验证的重点是转速峰值、流量、有效水头、蜗壳压力和尾水压力随水头变化是否符合物理规律，不强调启动稳定时间。

第四章可写：在统一目标函数、统一决策变量边界和同一仿真模型下，采用 GA、PSO、DE 三种传统优化算法进行对比。结果表明，PSO 在综合目标函数、转速超调抑制和压力响应改善方面表现较优，因此选取 PSO 优化结果作为推荐开机控制参数。
