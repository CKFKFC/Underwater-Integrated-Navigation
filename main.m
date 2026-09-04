% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 水下 ESKF 组合导航主程序。

clearvars;
close all;
clc;

% 将当前项目及子文件夹加入 MATLAB 路径。
addpath(genpath(pwd));

% 控制随机数，便于复现实验。
randomselect = 'random'; % 'random' or 'repeatlast'
ControlRandomNumber(randomselect);

% 读取全局配置。仿真时长可在 cfg.sim.duration 中设置。
cfg = setConfig();

% 读取真值或传感器量测数据，并初始化量测管理器。
meas = StateAndMeasurement(cfg);
meas.loadInputData();
meas.checkInputData();

% 如需理想量测，仅检查导入数据是否完整；本项目不在算法内生成轨迹。
if cfg.data.generateIdealMeasurement
    meas.generateIdealMeasurements();
end

% 初始化结果管理器。
results = FilterResults(cfg, meas);

% ESKF 是算法流程函数，ErrorStateKF 是内部滤波器类。
if cfg.algorithm.isESKFOn
    disp("ESKF running ...");
    results = ESKF(cfg, meas, results);
end

% 误差评估与结果保存。
results.computeErrors(meas);
results.saveToMat();

% 绘制二维水平轨迹对比和组合导航误差曲线。
plotter = ResultPlotter(cfg, meas, results);
plotter.plotTrajectory();
plotter.plotPositionError();
plotter.plotVelocityError();
plotter.plotAttitudeError();
plotter.plotRmse();
plotter.plotPositionComponents();
