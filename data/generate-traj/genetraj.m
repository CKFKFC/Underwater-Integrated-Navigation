function [inputData, outputFile, trajectoryInfo] = genetraj(varargin)
%GENETRAJ 生成当前工程可直接读取的导航输入数据。
%   GENETRAJ 是 data 目录下轨迹生成的统一公开入口。后续新增轨迹时，
%   应优先扩展 cfg.TrajectoryType 对应的内部生成逻辑，而不是新建入口函数。
%
%   水平轨迹类型由 cfg.TrajectoryType 选择：
%   "straight", "lawnmower", "rectangle", "figure8", "circle", 或 "sCurve"。
%
%   默认用法：
%       genetraj()
%
%   生成 3600 s 直线 AUV 轨迹：
%       genetraj("TrajectoryType", "straight", ...
%           "OutputFile", "data/input/navigation_input_straight_auv_3600s.mat")
%
%   生成 3600 s 割草机 AUV 轨迹：
%       genetraj("TrajectoryType", "lawnmower", ...
%           "OutputFile", "data/input/navigation_input_lawnmower_auv_3600s.mat")
%
%   生成定深长方形轨迹（直边前进，到顶点停车并原地右转 90 deg）：
%       genetraj("TrajectoryType", "rectangle", ...
%           "RectangleLength", 450.0, "RectangleWidth", 200.0, ...
%           "RectangleTurnDuration", 5.0, ...
%           "OutputFile", "data/input/navigation_input_rectangle_3600s.mat")
%   rectangle 固定深度为 InitialDepth，不叠加 DepthMotionType 或滚转激励。
%
%   重复实验建议修改 setTrajectoryOptions.m，或传入 cfg 结构体覆盖默认设置：
%       cfg = setTrajectoryOptions();
%       cfg.TrajectoryType = "lawnmower";
%       cfg.OutputFile = "data/input/my_lawnmower_case.mat";
%       genetraj(cfg);
%
%   生成的 inputData 结构体是 StateAndMeasurement 和 ESKF 流程读取的统一格式。

% 先读取默认配置，再用外部传入的 name-value 或 cfg 结构体覆盖。
cfg = setTrajectoryOptions();
overrides = parseTrajectoryOverrides(varargin{:});
cfg = applyOptionOverrides(cfg, overrides);
cfg = validateTrajectoryOptions(cfg);

% 输出文件路径允许由配置指定；留空时使用 data/input/navigation_input.mat。
outputFile = resolveOutputFile(cfg);

% 按“时间轴 -> 真值轨迹 -> 理想传感器 -> inputData”的顺序生成数据。
time = generateTimeVector(cfg.Duration, cfg.SampleInterval);
trajectory = generateTrajectoryShape(time, cfg);
sensorData = generateIdealSensors(time, trajectory, cfg);
[inputData, trajectoryInfo] = buildNavigationInputData(time, trajectory, sensorData, cfg);

if cfg.SaveToFile
    % 保存前确保输出目录存在；MAT 文件内保留本次使用的全部配置。
    outputFolder = fileparts(outputFile);
    if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
        mkdir(outputFolder);
    end

    trajectoryOptions = cfg;
    save(outputFile, "inputData", "trajectoryInfo", "trajectoryOptions");
else
    outputFile = "";
end

end
