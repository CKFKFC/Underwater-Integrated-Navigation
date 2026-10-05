function cfg = setTrajectoryOptions()
%SETTRAJECTORYOPTIONS 设置 genetraj 使用的默认轨迹和传感器参数。
%   这个文件是最适合人工修改的配置入口。无论生成直线、割草机还是其他轨迹，
%   都应通过 genetraj 调用：
%
%       genetraj()
%
%   切换水平轨迹时，修改 cfg.TrajectoryType：
%       "straight", "lawnmower", "figure8", "circle", 或 "sCurve"。
%
%   坐标和符号约定：
%   - 水平轨迹先在局部 ENU 坐标中设计，单位 m。
%   - 输出位置统一转换为纬经高 [latitude, longitude, height]。
%   - 深度 depth 向下为正，高度 height 向上为正。
%   - b 系为 RFU [right, forward, up]，n 系为 ENU [east, north, up]。
%   - 欧拉角为 [roll, pitch, yaw]：右倾、抬头、北零顺时针航向为正。

cfg = struct();

%% 输出文件
% OutputFile 为空时，默认保存到 data/input/navigation_input.mat。
cfg.OutputFile = "";

% 设为 false 时只返回 inputData，不写 MAT 文件，适合调试和单元测试。
cfg.SaveToFile = true;

%% 时间设置
% 轨迹总时长，单位 s。参考工程的直线和割草机轨迹均采用约 3600 s。
cfg.Duration = 3600.0;

% 主采样周期，也是 IMU 采样周期，单位 s。
% 建议与 setConfig.m 中的 cfg.sim.dt 保持一致。
cfg.SampleInterval = 0.01;

%% 初始地理位置
% 初始纬度和经度，单位 deg。这里沿用参考工程附近的初始位置。
cfg.InitialLatitudeDeg = 34.246048;
cfg.InitialLongitudeDeg = 108.909664;

% 初始深度，向下为正，单位 m。
cfg.InitialDepth = 30.0;

% 水面参考高度，单位 m。应与 setConfig.m 中 cfg.reference.surfaceAltitude 一致。
cfg.SurfaceAltitude = 0.0;

%% 水平轨迹形状
% 支持的 TrajectoryType：
% "straight"  : 直线匀速 AUV 运动。
% "lawnmower" : 割草机测线轨迹，包含 180 deg 掉头。
% "figure8"   : 8 字形运动，适合提供多方向激励。
% "circle"    : 圆形运动，适合稳定转弯测试。
% "sCurve"    : 正弦横向摆动曲线。
cfg.TrajectoryType = "straight";

% figure8/circle 的水平尺度，单位 m。
cfg.HorizontalRadius = 70.0;

% figure8/circle 的主要机动周期，单位 s。
cfg.HorizontalPeriod = 180.0;

% straight/sCurve/lawnmower 的前进速度，单位 m/s。
cfg.StraightSpeed = 1.0;

% straight 直线航向角，单位 deg。0 deg 表示向北，90 deg 表示向东。
cfg.CourseDeg = 0.0;

% lawnmower 割草机轨迹几何参数，单位 m。
% 相邻测线间距必须不小于 2 倍转弯半径；如果更大，会在两个 90 deg
% 定半径转弯之间增加一段横向连接段。
cfg.LawnmowerLegLength = 450.0;
cfg.LawnmowerLaneSpacing = 50.0;
cfg.LawnmowerTurnRadius = 25.0;

% sCurve 的横向摆动幅值和周期。
cfg.LateralAmplitude = 30.0;
cfg.LateralPeriod = 120.0;

%% 深度轨迹形状
% 支持的 DepthMotionType：
% "auvHeave"  : 参考 AUV 轨迹风格的小幅低频升沉。
% "sine"      : 正弦深度变化。
% "constant"  : 固定深度。
% "diveClimb" : 一次下潜-上浮周期。
cfg.DepthMotionType = "auvHeave";

% 深度变化幅值，向下为正，单位 m。
cfg.DepthAmplitude = 8.0;

% 深度变化周期，单位 s。
cfg.DepthPeriod = 90.0;

% 深度正弦初相位，单位 deg。生成器会平移曲线，使 t=0 的深度仍等于 InitialDepth。
cfg.DepthPhaseDeg = 30.0;

%% 姿态设置
% 额外滚转角幅值，单位 deg。航向和俯仰主要由速度方向自动生成。
cfg.RollAmplitudeDeg = 3.0;

%% 传感器输出周期
% DVL 体坐标速度输出周期，单位 s。
cfg.DvlSampleInterval = 0.10;

% 真值杆臂：INS 指向 DVL，在 INS RFU [右; 前; 上] 三轴下表达，单位 m。
% 与滤波配置 cfg.sensor.dvl.leverArmBody 分别设置，支持模拟标定误差。
cfg.DvlLeverArmBody = [0.7; -0.8; -0.5];

% 深度计输出周期，单位 s。
cfg.DepthSampleInterval = 0.10;

% GPS 纬经高输出周期，单位 s。水下场景如不使用 GPS，应在 setConfig.m 中关闭。
cfg.GpsSampleInterval = 1.00;

end
