function cfg = setConfig()
%SETCONFIG 创建水下 ESKF 组合导航全局配置。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 配置项目路径、数据模式、仿真时长、传感器噪声和算法开关。

projectRoot = fileparts(fileparts(mfilename("fullpath")));

cfg = struct();

%% 项目路径配置
cfg.path.projectRoot = projectRoot;
cfg.path.inputFolder = fullfile(projectRoot, "data", "input");
cfg.path.outputFolder = fullfile(projectRoot, "data", "output");

%% 仿真配置
% 主采样周期，单位 s。应与轨迹生成时的 SampleInterval 保持一致。
cfg.sim.dt = 0.01;

% Monte Carlo 次数。实测数据模式下会自动改为 1。
cfg.sim.runs = 1;

% 实际参与算法运行的仿真时长，单位 s。
% inf 或 [] 表示使用输入 MAT 文件中的完整轨迹；例如设为 360.0 时，
% 即使输入轨迹为 3600 s，算法、误差计算和绘图也只运行前 360 s。
cfg.sim.duration = 2000;

%% 数据配置
% mode = "simulation" 时，输入文件应包含真值和理想传感器数据。
% mode = "real" 时，输入文件可包含已经带噪的实测传感器数据。
cfg.data.mode = "simulation"; % "simulation" or "real"
cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input_straight_auv_3600s.mat");
cfg.data.generateIdealMeasurement = false;
cfg.data.coordinateFrame = "ENU";
cfg.data.positionType = "latitude-longitude-height";

%% 参考高度
% 水面参考高度，与纬经高使用同一高度基准。
% ENU 中高度向上为正，深度向下为正。
cfg.reference.surfaceAltitude = 0.0;

%% 传感器开关
cfg.sensor.imu.isEnabled = true;
cfg.sensor.dvl.isEnabled = true;
cfg.sensor.depth.isEnabled = false;
cfg.sensor.gps.isEnabled = false;

% DVL 量测统一使用载体坐标系速度。
% 可用时间为 N-by-2 矩阵，每行表示 [开始时间, 结束时间]，单位 s。
cfg.sensor.dvl.availableTime = [0.0, inf];
cfg.sensor.gps.availableTime = [0.0, inf];

% 异步量测时间匹配容差。
cfg.measurement.timeTolerance = 0.5 * cfg.sim.dt;

%% IMU 误差模型
% 常值零偏 + 白噪声。工程指标在这里设置，算法内部统一转换为 SI 单位。
standardGravity = 9.80665;
secondsPerHour = 3600.0;
sampleRate = 1.0 / cfg.sim.dt;
microGToMeterPerSecondSquared = standardGravity * 1.0e-6;
degreePerHourToRadianPerSecond = deg2rad(1.0) / secondsPerHour;
degreePerSqrtHourToRadianPerSqrtSecond = deg2rad(1.0) / sqrt(secondsPerHour);

% 常值零偏 1σ。每次 Monte Carlo 运行生成一次，整段轨迹内保持不变。
cfg.noise.imu.accelBiasStdMicroG = [100.0; 100.0; 100.0];     % 加速度计零偏: ug
cfg.noise.imu.gyroBiasStdDegPerHour = [1.0; 1.0; 1.0];        % 陀螺零偏: deg/h
cfg.noise.imu.accelBiasStd = cfg.noise.imu.accelBiasStdMicroG ...
    * microGToMeterPerSecondSquared;                          % m/s^2
cfg.noise.imu.gyroBiasStd = cfg.noise.imu.gyroBiasStdDegPerHour ...
    * degreePerHourToRadianPerSecond;                         % rad/s

% 白噪声连续强度。
% 陀螺用角随机游走系数 deg/sqrt(h)，转换为 rad/sqrt(s)。
% 加速度计用噪声密度 micro-g/sqrt(Hz)，转换为 (m/s^2)/sqrt(Hz)。
cfg.noise.imu.gyroRandomWalkDegPerSqrtHour = [0.15; 0.15; 0.15];
cfg.noise.imu.accelRandomWalkMicroGPerSqrtHz = [15.0; 15.0; 15.0];
cfg.noise.imu.gyroNoiseDensity = cfg.noise.imu.gyroRandomWalkDegPerSqrtHour ...
    * degreePerSqrtHourToRadianPerSqrtSecond;                  % rad/sqrt(s)
cfg.noise.imu.accelNoiseDensity = cfg.noise.imu.accelRandomWalkMicroGPerSqrtHz ...
    * microGToMeterPerSecondSquared;                           % (m/s^2)/sqrt(Hz)

% 每个采样点的 IMU 白噪声标准差，用于给理想 IMU 逐点加噪。
% 若白噪声连续强度为 N，则采样周期 dt 下的离散量测标准差为 N/sqrt(dt)。
cfg.noise.imu.gyroStd = cfg.noise.imu.gyroNoiseDensity * sqrt(sampleRate);    % rad/s
cfg.noise.imu.accelStd = cfg.noise.imu.accelNoiseDensity * sqrt(sampleRate);  % m/s^2

%% 外部传感器噪声
% 外部量测噪声为每次量测的 1σ 标准差。
cfg.noise.dvl.velocityStd = [0.3; 0.3; 0.3]; % 体坐标速度: m/s
cfg.noise.depth.depthStd = 0.10;             % 深度: m
cfg.noise.gps.positionStd = [1.0; 1.0; 2.0]; % ENU 位置: m

%% 算法配置
cfg.algorithm.isESKFOn = true;

% 误差状态模型配置。只需切换 profile 即可选择状态组合：
% ins9  = 姿态误差 + 速度误差 + 位置误差；
% ins15 = ins9 + 陀螺零偏误差 + 加速度计零偏误差。
cfg.algorithm.stateModel.profile = "ins15";

% 初始误差标准差用于每次 MC 的导航初值扰动和滤波初始协方差 P0。
% 状态顺序由所选 profile 中的状态块顺序统一生成。
cfg.algorithm.initialError.attitudeStdDeg = [1.0; 1.0; 2.0];        % roll/pitch/yaw: deg
cfg.algorithm.initialError.velocityStd = [0.1; 0.1; 0.1];        % ENU 速度: m/s
cfg.algorithm.initialError.positionStd = [1.0; 1.0; 1.0];           % ENU 位置: m
cfg.algorithm.initialError.gyroBiasStdDegPerHour = [1.0; 1.0; 1.0]; % 陀螺零偏估计误差: deg/h
cfg.algorithm.initialError.accelBiasStdMicroG = [100.0; 100.0; 100.0]; % 加速度计零偏估计误差: micro-g

% 滤波器内部统一使用 SI 单位，避免在算法类中再次硬编码单位换算。
cfg.algorithm.initialError.attitudeStd = deg2rad(cfg.algorithm.initialError.attitudeStdDeg);
cfg.algorithm.initialError.gyroBiasStd = cfg.algorithm.initialError.gyroBiasStdDegPerHour ...
    * degreePerHourToRadianPerSecond;                            % rad/s
cfg.algorithm.initialError.accelBiasStd = cfg.algorithm.initialError.accelBiasStdMicroG ...
    * microGToMeterPerSecondSquared;                              % m/s^2

%% 结果输出配置
cfg.result.outputFolder = cfg.path.outputFolder;
cfg.result.fileName = "eskf_results.mat";

%% 数据模式派生设置
% 实测数据通常已经包含噪声，因此默认不做传感器噪声 Monte Carlo 平均。
switch cfg.data.mode
    case "simulation"
        cfg.data.isSensorNoiseMonteCarlo = true;
    case "real"
        cfg.sim.runs = 1;
        cfg.data.generateIdealMeasurement = false;
        cfg.data.isSensorNoiseMonteCarlo = false;
    otherwise
        error("setConfig:InvalidDataMode", ...
            "cfg.data.mode must be either ""simulation"" or ""real"".");
end

end
