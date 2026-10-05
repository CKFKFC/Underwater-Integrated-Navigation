function [inputData, trajectoryInfo] = buildNavigationInputData(time, trajectory, sensorData, cfg)
%BUILDNAVIGATIONINPUTDATA 组装 StateAndMeasurement 可直接读取的 inputData。

inputData = struct();
inputData.time = time;
inputData.metadata = struct();
inputData.metadata.bodyFrame = "RFU";
inputData.metadata.navigationFrame = "ENU";
inputData.metadata.dvlLeverArmBody = cfg.DvlLeverArmBody;
inputData.metadata.dvlVelocityReference = "DVL installation point, Earth-relative velocity";
inputData.metadata.bodyAxisOrder = ["right", "forward", "up"];
inputData.metadata.eulerOrder = ["roll", "pitch", "yaw"];
inputData.metadata.eulerConvention = ...
    "positive right-bank, nose-up, north-zero clockwise heading";

% 初始状态用于初始化惯导名义状态和 ESKF 误差状态。
inputData.initial = struct();
inputData.initial.positionLlh = trajectory.PositionLlh(1, :).';
inputData.initial.velocityEnu = trajectory.VelocityEnu(1, :).';
inputData.initial.attitudeCbn = trajectory.AttitudeCbn(:, :, 1);
inputData.initial.attitudeEuler = trajectory.AttitudeEuler(1, :).';
inputData.initial.gyroBias = zeros(3, 1);
inputData.initial.accelBias = zeros(3, 1);

% truth 字段只用于误差评估和绘图，不直接作为滤波量测。
inputData.truth = struct();
inputData.truth.positionLlh = trajectory.PositionLlh;
inputData.truth.velocityEnu = trajectory.VelocityEnu;
inputData.truth.attitudeCbn = trajectory.AttitudeCbn;
inputData.truth.attitudeEuler = trajectory.AttitudeEuler;

% IMU 与主时间轴同频，gyro/accel 均按 RFU [right, forward, up] 表达。
inputData.imu = struct();
inputData.imu.time = sensorData.Imu.Time;
inputData.imu.gyro = sensorData.Imu.Gyro;
inputData.imu.accel = sensorData.Imu.Accel;

% DVL 量测为 RFU 体坐标系速度，valid 用于模拟缺测或无效量测。
inputData.dvl = struct();
inputData.dvl.time = sensorData.Dvl.Time;
inputData.dvl.velocityBody = sensorData.Dvl.VelocityBody;
inputData.dvl.valid = sensorData.Dvl.Valid;

% 深度计量测为向下为正的 depth。
inputData.depth = struct();
inputData.depth.time = sensorData.Depth.Time;
inputData.depth.depth = sensorData.Depth.Depth;
inputData.depth.valid = sensorData.Depth.Valid;

% GPS 量测为纬经高，水下仿真中可以在配置中关闭。
inputData.gps = struct();
inputData.gps.time = sensorData.Gps.Time;
inputData.gps.positionLlh = sensorData.Gps.PositionLlh;
inputData.gps.valid = sensorData.Gps.Valid;

trajectoryInfo = struct();
trajectoryInfo.time = time;
trajectoryInfo.positionEnu = trajectory.PositionEnu;
trajectoryInfo.positionLlh = trajectory.PositionLlh;
trajectoryInfo.velocityEnu = trajectory.VelocityEnu;
trajectoryInfo.attitudeEuler = trajectory.AttitudeEuler;
trajectoryInfo.referenceLlh = trajectory.ReferenceLlh;
trajectoryInfo.bodyFrame = "RFU";
trajectoryInfo.navigationFrame = "ENU";
trajectoryInfo.horizontalInfo = trajectory.HorizontalInfo;
trajectoryInfo.depthInfo = trajectory.DepthInfo;
trajectoryInfo.coordinateNote = [
    "positionEnu 仅用于轨迹形状设计和绘图检查；"
    "inputData.initial.positionLlh 和 inputData.truth.positionLlh 才是惯导解算使用的位置。"
    ];
trajectoryInfo.options = cfg;

end
