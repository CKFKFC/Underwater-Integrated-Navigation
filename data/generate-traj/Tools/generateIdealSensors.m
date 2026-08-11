function sensorData = generateIdealSensors(time, trajectory, cfg)
%GENERATEIDEALSENSORS 由轨迹真值生成无噪声传感器数据。
%   这里不添加随机噪声。仿真模式下，StateAndMeasurement.prepareMonteCarloRun
%   会根据 setConfig.m 中的噪声参数，为每次蒙特卡洛实验重新加噪。

% IMU 与主时间轴同频，由相邻导航真值反推理想陀螺和加速度计输出。
[gyro, accel] = createIdealImu( ...
    time, trajectory.PositionLlh, trajectory.VelocityEnu, trajectory.AttitudeCbn);

numSamples = numel(time);

% DVL、深度计和 GPS 可以低频输出，这里把各自采样周期映射到主时间轴索引。
dvlIndex = makeSensorIndex(numSamples, cfg.SampleInterval, cfg.DvlSampleInterval);
depthIndex = makeSensorIndex(numSamples, cfg.SampleInterval, cfg.DepthSampleInterval);
gpsIndex = makeSensorIndex(numSamples, cfg.SampleInterval, cfg.GpsSampleInterval);

% DVL 量测使用体坐标系速度，不能直接使用 ENU 速度。
dvlVelocityBody = createDvlVelocityBody( ...
    trajectory.VelocityEnu(dvlIndex, :), trajectory.AttitudeCbn(:, :, dvlIndex));

% ENU 高度向上为正，深度计深度向下为正。
depth = cfg.SurfaceAltitude - trajectory.PositionLlh(depthIndex, 3);

sensorData = struct();

% IMU 字段命名与 StateAndMeasurement 中的读取逻辑保持一致。
sensorData.Imu = struct();
sensorData.Imu.Time = time;
sensorData.Imu.Gyro = gyro;
sensorData.Imu.Accel = accel;

% 低频传感器均提供 valid 标志，便于后续统一处理缺测或异常数据。
sensorData.Dvl = struct();
sensorData.Dvl.Time = time(dvlIndex);
sensorData.Dvl.VelocityBody = dvlVelocityBody;
sensorData.Dvl.Valid = true(numel(dvlIndex), 1);

sensorData.Depth = struct();
sensorData.Depth.Time = time(depthIndex);
sensorData.Depth.Depth = depth;
sensorData.Depth.Valid = true(numel(depthIndex), 1);

sensorData.Gps = struct();
sensorData.Gps.Time = time(gpsIndex);
sensorData.Gps.PositionLlh = trajectory.PositionLlh(gpsIndex, :);
sensorData.Gps.Valid = true(numel(gpsIndex), 1);

end
