function sensorIndex = makeSensorIndex(numSamples, imuSampleInterval, sensorSampleInterval)
%MAKESENSORINDEX 将低频传感器采样周期映射到主 IMU 时间轴索引。

% 按周期比取整得到主时间轴上的采样步长，至少每 1 个 IMU 点取一次。
step = max(1, round(sensorSampleInterval / imuSampleInterval));
sensorIndex = (1:step:numSamples).';

% 强制包含最后一帧，保证低频传感器数据覆盖完整仿真时长。
if sensorIndex(end) ~= numSamples
    sensorIndex = [sensorIndex; numSamples];
end

end
