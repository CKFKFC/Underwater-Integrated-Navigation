function [gyro, accel] = createIdealImu(time, positionLlh, velocityEnu, attitudeCbn)
%CREATEIDEALIMU 由相邻两帧导航真值反推理想 IMU 输出。
%   gyro  是载体相对惯性空间的角速度，体坐标表达，单位 rad/s。
%   accel 是比力，体坐标表达，单位 m/s^2。

numSamples = numel(time);
gyro = zeros(numSamples, 3);
accel = zeros(numSamples, 3);

% 第 sampleIndex 帧 IMU 由 sampleIndex-1 到 sampleIndex 的导航状态变化反推。
for sampleIndex = 2:numSamples
    dt = time(sampleIndex) - time(sampleIndex - 1);
    [gyroBody, accelBody] = inverseInsStep( ...
        attitudeCbn(:, :, sampleIndex - 1), velocityEnu(sampleIndex - 1, :).', ...
        positionLlh(sampleIndex - 1, :).', attitudeCbn(:, :, sampleIndex), ...
        velocityEnu(sampleIndex, :).', positionLlh(sampleIndex, :).', dt);

    gyro(sampleIndex, :) = gyroBody.';
    accel(sampleIndex, :) = accelBody.';
end

% StateAndMeasurement 要求 IMU 与主时间轴同长。第 1 帧没有前一帧可反推，
% 因此用第 2 帧填充占位；ESKF 从 sampleIndex=2 开始真正使用 IMU。
gyro(1, :) = gyro(2, :);
accel(1, :) = accel(2, :);

end
