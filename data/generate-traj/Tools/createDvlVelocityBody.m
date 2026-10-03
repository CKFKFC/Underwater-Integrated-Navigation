function velocityBody = createDvlVelocityBody(velocityEnu, attitudeCbn)
%CREATEDVLVELOCITYBODY 将 ENU 速度转换为 DVL 的 RFU 体坐标速度。
%   当前项目规定 DVL 量测字段必须是 dvl.velocityBody，而不是 ENU 速度。

numSamples = size(velocityEnu, 1);
velocityBody = zeros(numSamples, 3);
for sampleIndex = 1:numSamples
    % Cbn 为体坐标到 ENU 的转换矩阵，因此转置后得到 ENU 到体坐标的 Cnb。
    Cnb = attitudeCbn(:, :, sampleIndex).';
    velocityBody(sampleIndex, :) = (Cnb * velocityEnu(sampleIndex, :).').';
end

end
