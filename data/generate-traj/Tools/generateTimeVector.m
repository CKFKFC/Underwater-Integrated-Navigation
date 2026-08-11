function time = generateTimeVector(duration, sampleInterval)
%GENERATETIMEVECTOR 生成从 0 开始的列向量时间轴。
%   用整数采样点生成，避免 0:dt:T 的浮点累积误差。

numSamples = floor(duration / sampleInterval) + 1;
time = (0:numSamples - 1).' * sampleInterval;

end
