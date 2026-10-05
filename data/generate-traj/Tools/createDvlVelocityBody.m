function velocityBody = createDvlVelocityBody(velocityEnu, attitudeCbn, gyroBody, positionLlh, leverArmBody)
%CREATEDVLVELOCITYBODY 生成 DVL 安装点的对地速度，以 INS RFU 三轴表达。
%   真值采用 omega_eb^b×l，滤波中使用 omega_ib^b×l 是单独的工程近似。
%   gyroBody 为理想 omega_ib^b，rad/s；杆臂从 INS 指向 DVL，单位 m。
arguments
    velocityEnu (:, 3) double
    attitudeCbn (3, 3, :) double
    gyroBody (:, 3) double = zeros(0, 3)
    positionLlh (:, 3) double = zeros(0, 3)
    leverArmBody (3, 1) double {mustBeReal, mustBeFinite} = zeros(3, 1)
end

numSamples = size(velocityEnu, 1);
hasLeverArm = any(leverArmBody ~= 0.0);
if size(attitudeCbn, 3) ~= numSamples
    error("createDvlVelocityBody:InvalidAttitudeSize", "Attitude and velocity sample counts must match.");
end
if hasLeverArm && (size(gyroBody, 1) ~= numSamples || size(positionLlh, 1) ~= numSamples)
    error("createDvlVelocityBody:MissingLeverArmInputs", ...
        "A nonzero lever arm requires ideal gyro and position at every DVL sample.");
end
constants = wgs84ConstantsLocal();
velocityBody = zeros(numSamples, 3);
for sampleIndex = 1:numSamples
    % Cbn 为体坐标到 ENU 的转换矩阵，因此转置后得到 ENU 到体坐标的 Cnb。
    Cnb = attitudeCbn(:, :, sampleIndex).';
    velocityBody(sampleIndex, :) = (Cnb * velocityEnu(sampleIndex, :).').';
    if hasLeverArm
        latitude = positionLlh(sampleIndex, 1);
        earthRateEnu = constants.wie * [0.0; cos(latitude); sin(latitude)];
        angularRateEarthBody = gyroBody(sampleIndex, :).' - Cnb * earthRateEnu;
        velocityBody(sampleIndex, :) = velocityBody(sampleIndex, :) ...
            + cross(angularRateEarthBody, leverArmBody).';
    end
end

end
