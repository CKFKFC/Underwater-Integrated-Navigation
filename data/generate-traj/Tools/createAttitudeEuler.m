function attitudeEuler = createAttitudeEuler(time, velocityEnu, cfg)
%CREATEATTITUDEEULER 由速度方向生成 RFU [roll, pitch, yaw] 姿态真值。
%   yaw 北向为零、顺时针为正；pitch 抬头为正；roll 右侧下沉为正。

% 航向角由水平速度方向给出；atan2(east, north) 表示北零顺时针航向。
horizontalSpeed = hypot(velocityEnu(:, 1), velocityEnu(:, 2));
yaw = unwrap(atan2(velocityEnu(:, 1), velocityEnu(:, 2)));

% 俯仰角由垂向速度和水平速度给出，向上速度对应正俯仰。
pitch = atan2(velocityEnu(:, 3), max(horizontalSpeed, eps));

% 滚转角只作为小幅姿态激励，不参与决定轨迹位置。
roll = createRollAngle(time, yaw, cfg);

attitudeEuler = [roll, pitch, yaw];

end

function roll = createRollAngle(time, yaw, cfg)
%CREATEROLLANGLE 生成姿态真值中的小幅滚转角。

trajectoryType = lower(string(cfg.TrajectoryType));
if trajectoryType == "lawnmower"
    % 割草机掉头阶段根据航向角速度给出滚转，直线测线段滚转接近 0。
    yawRate = gradient(yaw, time);
    yawRateScale = max(cfg.StraightSpeed / cfg.LawnmowerTurnRadius, eps);
    roll = deg2rad(cfg.RollAmplitudeDeg) * max(-1.0, min(1.0, yawRate / yawRateScale));
else
    % 其他轨迹使用低频正弦滚转，提供温和姿态激励。
    rollOmega = pi / cfg.HorizontalPeriod;
    roll = deg2rad(cfg.RollAmplitudeDeg) * sin(rollOmega * time);
end

end
