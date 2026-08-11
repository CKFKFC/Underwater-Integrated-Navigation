function trajectory = generateTrajectoryShape(time, cfg)
%GENERATETRAJECTORYSHAPE 生成轨迹形状并转换为纬经高真值。
%   局部 ENU 只用于轨迹设计和可视化；最终滤波输入使用 PositionLlh。

% 水平轨迹和垂向轨迹分开生成，最后合成为完整 ENU 位置和速度。
[horizontalPosition, horizontalVelocity, horizontalInfo] = generateHorizontalMotion(time, cfg);
[upPosition, upVelocity, depthInfo] = generateDepthMotion(time, cfg);

positionEnu = [
    horizontalPosition(:, 1), horizontalPosition(:, 2), upPosition
    ];
velocityEnu = [
    horizontalVelocity(:, 1), horizontalVelocity(:, 2), upVelocity
    ];

% 高度 h 向上为正，深度 depth 向下为正，所以初始高度等于水面高度减初始深度。
initialHeight = cfg.SurfaceAltitude - cfg.InitialDepth;
referenceLlh = [
    deg2rad(cfg.InitialLatitudeDeg)
    deg2rad(cfg.InitialLongitudeDeg)
    initialHeight
    ];

% 将每个 ENU 偏移点转换为纬经高，作为惯导和滤波输入使用的位置真值。
positionLlh = zeros(numel(time), 3);
for sampleIndex = 1:numel(time)
    positionLlh(sampleIndex, :) = enuOffsetToLlhLocal(referenceLlh, positionEnu(sampleIndex, :).').';
end

% 姿态先用欧拉角描述，再转换为体坐标到导航坐标的 DCM。
attitudeEuler = createAttitudeEuler(time, velocityEnu, cfg);
attitudeCbn = zeros(3, 3, numel(time));
for sampleIndex = 1:numel(time)
    attitudeCbn(:, :, sampleIndex) = dcmFromEulerLocal(attitudeEuler(sampleIndex, :).');
end

trajectory = struct();
trajectory.PositionEnu = positionEnu;
trajectory.PositionLlh = positionLlh;
trajectory.VelocityEnu = velocityEnu;
trajectory.AttitudeEuler = attitudeEuler;
trajectory.AttitudeCbn = attitudeCbn;
trajectory.ReferenceLlh = referenceLlh;
trajectory.HorizontalInfo = horizontalInfo;
trajectory.DepthInfo = depthInfo;

end
