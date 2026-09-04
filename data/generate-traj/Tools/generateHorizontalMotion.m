function [position, velocity, info] = generateHorizontalMotion(time, cfg)
%GENERATEHORIZONTALMOTION 生成局部 ENU 坐标下的水平轨迹。
%   position 和 velocity 分别包含 east/north 位置与速度分量，单位为 m 和 m/s。

% 轨迹类型统一从 cfg.TrajectoryType 读取；外部只需要调用 genetraj。
trajectoryType = lower(string(cfg.TrajectoryType));
switch trajectoryType
    case "figure8"
        [position, velocity] = createFigure8Motion(time, cfg);
        description = "figure-8 horizontal motion";
    case "circle"
        [position, velocity] = createCircleMotion(time, cfg);
        description = "circular horizontal motion";
    case "straight"
        [position, velocity] = createStraightMotion(time, cfg);
        description = "straight constant-speed motion";
    case "lawnmower"
        [position, velocity] = createLawnmowerMotion(time, cfg);
        description = "lawnmower AUV survey pattern";
    case "scurve"
        [position, velocity] = createSCurveMotion(time, cfg);
        description = "sinusoidal lateral s-curve";
    otherwise
        error("genetraj:InvalidTrajectoryType", ...
            "Unsupported TrajectoryType: %s.", cfg.TrajectoryType);
end

info = struct();
info.Type = cfg.TrajectoryType;
info.Description = description;
info.Coordinate = "local ENU horizontal position [east, north], unit m";

if trajectoryType == "lawnmower"
    % 记录割草机几何信息，便于后续检查相邻测线间距和掉头半径。
    info.LegLength = cfg.LawnmowerLegLength;
    info.LaneSpacing = cfg.LawnmowerLaneSpacing;
    info.TurnRadius = cfg.LawnmowerTurnRadius;
    info.TurnConnectorLength = cfg.LawnmowerLaneSpacing ...
        - 2.0 * cfg.LawnmowerTurnRadius;
end

end

function [position, velocity] = createFigure8Motion(time, cfg)
%CREATEFIGURE8MOTION 生成 8 字形水平轨迹。

% 采用解析函数生成位置和速度，避免数值差分带来的速度噪声。
omega = 2.0 * pi / cfg.HorizontalPeriod;
east = cfg.HorizontalRadius * sin(omega * time);
north = 0.5 * cfg.HorizontalRadius * sin(2.0 * omega * time);

eastVelocity = cfg.HorizontalRadius * omega * cos(omega * time);
northVelocity = cfg.HorizontalRadius * omega * cos(2.0 * omega * time);

position = [east, north];
velocity = [eastVelocity, northVelocity];

end

function [position, velocity] = createCircleMotion(time, cfg)
%CREATECIRCLEMOTION 生成从原点出发的圆形水平轨迹。

% 圆心位于初始点北侧，使 t=0 时位置正好为 [0, 0]。
omega = 2.0 * pi / cfg.HorizontalPeriod;
east = cfg.HorizontalRadius * sin(omega * time);
north = cfg.HorizontalRadius * (1.0 - cos(omega * time));

eastVelocity = cfg.HorizontalRadius * omega * cos(omega * time);
northVelocity = cfg.HorizontalRadius * omega * sin(omega * time);

position = [east, north];
velocity = [eastVelocity, northVelocity];

end

function [position, velocity] = createStraightMotion(time, cfg)
%CREATESTRAIGHTMOTION 生成匀速直线轨迹。

% CourseDeg 与 RFU 欧拉航向一致：北零、顺时针为正。
course = deg2rad(cfg.CourseDeg);
eastVelocity = cfg.StraightSpeed * sin(course) * ones(size(time));
northVelocity = cfg.StraightSpeed * cos(course) * ones(size(time));

east = eastVelocity .* time;
north = northVelocity .* time;

position = [east, north];
velocity = [eastVelocity, northVelocity];

end

function [position, velocity] = createLawnmowerMotion(time, cfg)
%CREATELAWNMOWERMOTION 生成局部 ENU 平面内的割草机测线轨迹。

numSamples = numel(time);
position = zeros(numSamples, 2);
velocity = zeros(numSamples, 2);

% 割草机模型由“直线测线 + 两个 90 deg 定半径转弯 + 可选横向连接段”组成。
speed = cfg.StraightSpeed;
legLength = cfg.LawnmowerLegLength;
laneSpacing = cfg.LawnmowerLaneSpacing;
turnRadius = cfg.LawnmowerTurnRadius;
connectorLength = laneSpacing - 2.0 * turnRadius;
heading = pi / 2.0;
turnDirection = -1.0;
currentPosition = [0.0, 0.0];
segmentType = "leg";
distanceInSegment = 0.0;
turnAngle = 0.0;

velocity(1, :) = speed * [cos(heading), sin(heading)];
for sampleIndex = 2:numSamples
    dtRemaining = time(sampleIndex) - time(sampleIndex - 1);
    while dtRemaining > eps
        % 一个采样周期可能跨过多个轨迹段，因此用 while 消耗剩余时间。
        switch segmentType
            case "leg"
                [currentPosition, distanceInSegment, dtRemaining, isDone] = ...
                    advanceStraightSegment(currentPosition, heading, speed, ...
                    distanceInSegment, legLength, dtRemaining);
                if isDone
                    segmentType = "turn1";
                    distanceInSegment = 0.0;
                    turnAngle = 0.0;
                end

            case "turn1"
                [currentPosition, heading, turnAngle, dtRemaining, isDone] = ...
                    advanceTurnSegment(currentPosition, heading, speed, ...
                    turnRadius, turnDirection, turnAngle, pi / 2.0, ...
                    dtRemaining);
                if isDone
                    turnAngle = 0.0;
                    if connectorLength > 1.0e-10
                        segmentType = "connector";
                    else
                        segmentType = "turn2";
                    end
                end

            case "connector"
                [currentPosition, distanceInSegment, dtRemaining, isDone] = ...
                    advanceStraightSegment(currentPosition, heading, speed, ...
                    distanceInSegment, connectorLength, dtRemaining);
                if isDone
                    segmentType = "turn2";
                    distanceInSegment = 0.0;
                    turnAngle = 0.0;
                end

            case "turn2"
                [currentPosition, heading, turnAngle, dtRemaining, isDone] = ...
                    advanceTurnSegment(currentPosition, heading, speed, ...
                    turnRadius, turnDirection, turnAngle, pi / 2.0, ...
                    dtRemaining);
                if isDone
                    segmentType = "leg";
                    distanceInSegment = 0.0;
                    turnAngle = 0.0;
                    turnDirection = -turnDirection;
                end
        end
    end

    position(sampleIndex, :) = currentPosition;
    velocity(sampleIndex, :) = speed * [cos(heading), sin(heading)];
end

end

function [position, distanceInSegment, dtRemaining, isDone] = ...
    advanceStraightSegment(position, heading, speed, distanceInSegment, ...
    segmentLength, dtRemaining)
%ADVANCESTRAIGHTSEGMENT 沿有限长度直线段推进。

% 如果本采样周期走不到段末，则只推进 stepTime；否则切换到下一段。
distanceRemaining = segmentLength - distanceInSegment;
stepTime = min(dtRemaining, distanceRemaining / speed);
position = position + speed * stepTime * [cos(heading), sin(heading)];
distanceInSegment = distanceInSegment + speed * stepTime;
dtRemaining = dtRemaining - stepTime;
isDone = distanceInSegment >= segmentLength - 1.0e-10;

end

function [position, heading, turnAngle, dtRemaining, isDone] = ...
    advanceTurnSegment(position, heading, speed, turnRadius, turnDirection, ...
    turnAngle, targetTurnAngle, dtRemaining)
%ADVANCETURNSEGMENT 沿有限角度的定半径圆弧推进。

% 使用中点航向近似本小步位移，保证短采样周期下的弧线位置足够平滑。
yawRate = turnDirection * speed / turnRadius;
turnTimeRemaining = (targetTurnAngle - turnAngle) / abs(yawRate);
stepTime = min(dtRemaining, turnTimeRemaining);
middleHeading = heading + 0.5 * yawRate * stepTime;
position = position + speed * stepTime * ...
    [cos(middleHeading), sin(middleHeading)];
heading = heading + yawRate * stepTime;
turnAngle = turnAngle + abs(yawRate) * stepTime;
dtRemaining = dtRemaining - stepTime;
isDone = turnAngle >= targetTurnAngle - 1.0e-10;

end

function [position, velocity] = createSCurveMotion(time, cfg)
%CREATESCURVEMOTION 生成向东前进的 S 形水平轨迹。

% 东向保持匀速，北向叠加正弦摆动。
omega = 2.0 * pi / cfg.LateralPeriod;
east = cfg.StraightSpeed * time;
north = cfg.LateralAmplitude * sin(omega * time);

eastVelocity = cfg.StraightSpeed * ones(size(time));
northVelocity = cfg.LateralAmplitude * omega * cos(omega * time);

position = [east, north];
velocity = [eastVelocity, northVelocity];

end
