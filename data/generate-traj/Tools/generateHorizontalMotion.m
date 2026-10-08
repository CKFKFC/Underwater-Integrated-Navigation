function [position, velocity, info, heading] = generateHorizontalMotion(time, cfg)
%GENERATEHORIZONTALMOTION 生成局部 ENU 坐标下的水平轨迹。
%   position 和 velocity 分别包含 east/north 位置与速度分量，单位为 m 和 m/s。
%   heading 为可选的北零顺时针航向（rad）；空值表示由速度方向生成航向。

% 轨迹类型统一从 cfg.TrajectoryType 读取；外部只需要调用 genetraj。
trajectoryType = lower(string(cfg.TrajectoryType));
heading = zeros(0, 1);
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
    case "rectangle"
        [position, velocity, heading, isTurning] = createRectangleMotion(time, cfg);
        description = "constant-depth rectangle with in-place heading changes";
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
elseif trajectoryType == "rectangle"
    info.Length = cfg.RectangleLength;
    info.Width = cfg.RectangleWidth;
    info.TurnDuration = cfg.RectangleTurnDuration;
    info.Perimeter = 2.0*(cfg.RectangleLength + cfg.RectangleWidth);
    info.CycleDuration = info.Perimeter/cfg.StraightSpeed + 4.0*cfg.RectangleTurnDuration;
    info.IsTurning = isTurning;
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

function [position, velocity, heading, isTurning] = createRectangleMotion(time, cfg)
%CREATERECTANGLEMOTION 沿长方形四边前进，在顶点停止平移并原地右转 90 deg。

legLengths = [cfg.RectangleLength, cfg.RectangleWidth, ...
    cfg.RectangleLength, cfg.RectangleWidth];
legDurations = legLengths/cfg.StraightSpeed;
sideEnds = cumsum(legDurations + cfg.RectangleTurnDuration);
cycleTime = mod(time, sideEnds(end));
completedCycles = floor(time/sideEnds(end));
course = deg2rad(cfg.CourseDeg);

position = zeros(numel(time), 2);
velocity = zeros(numel(time), 2);
heading = course + 2.0*pi*completedCycles;
isTurning = false(size(time));
sideStart = 0.0;
startPosition = [0.0, 0.0];
directions = [0.0, 1.0; 1.0, 0.0; 0.0, -1.0; -1.0, 0.0];

% 按各段持续时间解析求值，停车转向期间位置保持在同一个顶点。
for sideIndex = 1:4
    forward = directions(sideIndex, :);
    onSide = (cycleTime >= sideStart) & (cycleTime < sideEnds(sideIndex));
    sideTime = cycleTime - sideStart;
    onLeg = onSide & (sideTime < legDurations(sideIndex));
    onTurn = onSide & ~onLeg;

    position(onLeg, :) = startPosition + cfg.StraightSpeed*sideTime(onLeg).*forward;
    velocity(onLeg, 1) = cfg.StraightSpeed*forward(1);
    velocity(onLeg, 2) = cfg.StraightSpeed*forward(2);

    vertex = startPosition + legLengths(sideIndex)*forward;
    position(onTurn, 1) = vertex(1);
    position(onTurn, 2) = vertex(2);
    heading(onSide) = heading(onSide) + (sideIndex-1)*pi/2.0;

    % 航向独立于平移速度，避免零速度时 atan2(0, 0) 把航向重置为北向。
    turnFraction = (sideTime(onTurn)-legDurations(sideIndex))/cfg.RectangleTurnDuration;
    heading(onTurn) = heading(onTurn) + (pi/2.0)*turnFraction;
    isTurning(onTurn) = true;

    startPosition = vertex;
    sideStart = sideEnds(sideIndex);
end

% CourseDeg 旋转整个长方形，使第一条长边沿指定航向。
rotation = [cos(course), -sin(course); sin(course), cos(course)];
position = position*rotation;
velocity = velocity*rotation;

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
