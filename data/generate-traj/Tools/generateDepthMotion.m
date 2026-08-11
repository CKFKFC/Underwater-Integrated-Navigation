function [upPosition, upVelocity, info] = generateDepthMotion(time, cfg)
%GENERATEDEPTHMOTION 生成局部 ENU 坐标下的垂向运动。
%   upPosition 向上为正；设计深度 depth 向下为正，二者符号相反。

% 深度类型统一从 cfg.DepthMotionType 读取，便于在 genetraj 中统一配置。
depthType = lower(string(cfg.DepthMotionType));
switch depthType
    case "sine"
        [depthOffset, depthRate] = createSineDepth(time, cfg);
        description = "sinusoidal depth motion";
    case "constant"
        depthOffset = zeros(size(time));
        depthRate = zeros(size(time));
        description = "constant-depth motion";
    case "diveclimb"
        [depthOffset, depthRate] = createDiveClimbDepth(time, cfg);
        description = "single dive-climb depth motion";
    case "auvheave"
        [depthOffset, depthRate] = createAuvHeaveDepth(time, cfg);
        description = "near-constant-depth AUV heave from reference trajectories";
    otherwise
        error("genetraj:InvalidDepthMotionType", ...
            "Unsupported DepthMotionType: %s.", cfg.DepthMotionType);
end

% ENU 的 up 轴向上为正，因此深度偏移和深度速度需要取反。
upPosition = -depthOffset;
upVelocity = -depthRate;

info = struct();
info.Type = cfg.DepthMotionType;
info.Description = description;
info.DepthOffset = depthOffset;
info.DepthRate = depthRate;

end

function [depthOffset, depthRate] = createSineDepth(time, cfg)
%CREATESINEDEPTH 生成相对 InitialDepth 的正弦深度偏移。

omega = 2.0 * pi / cfg.DepthPeriod;
phase = deg2rad(cfg.DepthPhaseDeg);
rawDepth = cfg.DepthAmplitude * sin(omega * time + phase);

% 平移曲线，保证 t=0 时深度偏移为 0，即初始深度等于 cfg.InitialDepth。
depthOffset = rawDepth - rawDepth(1);
depthRate = cfg.DepthAmplitude * omega * cos(omega * time + phase);

end

function [depthOffset, depthRate] = createDiveClimbDepth(time, cfg)
%CREATEDIVECLIMBDEPTH 生成一个平滑下潜-上浮深度周期。

% 余弦形状在周期起点速度为 0，适合作为平滑深度激励。
omega = 2.0 * pi / cfg.DepthPeriod;
depthOffset = 0.5 * cfg.DepthAmplitude * (1.0 - cos(omega * time));
depthRate = 0.5 * cfg.DepthAmplitude * omega * sin(omega * time);

end

function [depthOffset, depthRate] = createAuvHeaveDepth(time, cfg)
%CREATEAUVHEAVEDEPTH 生成近似定深航行中的低频小幅升沉。

trajectoryType = lower(string(cfg.TrajectoryType));
switch trajectoryType
    case "lawnmower"
        % 割草机轨迹采用略强的低频升沉，模拟测线航行中的缓慢深度变化。
        speed = cfg.StraightSpeed;
        heightOffset = 0.55 * sin(2.0 * pi * time / 1100.0 + 0.25) ...
            + 0.22 * sin(2.0 * pi * time / 310.0 + 1.0) ...
            + 0.18 * sin(2.0 * pi * speed * time / 700.0 + 0.6);
        heightRate = 0.55 * (2.0 * pi / 1100.0) * cos(2.0 * pi * time / 1100.0 + 0.25) ...
            + 0.22 * (2.0 * pi / 310.0) * cos(2.0 * pi * time / 310.0 + 1.0) ...
            + 0.18 * (2.0 * pi * speed / 700.0) * cos(2.0 * pi * speed * time / 700.0 + 0.6);
    otherwise
        % 其他轨迹采用更小幅值的缓慢高度扰动，避免垂向运动掩盖水平机动。
        heightOffset = 0.4 * sin(2.0 * pi * time / 850.0 + 0.3) ...
            + 0.15 * sin(2.0 * pi * time / 230.0 + 1.1);
        heightRate = 0.4 * (2.0 * pi / 850.0) * cos(2.0 * pi * time / 850.0 + 0.3) ...
            + 0.15 * (2.0 * pi / 230.0) * cos(2.0 * pi * time / 230.0 + 1.1);
end

% 先保证初始高度偏移为 0，再转换为向下为正的深度偏移。
heightOffset = heightOffset - heightOffset(1);
depthOffset = -heightOffset;
depthRate = -heightRate;

end
