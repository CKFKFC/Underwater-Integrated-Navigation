function cfg = validateTrajectoryOptions(cfg)
%VALIDATETRAJECTORYOPTIONS 校验 genetraj 的轨迹和传感器配置参数。

% 字符串类配置先统一转为 string，后续比较时兼容 char 和 string 输入。
cfg.OutputFile = string(cfg.OutputFile);
cfg.TrajectoryType = string(cfg.TrajectoryType);
cfg.DepthMotionType = string(cfg.DepthMotionType);

% 基础数值参数校验：先保证类型和正负号合法，再进入轨迹类型相关校验。
requireScalar(cfg.SaveToFile, "SaveToFile");
requirePositive(cfg.Duration, "Duration");
requirePositive(cfg.SampleInterval, "SampleInterval");
requireFinite(cfg.InitialLatitudeDeg, "InitialLatitudeDeg");
requireFinite(cfg.InitialLongitudeDeg, "InitialLongitudeDeg");
requireNonnegative(cfg.InitialDepth, "InitialDepth");
requireFinite(cfg.SurfaceAltitude, "SurfaceAltitude");
requirePositive(cfg.HorizontalRadius, "HorizontalRadius");
requirePositive(cfg.HorizontalPeriod, "HorizontalPeriod");
requirePositive(cfg.StraightSpeed, "StraightSpeed");
requireFinite(cfg.CourseDeg, "CourseDeg");
requirePositive(cfg.LawnmowerLegLength, "LawnmowerLegLength");
requirePositive(cfg.LawnmowerLaneSpacing, "LawnmowerLaneSpacing");
requirePositive(cfg.LawnmowerTurnRadius, "LawnmowerTurnRadius");
requireNonnegative(cfg.LateralAmplitude, "LateralAmplitude");
requirePositive(cfg.LateralPeriod, "LateralPeriod");
requireNonnegative(cfg.DepthAmplitude, "DepthAmplitude");
requirePositive(cfg.DepthPeriod, "DepthPeriod");
requireFinite(cfg.DepthPhaseDeg, "DepthPhaseDeg");
requireNonnegative(cfg.RollAmplitudeDeg, "RollAmplitudeDeg");
requirePositive(cfg.DvlSampleInterval, "DvlSampleInterval");
validateattributes(cfg.DvlLeverArmBody, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 3}, 'genetraj', 'DvlLeverArmBody');
cfg.DvlLeverArmBody = double(cfg.DvlLeverArmBody(:));
requirePositive(cfg.DepthSampleInterval, "DepthSampleInterval");
requirePositive(cfg.GpsSampleInterval, "GpsSampleInterval");

% 当前局部 ENU 转换采用参考点一阶近似，不适合靠近极区的纬度。
if abs(cfg.InitialLatitudeDeg) >= 89.0
    error("genetraj:InvalidLatitude", ...
        "InitialLatitudeDeg should stay away from the poles for this local ENU generator.");
end

% 水平轨迹类型必须与 generateHorizontalMotion 中的分支保持一致。
validTrajectoryTypes = ["figure8", "circle", "straight", "lawnmower", "sCurve"];
if ~any(strcmpi(cfg.TrajectoryType, validTrajectoryTypes))
    error("genetraj:InvalidTrajectoryType", ...
        "TrajectoryType must be one of: %s.", strjoin(validTrajectoryTypes, ", "));
end

% 深度轨迹类型必须与 generateDepthMotion 中的分支保持一致。
validDepthTypes = ["sine", "constant", "diveClimb", "auvHeave"];
if ~any(strcmpi(cfg.DepthMotionType, validDepthTypes))
    error("genetraj:InvalidDepthMotionType", ...
        "DepthMotionType must be one of: %s.", strjoin(validDepthTypes, ", "));
end

% 至少需要两帧，才能由相邻状态反推出第一段 IMU 数据。
if floor(cfg.Duration / cfg.SampleInterval) + 1 < 2
    error("genetraj:TooFewSamples", ...
        "Duration and SampleInterval must produce at least two samples.");
end

% 割草机掉头由两个 90 deg 定半径圆弧组成，测线间距不能小于转弯直径。
if strcmpi(cfg.TrajectoryType, "lawnmower") ...
        && cfg.LawnmowerLaneSpacing < 2.0 * cfg.LawnmowerTurnRadius
    error("genetraj:InvalidLawnmowerGeometry", ...
        "LawnmowerLaneSpacing must be at least 2 * LawnmowerTurnRadius.");
end

end

function requireScalar(value, fieldName)
%REQUIRESCALAR 要求参数为标量。
if ~isscalar(value)
    error("genetraj:InvalidOption", "%s must be a scalar value.", fieldName);
end
end

function requireFinite(value, fieldName)
%REQUIREFINITE 要求参数为有限数值标量。
if ~isscalar(value) || ~isnumeric(value) || ~isfinite(value)
    error("genetraj:InvalidOption", "%s must be a finite numeric scalar.", fieldName);
end
end

function requirePositive(value, fieldName)
%REQUIREPOSITIVE 要求参数为正数。
requireFinite(value, fieldName);
if value <= 0.0
    error("genetraj:InvalidOption", "%s must be positive.", fieldName);
end
end

function requireNonnegative(value, fieldName)
%REQUIRENONNEGATIVE 要求参数为非负数。
requireFinite(value, fieldName);
if value < 0.0
    error("genetraj:InvalidOption", "%s must be nonnegative.", fieldName);
end
end
