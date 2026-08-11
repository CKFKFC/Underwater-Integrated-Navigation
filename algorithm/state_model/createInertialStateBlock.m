function block = createInertialStateBlock(cfg, blockName)
%CREATEINERTIALSTATEBLOCK 创建一个惯导误差状态块定义。
%   状态块只描述状态的语义、维数和初始标准差，不负责分配全局索引。

arguments
    cfg struct
    blockName (1, 1) string
end

initialError = getInitialErrorConfig(cfg);

switch blockName
    case "Attitude"
        initialStd = getConfigStd(initialError, "attitudeStd", 3);
        unit = "rad";
        frame = "ENU";
    case "Velocity"
        initialStd = getConfigStd(initialError, "velocityStd", 3);
        unit = "m/s";
        frame = "ENU";
    case "Position"
        initialStd = getConfigStd(initialError, "positionStd", 3);
        unit = "m";
        frame = "ENU";
    case "GyroBias"
        initialStd = getConfigStd(initialError, "gyroBiasStd", 3);
        unit = "rad/s";
        frame = "Body";
    case "AccelBias"
        initialStd = getConfigStd(initialError, "accelBiasStd", 3);
        unit = "m/s^2";
        frame = "Body";
    otherwise
        error("createInertialStateBlock:UnknownBlock", ...
            "Unknown inertial error-state block: %s.", blockName);
end

block = struct();
block.Name = blockName;
block.Dimension = numel(initialStd);
block.InitialStd = initialStd;
block.Unit = unit;
block.Frame = frame;

end

function initialError = getInitialErrorConfig(cfg)
if ~isfield(cfg, "algorithm") || ~isfield(cfg.algorithm, "initialError")
    error("createInertialStateBlock:MissingInitialErrorConfig", ...
        "cfg.algorithm.initialError must be configured in setConfig.");
end
initialError = cfg.algorithm.initialError;
end

function stdVector = getConfigStd(config, fieldName, dimension)
fieldName = char(fieldName);
if ~isfield(config, fieldName)
    error("createInertialStateBlock:MissingInitialStd", ...
        "cfg.algorithm.initialError.%s must be configured in setConfig.", fieldName);
end

stdVector = double(config.(fieldName)(:));
if isscalar(stdVector)
    stdVector = repmat(stdVector, dimension, 1);
end
if numel(stdVector) ~= dimension
    error("createInertialStateBlock:InvalidInitialStd", ...
        "Initial standard deviation must be scalar or %d-by-1.", dimension);
end
end
