function block = createInertialStateBlock(cfg, blockName)
%CREATEINERTIALSTATEBLOCK 创建一个惯导误差状态块定义。
%   状态块只描述状态的语义、维数、单位和初始标准差，不负责分配全局索引。
%   所有标准差在配置层已经转换为 SI 单位，可直接用于随机初始误差和 P0。

arguments
    cfg struct
    blockName (1, 1) string
end

initialError = getInitialErrorConfig(cfg);

% 此处是状态名称与配置字段之间的唯一映射。未来新增状态块时，应在这里
% 明确其初始不确定度、单位和参考坐标系，再由 profile 决定是否启用。
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

% Dimension 从标准差向量推导，防止块定义维数与 P0 配置长度不一致。
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
% 配置允许标量表示三轴同值，也允许逐轴设置；输出始终规范为列向量。
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
