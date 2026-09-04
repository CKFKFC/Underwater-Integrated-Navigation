function block = createInertialStateBlock(cfg, blockName)
%CREATEINERTIALSTATEBLOCK 构造一个惯导误差状态块定义。
%   状态块只描述状态语义、单位、初值扰动和 P0 标准差，不负责分配全局索引。

arguments
    cfg struct
    blockName (1, 1) string
end

initialPerturbation = getInitialConfig(cfg, "initialPerturbation");
initialCovariance = getInitialConfig(cfg, "initialCovariance");

% 此处是状态名称与配置字段之间的唯一映射。未来新增状态块时，应在这里
% 明确其初始不确定度、单位和参考坐标系，再由 profile 决定是否启用。
switch blockName
    case "Attitude"
        fieldName = "attitudeStd";
        unit = "rad";
        frame = "ENU";
    case "Velocity"
        fieldName = "velocityStd";
        unit = "m/s";
        frame = "ENU";
    case "Position"
        fieldName = "positionStd";
        unit = "m";
        frame = "ENU";
    case "GyroBias"
        fieldName = "gyroBiasStd";
        unit = "rad/s";
        frame = "RFU";
    case "AccelBias"
        fieldName = "accelBiasStd";
        unit = "m/s^2";
        frame = "RFU";
    otherwise
        error("createInertialStateBlock:UnknownBlock", ...
            "Unknown inertial error-state block: %s.", blockName);
end

perturbationStd = getConfigStd(initialPerturbation, fieldName, 3, "initialPerturbation");
covarianceStd = getConfigStd(initialCovariance, fieldName, 3, "initialCovariance");

% Dimension 从标准差向量推导，防止状态块定义与配置长度不一致。
block = struct();
block.Name = blockName;
block.Dimension = numel(covarianceStd);
block.PerturbationStd = perturbationStd;
block.CovarianceStd = covarianceStd;
block.Unit = unit;
block.Frame = frame;

end

function initialConfig = getInitialConfig(cfg, configName)
if ~isfield(cfg, "algorithm") || ~isfield(cfg.algorithm, configName)
    error("createInertialStateBlock:MissingInitialConfig", ...
        "cfg.algorithm.%s must be configured in setConfig.", configName);
end
initialConfig = cfg.algorithm.(configName);
end

function stdVector = getConfigStd(config, fieldName, dimension, configName)
% 配置允许标量表示三轴同值，也允许逐轴设置；输出始终规范为列向量。
fieldName = char(fieldName);
if ~isfield(config, fieldName)
    error("createInertialStateBlock:MissingInitialStd", ...
        "cfg.algorithm.%s.%s must be configured in setConfig.", configName, fieldName);
end

stdVector = double(config.(fieldName)(:));
if isscalar(stdVector)
    stdVector = repmat(stdVector, dimension, 1);
end
if numel(stdVector) ~= dimension
    error("createInertialStateBlock:InvalidInitialStd", ...
        "cfg.algorithm.%s.%s must be scalar or %d-by-1.", ...
        configName, fieldName, dimension);
end
if any(~isfinite(stdVector)) || any(stdVector < 0.0)
    error("createInertialStateBlock:InvalidInitialStd", ...
        "cfg.algorithm.%s.%s must contain finite nonnegative values.", ...
        configName, fieldName);
end
end
