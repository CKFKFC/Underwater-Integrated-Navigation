function [profileName, blocks] = createInertialStateProfile(cfg)
%CREATEINERTIALSTATEPROFILE 按配置组合惯导误差状态块。
%   Profile 是切换状态模型的唯一入口，只负责声明“包含哪些块及其顺序”。
%   每个块的数学耦合由 InertialErrorStateModel 负责，滤波数值核心无需修改。

arguments
    cfg struct
end

if ~isfield(cfg, "algorithm") ...
        || ~isfield(cfg.algorithm, "stateModel") ...
        || ~isfield(cfg.algorithm.stateModel, "profile")
    error("createInertialStateProfile:MissingProfile", ...
        "cfg.algorithm.stateModel.profile must be configured.");
end

profileName = string(cfg.algorithm.stateModel.profile);
if ~isscalar(profileName) || ismissing(profileName)
    error("createInertialStateProfile:InvalidProfile", ...
        "cfg.algorithm.stateModel.profile must be one nonmissing string scalar.");
end
% 基础块 Attitude、Velocity、Position 始终排在最前。ins15 在相同 9 维基底
% 后追加两个零偏块，因此基础模型的语义和索引在不同 profile 间保持稳定。
switch profileName
    case "ins9"
        blockNames = ["Attitude", "Velocity", "Position"];
    case "ins15"
        blockNames = ["Attitude", "Velocity", "Position", "GyroBias", "AccelBias"];
    otherwise
        error("createInertialStateProfile:UnknownProfile", ...
            "Unknown state-model profile: %s. Supported profiles are ins9 and ins15.", ...
            profileName);
end

% 先预分配同构 struct 数组，再逐块填入配置，避免循环中动态增长数组。
blocks = repmat(createInertialStateBlock(cfg, blockNames(1)), 1, numel(blockNames));
for blockIndex = 2:numel(blockNames)
    blocks(blockIndex) = createInertialStateBlock(cfg, blockNames(blockIndex));
end

end
