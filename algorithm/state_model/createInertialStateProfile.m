function [profileName, blocks] = createInertialStateProfile(cfg)
%CREATEINERTIALSTATEPROFILE 根据配置组合惯导误差状态块。

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

blocks = repmat(createInertialStateBlock(cfg, blockNames(1)), 1, numel(blockNames));
for blockIndex = 2:numel(blockNames)
    blocks(blockIndex) = createInertialStateBlock(cfg, blockNames(blockIndex));
end

end
