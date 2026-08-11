function cfg = applyOptionOverrides(cfg, overrides)
%APPLYOPTIONOVERRIDES 将临时参数覆盖到默认配置中。

overrideNames = fieldnames(overrides);
for nameIndex = 1:numel(overrideNames)
    fieldName = overrideNames{nameIndex};

    % 只允许覆盖 setTrajectoryOptions.m 中已经声明的字段，避免拼写错误被静默忽略。
    if ~isfield(cfg, fieldName)
        error("genetraj:UnknownOption", ...
            "Unknown trajectory option: %s. Please check setTrajectoryOptions.m.", fieldName);
    end

    cfg.(fieldName) = overrides.(fieldName);
end

end
