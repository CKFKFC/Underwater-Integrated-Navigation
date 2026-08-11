function overrides = parseTrajectoryOverrides(varargin)
%PARSETRAJECTORYOVERRIDES 解析 genetraj 的临时参数覆盖。
%   支持两种形式：
%   1. genetraj(cfgStruct)
%   2. genetraj("Duration", 120.0, "TrajectoryType", "circle")

if nargin == 0
    % 没有外部覆盖参数时，返回空结构体，后续保持默认配置。
    overrides = struct();
    return;
end

if nargin == 1 && isstruct(varargin{1})
    % 单个结构体输入直接作为覆盖参数，适合脚本中批量修改配置。
    overrides = varargin{1};
    return;
end

if rem(nargin, 2) ~= 0
    error("genetraj:InvalidOverrides", ...
        "Temporary overrides must be a struct or name-value pairs.");
end

overrides = struct();
for inputIndex = 1:2:nargin
    fieldName = string(varargin{inputIndex});
    if ~isscalar(fieldName) || strlength(fieldName) == 0
        error("genetraj:InvalidOptionName", ...
            "Option names must be nonempty strings or character vectors.");
    end

    % name-value 输入先临时收集为结构体，字段合法性由 applyOptionOverrides 校验。
    overrides.(char(fieldName)) = varargin{inputIndex + 1};
end

end
