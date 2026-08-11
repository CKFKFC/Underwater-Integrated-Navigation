function ControlRandomNumber(randomselect)
%CONTROLRANDOMNUMBER 配置实验随机数。
% 作者: Kefan Chen
% 日期: 2026-07-03
% 功能: 保存或恢复随机数状态，便于复现实验。
%   CONTROLRANDOMNUMBER(RANDOMSELECT) 支持两种模式：
%   "random"     - 按当前时间初始化，并保存 randstates.mat。
%   "repeatlast" - 读取上次保存的随机数状态。

if nargin < 1 || isempty(randomselect)
    randomselect = "random";
else
    randomselect = lower(string(randomselect));
end

projectRoot = fileparts(fileparts(mfilename("fullpath")));
stateFile = fullfile(projectRoot, "randstates.mat");

switch randomselect
    case "random"
        rng("shuffle");
        randstate = rng();
        save(stateFile, "randstate");

    case "repeatlast"
        if isfile(stateFile)
            savedState = load(stateFile, "randstate");
            rng(savedState.randstate);
        else
            warning("ControlRandomNumber:StateFileNotFound", ...
                "randstates.mat was not found. A new random state will be created.");
            rng("shuffle");
            randstate = rng();
            save(stateFile, "randstate");
        end

    otherwise
        error("ControlRandomNumber:InvalidMode", ...
            "randomselect must be either ""random"" or ""repeatlast"".");
end

end


