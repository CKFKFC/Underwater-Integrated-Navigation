function outputFile = resolveOutputFile(cfg)
%RESOLVEOUTPUTFILE 计算轨迹数据的输出 MAT 文件路径。

if strlength(cfg.OutputFile) == 0
    % 未显式指定输出文件时，固定保存到 data/input/navigation_input.mat。
    generatorFolder = fileparts(mfilename("fullpath"));
    dataFolder = fileparts(generatorFolder);
    outputFile = fullfile(dataFolder, "input", "navigation_input.mat");
else
    % 指定 OutputFile 时完全尊重用户给出的相对路径或绝对路径。
    outputFile = cfg.OutputFile;
end

end
