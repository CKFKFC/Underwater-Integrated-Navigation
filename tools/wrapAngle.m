function angle = wrapAngle(angle)
%WRAPANGLE 将角度归一化到 [-pi, pi]。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 不依赖工具箱完成弧度角归一化。

angle = mod(angle + pi, 2.0 * pi) - pi;

end


