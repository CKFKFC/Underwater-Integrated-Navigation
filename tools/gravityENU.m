function gravity = gravityENU(latitude, altitude)
%GRAVITYENU 计算 ENU 坐标下的正常重力。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 返回 ENU 中向上为正的重力加速度。

constants = getWgs84Constants();
sinLatitude = sin(latitude);
gravityMagnitude = constants.gammaE ...
    * (1.0 + constants.k * sinLatitude^2) ...
    / sqrt(1.0 - constants.e2 * sinLatitude^2);
gravityMagnitude = gravityMagnitude * (1.0 - (2.0 * altitude / constants.a));
gravity = [0.0; 0.0; -gravityMagnitude];

end


