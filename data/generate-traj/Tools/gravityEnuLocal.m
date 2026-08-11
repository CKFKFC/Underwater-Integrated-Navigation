function gravity = gravityEnuLocal(latitude, altitude)
%GRAVITYENULOCAL 计算 ENU 坐标中向上为正的正常重力。

constants = wgs84ConstantsLocal();
sinLatitude = sin(latitude);

% 先计算椭球面正常重力，再按高度做一阶修正。
gravityMagnitude = constants.gammaE ...
    * (1.0 + constants.k * sinLatitude^2) ...
    / sqrt(1.0 - constants.e2 * sinLatitude^2);
gravityMagnitude = gravityMagnitude * (1.0 - (2.0 * altitude / constants.a));

% ENU 的 up 轴向上为正，重力方向向下，因此第三个分量为负。
gravity = [0.0; 0.0; -gravityMagnitude];

end
