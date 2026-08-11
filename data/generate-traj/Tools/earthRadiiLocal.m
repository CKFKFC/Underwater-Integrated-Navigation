function [RM, RN] = earthRadiiLocal(latitude)
%EARTHRADIILOCAL 计算 WGS-84 子午圈和卯酉圈曲率半径。

constants = wgs84ConstantsLocal();
sinLatitude = sin(latitude);

% RN 为卯酉圈曲率半径，RM 为子午圈曲率半径，输入纬度单位为 rad。
denominator = sqrt(1.0 - constants.e2 * sinLatitude^2);
RN = constants.a / denominator;
RM = constants.a * (1.0 - constants.e2) / denominator^3;

end
