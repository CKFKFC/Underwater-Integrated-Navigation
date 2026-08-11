function [RM, RN] = earthRadii(latitude)
%EARTHRADII 计算子午圈和卯酉圈曲率半径。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 返回指定纬度处的 WGS-84 曲率半径。

constants = getWgs84Constants();
sinLatitude = sin(latitude);
denominator = sqrt(1.0 - constants.e2 * sinLatitude^2);
RN = constants.a / denominator;
RM = constants.a * (1.0 - constants.e2) / denominator^3;

end


