function constants = getWgs84Constants()
%GETWGS84CONSTANTS 返回 ENU 惯导所需 WGS-84 常数。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 提供椭球、重力和地球自转常数。

constants = struct();
constants.a = 6378137.0;
constants.f = 1.0 / 298.257223563;
constants.e2 = constants.f * (2.0 - constants.f);
constants.wie = 7.292115e-5;
constants.mu = 3.986004418e14;
constants.gammaE = 9.7803253359;
constants.k = 0.00193185265241;

end


