function constants = wgs84ConstantsLocal()
%WGS84CONSTANTSLOCAL 返回轨迹生成和 IMU 反算所需的 WGS-84 常数。

constants = struct();

% WGS-84 椭球长半轴，单位 m。
constants.a = 6378137.0;

% WGS-84 椭球扁率和第一偏心率平方。
constants.f = 1.0 / 298.257223563;
constants.e2 = constants.f * (2.0 - constants.f);

% 地球自转角速度，单位 rad/s。
constants.wie = 7.292115e-5;

% 正常重力公式中的赤道重力和 Somigliana 常数。
constants.gammaE = 9.7803253359;
constants.k = 0.00193185265241;

end
