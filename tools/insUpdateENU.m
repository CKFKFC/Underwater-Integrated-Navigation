function [newCbn, newVelocityEnu, newPositionLlh] = insUpdateENU( ...
    oldCbn, oldVelocityEnu, oldPositionLlh, gyroBody, accelBody, dt)
%INSUPDATEENU 执行 ENU 坐标下的捷联惯导递推。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 更新姿态矩阵、ENU 速度和纬经高位置。

constants = getWgs84Constants();
latitude = oldPositionLlh(1);
altitude = oldPositionLlh(3);

alphaBody = gyroBody * dt;
alphaNorm = norm(alphaBody);
alphaSkew = skew(alphaBody);

wIeN = constants.wie * [0.0; cos(latitude); sin(latitude)];
[oldRM, oldRN] = earthRadii(latitude);
wEnN = [
    -oldVelocityEnu(2) / (oldRM + altitude)
    oldVelocityEnu(1) / (oldRN + altitude)
    oldVelocityEnu(1) * tan(latitude) / (oldRN + altitude)
    ];

if alphaNorm > 1.0e-8
    CavgBody = eye(3) ...
        + ((1.0 - cos(alphaNorm)) / alphaNorm^2) * alphaSkew ...
        + ((1.0 - (sin(alphaNorm) / alphaNorm)) / alphaNorm^2) * alphaSkew * alphaSkew;
    avgCbn = oldCbn * CavgBody - 0.5 * dt * skew(wEnN + wIeN) * oldCbn;
else
    avgCbn = oldCbn - 0.5 * dt * skew(wEnN + wIeN) * oldCbn;
end

accelEnu = avgCbn * accelBody;
newVelocityEnu = oldVelocityEnu ...
    + dt * accelEnu ...
    + dt * gravityENU(latitude, altitude) ...
    - dt * skew(wEnN + 2.0 * wIeN) * oldVelocityEnu;

newAltitude = altitude + 0.5 * dt * (oldVelocityEnu(3) + newVelocityEnu(3));

oldLatitudeRate = oldVelocityEnu(2) / (oldRM + altitude);
predictedLatitude = latitude + dt * oldLatitudeRate;
[predictedRM, ~] = earthRadii(predictedLatitude);
newLatitudeRate = newVelocityEnu(2) / (predictedRM + newAltitude);
newLatitude = latitude + 0.5 * dt * (oldLatitudeRate + newLatitudeRate);
[newRM, newRN] = earthRadii(newLatitude);

newLongitude = oldPositionLlh(2) + 0.5 * dt * ( ...
    oldVelocityEnu(1) / ((oldRN + altitude) * cos(latitude)) ...
    + newVelocityEnu(1) / ((newRN + newAltitude) * cos(newLatitude)));
newPositionLlh = [newLatitude; newLongitude; newAltitude];

newWEnN = [
    -newVelocityEnu(2) / (newRM + newAltitude)
    newVelocityEnu(1) / (newRN + newAltitude)
    newVelocityEnu(1) * tan(newLatitude) / (newRN + newAltitude)
    ];

if alphaNorm > 1.0e-8
    CnewBodyOldBody = eye(3) ...
        + (sin(alphaNorm) / alphaNorm) * alphaSkew ...
        + ((1.0 - cos(alphaNorm)) / alphaNorm^2) * alphaSkew * alphaSkew;
else
    CnewBodyOldBody = eye(3) + alphaSkew;
end

newCbn = (eye(3) - skew(wIeN + 0.5 * wEnN + 0.5 * newWEnN) * dt) ...
    * oldCbn * CnewBodyOldBody;
newCbn = orthonormalizeDcm(newCbn);

end


