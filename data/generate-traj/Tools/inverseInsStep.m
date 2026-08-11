function [gyroBody, accelBody] = inverseInsStep( ...
    oldCbn, oldVelocityEnu, oldPositionLlh, newCbn, newVelocityEnu, newPositionLlh, dt)
%INVERSEINSSTEP 按项目 ENU 捷联惯导方程反推一帧理想 IMU。
%   这里的符号约定与 tools/insUpdateENU.m 对齐，保证生成的数据能被当前
%   惯导解算正确积分。

constants = wgs84ConstantsLocal();
oldLatitude = oldPositionLlh(1);
oldAltitude = oldPositionLlh(3);
newLatitude = newPositionLlh(1);
newAltitude = newPositionLlh(3);

[oldRM, oldRN] = earthRadiiLocal(oldLatitude);
[newRM, newRN] = earthRadiiLocal(newLatitude);

% 地球自转角速度和导航系相对地球的转动角速度，均在 ENU 导航系表达。
oldWIeN = constants.wie * [0.0; cos(oldLatitude); sin(oldLatitude)];
oldWEnN = [
    -oldVelocityEnu(2) / (oldRM + oldAltitude)
    oldVelocityEnu(1) / (oldRN + oldAltitude)
    oldVelocityEnu(1) * tan(oldLatitude) / (oldRN + oldAltitude)
    ];

% 新旧两帧曲率半径可能不同，因此新时刻导航系转动也单独计算。
newWEnN = [
    -newVelocityEnu(2) / (newRM + newAltitude)
    newVelocityEnu(1) / (newRN + newAltitude)
    newVelocityEnu(1) * tan(newLatitude) / (newRN + newAltitude)
    ];

% 姿态反算：先去掉导航系自身旋转，再求体坐标增量旋转。
navigationCompensation = eye(3) - skewLocal(oldWIeN + 0.5 * oldWEnN + 0.5 * newWEnN) * dt;
bodyIncrement = oldCbn.' * (navigationCompensation \ newCbn);
bodyIncrement = orthonormalizeDcmLocal(bodyIncrement);
alphaBody = rotationVectorFromDcm(bodyIncrement);
gyroBody = alphaBody / dt;

% 比力反算：由速度增量扣除重力，并补偿地球自转和导航系转动项。
specificForceEnu = (newVelocityEnu - oldVelocityEnu) / dt ...
    - gravityEnuLocal(oldLatitude, oldAltitude) ...
    + skewLocal(oldWEnN + 2.0 * oldWIeN) * oldVelocityEnu;

% 将比力从导航系转换到体坐标系时，使用小角度积分平均姿态。
alphaNorm = norm(alphaBody);
alphaSkew = skewLocal(alphaBody);
if alphaNorm > 1.0e-8
    avgBodyIncrement = eye(3) ...
        + ((1.0 - cos(alphaNorm)) / alphaNorm^2) * alphaSkew ...
        + ((1.0 - sin(alphaNorm) / alphaNorm) / alphaNorm^2) * alphaSkew * alphaSkew;
    avgCbn = oldCbn * avgBodyIncrement - 0.5 * dt * skewLocal(oldWEnN + oldWIeN) * oldCbn;
else
    avgCbn = oldCbn - 0.5 * dt * skewLocal(oldWEnN + oldWIeN) * oldCbn;
end

accelBody = avgCbn.' * specificForceEnu;

end
