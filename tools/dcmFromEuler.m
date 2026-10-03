function Cbn = dcmFromEuler(euler)
%DCMFROMEULER 将 RFU 欧拉角转换为体到 ENU 的 DCM。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: b 系为右前上，n 系为东北天；航向北零顺时针，抬头和右倾为正。

euler = euler(:);
roll = euler(1);
pitch = euler(2);
yaw = euler(3);

cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
cy = cos(yaw);
sy = sin(yaw);

pitchAboutRight = [
    1.0, 0.0, 0.0
    0.0, cp, -sp
    0.0, sp, cp
    ];
rollAboutForward = [
    cr, 0.0, sr
    0.0, 1.0, 0.0
    -sr, 0.0, cr
    ];
headingAboutUp = [
    cy, sy, 0.0
    -sy, cy, 0.0
    0.0, 0.0, 1.0
    ];

Cbn = headingAboutUp * pitchAboutRight * rollAboutForward;

end


