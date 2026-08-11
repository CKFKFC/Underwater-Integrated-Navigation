function Cbn = dcmFromEuler(euler)
%DCMFROMEULER 将滚转俯仰航向角转换为体到 ENU 的 DCM。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 按 ZYX 航向-俯仰-滚转顺序构造 C_b_n。

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

Rx = [
    1.0, 0.0, 0.0
    0.0, cr, -sr
    0.0, sr, cr
    ];
Ry = [
    cp, 0.0, sp
    0.0, 1.0, 0.0
    -sp, 0.0, cp
    ];
Rz = [
    cy, -sy, 0.0
    sy, cy, 0.0
    0.0, 0.0, 1.0
    ];

Cbn = Rz * Ry * Rx;

end


