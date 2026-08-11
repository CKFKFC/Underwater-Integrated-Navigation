function Cbn = dcmFromEulerLocal(euler)
%DCMFROMEULERLOCAL 将 [roll; pitch; yaw] 转换为体坐标到 ENU 的 DCM。
%   旋转顺序与项目 tools/dcmFromEuler.m 保持一致，为 ZYX 航向-俯仰-滚转。

roll = euler(1);
pitch = euler(2);
yaw = euler(3);

cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
cy = cos(yaw);
sy = sin(yaw);

% 三个基础旋转矩阵按 Rz(yaw) * Ry(pitch) * Rx(roll) 组合。
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
