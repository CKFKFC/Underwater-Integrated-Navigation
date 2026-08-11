function euler = eulerFromDcm(Cbn)
%EULERFROMDCM 将体到 ENU 的 DCM 转为欧拉角。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 从 C_b_n 中提取 ZYX 欧拉角。

pitchArgument = -Cbn(3, 1);
pitchArgument = min(max(pitchArgument, -1.0), 1.0);
pitch = asin(pitchArgument);
roll = atan2(Cbn(3, 2), Cbn(3, 3));
yaw = atan2(Cbn(2, 1), Cbn(1, 1));
euler = [roll; pitch; yaw];

end


