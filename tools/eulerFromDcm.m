function euler = eulerFromDcm(Cbn)
%EULERFROMDCM 将 RFU 体到 ENU 的 DCM 转为欧拉角。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 提取 [roll; pitch; yaw]，航向北零顺时针，抬头和右倾为正。

pitchArgument = Cbn(3, 2);
pitchArgument = min(max(pitchArgument, -1.0), 1.0);
pitch = asin(pitchArgument);
roll = atan2(-Cbn(3, 1), Cbn(3, 3));
yaw = atan2(Cbn(1, 2), Cbn(2, 2));
euler = [roll; pitch; yaw];

end


