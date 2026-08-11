function positionLlh = enuOffsetToLlhLocal(referenceLlh, enuOffset)
%ENUOFFSETTOLLHLOCAL 将局部 ENU 位移转换为纬经高。
%   referenceLlh = [lat; lon; h]，单位为 rad、rad、m。
%   enuOffset = [east; north; up]，单位 m。

% 当前轨迹范围较小，使用参考点处的局部曲率半径做一阶 ENU 到 LLH 转换。
[RM, RN] = earthRadiiLocal(referenceLlh(1));
latitude = referenceLlh(1) + enuOffset(2) / (RM + referenceLlh(3));
longitude = referenceLlh(2) + enuOffset(1) / ((RN + referenceLlh(3)) * cos(referenceLlh(1)));
height = referenceLlh(3) + enuOffset(3);
positionLlh = [latitude; longitude; height];

end
