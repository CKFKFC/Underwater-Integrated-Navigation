function positionLlh = enuOffsetToLlh(referenceLlh, enuOffset)
%ENUOFFSETTOLLH 将局部 ENU 位移叠加到纬经高。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 将局部 ENU 修正量转换为纬经高修正。

referenceLlh = double(referenceLlh(:));
enuOffset = double(enuOffset(:));
[RM, RN] = earthRadii(referenceLlh(1));

latitude = referenceLlh(1) + enuOffset(2) / (RM + referenceLlh(3));
longitude = referenceLlh(2) + enuOffset(1) / ((RN + referenceLlh(3)) * cos(referenceLlh(1)));
height = referenceLlh(3) + enuOffset(3);
positionLlh = [latitude; longitude; height];

end


