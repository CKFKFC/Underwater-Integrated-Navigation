function errorEnu = llh2enuError(targetLlh, referenceLlh)
%LLH2ENUERROR 将纬经高差转换为局部 ENU 米级误差。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 计算目标相对参考点的局部 ENU 误差。

wasColumn = iscolumn(targetLlh);
targetRows = reshape(double(targetLlh), [], 3);
referenceRows = reshape(double(referenceLlh), [], 3);

if size(referenceRows, 1) == 1 && size(targetRows, 1) > 1
    referenceRows = repmat(referenceRows, size(targetRows, 1), 1);
end

errorRows = zeros(size(targetRows));
for rowIndex = 1:size(targetRows, 1)
    reference = referenceRows(rowIndex, :).';
    target = targetRows(rowIndex, :).';
    [RM, RN] = earthRadii(reference(1));
    deltaLatitude = target(1) - reference(1);
    deltaLongitude = wrapAngle(target(2) - reference(2));
    deltaHeight = target(3) - reference(3);

    east = deltaLongitude * (RN + reference(3)) * cos(reference(1));
    north = deltaLatitude * (RM + reference(3));
    up = deltaHeight;
    errorRows(rowIndex, :) = [east, north, up];
end

if wasColumn
    errorEnu = errorRows.';
else
    errorEnu = errorRows;
end

end


