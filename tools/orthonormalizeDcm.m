function C = orthonormalizeDcm(C)
%ORTHONORMALIZEDCM 将矩阵投影为正交姿态矩阵。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 保持姿态矩阵在递推和更新后正交。

[U, ~, V] = svd(C);
C = U * V.';
if det(C) < 0.0
    U(:, 3) = -U(:, 3);
    C = U * V.';
end

end


