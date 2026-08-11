function C = orthonormalizeDcmLocal(C)
%ORTHONORMALIZEDCMLOCAL 用 SVD 修正 DCM 的数值正交性。

% SVD 投影可把带有微小数值误差的矩阵拉回最近的正交矩阵。
[U, ~, V] = svd(C);
C = U * V.';
if det(C) < 0.0
    % det 为负时表示出现反射，需要翻转最后一列以保持右手旋转矩阵。
    U(:, 3) = -U(:, 3);
    C = U * V.';
end

end
