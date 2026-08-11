function S = skew(vector)
%SKEW 返回三维向量的反对称矩阵。
% 作者: Kefan Chen
% 日期: 2026-07-04
% 功能: 构造叉乘矩阵。

vector = vector(:);
S = [
    0.0, -vector(3), vector(2)
    vector(3), 0.0, -vector(1)
    -vector(2), vector(1), 0.0
    ];

end


