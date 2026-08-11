function S = skewLocal(vector)
%SKEWLOCAL 构造三维向量的反对称矩阵。
%   对任意三维向量 a，有 skewLocal(v) * a = cross(v, a)。

vector = vector(:);
S = [
    0.0, -vector(3), vector(2)
    vector(3), 0.0, -vector(1)
    -vector(2), vector(1), 0.0
    ];

end
