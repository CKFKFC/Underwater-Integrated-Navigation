function rotationVector = rotationVectorFromDcm(rotationMatrix)
%ROTATIONVECTORFROMDCM 从旋转矩阵提取旋转矢量。

% trace 计算旋转角；先限幅，避免浮点误差导致 acos 输入越界。
traceArgument = 0.5 * (trace(rotationMatrix) - 1.0);
traceArgument = min(max(traceArgument, -1.0), 1.0);
angle = acos(traceArgument);

% 反对称部分给出旋转轴乘以 sin(angle)。
axisTimesSine = 0.5 * [
    rotationMatrix(3, 2) - rotationMatrix(2, 3)
    rotationMatrix(1, 3) - rotationMatrix(3, 1)
    rotationMatrix(2, 1) - rotationMatrix(1, 2)
    ];

if angle > 1.0e-8
    rotationVector = axisTimesSine * angle / sin(angle);
else
    % 小角度时 sin(angle) 约等于 angle，直接使用反对称部分更稳定。
    rotationVector = axisTimesSine;
end

end
