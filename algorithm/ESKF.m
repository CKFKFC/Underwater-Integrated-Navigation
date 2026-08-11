%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数: 水下 ENU 坐标系 ESKF 组合导航流程
% 作者: Kefan Chen
% 日期: 2026-07-04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function results = ESKF(cfg, meas, results)
%ESKF 水下 ESKF 组合导航流程函数。
%   ESKF 只组织算法流程，误差状态模型由配置文件选择。

%% 初始化
algorithmName = "ESKF";
stateModel = InertialErrorStateModel(cfg);
time = meas.getTime();
numSamples = meas.getNumSamples();
progressStepCount = max(1, round(1.0 / cfg.sim.dt));
results.registerAlgorithm( ...
    algorithmName, cfg.sim.runs, numSamples, stateModel.GetMetadata());

%% 滤波主循环
for mc = 1:cfg.sim.runs
    %% 初始化本次 Monte Carlo
    % 每次 MC 重新生成传感器噪声。
    meas.prepareMonteCarloRun(mc);

    % 导航初值扰动和 P0 属于 ESKF 内部参数。
    [navSol, imuBias, initialP] = stateModel.CreateInitialNavigation(meas);
    fil = ErrorStateKF(stateModel, initialP);

    positionHistory = nan(3, numSamples);
    velocityHistory = nan(3, numSamples);
    eulerHistory = nan(3, numSamples);
    gyroBiasHistory = nan(3, numSamples);
    accelBiasHistory = nan(3, numSamples);

    positionHistory(:, 1) = navSol.PositionLlh(:);
    velocityHistory(:, 1) = navSol.VelocityEnu(:);
    eulerHistory(:, 1) = eulerFromDcm(navSol.Cbn);
    gyroBiasHistory(:, 1) = imuBias.GyroBias(:);
    accelBiasHistory(:, 1) = imuBias.AccelBias(:);

    for sampleIndex = 2:numSamples
        if mod(sampleIndex - 1, progressStepCount) == 0
            fprintf("第%d/%d次MC，仿真运行到第%.2f秒\n", mc, cfg.sim.runs, time(sampleIndex));
        end

        %% 准备当前滤波时刻
        dt = time(sampleIndex) - time(sampleIndex - 1);
        if ~isfinite(dt) || dt <= 0.0
            dt = cfg.sim.dt;
        end
        fil.Preparation(sampleIndex);

        %% 惯导解算
        % 惯导解算仍在滤波时间更新之外显式执行。
        imu = meas.getImu(sampleIndex);
        [navSol, correctedImu] = ErrorStateKF.PropagateNavigation(navSol, imuBias, imu, dt);

        %% 误差状态预测
        % 时间更新只递推所选模型的误差状态和协方差。
        fil.Predict(navSol, correctedImu, dt);

        %% DVL 量测更新
        dvl = meas.getDvl(sampleIndex);
        if dvl.Valid
            fil.UpdateDvl(navSol, dvl);
        end

        %% 深度量测更新
        depth = meas.getDepth(sampleIndex);
        if depth.Valid
            fil.UpdateDepth(navSol, depth);
        end

        %% GPS 量测更新
        gps = meas.getGps(sampleIndex);
        if gps.Valid
            fil.UpdateGps(navSol, gps);
        end

        %% 闭环反馈
        % 将估计误差注入导航解算结果。
        errorState = fil.GetErrorState();
        [navSol, imuBias] = stateModel.FeedbackNavigation(navSol, imuBias, errorState);

        % 反馈后必须清零误差状态，避免下一步重复补偿。
        fil.ResetErrorState();

        %% 保存当前时刻结果
        positionHistory(:, sampleIndex) = navSol.PositionLlh(:);
        velocityHistory(:, sampleIndex) = navSol.VelocityEnu(:);
        eulerHistory(:, sampleIndex) = eulerFromDcm(navSol.Cbn);
        gyroBiasHistory(:, sampleIndex) = imuBias.GyroBias(:);
        accelBiasHistory(:, sampleIndex) = imuBias.AccelBias(:);
    end

    results.Data.(algorithmName).PositionLlh(:, :, mc) = positionHistory.';
    results.Data.(algorithmName).VelocityEnu(:, :, mc) = velocityHistory.';
    results.Data.(algorithmName).Euler(:, :, mc) = eulerHistory.';
    results.Data.(algorithmName).GyroBias(:, :, mc) = gyroBiasHistory.';
    results.Data.(algorithmName).AccelBias(:, :, mc) = accelBiasHistory.';
end

end
