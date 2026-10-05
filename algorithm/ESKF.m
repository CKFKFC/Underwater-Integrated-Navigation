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

% stateModel 是本次运行的状态模型唯一来源。ESKF 主流程不判断 9/15 维，
% 只把导航状态交给模型构造矩阵，并把模型元数据随结果保存用于复现实验。
stateModel = InertialErrorStateModel(cfg);
time = meas.getTime();
numSamples = meas.getNumSamples();
isDelayEnabled = isfield(cfg, "sensorDelay") ...
    && logical(cfg.sensorDelay.isEnabled);
progressStepCount = max(1, round(1.0 / cfg.sim.dt));
results.registerAlgorithm( ...
    algorithmName, cfg.sim.runs, numSamples, stateModel.GetMetadata());

%% 滤波主循环
for mc = 1:cfg.sim.runs
    %% 初始化本次 Monte Carlo
    % 每次 MC 重新生成传感器噪声。
    meas.prepareMonteCarloRun(mc);

    % 导航初值扰动、P0 和滤波器维数均来自同一个状态布局，避免三者错位。
    [navSol, imuBias, initialP] = stateModel.CreateInitialNavigation(meas);
    fil = ErrorStateKF(stateModel, initialP);
    laggedNavSol = navSol;
    hasDelayedImu = false;
    lastImuSampleTime = time(1);
    lastFilterTime = time(1);

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

        %% 计算当前主时间步长
        dt = time(sampleIndex) - time(sampleIndex - 1);
        if ~isfinite(dt) || dt <= 0.0
            dt = cfg.sim.dt;
        end

        if isDelayEnabled
            %% 延时 IMU 解算与一阶前向补偿
            imu = meas.getImu(sampleIndex);
            if ~imu.Valid
                positionHistory(:, sampleIndex) = navSol.PositionLlh(:);
                velocityHistory(:, sampleIndex) = navSol.VelocityEnu(:);
                eulerHistory(:, sampleIndex) = eulerFromDcm(navSol.Cbn);
                gyroBiasHistory(:, sampleIndex) = imuBias.GyroBias(:);
                accelBiasHistory(:, sampleIndex) = imuBias.AccelBias(:);
                continue;
            end

            if hasDelayedImu
                imuDt = imu.SampleTime - lastImuSampleTime;
                if ~isfinite(imuDt) || imuDt <= 0.0
                    imuDt = cfg.sim.dt;
                end
                [laggedNavSol, correctedImu] = ErrorStateKF.PropagateNavigation( ...
                    laggedNavSol, imuBias, imu, imuDt);
            else
                correctedImu = ErrorStateKF.CompensateImu(imuBias, imu);
                hasDelayedImu = true;
            end

            navSol = SensorDelayCompensator.forwardImu( ...
                laggedNavSol, correctedImu, imu.DelaySeconds);
            lastImuSampleTime = imu.SampleTime;

            %% 将误差状态预测到导航计算机当前时刻
            predictDt = time(sampleIndex) - lastFilterTime;
            if ~isfinite(predictDt) || predictDt <= 0.0
                predictDt = dt;
            end
            fil.Preparation(sampleIndex);
            fil.Predict(navSol, correctedImu, predictDt);
            lastFilterTime = time(sampleIndex);

            %% 使用各外部量测采样时刻的临时导航状态构造新息
            dvl = meas.getDvl(sampleIndex);
            if dvl.Valid
                % navSol 的姿态仍属于最新已到达 IMU 的采样时刻。非零杆臂时
                % 将临时 DVL 姿态推到 DVL.SampleTime；角速度保持最新已到达值，
                % 是短延时的常角速度近似，不读取尚未到达的 IMU。
                attitudeDeltaTime = 0.0;
                if any(cfg.sensor.dvl.leverArmBody ~= 0.0)
                    attitudeDeltaTime = dvl.SampleTime - imu.SampleTime;
                end
                dvlNavSol = SensorDelayCompensator.backPropagateVelocity( ...
                    navSol, correctedImu, dvl.DelaySeconds, attitudeDeltaTime);
                fil.UpdateDvl(dvlNavSol, dvl, correctedImu);
            end

            depth = meas.getDepth(sampleIndex);
            if depth.Valid
                depthNavSol = SensorDelayCompensator.backPropagateHeight( ...
                    navSol, depth.DelaySeconds);
                fil.UpdateDepth(depthNavSol, depth);
            end

            gps = meas.getGps(sampleIndex);
            if gps.Valid
                gpsNavSol = SensorDelayCompensator.backPropagateNavigation( ...
                    navSol, correctedImu, gps.DelaySeconds);
                fil.UpdateGps(gpsNavSol, gps);
            end
        else
            %% 无延时惯导解算与误差状态预测
            fil.Preparation(sampleIndex);
            imu = meas.getImu(sampleIndex);
            [navSol, correctedImu] = ErrorStateKF.PropagateNavigation( ...
                navSol, imuBias, imu, dt);
            fil.Predict(navSol, correctedImu, dt);

            %% 无延时外部量测更新
            dvl = meas.getDvl(sampleIndex);
            if dvl.Valid
                fil.UpdateDvl(navSol, dvl, correctedImu);
            end

            depth = meas.getDepth(sampleIndex);
            if depth.Valid
                fil.UpdateDepth(navSol, depth);
            end

            gps = meas.getGps(sampleIndex);
            if gps.Valid
                fil.UpdateGps(navSol, gps);
            end
        end

        %% 闭环反馈
        % 将估计误差注入导航解算结果。
        errorState = fil.GetErrorState();
        imuBiasBeforeFeedback = imuBias;
        [navSol, imuBias] = stateModel.FeedbackNavigation(navSol, imuBias, errorState);

        if isDelayEnabled
            % 延时时间仅为 30 ms，将同一个导航误差修正量注入滞后状态，保证
            % 下一帧完整机械编排不会丢失当前量测反馈。零偏只在当前状态反馈一次。
            [laggedNavSol, ~] = stateModel.FeedbackNavigation( ...
                laggedNavSol, imuBiasBeforeFeedback, errorState);
        end

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
