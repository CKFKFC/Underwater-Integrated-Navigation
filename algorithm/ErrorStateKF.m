classdef ErrorStateKF < handle
    %ERRORSTATEKF 可变维误差状态卡尔曼滤波器数值核心。
    %   状态语义、动力学、量测模型和反馈由 InertialErrorStateModel 提供。
    %   本类只实现与状态维数无关的预测、量测更新和反馈后清零逻辑。

    properties (Access = private)
        Model_       % 提供当前 profile 对应的 F、G、H 和状态布局
        PreX_        % 本时刻预测前保存的误差状态
        PreP_        % 本时刻预测前保存的协方差
        CurX_        % 已完成当前预测/量测更新的误差状态
        CurP_        % 已完成当前预测/量测更新的协方差
        CurStep_     % 当前主时间轴样本序号，用于保留滤波时刻语义
    end

    methods
        %% 构造可变维误差状态滤波器
        function obj = ErrorStateKF(model, initialCovariance)
            arguments
                model InertialErrorStateModel
                initialCovariance double
            end

            stateDimension = model.Layout.Dimension;
            if ~isequal(size(initialCovariance), [stateDimension, stateDimension])
                error("ErrorStateKF:InvalidInitialCovariance", ...
                    "Initial covariance must be %d-by-%d for profile %s.", ...
                    stateDimension, stateDimension, model.ProfileName);
            end

            % 状态维数只从模型布局读取；更换 profile 不需要修改本类的分配逻辑。
            obj.Model_ = model;
            obj.PreX_ = zeros(stateDimension, 1);
            obj.CurX_ = zeros(stateDimension, 1);
            obj.PreP_ = initialCovariance;
            obj.CurP_ = initialCovariance;
            obj.CurStep_ = 0;
        end

        %% 准备进入新的滤波时刻
        function Preparation(obj, sampleIndex)
            %PREPARATION 保存上一个滤波时刻的误差状态与协方差。
            %   同一时刻的多个量测都更新 CurX_/CurP_；进入下一时刻前，再将其
            %   固化为预测所使用的 PreX_/PreP_，避免量测更新之间互相覆盖。
            obj.CurStep_ = sampleIndex;
            obj.PreX_ = obj.CurX_;
            obj.PreP_ = obj.CurP_;
        end

        %% 执行误差状态预测
        function Predict(obj, navSol, correctedImu, dt)
            %PREDICT 递推误差状态，不递推名义导航解。
            [F, G, Qc] = obj.Model_.BuildErrorDynamics(navSol, correctedImu);
            stateDimension = obj.Model_.Layout.Dimension;

            % 当前采样周期较短，采用一阶离散化 Phi=I+F*dt；连续白噪声通过
            % Qd=G*Qc*G'*dt 映射到离散协方差。两者维数均由模型自动决定。
            Phi = eye(stateDimension) + F * dt;
            Qd = G * Qc * G.' * dt;

            obj.CurX_ = Phi * obj.PreX_;
            obj.CurP_ = Phi * obj.PreP_ * Phi.' + Qd;
            obj.CurP_ = ErrorStateKF.SymmetrizeCovariance(obj.CurP_);
        end

        %% 执行 DVL 量测更新
        function UpdateDvl(obj, navSol, measurement, correctedImu)
            %UPDATEDVL 执行 DVL 速度量测更新。
            arguments
                obj
                navSol struct
                measurement struct
                correctedImu struct = struct()
            end
            [residual, H, R] = obj.Model_.BuildDvlMeasurement(navSol, measurement, correctedImu);
            obj.KalmanUpdate(residual, H, R);
        end

        %% 执行深度量测更新
        function UpdateDepth(obj, navSol, measurement)
            %UPDATEDEPTH 执行深度量测更新。
            [residual, H, R] = obj.Model_.BuildDepthMeasurement(navSol, measurement);
            obj.KalmanUpdate(residual, H, R);
        end

        %% 执行 GPS 量测更新
        function UpdateGps(obj, navSol, measurement)
            %UPDATEGPS 执行 GPS 位置量测更新。
            [residual, H, R] = obj.Model_.BuildGpsMeasurement(navSol, measurement);
            obj.KalmanUpdate(residual, H, R);
        end

        %% 获取当前误差状态
        function errorState = GetErrorState(obj)
            %GETERRORSTATE 返回当前误差状态估计。
            errorState = obj.CurX_;
        end

        %% 获取当前协方差
        function covariance = GetCovariance(obj)
            %GETCOVARIANCE 返回当前误差状态协方差。
            covariance = obj.CurP_;
        end

        %% 闭环反馈后清零误差状态
        function ResetErrorState(obj)
            %RESETERRORSTATE 闭环反馈后清零误差状态均值。
            %   协方差表示反馈后的剩余不确定度，不能随状态均值一起清零。
            obj.CurX_ = zeros(obj.Model_.Layout.Dimension, 1);
            obj.PreX_ = obj.CurX_;
        end
    end

    methods (Static)
        %% 执行零偏补偿后的惯导机械编排
        function [navSol, correctedImu] = PropagateNavigation(navSol, imuBias, imu, dt)
            %PROPAGATENAVIGATION 执行零偏补偿后的惯导机械编排。
            %   先用当前名义零偏补偿原始 IMU，再将补偿后的角速度和比力同时
            %   提供给机械编排与误差状态动力学，保证两条传播链使用相同输入。
            correctedImu = ErrorStateKF.CompensateImu(imuBias, imu);

            [navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh] = insUpdateENU( ...
                navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh, ...
                correctedImu.Gyro, correctedImu.Accel, dt);
        end

        %% 执行 IMU 名义零偏补偿
        function correctedImu = CompensateImu(imuBias, imu)
            %COMPENSATEIMU 使用当前名义零偏修正原始 IMU 量测。
            correctedImu = struct();
            correctedImu.Time = imu.Time;
            correctedImu.Gyro = imu.Gyro - imuBias.GyroBias;
            correctedImu.Accel = imu.Accel - imuBias.AccelBias;
        end
    end

    methods (Access = private)
        %% 执行通用线性卡尔曼量测更新
        function KalmanUpdate(obj, residual, H, R)
            %KALMANUPDATE 对任意维量测执行一次线性误差状态更新。
            residual = residual(:);

            % 多量测在同一时刻串行到达时，CurX_ 可能已被前一个量测更新，
            % 因此创新必须扣除 H*CurX_，不能直接把原始残差重复当作创新。
            innovation = residual - H * obj.CurX_;
            S = H * obj.CurP_ * H.' + R;
            K = (obj.CurP_ * H.') / S;
            obj.CurX_ = obj.CurX_ + K * innovation;

            % 使用 Joseph 形式更新协方差。相比简式 (I-KH)P，它在有限精度下
            % 更能保持半正定性；最后显式对称化用于消除舍入产生的微小非对称。
            identityMatrix = eye(size(obj.CurP_));
            obj.CurP_ = (identityMatrix - K * H) * obj.CurP_ ...
                * (identityMatrix - K * H).' + K * R * K.';
            obj.CurP_ = ErrorStateKF.SymmetrizeCovariance(obj.CurP_);
        end
    end

    methods (Static, Access = private)
        %% 消除协方差的浮点非对称误差
        function covariance = SymmetrizeCovariance(covariance)
            %SYMMETRIZECOVARIANCE 消除浮点矩阵运算导致的反对称舍入误差。
            covariance = 0.5 * (covariance + covariance.');
        end
    end
end
