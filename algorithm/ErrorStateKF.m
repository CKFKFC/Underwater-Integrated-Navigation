classdef ErrorStateKF < handle
    %ERRORSTATEKF 可变维误差状态卡尔曼滤波器数值核心。
    %   状态语义、动力学、量测模型和反馈由 InertialErrorStateModel 提供。

    properties (Access = private)
        Model_
        PreX_
        PreP_
        CurX_
        CurP_
        CurStep_
    end

    methods
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

            obj.Model_ = model;
            obj.PreX_ = zeros(stateDimension, 1);
            obj.CurX_ = zeros(stateDimension, 1);
            obj.PreP_ = initialCovariance;
            obj.CurP_ = initialCovariance;
            obj.CurStep_ = 0;
        end

        function Preparation(obj, sampleIndex)
            %PREPARATION 保存上一个滤波时刻的误差状态与协方差。
            obj.CurStep_ = sampleIndex;
            obj.PreX_ = obj.CurX_;
            obj.PreP_ = obj.CurP_;
        end

        function Predict(obj, navSol, correctedImu, dt)
            %PREDICT 递推误差状态，不递推名义导航解。
            [F, G, Qc] = obj.Model_.BuildErrorDynamics(navSol, correctedImu);
            stateDimension = obj.Model_.Layout.Dimension;
            Phi = eye(stateDimension) + F * dt;
            Qd = G * Qc * G.' * dt;

            obj.CurX_ = Phi * obj.PreX_;
            obj.CurP_ = Phi * obj.PreP_ * Phi.' + Qd;
            obj.CurP_ = ErrorStateKF.SymmetrizeCovariance(obj.CurP_);
        end

        function UpdateDvl(obj, navSol, measurement)
            %UPDATEDVL 执行 DVL 速度量测更新。
            [residual, H, R] = obj.Model_.BuildDvlMeasurement(navSol, measurement);
            obj.KalmanUpdate(residual, H, R);
        end

        function UpdateDepth(obj, navSol, measurement)
            %UPDATEDEPTH 执行深度量测更新。
            [residual, H, R] = obj.Model_.BuildDepthMeasurement(navSol, measurement);
            obj.KalmanUpdate(residual, H, R);
        end

        function UpdateGps(obj, navSol, measurement)
            %UPDATEGPS 执行 GPS 位置量测更新。
            [residual, H, R] = obj.Model_.BuildGpsMeasurement(navSol, measurement);
            obj.KalmanUpdate(residual, H, R);
        end

        function errorState = GetErrorState(obj)
            %GETERRORSTATE 返回当前误差状态估计。
            errorState = obj.CurX_;
        end

        function covariance = GetCovariance(obj)
            %GETCOVARIANCE 返回当前误差状态协方差。
            covariance = obj.CurP_;
        end

        function ResetErrorState(obj)
            %RESETERRORSTATE 闭环反馈后清零误差状态均值。
            obj.CurX_ = zeros(obj.Model_.Layout.Dimension, 1);
            obj.PreX_ = obj.CurX_;
        end
    end

    methods (Static)
        function [navSol, correctedImu] = PropagateNavigation(navSol, imuBias, imu, dt)
            %PROPAGATENAVIGATION 执行零偏补偿后的惯导机械编排。
            correctedImu = struct();
            correctedImu.Time = imu.Time;
            correctedImu.Gyro = imu.Gyro - imuBias.GyroBias;
            correctedImu.Accel = imu.Accel - imuBias.AccelBias;

            [navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh] = insUpdateENU( ...
                navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh, ...
                correctedImu.Gyro, correctedImu.Accel, dt);
        end
    end

    methods (Access = private)
        function KalmanUpdate(obj, residual, H, R)
            residual = residual(:);

            % 多量测连续更新时，扣除当前误差状态预测的量测分量。
            innovation = residual - H * obj.CurX_;
            S = H * obj.CurP_ * H.' + R;
            K = (obj.CurP_ * H.') / S;
            obj.CurX_ = obj.CurX_ + K * innovation;

            identityMatrix = eye(size(obj.CurP_));
            obj.CurP_ = (identityMatrix - K * H) * obj.CurP_ ...
                * (identityMatrix - K * H).' + K * R * K.';
            obj.CurP_ = ErrorStateKF.SymmetrizeCovariance(obj.CurP_);
        end
    end

    methods (Static, Access = private)
        function covariance = SymmetrizeCovariance(covariance)
            covariance = 0.5 * (covariance + covariance.');
        end
    end
end
