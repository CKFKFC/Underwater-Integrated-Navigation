classdef ErrorStateKF < handle
    %ERRORSTATEKF 15 维误差状态卡尔曼滤波器。
    % 作者: Kefan Chen
    % 日期: 2026-07-04
    % 功能: 负责 ESKF 参数、误差状态预测、量测更新和闭环反馈。

    properties
        cfg_
        param_
        preX_
        preP_
        curX_
        curP_
        curStep_
    end

    methods
        %% constructor
        function obj = ErrorStateKF(cfg, param, initialCovariance)
            obj.cfg_ = cfg;
            obj.param_ = param;
            obj.preX_ = zeros(15, 1);
            obj.curX_ = zeros(15, 1);
            obj.preP_ = initialCovariance;
            obj.curP_ = initialCovariance;
            obj.curStep_ = 0;
        end

        %% preparation
        function Preparation(obj, sampleIndex)
            obj.curStep_ = sampleIndex;
            obj.preX_ = obj.curX_;
            obj.preP_ = obj.curP_;
        end

        %% prediction
        function Predict(obj, navSol, correctedImu, dt)
            % 这里只递推误差状态，不递推导航解算量。
            [F, G, Qc] = obj.BuildErrorDynamics(navSol, correctedImu);
            Phi = eye(15) + F * dt;
            Qd = G * Qc * G.' * dt;

            obj.curX_ = Phi * obj.preX_;
            obj.curP_ = Phi * obj.preP_ * Phi.' + Qd;
            obj.curP_ = ErrorStateKF.SymmetrizeCovariance(obj.curP_);
        end

        %% DVL measurement update
        function UpdateDvl(obj, navSol, measurement)
            % DVL 量测统一采用载体坐标系速度。
            Cnb = navSol.Cbn.';
            predictedVelocityBody = Cnb * navSol.VelocityEnu;
            residual = measurement.VelocityBody - predictedVelocityBody;

            H = zeros(3, 15);
            H(:, 1:3) = Cnb * skew(navSol.VelocityEnu);
            H(:, 4:6) = Cnb;
            obj.KalmanUpdate(residual, H, measurement.R);
        end

        %% depth measurement update
        function UpdateDepth(obj, navSol, measurement)
            % ENU 高度向上为正，深度向下为正。
            predictedDepth = obj.cfg_.reference.surfaceAltitude - navSol.PositionLlh(3);
            residual = measurement.Depth - predictedDepth;

            H = zeros(1, 15);
            H(1, 9) = -1.0;
            obj.KalmanUpdate(residual, H, measurement.R);
        end

        %% GPS measurement update
        function UpdateGps(obj, navSol, measurement)
            % GPS 纬经高残差转换为局部 ENU 米级误差。
            residual = llh2enuError(measurement.PositionLlh, navSol.PositionLlh);

            H = zeros(3, 15);
            H(:, 7:9) = eye(3);
            obj.KalmanUpdate(residual, H, measurement.R);
        end

        %% get current error state
        function errorState = GetErrorState(obj)
            errorState = obj.curX_;
        end

        %% get current covariance
        function covariance = GetCovariance(obj)
            covariance = obj.curP_;
        end

        %% reset error state after feedback
        function ResetErrorState(obj)
            % 反馈式组合导航必须在反馈后清零误差状态。
            obj.curX_ = zeros(15, 1);
            obj.preX_ = obj.curX_;
        end
    end

    methods (Static)
        %% create default ESKF parameters
        function param = CreateDefaultParameters(cfg)
            %CREATEDEFAULTPARAMETERS 定义 ESKF 算法内部参数。

            if nargin < 1
                error("ErrorStateKF:MissingConfig", ...
                    "ESKF initial error settings must be provided by setConfig.");
            end

            % 初始误差标准差顺序：姿态、速度、位置、陀螺零偏、加速度计零偏。
            % 这里仅读取 setConfig 中已经换算好的 SI 单位参数。
            initialError = ErrorStateKF.GetInitialErrorConfig(cfg);
            param = struct();
            param.InitialStd.Attitude = ErrorStateKF.GetConfigStd(initialError, "attitudeStd", 3);
            param.InitialStd.Velocity = ErrorStateKF.GetConfigStd(initialError, "velocityStd", 3);
            param.InitialStd.Position = ErrorStateKF.GetConfigStd(initialError, "positionStd", 3);
            param.InitialStd.GyroBias = ErrorStateKF.GetConfigStd(initialError, "gyroBiasStd", 3);
            param.InitialStd.AccelBias = ErrorStateKF.GetConfigStd(initialError, "accelBiasStd", 3);

            param.InitialErrorStd = [
                param.InitialStd.Attitude
                param.InitialStd.Velocity
                param.InitialStd.Position
                param.InitialStd.GyroBias
                param.InitialStd.AccelBias
                ];
        end

        %% create initial navigation solution
        function [navSol, imuBias, covariance] = CreateInitialNavigation(meas, param)
            %CREATEINITIALNAVIGATION 生成一次 MC 的导航初值和 P0。

            navSol = meas.getInitialNavigation();
            imuBias = meas.getInitialImuBias();
            initialError = param.InitialErrorStd .* randn(15, 1);
            initialNavigationError = initialError;

            % 真实 IMU 零偏已在每次 MC 的传感器数据中生成。
            % 初始导航解不再额外随机注入零偏估计，P0 仍保留零偏不确定度。
            initialNavigationError(10:15) = 0.0;

            % 给导航初值加入一次 MC 的初始误差。
            [navSol, imuBias] = ErrorStateKF.FeedbackNavigation(navSol, imuBias, initialNavigationError);
            covariance = diag(param.InitialErrorStd.^2);
        end

        %% propagate navigation solution
        function [navSol, correctedImu] = PropagateNavigation(navSol, imuBias, imu, dt)
            %PROPAGATENAVIGATION 执行零偏补偿后的惯导机械编排。

            % 惯导解算使用反馈后的零偏进行 IMU 补偿。
            correctedImu = struct();
            correctedImu.Time = imu.Time;
            correctedImu.Gyro = imu.Gyro - imuBias.GyroBias;
            correctedImu.Accel = imu.Accel - imuBias.AccelBias;

            [navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh] = insUpdateENU( ...
                navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh, ...
                correctedImu.Gyro, correctedImu.Accel, dt);
        end

        %% feedback error state
        function [navSol, imuBias] = FeedbackNavigation(navSol, imuBias, errorState)
            %FEEDBACKNAVIGATION 将 15 维误差状态反馈到导航解算结果。

            attitudeError = errorState(1:3);
            velocityError = errorState(4:6);
            positionError = errorState(7:9);
            gyroBiasError = errorState(10:12);
            accelBiasError = errorState(13:15);

            navSol.Cbn = orthonormalizeDcm((eye(3) + skew(attitudeError)) * navSol.Cbn);
            navSol.VelocityEnu = navSol.VelocityEnu + velocityError;
            navSol.PositionLlh = enuOffsetToLlh(navSol.PositionLlh, positionError);
            imuBias.GyroBias = imuBias.GyroBias + gyroBiasError;
            imuBias.AccelBias = imuBias.AccelBias + accelBiasError;
        end
    end

    methods (Access = private)
        %% build continuous error model
        function [F, G, Qc] = BuildErrorDynamics(obj, navSol, correctedImu)
            constants = getWgs84Constants();
            latitude = navSol.PositionLlh(1);
            altitude = navSol.PositionLlh(3);
            velocityEnu = navSol.VelocityEnu;
            Cbn = navSol.Cbn;
            earthModel = ErrorStateKF.BuildEarthErrorModel(constants, latitude, altitude, velocityEnu);
            fEnu = Cbn * correctedImu.Accel;
            wInN = earthModel.WIeN + earthModel.WEnN;
            wVelocityN = (2.0 * earthModel.WIeN) + earthModel.WEnN;
            angularRatePositionJacobian = earthModel.WIePositionJacobian ...
                + earthModel.WEnPositionJacobian;
            velocityPositionJacobian = skew(velocityEnu) ...
                * ((2.0 * earthModel.WIePositionJacobian) + earthModel.WEnPositionJacobian) ...
                + earthModel.GravityPositionJacobian;

            % 误差状态顺序：姿态、速度、位置、陀螺零偏、加速度计零偏。
            % 速度和位置误差在本工程中定义为真值相对当前导航解的 ENU 修正量。
            F = zeros(15, 15);
            F(1:3, 1:3) = -skew(wInN);
            F(1:3, 4:6) = -earthModel.WEnVelocityJacobian;
            F(1:3, 7:9) = -angularRatePositionJacobian * earthModel.LlhFromEnuJacobian;
            F(1:3, 10:12) = -Cbn;

            F(4:6, 1:3) = -skew(fEnu);
            F(4:6, 4:6) = skew(velocityEnu) * earthModel.WEnVelocityJacobian - skew(wVelocityN);
            F(4:6, 7:9) = velocityPositionJacobian * earthModel.LlhFromEnuJacobian;
            F(4:6, 13:15) = -Cbn;

            F(7:9, 4:6) = eye(3);
            F(7:9, 7:9) = earthModel.LocalPositionJacobian;

            G = zeros(15, 6);
            G(1:3, 1:3) = -Cbn;
            G(4:6, 4:6) = -Cbn;

            gyroNoise = ErrorStateKF.ExpandStd(obj.cfg_.noise.imu.gyroNoiseDensity, 3);
            accelNoise = ErrorStateKF.ExpandStd(obj.cfg_.noise.imu.accelNoiseDensity, 3);
            Qc = diag([
                gyroNoise.^2
                accelNoise.^2
                ]);
        end

        %% Kalman update
        function KalmanUpdate(obj, residual, H, R)
            residual = residual(:);

            % 多量测连续更新时，需要扣除当前误差状态预测量。
            innovation = residual - H * obj.curX_;
            S = H * obj.curP_ * H.' + R;
            K = (obj.curP_ * H.') / S;
            obj.curX_ = obj.curX_ + K * innovation;

            identityMatrix = eye(size(obj.curP_));
            obj.curP_ = (identityMatrix - K * H) * obj.curP_ * (identityMatrix - K * H).' + K * R * K.';
            obj.curP_ = ErrorStateKF.SymmetrizeCovariance(obj.curP_);
        end
    end

    methods (Static, Access = private)
        function model = BuildEarthErrorModel(constants, latitude, altitude, velocityEnu)
            [RM, RN] = earthRadii(latitude);
            [dRMdLatitude, dRNdLatitude] = ErrorStateKF.EarthRadiiDerivatives(constants, latitude);

            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            tanLatitude = tan(latitude);
            secLatitude = 1.0 / cosLatitude;
            meridianRadius = RM + altitude;
            transverseRadius = RN + altitude;
            longitudeRadius = transverseRadius * cosLatitude;

            model = struct();
            model.WIeN = constants.wie * [0.0; cosLatitude; sinLatitude];
            model.WEnN = [
                -velocityEnu(2) / meridianRadius
                velocityEnu(1) / transverseRadius
                velocityEnu(1) * tanLatitude / transverseRadius
                ];
            model.LlhFromEnuJacobian = [
                0.0, 1.0 / meridianRadius, 0.0
                1.0 / longitudeRadius, 0.0, 0.0
                0.0, 0.0, 1.0
                ];

            model.WIePositionJacobian = zeros(3, 3);
            model.WIePositionJacobian(2, 1) = -constants.wie * sinLatitude;
            model.WIePositionJacobian(3, 1) = constants.wie * cosLatitude;

            model.WEnVelocityJacobian = [
                0.0, -1.0 / meridianRadius, 0.0
                1.0 / transverseRadius, 0.0, 0.0
                tanLatitude / transverseRadius, 0.0, 0.0
                ];

            model.WEnPositionJacobian = zeros(3, 3);
            model.WEnPositionJacobian(1, 1) = velocityEnu(2) * dRMdLatitude / meridianRadius^2;
            model.WEnPositionJacobian(1, 3) = velocityEnu(2) / meridianRadius^2;
            model.WEnPositionJacobian(2, 1) = -velocityEnu(1) * dRNdLatitude / transverseRadius^2;
            model.WEnPositionJacobian(2, 3) = -velocityEnu(1) / transverseRadius^2;
            model.WEnPositionJacobian(3, 1) = velocityEnu(1) * ( ...
                secLatitude^2 / transverseRadius ...
                - tanLatitude * dRNdLatitude / transverseRadius^2);
            model.WEnPositionJacobian(3, 3) = -velocityEnu(1) * tanLatitude / transverseRadius^2;

            model.GravityPositionJacobian = ErrorStateKF.BuildGravityPositionJacobian( ...
                constants, latitude, altitude);
            model.LocalPositionJacobian = ErrorStateKF.BuildLocalPositionJacobian( ...
                latitude, velocityEnu, meridianRadius, transverseRadius, ...
                dRMdLatitude, dRNdLatitude);
        end

        function [dRMdLatitude, dRNdLatitude] = EarthRadiiDerivatives(constants, latitude)
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            denominator = 1.0 - constants.e2 * sinLatitude^2;

            dRNdLatitude = constants.a * constants.e2 * sinLatitude * cosLatitude ...
                / denominator^(3.0 / 2.0);
            dRMdLatitude = 3.0 * constants.a * (1.0 - constants.e2) ...
                * constants.e2 * sinLatitude * cosLatitude / denominator^(5.0 / 2.0);
        end

        function jacobian = BuildGravityPositionJacobian(constants, latitude, altitude)
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            denominator = 1.0 - constants.e2 * sinLatitude^2;
            sqrtDenominator = sqrt(denominator);
            latitudeTerm = 1.0 + constants.k * sinLatitude^2;
            altitudeTerm = 1.0 - (2.0 * altitude / constants.a);

            normalGravity = constants.gammaE * latitudeTerm / sqrtDenominator;
            dNormalGravityDLatitude = constants.gammaE * ( ...
                (2.0 * constants.k * sinLatitude * cosLatitude) / sqrtDenominator ...
                + latitudeTerm * constants.e2 * sinLatitude * cosLatitude / denominator^(3.0 / 2.0));

            dGravityDLatitude = dNormalGravityDLatitude * altitudeTerm;
            dGravityDAltitude = -2.0 * normalGravity / constants.a;

            jacobian = zeros(3, 3);
            jacobian(3, 1) = -dGravityDLatitude;
            jacobian(3, 3) = -dGravityDAltitude;
        end

        function jacobian = BuildLocalPositionJacobian( ...
                latitude, velocityEnu, meridianRadius, transverseRadius, dRMdLatitude, dRNdLatitude)
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            longitudeRadius = transverseRadius * cosLatitude;
            latitudeRate = velocityEnu(2) / meridianRadius;
            altitudeRate = velocityEnu(3);

            dMeridianRadiusDt = dRMdLatitude * latitudeRate + altitudeRate;
            dLongitudeRadiusDt = (dRNdLatitude * latitudeRate + altitudeRate) * cosLatitude ...
                - transverseRadius * sinLatitude * latitudeRate;

            metricRateJacobian = zeros(3, 3);
            metricRateJacobian(1, 1) = dLongitudeRadiusDt / longitudeRadius;
            metricRateJacobian(2, 2) = dMeridianRadiusDt / meridianRadius;

            enuFromLlhJacobian = [
                0.0, longitudeRadius, 0.0
                meridianRadius, 0.0, 0.0
                0.0, 0.0, 1.0
                ];
            llhFromEnuJacobian = [
                0.0, 1.0 / meridianRadius, 0.0
                1.0 / longitudeRadius, 0.0, 0.0
                0.0, 0.0, 1.0
                ];

            positionRateJacobian = zeros(3, 3);
            positionRateJacobian(1, 1) = -velocityEnu(2) * dRMdLatitude / meridianRadius^2;
            positionRateJacobian(1, 3) = -velocityEnu(2) / meridianRadius^2;

            dLongitudeRadiusDLatitude = dRNdLatitude * cosLatitude - transverseRadius * sinLatitude;
            positionRateJacobian(2, 1) = -velocityEnu(1) ...
                * dLongitudeRadiusDLatitude / longitudeRadius^2;
            positionRateJacobian(2, 3) = -velocityEnu(1) * cosLatitude / longitudeRadius^2;

            jacobian = metricRateJacobian ...
                + enuFromLlhJacobian * positionRateJacobian * llhFromEnuJacobian;
        end

        function covariance = SymmetrizeCovariance(covariance)
            covariance = 0.5 * (covariance + covariance.');
        end

        function initialError = GetInitialErrorConfig(cfg)
            if ~isstruct(cfg) ...
                    || ~isfield(cfg, 'algorithm') ...
                    || ~isfield(cfg.algorithm, 'initialError')
                error("ErrorStateKF:MissingInitialErrorConfig", ...
                    "cfg.algorithm.initialError must be configured in setConfig.");
            end
            initialError = cfg.algorithm.initialError;
        end

        function stdVector = GetConfigStd(config, fieldName, columnCount)
            fieldName = char(fieldName);
            if ~isfield(config, fieldName)
                error("ErrorStateKF:MissingInitialStd", ...
                    "cfg.algorithm.initialError.%s must be configured in setConfig.", fieldName);
            end
            stdVector = ErrorStateKF.ExpandStd(config.(fieldName), columnCount);
        end

        function stdVector = ExpandStd(stdValue, columnCount)
            stdVector = double(stdValue(:));
            if isscalar(stdVector)
                stdVector = repmat(stdVector, columnCount, 1);
            end
            if numel(stdVector) ~= columnCount
                error("ErrorStateKF:InvalidNoiseStd", ...
                    "Noise standard deviation must be scalar or %d-by-1.", columnCount);
            end
        end
    end
end
