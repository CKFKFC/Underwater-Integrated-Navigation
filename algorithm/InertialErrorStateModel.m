classdef InertialErrorStateModel
    %INERTIALERRORSTATEMODEL 可配置惯导误差状态模型。
    %   该类组合状态块，并集中生成动力学、量测模型和闭环反馈。

    properties (SetAccess = private)
        Cfg
        ProfileName
        Blocks
        Layout
        InitialErrorStd
    end

    methods
        function obj = InertialErrorStateModel(cfg)
            arguments
                cfg struct
            end

            [obj.ProfileName, obj.Blocks] = createInertialStateProfile(cfg);
            obj.Cfg = cfg;
            obj.Layout = buildStateLayout(obj.Blocks);
            obj.InitialErrorStd = vertcat(obj.Blocks.InitialStd);
        end

        function [navSol, imuBias, covariance] = CreateInitialNavigation(obj, meas)
            %CREATEINITIALNAVIGATION 生成一次 Monte Carlo 的导航初值和 P0。
            navSol = meas.getInitialNavigation();
            imuBias = meas.getInitialImuBias();
            initialError = obj.InitialErrorStd .* randn(obj.Layout.Dimension, 1);
            initialNavigationError = initialError;

            % 真实 IMU 零偏已由量测对象生成，不向名义零偏估计重复注入误差。
            if obj.HasState("GyroBias")
                initialNavigationError(obj.Layout.Index.GyroBias) = 0.0;
            end
            if obj.HasState("AccelBias")
                initialNavigationError(obj.Layout.Index.AccelBias) = 0.0;
            end

            [navSol, imuBias] = obj.FeedbackNavigation( ...
                navSol, imuBias, initialNavigationError);
            covariance = diag(obj.InitialErrorStd.^2);
        end

        function [F, G, Qc] = BuildErrorDynamics(obj, navSol, correctedImu)
            %BUILDERRORDYNAMICS 生成当前状态组合对应的连续误差模型。
            attitudeIndex = obj.Layout.Index.Attitude;
            velocityIndex = obj.Layout.Index.Velocity;
            positionIndex = obj.Layout.Index.Position;

            constants = getWgs84Constants();
            latitude = navSol.PositionLlh(1);
            altitude = navSol.PositionLlh(3);
            velocityEnu = navSol.VelocityEnu;
            Cbn = navSol.Cbn;
            earthModel = InertialErrorStateModel.BuildEarthErrorModel( ...
                constants, latitude, altitude, velocityEnu);
            fEnu = Cbn * correctedImu.Accel;
            wInN = earthModel.WIeN + earthModel.WEnN;
            wVelocityN = (2.0 * earthModel.WIeN) + earthModel.WEnN;
            angularRatePositionJacobian = earthModel.WIePositionJacobian ...
                + earthModel.WEnPositionJacobian;
            velocityPositionJacobian = skew(velocityEnu) ...
                * ((2.0 * earthModel.WIePositionJacobian) + earthModel.WEnPositionJacobian) ...
                + earthModel.GravityPositionJacobian;

            stateDimension = obj.Layout.Dimension;
            F = zeros(stateDimension, stateDimension);
            F(attitudeIndex, attitudeIndex) = -skew(wInN);
            F(attitudeIndex, velocityIndex) = -earthModel.WEnVelocityJacobian;
            F(attitudeIndex, positionIndex) = ...
                -angularRatePositionJacobian * earthModel.LlhFromEnuJacobian;

            F(velocityIndex, attitudeIndex) = -skew(fEnu);
            F(velocityIndex, velocityIndex) = ...
                skew(velocityEnu) * earthModel.WEnVelocityJacobian - skew(wVelocityN);
            F(velocityIndex, positionIndex) = ...
                velocityPositionJacobian * earthModel.LlhFromEnuJacobian;

            F(positionIndex, velocityIndex) = eye(3);
            F(positionIndex, positionIndex) = earthModel.LocalPositionJacobian;

            if obj.HasState("GyroBias")
                F(attitudeIndex, obj.Layout.Index.GyroBias) = -Cbn;
            end
            if obj.HasState("AccelBias")
                F(velocityIndex, obj.Layout.Index.AccelBias) = -Cbn;
            end

            G = zeros(stateDimension, 6);
            G(attitudeIndex, 1:3) = -Cbn;
            G(velocityIndex, 4:6) = -Cbn;

            gyroNoise = InertialErrorStateModel.ExpandStd( ...
                obj.Cfg.noise.imu.gyroNoiseDensity, 3);
            accelNoise = InertialErrorStateModel.ExpandStd( ...
                obj.Cfg.noise.imu.accelNoiseDensity, 3);
            Qc = diag([gyroNoise.^2; accelNoise.^2]);
        end

        function [residual, H, R] = BuildDvlMeasurement(obj, navSol, measurement)
            %BUILDDVLMEASUREMENT 生成体坐标系 DVL 速度量测模型。
            Cnb = navSol.Cbn.';
            predictedVelocityBody = Cnb * navSol.VelocityEnu;
            residual = measurement.VelocityBody - predictedVelocityBody;

            H = zeros(3, obj.Layout.Dimension);
            H(:, obj.Layout.Index.Attitude) = Cnb * skew(navSol.VelocityEnu);
            H(:, obj.Layout.Index.Velocity) = Cnb;
            R = measurement.R;
        end

        function [residual, H, R] = BuildDepthMeasurement(obj, navSol, measurement)
            %BUILDDEPTHMEASUREMENT 生成深度量测模型。
            predictedDepth = obj.Cfg.reference.surfaceAltitude - navSol.PositionLlh(3);
            residual = measurement.Depth - predictedDepth;

            H = zeros(1, obj.Layout.Dimension);
            positionIndex = obj.Layout.Index.Position;
            H(1, positionIndex(3)) = -1.0;
            R = measurement.R;
        end

        function [residual, H, R] = BuildGpsMeasurement(obj, navSol, measurement)
            %BUILDGPSMEASUREMENT 生成 GPS 位置量测模型。
            residual = llh2enuError(measurement.PositionLlh, navSol.PositionLlh);

            H = zeros(3, obj.Layout.Dimension);
            H(:, obj.Layout.Index.Position) = eye(3);
            R = measurement.R;
        end

        function [navSol, imuBias] = FeedbackNavigation(obj, navSol, imuBias, errorState)
            %FEEDBACKNAVIGATION 按状态块语义反馈误差状态。
            if numel(errorState) ~= obj.Layout.Dimension
                error("InertialErrorStateModel:InvalidErrorState", ...
                    "Error state must contain %d elements for profile %s.", ...
                    obj.Layout.Dimension, obj.ProfileName);
            end
            errorState = errorState(:);

            attitudeError = errorState(obj.Layout.Index.Attitude);
            velocityError = errorState(obj.Layout.Index.Velocity);
            positionError = errorState(obj.Layout.Index.Position);

            navSol.Cbn = orthonormalizeDcm( ...
                (eye(3) + skew(attitudeError)) * navSol.Cbn);
            navSol.VelocityEnu = navSol.VelocityEnu + velocityError;
            navSol.PositionLlh = enuOffsetToLlh(navSol.PositionLlh, positionError);

            if obj.HasState("GyroBias")
                imuBias.GyroBias = imuBias.GyroBias ...
                    + errorState(obj.Layout.Index.GyroBias);
            end
            if obj.HasState("AccelBias")
                imuBias.AccelBias = imuBias.AccelBias ...
                    + errorState(obj.Layout.Index.AccelBias);
            end
        end

        function isPresent = HasState(obj, blockName)
            %HASSTATE 判断当前模型是否包含指定状态块。
            isPresent = isfield(obj.Layout.Has, char(blockName));
        end

        function metadata = GetMetadata(obj)
            %GETMETADATA 返回可随结果保存的状态模型描述。
            metadata = struct();
            metadata.Profile = obj.ProfileName;
            metadata.Dimension = obj.Layout.Dimension;
            metadata.BlockNames = obj.Layout.BlockNames;
            metadata.BlockDimensions = obj.Layout.BlockDimensions;
            metadata.Index = obj.Layout.Index;
        end
    end

    methods (Static, Access = private)
        function model = BuildEarthErrorModel(constants, latitude, altitude, velocityEnu)
            [RM, RN] = earthRadii(latitude);
            [dRMdLatitude, dRNdLatitude] = ...
                InertialErrorStateModel.EarthRadiiDerivatives(constants, latitude);

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
            model.WEnPositionJacobian(1, 1) = ...
                velocityEnu(2) * dRMdLatitude / meridianRadius^2;
            model.WEnPositionJacobian(1, 3) = velocityEnu(2) / meridianRadius^2;
            model.WEnPositionJacobian(2, 1) = ...
                -velocityEnu(1) * dRNdLatitude / transverseRadius^2;
            model.WEnPositionJacobian(2, 3) = -velocityEnu(1) / transverseRadius^2;
            model.WEnPositionJacobian(3, 1) = velocityEnu(1) * ( ...
                secLatitude^2 / transverseRadius ...
                - tanLatitude * dRNdLatitude / transverseRadius^2);
            model.WEnPositionJacobian(3, 3) = ...
                -velocityEnu(1) * tanLatitude / transverseRadius^2;

            model.GravityPositionJacobian = ...
                InertialErrorStateModel.BuildGravityPositionJacobian( ...
                constants, latitude, altitude);
            model.LocalPositionJacobian = ...
                InertialErrorStateModel.BuildLocalPositionJacobian( ...
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
                * constants.e2 * sinLatitude * cosLatitude ...
                / denominator^(5.0 / 2.0);
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
                + latitudeTerm * constants.e2 * sinLatitude * cosLatitude ...
                / denominator^(3.0 / 2.0));

            dGravityDLatitude = dNormalGravityDLatitude * altitudeTerm;
            dGravityDAltitude = -2.0 * normalGravity / constants.a;

            jacobian = zeros(3, 3);
            jacobian(3, 1) = -dGravityDLatitude;
            jacobian(3, 3) = -dGravityDAltitude;
        end

        function jacobian = BuildLocalPositionJacobian( ...
                latitude, velocityEnu, meridianRadius, transverseRadius, ...
                dRMdLatitude, dRNdLatitude)
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            longitudeRadius = transverseRadius * cosLatitude;
            latitudeRate = velocityEnu(2) / meridianRadius;
            altitudeRate = velocityEnu(3);

            dMeridianRadiusDt = dRMdLatitude * latitudeRate + altitudeRate;
            dLongitudeRadiusDt = ...
                (dRNdLatitude * latitudeRate + altitudeRate) * cosLatitude ...
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
            positionRateJacobian(1, 1) = ...
                -velocityEnu(2) * dRMdLatitude / meridianRadius^2;
            positionRateJacobian(1, 3) = -velocityEnu(2) / meridianRadius^2;

            dLongitudeRadiusDLatitude = ...
                dRNdLatitude * cosLatitude - transverseRadius * sinLatitude;
            positionRateJacobian(2, 1) = -velocityEnu(1) ...
                * dLongitudeRadiusDLatitude / longitudeRadius^2;
            positionRateJacobian(2, 3) = ...
                -velocityEnu(1) * cosLatitude / longitudeRadius^2;

            jacobian = metricRateJacobian ...
                + enuFromLlhJacobian * positionRateJacobian * llhFromEnuJacobian;
        end

        function stdVector = ExpandStd(stdValue, dimension)
            stdVector = double(stdValue(:));
            if isscalar(stdVector)
                stdVector = repmat(stdVector, dimension, 1);
            end
            if numel(stdVector) ~= dimension
                error("InertialErrorStateModel:InvalidNoiseStd", ...
                    "Noise standard deviation must be scalar or %d-by-1.", dimension);
            end
        end
    end
end
