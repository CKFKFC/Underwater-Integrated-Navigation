classdef InertialErrorStateModel
    %INERTIALERRORSTATEMODEL 可配置惯导误差状态模型。
    %   该类是“状态语义层”：根据 profile 组合状态块，并集中生成动力学、
    %   量测模型和闭环反馈。ErrorStateKF 只消费本类生成的矩阵，因此不需要
    %   知道状态总维数，也不应出现与具体状态位置有关的硬编码索引。

    properties (SetAccess = private)
        Cfg                % 完整算法配置，供噪声模型和量测模型读取
        ProfileName        % 当前状态组合名称，例如 ins9 或 ins15
        Blocks             % 按状态向量顺序排列的状态块定义
        Layout             % 状态总维数及每个状态块的全局索引
        InitialErrorStd    % 严格按 Layout 顺序拼接的初始标准差
    end

    methods
        %% 构造可配置状态模型
        function obj = InertialErrorStateModel(cfg)
            arguments
                cfg struct
            end

            % Profile 只决定“包含哪些状态块及其顺序”；全局索引由布局生成器
            % 统一分配，后续动力学、量测和反馈均通过 Layout 按名称取索引。
            [obj.ProfileName, obj.Blocks] = createInertialStateProfile(cfg);
            obj.Cfg = cfg;
            obj.Layout = buildStateLayout(obj.Blocks);
            obj.InitialErrorStd = vertcat(obj.Blocks.InitialStd);
        end

        %% 生成一次 Monte Carlo 的导航初值和初始协方差
        function [navSol, imuBias, covariance] = CreateInitialNavigation(obj, meas)
            %CREATEINITIALNAVIGATION 生成一次 Monte Carlo 的导航初值和 P0。
            navSol = meas.getInitialNavigation();
            imuBias = meas.getInitialImuBias();

            % 初始误差和 P0 使用完全相同的状态顺序，避免新增状态后扰动向量与
            % 协方差对角线错位。randn 的长度随所选 profile 自动变化。
            initialError = obj.InitialErrorStd .* randn(obj.Layout.Dimension, 1);
            initialNavigationError = initialError;

            % 真实 IMU 零偏已由量测对象生成。零偏状态在 P0 中仍保留不确定度，
            % 但初始化时不再向名义零偏估计重复注入随机误差。
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

        %% 构建连续时间惯导误差动力学
        function [F, G, Qc] = BuildErrorDynamics(obj, navSol, correctedImu)
            %BUILDERRORDYNAMICS 生成当前状态组合对应的连续误差模型。
            %   本工程误差定义为“真值相对当前导航解的修正量”。F 的行表示
            %   被影响的状态块，列表示误差来源；基础 9 维块在所有 profile 中存在。
            attitudeIndex = obj.Layout.Index.Attitude;
            velocityIndex = obj.Layout.Index.Velocity;
            positionIndex = obj.Layout.Index.Position;

            % 将地球自转、运输角速度、曲率半径和重力梯度统一整理成局部雅可比。
            % 地球模型先对纬经高求导，随后通过 LlhFromEnuJacobian 转换为对
            % ENU 位置误差求导，保证 F 中位置状态始终使用米制 ENU 定义。
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

            % 基础 9 维惯导误差模型：姿态、速度、位置三个 3 维状态块。
            % 使用语义索引后，状态块的全局位置由 profile 决定，而公式本身不变。
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

            % 可选零偏状态只在当前 profile 包含它们时加入耦合。陀螺零偏驱动
            % 姿态误差，加速度计零偏驱动速度误差；不存在时相应行列根本不分配。
            if obj.HasState("GyroBias")
                F(attitudeIndex, obj.Layout.Index.GyroBias) = -Cbn;
            end
            if obj.HasState("AccelBias")
                F(velocityIndex, obj.Layout.Index.AccelBias) = -Cbn;
            end

            % 连续白噪声向量固定为 [陀螺噪声; 加速度计噪声]。G 将体坐标噪声
            % 投影到 ENU 姿态/速度误差，Qc 保存对应的连续时间噪声强度。
            G = zeros(stateDimension, 6);
            G(attitudeIndex, 1:3) = -Cbn;
            G(velocityIndex, 4:6) = -Cbn;

            gyroNoise = InertialErrorStateModel.ExpandStd( ...
                obj.Cfg.noise.imu.gyroNoiseDensity, 3);
            accelNoise = InertialErrorStateModel.ExpandStd( ...
                obj.Cfg.noise.imu.accelNoiseDensity, 3);
            Qc = diag([gyroNoise.^2; accelNoise.^2]);
        end

        %% 构建 DVL 速度量测模型
        function [residual, H, R] = BuildDvlMeasurement(obj, navSol, measurement)
            %BUILDDVLMEASUREMENT 生成体坐标系 DVL 速度量测模型。
            %   残差统一采用 z-h(x)。线性化 Cnb*v 后，姿态误差和 ENU 速度
            %   误差进入 H；当前未参与该量测的可选状态列保持为零。
            Cnb = navSol.Cbn.';
            predictedVelocityBody = Cnb * navSol.VelocityEnu;
            residual = measurement.VelocityBody - predictedVelocityBody;

            H = zeros(3, obj.Layout.Dimension);
            H(:, obj.Layout.Index.Attitude) = Cnb * skew(navSol.VelocityEnu);
            H(:, obj.Layout.Index.Velocity) = Cnb;
            R = measurement.R;
        end

        %% 构建深度量测模型
        function [residual, H, R] = BuildDepthMeasurement(obj, navSol, measurement)
            %BUILDDEPTHMEASUREMENT 生成深度量测模型。
            %   ENU 高度向上为正，而深度向下为正，因此高度误差对应的 H 元素为 -1。
            predictedDepth = obj.Cfg.reference.surfaceAltitude - navSol.PositionLlh(3);
            residual = measurement.Depth - predictedDepth;

            H = zeros(1, obj.Layout.Dimension);
            positionIndex = obj.Layout.Index.Position;
            H(1, positionIndex(3)) = -1.0;
            R = measurement.R;
        end

        %% 构建 GPS 位置量测模型
        function [residual, H, R] = BuildGpsMeasurement(obj, navSol, measurement)
            %BUILDGPSMEASUREMENT 生成 GPS 位置量测模型。
            %   经纬高差先转换为米制 ENU 残差，使其与 Position 状态块定义一致。
            residual = llh2enuError(measurement.PositionLlh, navSol.PositionLlh);

            H = zeros(3, obj.Layout.Dimension);
            H(:, obj.Layout.Index.Position) = eye(3);
            R = measurement.R;
        end

        %% 将误差状态闭环反馈到名义导航状态
        function [navSol, imuBias] = FeedbackNavigation(obj, navSol, imuBias, errorState)
            %FEEDBACKNAVIGATION 按状态块语义反馈误差状态。
            %   反馈接口只接受与当前 Layout 等长的向量。统一转换为列向量，防止
            %   MATLAB 隐式扩展把 3 维修正量意外扩展成 3-by-3 矩阵。
            if numel(errorState) ~= obj.Layout.Dimension
                error("InertialErrorStateModel:InvalidErrorState", ...
                    "Error state must contain %d elements for profile %s.", ...
                    obj.Layout.Dimension, obj.ProfileName);
            end
            errorState = errorState(:);

            attitudeError = errorState(obj.Layout.Index.Attitude);
            velocityError = errorState(obj.Layout.Index.Velocity);
            positionError = errorState(obj.Layout.Index.Position);

            % 姿态误差采用左乘小角度修正；速度和位置采用加性修正。DCM 修正后
            % 重新正交化，避免多次闭环反馈造成旋转矩阵逐步偏离 SO(3)。
            navSol.Cbn = orthonormalizeDcm( ...
                (eye(3) + skew(attitudeError)) * navSol.Cbn);
            navSol.VelocityEnu = navSol.VelocityEnu + velocityError;
            navSol.PositionLlh = enuOffsetToLlh(navSol.PositionLlh, positionError);

            % 名义 IMU 零偏仅由实际存在的零偏状态修正。因此 ins9 会保持输入的
            % 零偏估计不变，ins15 则会把滤波估计闭环反馈到 IMU 补偿量。
            if obj.HasState("GyroBias")
                imuBias.GyroBias = imuBias.GyroBias ...
                    + errorState(obj.Layout.Index.GyroBias);
            end
            if obj.HasState("AccelBias")
                imuBias.AccelBias = imuBias.AccelBias ...
                    + errorState(obj.Layout.Index.AccelBias);
            end
        end

        %% 判断当前模型是否包含指定状态块
        function isPresent = HasState(obj, blockName)
            %HASSTATE 判断当前模型是否包含指定状态块。
            isPresent = isfield(obj.Layout.Has, char(blockName));
        end

        %% 导出可随滤波结果保存的状态模型元数据
        function metadata = GetMetadata(obj)
            %GETMETADATA 返回可随结果保存的状态模型描述。
            %   元数据不参与滤波运算，用于复现实验时确认 profile、维数和索引。
            metadata = struct();
            metadata.Profile = obj.ProfileName;
            metadata.Dimension = obj.Layout.Dimension;
            metadata.BlockNames = obj.Layout.BlockNames;
            metadata.BlockDimensions = obj.Layout.BlockDimensions;
            metadata.Index = obj.Layout.Index;
        end
    end

    methods (Static, Access = private)
        %% 构建 ENU 误差方程所需的地球参数和雅可比
        function model = BuildEarthErrorModel(constants, latitude, altitude, velocityEnu)
            %BUILDEARTHERRORMODEL 汇总 ENU 误差方程需要的地球参数及雅可比。
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

            % WIeN 为地球自转角速度，WEnN 为载体相对地球运动引起的运输角速度。
            % LlhFromEnuJacobian 将 [东, 北, 天] 米制位置误差映射为纬经高微分。
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

            % 下列两个位置雅可比先以纬经高为自变量构造。在 BuildErrorDynamics
            % 中再右乘 LlhFromEnuJacobian，得到对 ENU 位置状态的导数。
            model.WIePositionJacobian = zeros(3, 3);
            model.WIePositionJacobian(2, 1) = -constants.wie * sinLatitude;
            model.WIePositionJacobian(3, 1) = constants.wie * cosLatitude;

            % 运输角速度同时受 ENU 速度和地理位置影响，分别保存两类偏导，
            % 便于在姿态误差方程和速度误差方程中复用。
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

        %% 计算曲率半径对纬度的导数
        function [dRMdLatitude, dRNdLatitude] = EarthRadiiDerivatives(constants, latitude)
            %EARTHRADIIDERIVATIVES 计算子午圈和卯酉圈曲率半径对纬度的导数。
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            denominator = 1.0 - constants.e2 * sinLatitude^2;

            dRNdLatitude = constants.a * constants.e2 * sinLatitude * cosLatitude ...
                / denominator^(3.0 / 2.0);
            dRMdLatitude = 3.0 * constants.a * (1.0 - constants.e2) ...
                * constants.e2 * sinLatitude * cosLatitude ...
                / denominator^(5.0 / 2.0);
        end

        %% 构建重力对位置的雅可比
        function jacobian = BuildGravityPositionJacobian(constants, latitude, altitude)
            %BUILDGRAVITYPOSITIONJACOBIAN 计算重力对纬度和高度的局部敏感度。
            %   ENU 重力主要作用于天向，因此该雅可比只有第三行存在非零元素。
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

        %% 构建米制 ENU 位置误差雅可比
        function jacobian = BuildLocalPositionJacobian( ...
                latitude, velocityEnu, meridianRadius, transverseRadius, ...
                dRMdLatitude, dRNdLatitude)
            %BUILDLOCALPOSITIONJACOBIAN 描述曲率尺度变化对 ENU 位置误差的影响。
            %   经纬度运动使用随纬度、高度变化的曲率半径。这里把尺度因子的
            %   时间变化和位置速率对纬经高的偏导合成为米制 ENU 位置雅可比。
            sinLatitude = sin(latitude);
            cosLatitude = cos(latitude);
            longitudeRadius = transverseRadius * cosLatitude;
            latitudeRate = velocityEnu(2) / meridianRadius;
            altitudeRate = velocityEnu(3);

            dMeridianRadiusDt = dRMdLatitude * latitudeRate + altitudeRate;
            dLongitudeRadiusDt = ...
                (dRNdLatitude * latitudeRate + altitudeRate) * cosLatitude ...
                - transverseRadius * sinLatitude * latitudeRate;

            % metricRateJacobian 表示 ENU 度量尺度随轨迹变化产生的误差增长。
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

            % positionRateJacobian 先描述纬经高位置速率对纬经高的偏导，最后
            % 通过 ENU<->LLH 两个雅可比转换回 Position 状态块采用的 ENU 表达。
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

        %% 将标量或分轴标准差规范为列向量
        function stdVector = ExpandStd(stdValue, dimension)
            %EXPANDSTD 允许配置使用一个各轴相同的标量或完整的分轴标准差。
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
