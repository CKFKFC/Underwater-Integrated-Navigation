classdef SensorDelayCompensator
    %SENSORDELAYCOMPENSATOR 一阶传感器总线延时状态补偿工具。
    %   IMU 延时补偿仅外推速度和位置，姿态保持不变。外部量测补偿按
    %   量测类型回推所需的导航状态。速度补偿使用与惯导机械编排一致的
    %   导航系加速度，包括比力、重力、地球自转和运输角速度项。

    methods (Static)
        function navSol = forwardImu(navSol, correctedImu, delaySeconds)
            %FORWARDIMU 将滞后 IMU 导航解一阶外推到计算机当前时刻。
            arguments
                navSol struct
                correctedImu struct
                delaySeconds (1, 1) double {mustBeNonnegative, mustBeFinite}
            end

            sampleVelocity = navSol.VelocityEnu(:);
            accelerationEnu = SensorDelayCompensator.navigationAcceleration( ...
                navSol, correctedImu);
            navSol.PositionLlh = SensorDelayCompensator.propagatePosition( ...
                navSol.PositionLlh, sampleVelocity, delaySeconds);
            navSol.VelocityEnu = sampleVelocity + accelerationEnu * delaySeconds;
        end

        function navSol = backPropagateNavigation(navSol, correctedImu, delaySeconds)
            %BACKPROPAGATENAVIGATION 回推位置和速度，供 GNSS 和状态同步使用。
            arguments
                navSol struct
                correctedImu struct
                delaySeconds (1, 1) double {mustBeNonnegative, mustBeFinite}
            end

            currentVelocity = navSol.VelocityEnu(:);
            accelerationEnu = SensorDelayCompensator.navigationAcceleration( ...
                navSol, correctedImu);
            navSol.PositionLlh = SensorDelayCompensator.propagatePosition( ...
                navSol.PositionLlh, currentVelocity, -delaySeconds);
            navSol.VelocityEnu = currentVelocity - accelerationEnu * delaySeconds;
        end

        function navSol = backPropagateVelocity(navSol, correctedImu, delaySeconds, attitudeDeltaTime)
            %BACKPROPAGATEVELOCITY 回推速度，并可将姿态对齐到 DVL 采样时刻。
            %   attitudeDeltaTime 为 DVL 采样时刻减当前姿态所属的 IMU 采样时刻，
            %   可正可负。默认零保留旧行为；角速度采用最新已到达值作短时保持。
            arguments
                navSol struct
                correctedImu struct
                delaySeconds (1, 1) double {mustBeNonnegative, mustBeFinite}
                attitudeDeltaTime (1, 1) double {mustBeFinite} = 0.0
            end

            accelerationEnu = SensorDelayCompensator.navigationAcceleration( ...
                navSol, correctedImu);
            navSol.VelocityEnu = navSol.VelocityEnu(:) ...
                - accelerationEnu * delaySeconds;
            if attitudeDeltaTime ~= 0.0
                % 姿态传播仍保留地球自转和运输角速度；忽略地球自转的近似
                % 仅用于 DVL 杆臂叉乘，不用于惯导姿态传播。
                latitude = navSol.PositionLlh(1);
                altitude = navSol.PositionLlh(3);
                [RM, RN] = earthRadii(latitude);
                constants = getWgs84Constants();
                velocityEnu = navSol.VelocityEnu;
                wInN = constants.wie * [0.0; cos(latitude); sin(latitude)] ...
                    + [-velocityEnu(2) / (RM + altitude); ...
                    velocityEnu(1) / (RN + altitude); ...
                    velocityEnu(1) * tan(latitude) / (RN + altitude)];
                navSol.Cbn = expm(-skew(wInN) * attitudeDeltaTime) * navSol.Cbn ...
                    * expm(skew(correctedImu.Gyro(:)) * attitudeDeltaTime);
            end
        end

        function navSol = backPropagateHeight(navSol, delaySeconds)
            %BACKPROPAGATEHEIGHT 仅回推高度，供深度计历史量测构造新息。
            arguments
                navSol struct
                delaySeconds (1, 1) double {mustBeNonnegative, mustBeFinite}
            end

            navSol.PositionLlh = navSol.PositionLlh(:);
            navSol.PositionLlh(3) = navSol.PositionLlh(3) ...
                - navSol.VelocityEnu(3) * delaySeconds;
        end
    end

    methods (Static, Access = private)
        function accelerationEnu = navigationAcceleration(navSol, correctedImu)
            %NAVIGATIONACCELERATION 计算当前导航状态对应的 ENU 速度导数。
            positionLlh = navSol.PositionLlh(:);
            velocityEnu = navSol.VelocityEnu(:);
            latitude = positionLlh(1);
            altitude = positionLlh(3);
            constants = getWgs84Constants();
            [meridianRadius, transverseRadius] = earthRadii(latitude);

            earthRateEnu = constants.wie * [0.0; cos(latitude); sin(latitude)];
            transportRateEnu = [
                -velocityEnu(2) / (meridianRadius + altitude)
                velocityEnu(1) / (transverseRadius + altitude)
                velocityEnu(1) * tan(latitude) / (transverseRadius + altitude)
                ];

            accelerationEnu = navSol.Cbn * correctedImu.Accel(:) ...
                + gravityENU(latitude, altitude) ...
                - skew(transportRateEnu + 2.0 * earthRateEnu) * velocityEnu;
        end

        function positionLlh = propagatePosition(positionLlh, velocityEnu, deltaTime)
            %PROPAGATEPOSITION 使用固定 ENU 速度一阶推算纬经高。
            positionLlh = positionLlh(:);
            velocityEnu = velocityEnu(:);
            latitude = positionLlh(1);
            altitude = positionLlh(3);
            [meridianRadius, transverseRadius] = earthRadii(latitude);

            positionLlh(1) = latitude + velocityEnu(2) * deltaTime ...
                / (meridianRadius + altitude);
            positionLlh(2) = positionLlh(2) + velocityEnu(1) * deltaTime ...
                / ((transverseRadius + altitude) * cos(latitude));
            positionLlh(3) = altitude + velocityEnu(3) * deltaTime;
        end
    end
end
