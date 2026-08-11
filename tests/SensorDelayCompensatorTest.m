classdef SensorDelayCompensatorTest < matlab.unittest.TestCase
    %SENSORDELAYCOMPENSATORTEST 验证一阶传感器延时状态补偿公式。

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, IncludingSubfolders=true);
            testCase.applyFixture(fixture);
        end
    end

    methods (Test)
        function forwardImuUpdatesPositionAndVelocity(testCase)
            navSol = SensorDelayCompensatorTest.createNavigation();
            correctedImu = SensorDelayCompensatorTest.createImu();
            delaySeconds = 0.03;
            expectedPosition = SensorDelayCompensatorTest.propagatePosition( ...
                navSol.PositionLlh, navSol.VelocityEnu, delaySeconds);
            expectedVelocity = navSol.VelocityEnu ...
                + SensorDelayCompensatorTest.navigationAcceleration( ...
                navSol, correctedImu) * delaySeconds;

            actual = SensorDelayCompensator.forwardImu( ...
                navSol, correctedImu, delaySeconds);

            testCase.verifyEqual(actual.Cbn, navSol.Cbn, AbsTol=0.0);
            testCase.verifyEqual(actual.PositionLlh, expectedPosition, AbsTol=1.0e-14);
            testCase.verifyEqual(actual.VelocityEnu, expectedVelocity, AbsTol=1.0e-14);
        end

        function backPropagateNavigationUpdatesPositionAndVelocity(testCase)
            navSol = SensorDelayCompensatorTest.createNavigation();
            correctedImu = SensorDelayCompensatorTest.createImu();
            delaySeconds = 0.02;
            expectedPosition = SensorDelayCompensatorTest.propagatePosition( ...
                navSol.PositionLlh, navSol.VelocityEnu, -delaySeconds);
            expectedVelocity = navSol.VelocityEnu ...
                - SensorDelayCompensatorTest.navigationAcceleration( ...
                navSol, correctedImu) * delaySeconds;

            actual = SensorDelayCompensator.backPropagateNavigation( ...
                navSol, correctedImu, delaySeconds);

            testCase.verifyEqual(actual.Cbn, navSol.Cbn, AbsTol=0.0);
            testCase.verifyEqual(actual.PositionLlh, expectedPosition, AbsTol=1.0e-14);
            testCase.verifyEqual(actual.VelocityEnu, expectedVelocity, AbsTol=1.0e-14);
        end

        function backPropagateVelocityLeavesPositionUnchanged(testCase)
            navSol = SensorDelayCompensatorTest.createNavigation();
            correctedImu = SensorDelayCompensatorTest.createImu();
            delaySeconds = 0.02;
            expectedVelocity = navSol.VelocityEnu ...
                - SensorDelayCompensatorTest.navigationAcceleration( ...
                navSol, correctedImu) * delaySeconds;

            actual = SensorDelayCompensator.backPropagateVelocity( ...
                navSol, correctedImu, delaySeconds);

            testCase.verifyEqual(actual.Cbn, navSol.Cbn, AbsTol=0.0);
            testCase.verifyEqual(actual.PositionLlh, navSol.PositionLlh, AbsTol=0.0);
            testCase.verifyEqual(actual.VelocityEnu, expectedVelocity, AbsTol=1.0e-14);
        end

        function backPropagateHeightOnlyChangesAltitude(testCase)
            navSol = SensorDelayCompensatorTest.createNavigation();
            delaySeconds = 0.02;
            expectedPosition = navSol.PositionLlh;
            expectedPosition(3) = expectedPosition(3) ...
                - navSol.VelocityEnu(3) * delaySeconds;

            actual = SensorDelayCompensator.backPropagateHeight(navSol, delaySeconds);

            testCase.verifyEqual(actual.Cbn, navSol.Cbn, AbsTol=0.0);
            testCase.verifyEqual(actual.VelocityEnu, navSol.VelocityEnu, AbsTol=0.0);
            testCase.verifyEqual(actual.PositionLlh, expectedPosition, AbsTol=1.0e-14);
        end

        function stationarySpecificForceDoesNotCreateVelocity(testCase)
            navSol = struct();
            navSol.Cbn = eye(3);
            navSol.VelocityEnu = zeros(3, 1);
            navSol.PositionLlh = [deg2rad(30.0); deg2rad(120.0); -50.0];
            correctedImu = struct();
            correctedImu.Time = 0.0;
            correctedImu.Gyro = zeros(3, 1);
            correctedImu.Accel = -gravityENU( ...
                navSol.PositionLlh(1), navSol.PositionLlh(3));

            actual = SensorDelayCompensator.forwardImu(navSol, correctedImu, 0.03);

            testCase.verifyEqual(actual.Cbn, navSol.Cbn, AbsTol=0.0);
            testCase.verifyEqual(actual.PositionLlh, navSol.PositionLlh, AbsTol=1.0e-14);
            testCase.verifyEqual(actual.VelocityEnu, zeros(3, 1), AbsTol=1.0e-14);
        end

        function firstOrderVelocityMatchesMechanizationDerivative(testCase)
            navSol = SensorDelayCompensatorTest.createNavigation();
            correctedImu = SensorDelayCompensatorTest.createImu();
            deltaTime = 1.0e-6;
            [~, mechanizedVelocity, ~] = insUpdateENU( ...
                navSol.Cbn, navSol.VelocityEnu, navSol.PositionLlh, ...
                correctedImu.Gyro, correctedImu.Accel, deltaTime);

            actual = SensorDelayCompensator.forwardImu( ...
                navSol, correctedImu, deltaTime);

            testCase.verifyEqual( ...
                actual.VelocityEnu, mechanizedVelocity, AbsTol=1.0e-10);
        end
    end

    methods (Static, Access = private)
        function navSol = createNavigation()
            navSol = struct();
            navSol.Cbn = dcmFromEuler(deg2rad([2.0; -3.0; 15.0]));
            navSol.VelocityEnu = [4.0; 3.0; -1.0];
            navSol.PositionLlh = [deg2rad(30.0); deg2rad(120.0); -50.0];
        end

        function correctedImu = createImu()
            correctedImu = struct();
            correctedImu.Time = 1.0;
            correctedImu.Gyro = [0.01; -0.02; 0.03];
            correctedImu.Accel = [0.5; -0.25; 1.0];
        end

        function positionLlh = propagatePosition(positionLlh, velocityEnu, deltaTime)
            latitude = positionLlh(1);
            altitude = positionLlh(3);
            [meridianRadius, transverseRadius] = earthRadii(latitude);
            positionLlh(1) = positionLlh(1) + velocityEnu(2) * deltaTime ...
                / (meridianRadius + altitude);
            positionLlh(2) = positionLlh(2) + velocityEnu(1) * deltaTime ...
                / ((transverseRadius + altitude) * cos(latitude));
            positionLlh(3) = positionLlh(3) + velocityEnu(3) * deltaTime;
        end

        function accelerationEnu = navigationAcceleration(navSol, correctedImu)
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
    end
end
