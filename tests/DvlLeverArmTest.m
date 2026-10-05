classdef DvlLeverArmTest < matlab.unittest.TestCase
    %DVLLEVERARMTEST 验证已知 DVL 杆臂的物理方向、线性化及端到端补偿。

    properties (TestParameter)
        stateProfile = {"ins9", "ins15"};
        delayEnabled = {false, true};
        timeOffset = {-0.02, 0.0, 0.01};
        badLeverArm = {[1; 2], [1; NaN; 3], [1; Inf; 3], [1; 2; 3i]};
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                projectRoot, IncludingSubfolders=true));
        end
    end

    methods (Test)
        function zeroLeverArmPreservesOriginalModel(testCase, stateProfile)
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = stateProfile;
            cfg.sensor.dvl.leverArmBody = zeros(3, 1);
            model = InertialErrorStateModel(cfg);
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();
            expectedH = zeros(3, model.Layout.Dimension);
            expectedH(:, model.Layout.Index.Attitude) = navSol.Cbn.' * skew(navSol.VelocityEnu);
            expectedH(:, model.Layout.Index.Velocity) = navSol.Cbn.';

            [residual, H, R] = model.BuildDvlMeasurement(navSol, measurement, imu);
            [legacyResidual, legacyH, legacyR] = model.BuildDvlMeasurement(navSol, measurement);

            testCase.verifyEqual(residual, measurement.VelocityBody ...
                - navSol.Cbn.' * navSol.VelocityEnu, AbsTol=1.0e-14);
            testCase.verifyEqual(H, expectedH, AbsTol=0.0);
            testCase.verifyEqual(R, measurement.R, AbsTol=0.0);
            testCase.verifyEqual(legacyResidual, residual, AbsTol=0.0);
            testCase.verifyEqual(legacyH, H, AbsTol=0.0);
            testCase.verifyEqual(legacyR, R, AbsTol=0.0);
        end

        function positiveUpRotationMovesRightLeverForward(testCase, stateProfile)
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = stateProfile;
            cfg.sensor.dvl.leverArmBody = [1; 0; 0];
            model = InertialErrorStateModel(cfg);
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();
            navSol.Cbn = eye(3);
            navSol.VelocityEnu = zeros(3, 1);
            imu.Gyro = [0; 0; 0.1];
            measurement.VelocityBody = [0; 0.1; 0];

            residual = model.BuildDvlMeasurement(navSol, measurement, imu);

            testCase.verifyEqual(residual, zeros(3, 1), AbsTol=1.0e-15);
        end

        function jacobianMatchesFeedbackConvention(testCase, stateProfile)
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = stateProfile;
            cfg.sensor.dvl.leverArmBody = [1.2; -0.4; 0.25];
            model = InertialErrorStateModel(cfg);
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();

            [~, H] = model.BuildDvlMeasurement(navSol, measurement, imu);
            numericalH = DvlLeverArmTest.finiteDifference(model, navSol, imu, measurement);

            testCase.verifyEqual(H, numericalH, AbsTol=2.0e-8);
        end

        function gyroNoiseMapsIntoTransverseVelocity(testCase)
            cfg = setConfig();
            cfg.sensor.dvl.leverArmBody = [2; 0; 0];
            cfg.noise.imu.gyroStd = [0.01; 0.02; 0.03];
            model = InertialErrorStateModel(cfg);
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();

            [~, ~, R] = model.BuildDvlMeasurement(navSol, measurement, imu);

            testCase.verifyEqual(R - measurement.R, diag([0; 0.0036; 0.0016]), AbsTol=1.0e-15);
        end

        function biasCompensationFeedsLeverVelocity(testCase)
            cfg = setConfig();
            cfg.sensor.dvl.leverArmBody = [1; 0; 0];
            model = InertialErrorStateModel(cfg);
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();
            navSol.VelocityEnu = zeros(3, 1);
            bias = struct('GyroBias', [0; 0; 0.03], 'AccelBias', zeros(3, 1));
            imu.Gyro = [0; 0; 0.13];
            measurement.VelocityBody = [0; 0.1; 0];
            correctedImu = ErrorStateKF.CompensateImu(bias, imu);

            residual = model.BuildDvlMeasurement(navSol, measurement, correctedImu);

            testCase.verifyEqual(residual, zeros(3, 1), AbsTol=1.0e-15);
        end

        function nonzeroLeverRequiresGyro(testCase)
            cfg = setConfig();
            cfg.sensor.dvl.leverArmBody = [1; 0; 0];
            model = InertialErrorStateModel(cfg);
            [navSol, ~, measurement] = DvlLeverArmTest.createInputs();
            filter = ErrorStateKF(model, eye(model.Layout.Dimension));

            testCase.verifyError(@() filter.UpdateDvl(navSol, measurement), ...
                "InertialErrorStateModel:MissingDvlGyro");
        end

        function invalidLeverArmIsRejected(testCase, badLeverArm)
            cfg = setConfig();
            cfg.sensor.dvl.leverArmBody = badLeverArm;

            testCase.verifyError(@() InertialErrorStateModel(cfg), ...
                "InertialErrorStateModel:InvalidDvlLeverArm");
        end

        function stationaryTruthHasNoLeverVelocity(testCase)
            [navSol, imu, measurement] = DvlLeverArmTest.createInputs();
            navSol.VelocityEnu = zeros(3, 1);
            constants = getWgs84Constants();
            latitude = navSol.PositionLlh(1);
            imu.Gyro = navSol.Cbn.' * (constants.wie * [0; cos(latitude); sin(latitude)]);
            leverArm = [1.2; -0.4; 0.25];
            velocity = createDvlVelocityBody(zeros(1, 3), navSol.Cbn, ...
                imu.Gyro.', navSol.PositionLlh.', leverArm);
            cfg = setConfig();
            cfg.sensor.dvl.leverArmBody = leverArm;
            model = InertialErrorStateModel(cfg);
            measurement.VelocityBody = velocity.';

            residual = model.BuildDvlMeasurement(navSol, measurement, imu);

            testCase.verifyEqual(velocity, zeros(1, 3), AbsTol=1.0e-15);
            testCase.verifyLessThanOrEqual(norm(residual), constants.wie * norm(leverArm));
        end

        function delayAttitudeUsesSignedSampleTimeOffset(testCase, timeOffset)
            [navSol, imu, ~] = DvlLeverArmTest.createInputs();
            navSol.Cbn = eye(3);
            navSol.VelocityEnu = zeros(3, 1);
            navSol.PositionLlh(1) = 0.0;
            constants = getWgs84Constants();
            % 在赤道绕北轴转动，与地球自转轴平行，闭式解为纯 y 轴转动。
            imu.Gyro = [0; 0.2 + constants.wie; 0];
            imu.Accel = -gravityENU(0, navSol.PositionLlh(3));
            angle = 0.2 * timeOffset;
            expectedCbn = [cos(angle), 0, sin(angle); 0, 1, 0; -sin(angle), 0, cos(angle)];

            actual = SensorDelayCompensator.backPropagateVelocity(navSol, imu, 0.02, timeOffset);

            testCase.verifyEqual(actual.Cbn, expectedCbn, AbsTol=1.0e-14);
            testCase.verifyEqual(actual.VelocityEnu, zeros(3, 1), AbsTol=1.0e-14);
        end

        function turningTrajectoryBenefitsFromCompensation(testCase, stateProfile, delayEnabled)
            trajectoryCfg = setTrajectoryOptions();
            trajectoryCfg.Duration = 8.0;
            trajectoryCfg.TrajectoryType = "circle";
            trajectoryCfg.HorizontalRadius = 10.0;
            trajectoryCfg.HorizontalPeriod = 20.0;
            trajectoryCfg.DepthMotionType = "constant";
            trajectoryCfg.RollAmplitudeDeg = 0.0;
            trajectoryCfg.DvlLeverArmBody = [1.2; -0.4; 0.25];
            trajectoryCfg.SaveToFile = false;
            inputData = genetraj(trajectoryCfg);
            inputFile = string(tempname) + ".mat";
            save(inputFile, "inputData");
            testCase.addTeardown(@() delete(inputFile));
            cfg = setConfig();
            cfg.data.file = inputFile;
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.sim.runs = 1;
            cfg.algorithm.stateModel.profile = stateProfile;
            cfg.algorithm.initialPerturbation.isEnabled = false;
            cfg.sensor.dvl.isEnabled = true;
            cfg.sensorDelay.isEnabled = delayEnabled;
            cfg.sensor.dvl.leverArmBody = trajectoryCfg.DvlLeverArmBody;

            [compensatedRmse, result] = DvlLeverArmTest.runIntegration(cfg, inputData);
            cfg.sensor.dvl.leverArmBody = zeros(3, 1);
            uncompensatedRmse = DvlLeverArmTest.runIntegration(cfg, inputData);

            testCase.verifyLessThan(compensatedRmse, 0.35 * uncompensatedRmse);
            testCase.verifyTrue(all(isfinite(result.PositionLlh), "all"));
            testCase.verifyTrue(all(isfinite(result.Euler), "all"));
            testCase.verifyEqual(result.StateModel.DvlLeverArmBody, ...
                inputData.metadata.dvlLeverArmBody, AbsTol=0.0);
        end
    end

    methods (Static, Access = private)
        function [navSol, imu, measurement] = createInputs()
            navSol = struct('Cbn', dcmFromEuler(deg2rad([2; -3; 25])), ...
                'VelocityEnu', [1.2; -0.6; 0.3], ...
                'PositionLlh', [deg2rad(30); deg2rad(120); -30]);
            imu = struct('Time', 0.0, 'Gyro', [0.1; -0.2; 0.3], 'Accel', [0; 0; 9.8]);
            measurement = struct('VelocityBody', [0.6; 0.1; -0.3], 'R', 0.01 * eye(3));
        end

        function H = finiteDifference(model, navSol, imu, measurement)
            bias = struct('GyroBias', zeros(3, 1), 'AccelBias', zeros(3, 1));
            step = 1.0e-6;
            H = zeros(3, model.Layout.Dimension);
            for column = 1:model.Layout.Dimension
                perturbation = zeros(model.Layout.Dimension, 1);
                perturbation(column) = step;
                [plusNav, plusBias] = model.FeedbackNavigation(navSol, bias, perturbation);
                [minusNav, minusBias] = model.FeedbackNavigation(navSol, bias, -perturbation);
                plusImu = ErrorStateKF.CompensateImu(plusBias, imu);
                minusImu = ErrorStateKF.CompensateImu(minusBias, imu);
                plusResidual = model.BuildDvlMeasurement(plusNav, measurement, plusImu);
                minusResidual = model.BuildDvlMeasurement(minusNav, measurement, minusImu);
                H(:, column) = -(plusResidual - minusResidual) / (2.0 * step);
            end
        end

        function [velocityRmse, result] = runIntegration(cfg, inputData)
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            results = FilterResults(cfg, meas);
            results = ESKF(cfg, meas, results);
            result = results.Data.ESKF;
            velocityError = result.VelocityEnu(:, :, 1) - inputData.truth.velocityEnu;
            velocityRmse = sqrt(mean(sum(velocityError.^2, 2)));
        end
    end
end
