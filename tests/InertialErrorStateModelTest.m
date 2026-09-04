classdef InertialErrorStateModelTest < matlab.unittest.TestCase
    %INERTIALERRORSTATEMODELTEST 验证可配置状态模型的布局和公共行为。

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, "IncludingSubfolders", true);
            testCase.applyFixture(fixture);
        end
    end

    methods (TestMethodSetup)
        function resetRandomSeed(testCase)
            originalRng = rng;
            testCase.addTeardown(@() rng(originalRng));
            rng(20260824, "twister");
        end
    end

    methods (Test)
        function ins9ProfileBuildsExpectedLayout(testCase)
            model = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins9"));

            testCase.verifyEqual(model.ProfileName, "ins9");
            testCase.verifyEqual(model.Layout.Dimension, 9);
            testCase.verifyEqual(model.Layout.BlockNames, ...
                ["Attitude", "Velocity", "Position"]);
            testCase.verifyEqual(model.Layout.Index.Attitude, 1:3);
            testCase.verifyEqual(model.Layout.Index.Velocity, 4:6);
            testCase.verifyEqual(model.Layout.Index.Position, 7:9);
            testCase.verifyFalse(model.HasState("GyroBias"));
        end

        function ins15ProfileBuildsExpectedLayout(testCase)
            model = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins15"));

            testCase.verifyEqual(model.ProfileName, "ins15");
            testCase.verifyEqual(model.Layout.Dimension, 15);
            testCase.verifyEqual(model.Layout.BlockNames, ...
                ["Attitude", "Velocity", "Position", "GyroBias", "AccelBias"]);
            testCase.verifyEqual(model.Layout.Index.GyroBias, 10:12);
            testCase.verifyEqual(model.Layout.Index.AccelBias, 13:15);
            testCase.verifyTrue(model.HasState("GyroBias"));
        end

        function unknownProfileIsRejected(testCase)
            cfg = InertialErrorStateModelTest.createConfig("ins21-unknown");

            testCase.verifyError(@() InertialErrorStateModel(cfg), ...
                "createInertialStateProfile:UnknownProfile");
        end

        function dynamicsFollowSelectedDimension(testCase)
            model9 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins9"));
            model15 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins15"));
            [navSol, correctedImu] = InertialErrorStateModelTest.createNavigationInputs();

            [F9, G9, Qc9] = model9.BuildErrorDynamics(navSol, correctedImu);
            [F15, G15, Qc15] = model15.BuildErrorDynamics(navSol, correctedImu);

            testCase.verifySize(F9, [9, 9]);
            testCase.verifySize(G9, [9, 6]);
            testCase.verifySize(F15, [15, 15]);
            testCase.verifySize(G15, [15, 6]);
            testCase.verifySize(Qc9, [6, 6]);
            testCase.verifyEqual(Qc15, Qc9);
            testCase.verifyEqual(F15(1:9, 1:9), F9, "AbsTol", 1.0e-15);
            testCase.verifyEqual(F15(1:3, 10:12), -navSol.Cbn, "AbsTol", 1.0e-15);
            testCase.verifyEqual(F15(4:6, 13:15), -navSol.Cbn, "AbsTol", 1.0e-15);
        end

        function measurementsFollowSelectedDimension(testCase)
            model9 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins9"));
            model15 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins15"));
            [navSol, ~] = InertialErrorStateModelTest.createNavigationInputs();
            [dvl, depth, gps] = InertialErrorStateModelTest.createMeasurements(navSol);

            [~, dvlH9] = model9.BuildDvlMeasurement(navSol, dvl);
            [~, depthH9] = model9.BuildDepthMeasurement(navSol, depth);
            [~, gpsH9] = model9.BuildGpsMeasurement(navSol, gps);
            [~, dvlH15] = model15.BuildDvlMeasurement(navSol, dvl);
            [~, depthH15] = model15.BuildDepthMeasurement(navSol, depth);
            [~, gpsH15] = model15.BuildGpsMeasurement(navSol, gps);

            testCase.verifySize(dvlH9, [3, 9]);
            testCase.verifySize(depthH9, [1, 9]);
            testCase.verifySize(gpsH9, [3, 9]);
            testCase.verifySize(dvlH15, [3, 15]);
            testCase.verifySize(depthH15, [1, 15]);
            testCase.verifySize(gpsH15, [3, 15]);
            testCase.verifyEqual(depthH9(9), -1.0);
            testCase.verifyEqual(gpsH9(:, 7:9), eye(3));
            testCase.verifyEqual(dvlH15(:, 10:15), zeros(3, 6));
        end

        function feedbackOnlyTouchesPresentStates(testCase)
            model9 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins9"));
            model15 = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins15"));
            [navSol, ~] = InertialErrorStateModelTest.createNavigationInputs();
            imuBias.GyroBias = [1.0; 2.0; 3.0];
            imuBias.AccelBias = [4.0; 5.0; 6.0];
            error9 = zeros(9, 1);
            error15 = [error9; 0.1; 0.2; 0.3; 0.4; 0.5; 0.6];

            [~, bias9] = model9.FeedbackNavigation(navSol, imuBias, error9);
            [~, bias15] = model15.FeedbackNavigation(navSol, imuBias, error15);

            testCase.verifyEqual(bias9, imuBias);
            testCase.verifyEqual(bias15.GyroBias, [1.1; 2.2; 3.3], "AbsTol", 1.0e-15);
            testCase.verifyEqual(bias15.AccelBias, [4.4; 5.5; 6.6], "AbsTol", 1.0e-15);
        end

        function feedbackAcceptsRowVectorAndRejectsWrongDimension(testCase)
            model = InertialErrorStateModel( ...
                InertialErrorStateModelTest.createConfig("ins9"));
            [navSol, ~] = InertialErrorStateModelTest.createNavigationInputs();
            imuBias.GyroBias = zeros(3, 1);
            imuBias.AccelBias = zeros(3, 1);

            [updatedNavigation, ~] = model.FeedbackNavigation( ...
                navSol, imuBias, zeros(1, 9));

            testCase.verifySize(updatedNavigation.VelocityEnu, [3, 1]);
            testCase.verifyError( ...
                @() model.FeedbackNavigation(navSol, imuBias, zeros(8, 1)), ...
                "InertialErrorStateModel:InvalidErrorState");
        end

        function filterCoreSupportsBothDimensions(testCase)
            filter9 = InertialErrorStateModelTest.createFilter("ins9");
            filter15 = InertialErrorStateModelTest.createFilter("ins15");
            [navSol, correctedImu] = InertialErrorStateModelTest.createNavigationInputs();
            [dvl, depth, gps] = InertialErrorStateModelTest.createMeasurements(navSol);

            filter9.Preparation(2);
            filter15.Preparation(2);
            filter9.Predict(navSol, correctedImu, 0.01);
            filter15.Predict(navSol, correctedImu, 0.01);
            filter9.UpdateDvl(navSol, dvl);
            filter9.UpdateDepth(navSol, depth);
            filter9.UpdateGps(navSol, gps);
            filter15.UpdateDvl(navSol, dvl);
            filter15.UpdateDepth(navSol, depth);
            filter15.UpdateGps(navSol, gps);

            covariance9 = filter9.GetCovariance();
            covariance15 = filter15.GetCovariance();
            testCase.verifySize(filter9.GetErrorState(), [9, 1]);
            testCase.verifySize(filter15.GetErrorState(), [15, 1]);
            testCase.verifyEqual(covariance9, covariance9.', "AbsTol", 1.0e-15);
            testCase.verifyEqual(covariance15, covariance15.', "AbsTol", 1.0e-15);

            filter9.ResetErrorState();
            filter15.ResetErrorState();
            testCase.verifyEqual(filter9.GetErrorState(), zeros(9, 1));
            testCase.verifyEqual(filter15.GetErrorState(), zeros(15, 1));
        end

        function disabledPerturbationPreservesNominalStateAndNonzeroP0(testCase)
            cfg = InertialErrorStateModelTest.createConfig("ins15");
            cfg.algorithm.initialPerturbation.isEnabled = false;
            cfg.algorithm.initialPerturbation.attitudeStd = 10.0 * ones(3, 1);
            cfg.algorithm.initialPerturbation.velocityStd = 10.0 * ones(3, 1);
            cfg.algorithm.initialPerturbation.positionStd = 10.0 * ones(3, 1);
            meas = InertialErrorStateModelTest.createMeasurement(cfg);
            expectedNavigation = meas.getInitialNavigation();
            expectedBias = meas.getInitialImuBias();
            model = InertialErrorStateModel(cfg);
            expectedCovariance = diag(model.InitialCovarianceStd.^2);

            [actualNavigation, actualBias, actualCovariance] = ...
                model.CreateInitialNavigation(meas);

            testCase.verifyEqual(actualNavigation.Cbn, expectedNavigation.Cbn, AbsTol=0.0);
            testCase.verifyEqual( ...
                actualNavigation.VelocityEnu, expectedNavigation.VelocityEnu, AbsTol=0.0);
            testCase.verifyEqual( ...
                actualNavigation.PositionLlh, expectedNavigation.PositionLlh, AbsTol=0.0);
            testCase.verifyEqual(actualBias, expectedBias, AbsTol=0.0);
            testCase.verifyEqual(actualCovariance, expectedCovariance, AbsTol=1.0e-18);
            testCase.verifyGreaterThan(norm(actualCovariance, "fro"), 0.0);
        end

        function perturbationAndP0CanBeConfiguredIndependently(testCase)
            cfg = InertialErrorStateModelTest.createConfig("ins9");
            cfg.algorithm.initialPerturbation.isEnabled = true;
            cfg.algorithm.initialPerturbation.attitudeStd = zeros(3, 1);
            cfg.algorithm.initialPerturbation.velocityStd = [1.0; 0.0; 0.0];
            cfg.algorithm.initialPerturbation.positionStd = zeros(3, 1);
            cfg.algorithm.initialCovariance.attitudeStd = zeros(3, 1);
            cfg.algorithm.initialCovariance.velocityStd = zeros(3, 1);
            cfg.algorithm.initialCovariance.positionStd = zeros(3, 1);
            meas = InertialErrorStateModelTest.createMeasurement(cfg);
            expectedNavigation = meas.getInitialNavigation();
            model = InertialErrorStateModel(cfg);

            [actualNavigation, ~, actualCovariance] = ...
                model.CreateInitialNavigation(meas);

            testCase.verifyNotEqual( ...
                actualNavigation.VelocityEnu(1), expectedNavigation.VelocityEnu(1));
            testCase.verifyEqual( ...
                actualNavigation.VelocityEnu(2:3), expectedNavigation.VelocityEnu(2:3), AbsTol=0.0);
            testCase.verifyEqual(actualCovariance, zeros(9, 9), AbsTol=0.0);
        end
    end

    methods (Static, Access = private)
        function cfg = createConfig(profileName)
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = profileName;
        end

        function [navSol, correctedImu] = createNavigationInputs()
            navSol = struct();
            navSol.Cbn = dcmFromEuler(deg2rad([2.0; -1.0; 20.0]));
            navSol.VelocityEnu = [2.0; 1.0; -0.2];
            navSol.PositionLlh = [deg2rad(30.0); deg2rad(114.0); -50.0];

            correctedImu = struct();
            correctedImu.Time = 0.01;
            correctedImu.Gyro = [1.0e-3; -2.0e-3; 3.0e-3];
            correctedImu.Accel = [0.1; 0.2; 9.7];
        end

        function [dvl, depth, gps] = createMeasurements(navSol)
            dvl = struct();
            dvl.VelocityBody = navSol.Cbn.' * navSol.VelocityEnu + [0.1; -0.1; 0.05];
            dvl.R = diag([0.1, 0.1, 0.1].^2);

            depth = struct();
            depth.Depth = -navSol.PositionLlh(3) + 0.2;
            depth.R = 0.1^2;

            gps = struct();
            gps.PositionLlh = enuOffsetToLlh(navSol.PositionLlh, [1.0; -2.0; 0.5]);
            gps.R = diag([1.0, 1.0, 2.0].^2);
        end

        function filter = createFilter(profileName)
            cfg = InertialErrorStateModelTest.createConfig(profileName);
            model = InertialErrorStateModel(cfg);
            filter = ErrorStateKF(model, eye(model.Layout.Dimension));
        end

        function meas = createMeasurement(cfg)
            cfg.sim.duration = 0.02;
            cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input.mat");
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.sensorDelay.isEnabled = false;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
        end
    end
end
