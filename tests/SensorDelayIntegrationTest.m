classdef SensorDelayIntegrationTest < matlab.unittest.TestCase
    %SENSORDELAYINTEGRATIONTEST 验证固定总线延时调度和短程 ESKF 集成。

    properties (TestParameter)
        stateProfile = {"ins9", "ins15"};
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, IncludingSubfolders=true);
            testCase.applyFixture(fixture);
        end
    end

    methods (TestMethodSetup)
        function resetRandomSeed(testCase)
            originalRng = rng;
            testCase.addTeardown(@() rng(originalRng));
            rng(20260811, "twister");
        end
    end

    methods (Test)
        function delaySwitchRestoresImmediateArrival(testCase)
            cfg = SensorDelayIntegrationTest.createMeasurementConfig(false);
            meas = SensorDelayIntegrationTest.createMeasurements(cfg);

            imu = meas.getImu(1);
            dvl = meas.getDvl(1);
            depth = meas.getDepth(1);
            gps = meas.getGps(1);

            testCase.verifyTrue(imu.Valid);
            testCase.verifyTrue(dvl.Valid);
            testCase.verifyTrue(depth.Valid);
            testCase.verifyTrue(gps.Valid);
            testCase.verifyEqual(imu.DelaySteps, 0);
            testCase.verifyEqual(dvl.DelaySteps, 0);
            testCase.verifyEqual(depth.DelaySteps, 0);
            testCase.verifyEqual(gps.DelaySteps, 0);
            testCase.verifyEqual(imu.SampleTime, imu.ArrivalTime, AbsTol=1.0e-15);
        end

        function fixedDelayMapsSamplesToArrivalIndices(testCase)
            cfg = SensorDelayIntegrationTest.createMeasurementConfig(true);
            meas = SensorDelayIntegrationTest.createMeasurements(cfg);
            time = meas.getTime();

            earlyImu = meas.getImu(3);
            imu = meas.getImu(4);
            dvl = meas.getDvl(3);
            depth = meas.getDepth(3);
            gps = meas.getGps(3);

            testCase.verifyFalse(earlyImu.Valid);
            testCase.verifyTrue(imu.Valid);
            testCase.verifyEqual(imu.SampleTime, time(1), AbsTol=1.0e-15);
            testCase.verifyEqual(imu.ArrivalTime, time(4), AbsTol=1.0e-15);
            testCase.verifyEqual(imu.DelaySteps, 3);
            testCase.verifyEqual(imu.DelaySeconds, 0.03, AbsTol=1.0e-15);
            testCase.verifyTrue(dvl.Valid);
            testCase.verifyTrue(depth.Valid);
            testCase.verifyTrue(gps.Valid);
            testCase.verifyEqual(dvl.SampleTime, time(1), AbsTol=1.0e-15);
            testCase.verifyEqual(depth.SampleTime, time(1), AbsTol=1.0e-15);
            testCase.verifyEqual(gps.SampleTime, time(1), AbsTol=1.0e-15);
            testCase.verifyEqual(dvl.ArrivalTime, time(3), AbsTol=1.0e-15);
            testCase.verifyEqual(depth.DelaySteps, 2);
            testCase.verifyEqual(gps.DelaySeconds, 0.02, AbsTol=1.0e-15);
        end

        function invalidDelayStepsAreRejected(testCase)
            cfg = SensorDelayIntegrationTest.createMeasurementConfig(true);
            cfg.sensorDelay.imu.delaySteps = 4;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();

            testCase.verifyError( ...
                @() meas.checkInputData(), "StateAndMeasurement:InvalidDelaySteps");
        end

        function numericDelaySwitchIsRejected(testCase)
            cfg = SensorDelayIntegrationTest.createMeasurementConfig(true);
            cfg.sensorDelay.isEnabled = 1;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();

            testCase.verifyError( ...
                @() meas.checkInputData(), "StateAndMeasurement:InvalidDelaySwitch");
        end

        function delayedEskfRemainsFinite(testCase, stateProfile)
            cfg = SensorDelayIntegrationTest.createMeasurementConfig(true);
            cfg.algorithm.stateModel.profile = stateProfile;
            cfg.sim.duration = 2.0;
            meas = SensorDelayIntegrationTest.createMeasurements(cfg);
            results = FilterResults(cfg, meas);

            results = ESKF(cfg, meas, results);
            result = results.Data.ESKF;

            testCase.verifyTrue(all(isfinite(result.PositionLlh), "all"));
            testCase.verifyTrue(all(isfinite(result.VelocityEnu), "all"));
            testCase.verifyTrue(all(isfinite(result.Euler), "all"));
            testCase.verifyTrue(all(isfinite(result.GyroBias), "all"));
            testCase.verifyTrue(all(isfinite(result.AccelBias), "all"));
        end

        function delayCompensationPreservesVerticalAccuracy(testCase)
            [noDelayRmse, delayRmse] = ...
                SensorDelayIntegrationTest.compareVerticalPositionRmse();

            testCase.verifyLessThan(abs(delayRmse - noDelayRmse), 0.05);
        end
    end

    methods (Static, Access = private)
        function cfg = createMeasurementConfig(isDelayEnabled)
            cfg = setConfig();
            cfg.sim.runs = 1;
            cfg.sim.duration = 0.2;
            cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input.mat");
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.sensor.dvl.isEnabled = true;
            cfg.sensor.dvl.leverArmBody = zeros(3, 1);
            cfg.sensor.depth.isEnabled = true;
            cfg.sensor.gps.isEnabled = true;
            cfg.sensorDelay.isEnabled = isDelayEnabled;
        end

        function meas = createMeasurements(cfg)
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            meas.prepareMonteCarloRun(1);
        end

        function [noDelayRmse, delayRmse] = compareVerticalPositionRmse()
            baseCfg = setConfig();
            baseCfg.sensor.dvl.leverArmBody = zeros(3, 1);
            baseCfg.sim.duration = 30.0;
            baseCfg.sim.runs = 1;
            baseCfg.data.isSensorNoiseMonteCarlo = false;
            baseCfg.sensor.depth.isEnabled = false;
            baseCfg.sensor.gps.isEnabled = false;
            randomState = rng;

            noDelayCfg = baseCfg;
            noDelayCfg.sensorDelay.isEnabled = false;
            noDelayMeas = StateAndMeasurement(noDelayCfg);
            noDelayMeas.loadInputData();
            noDelayMeas.checkInputData();
            noDelayResults = FilterResults(noDelayCfg, noDelayMeas);
            evalc("noDelayResults = ESKF(noDelayCfg, noDelayMeas, noDelayResults);");
            noDelayResults.computeErrors(noDelayMeas);
            noDelayError = noDelayResults.Error.ESKF.PositionEnu(:, 3, 1);
            noDelayRmse = sqrt(mean(noDelayError.^2, "omitnan"));

            rng(randomState);
            delayCfg = baseCfg;
            delayCfg.sensorDelay.isEnabled = true;
            delayMeas = StateAndMeasurement(delayCfg);
            delayMeas.loadInputData();
            delayMeas.checkInputData();
            delayResults = FilterResults(delayCfg, delayMeas);
            evalc("delayResults = ESKF(delayCfg, delayMeas, delayResults);");
            delayResults.computeErrors(delayMeas);
            delayError = delayResults.Error.ESKF.PositionEnu(:, 3, 1);
            delayRmse = sqrt(mean(delayError.^2, "omitnan"));
        end
    end
end
