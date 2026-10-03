classdef ImuBiasInjectionTest < matlab.unittest.TestCase
    %IMUBIASINJECTIONTEST 验证状态块驱动的 IMU 常值零偏注入。

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
        function ins9ProfileLeavesIdealImuUnbiased(testCase)
            cfg = ImuBiasInjectionTest.createConfig("ins9");
            actualMeasurement = ImuBiasInjectionTest.createMeasurements(cfg);
            idealMeasurement = ImuBiasInjectionTest.createIdealMeasurements(cfg);

            actualImu = actualMeasurement.getImu(1);
            idealImu = idealMeasurement.getImu(1);

            testCase.verifyEqual(actualImu.Gyro, idealImu.Gyro, AbsTol=0.0);
            testCase.verifyEqual(actualImu.Accel, idealImu.Accel, AbsTol=0.0);
        end

        function ins15ProfileAddsRunConstantBias(testCase)
            cfg = ImuBiasInjectionTest.createConfig("ins15");
            actualMeasurement = ImuBiasInjectionTest.createMeasurements(cfg);
            idealMeasurement = ImuBiasInjectionTest.createIdealMeasurements(cfg);

            firstActualImu = actualMeasurement.getImu(1);
            secondActualImu = actualMeasurement.getImu(2);
            firstIdealImu = idealMeasurement.getImu(1);
            secondIdealImu = idealMeasurement.getImu(2);
            firstDifference = [
                firstActualImu.Gyro - firstIdealImu.Gyro
                firstActualImu.Accel - firstIdealImu.Accel
                ];
            secondDifference = [
                secondActualImu.Gyro - secondIdealImu.Gyro
                secondActualImu.Accel - secondIdealImu.Accel
                ];

            testCase.verifyGreaterThan(norm(firstDifference), 0.0);
            testCase.verifyEqual(secondDifference, firstDifference, AbsTol=1.0e-15);
        end

        function ins9ProfileKeepsWhiteNoiseEnabled(testCase)
            cfg = ImuBiasInjectionTest.createConfig("ins9");
            cfg.noise.imu.gyroStd = 1.0e-5 * ones(3, 1);
            cfg.noise.imu.accelStd = 1.0e-3 * ones(3, 1);
            actualMeasurement = ImuBiasInjectionTest.createMeasurements(cfg);
            idealMeasurement = ImuBiasInjectionTest.createIdealMeasurements(cfg);

            actualImu = actualMeasurement.getImu(1);
            idealImu = idealMeasurement.getImu(1);
            measurementDifference = [
                actualImu.Gyro - idealImu.Gyro
                actualImu.Accel - idealImu.Accel
                ];

            testCase.verifyGreaterThan(norm(measurementDifference), 0.0);
        end
    end

    methods (Static, Access = private)
        function cfg = createConfig(profileName)
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = profileName;
            cfg.sim.duration = 0.02;
            cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input.mat");
            cfg.data.isSensorNoiseMonteCarlo = true;
            cfg.sensorDelay.isEnabled = false;
            cfg.noise.imu.gyroStd = zeros(3, 1);
            cfg.noise.imu.accelStd = zeros(3, 1);
        end

        function measurement = createMeasurements(cfg)
            measurement = StateAndMeasurement(cfg);
            measurement.loadInputData();
            measurement.checkInputData();
            measurement.prepareMonteCarloRun(1);
        end

        function measurement = createIdealMeasurements(cfg)
            cfg.data.isSensorNoiseMonteCarlo = false;
            measurement = ImuBiasInjectionTest.createMeasurements(cfg);
        end
    end
end
