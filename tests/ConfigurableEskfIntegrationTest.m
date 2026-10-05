classdef ConfigurableEskfIntegrationTest < matlab.unittest.TestCase
    %CONFIGURABLEESKFINTEGRATIONTEST 验证 9/15 维短程 ESKF 集成行为。

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, "IncludingSubfolders", true);
            testCase.applyFixture(fixture);
        end
    end

    methods (Test)
        function ins9ShortRunIsFiniteAndKeepsBiasFixed(testCase)
            result = ConfigurableEskfIntegrationTest.runShortIntegration(testCase, "ins9");
            biasChange = [
                diff(result.GyroBias, 1, 1)
                diff(result.AccelBias, 1, 1)
                ];

            testCase.verifyEqual(result.StateModel.Profile, "ins9");
            testCase.verifyEqual(result.StateModel.Dimension, 9);
            testCase.verifyEqual(result.StateModel.BodyFrame, "RFU");
            testCase.verifyEqual(result.StateModel.NavigationFrame, "ENU");
            testCase.verifyTrue(all(isfinite(result.PositionLlh), "all"));
            testCase.verifyTrue(all(isfinite(result.VelocityEnu), "all"));
            testCase.verifyTrue(all(isfinite(result.Euler), "all"));
            testCase.verifyEqual(biasChange, zeros(size(biasChange)), "AbsTol", 0.0);
        end

        function ins15UsesRfuInputAndRemainsFinite(testCase)
            result = ConfigurableEskfIntegrationTest.runShortIntegration(testCase, "ins15");

            testCase.verifyEqual(result.StateModel.Profile, "ins15");
            testCase.verifyEqual(result.StateModel.Dimension, 15);
            testCase.verifyEqual(result.StateModel.BodyFrame, "RFU");
            testCase.verifyEqual(result.StateModel.NavigationFrame, "ENU");
            testCase.verifyTrue(all(isfinite(result.PositionLlh), "all"));
            testCase.verifyTrue(all(isfinite(result.VelocityEnu), "all"));
            testCase.verifyTrue(all(isfinite(result.Euler), "all"));
            testCase.verifyEqual(result.Euler(1, :, 1), zeros(1, 3), AbsTol=1.0e-14);
        end
    end

    methods (Static, Access = private)
        function result = runShortIntegration(testCase, profileName)
            % 每次生成采用当前 RFU 约定的短轨迹，避免依赖旧坐标系 MAT 文件。
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            trajectoryCfg = setTrajectoryOptions();
            trajectoryCfg.Duration = 2.0;
            trajectoryCfg.DepthMotionType = "constant";
            trajectoryCfg.RollAmplitudeDeg = 0.0;
            trajectoryCfg.SaveToFile = false;
            inputData = genetraj(trajectoryCfg);
            inputFile = string(tempname) + ".mat";
            save(inputFile, "inputData");
            testCase.addTeardown(@() delete(inputFile));

            cfg = setConfig();
            cfg.algorithm.stateModel.profile = profileName;
            cfg.algorithm.initialPerturbation.isEnabled = false;
            cfg.sim.runs = 1;
            cfg.sim.duration = 2.0;
            cfg.data.file = inputFile;
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.sensor.dvl.isEnabled = true;
            cfg.sensor.dvl.leverArmBody = zeros(3, 1);
            cfg.sensor.depth.isEnabled = false;
            cfg.sensor.gps.isEnabled = false;
            cfg.sensorDelay.isEnabled = false;
            cfg.result.outputFolder = fullfile(projectRoot, "data", "output");

            rng(20260811, "twister");
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            results = FilterResults(cfg, meas);
            results = ESKF(cfg, meas, results);
            result = results.Data.ESKF;
        end
    end
end
