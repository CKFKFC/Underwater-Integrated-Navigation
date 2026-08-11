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
            result = ConfigurableEskfIntegrationTest.runShortIntegration("ins9");
            biasChange = [
                diff(result.GyroBias, 1, 1)
                diff(result.AccelBias, 1, 1)
                ];

            testCase.verifyEqual(result.StateModel.Profile, "ins9");
            testCase.verifyEqual(result.StateModel.Dimension, 9);
            testCase.verifyTrue(all(isfinite(result.PositionLlh), "all"));
            testCase.verifyTrue(all(isfinite(result.VelocityEnu), "all"));
            testCase.verifyTrue(all(isfinite(result.Euler), "all"));
            testCase.verifyEqual(biasChange, zeros(size(biasChange)), "AbsTol", 0.0);
        end

        function ins15MatchesLegacyBaseline(testCase)
            result = ConfigurableEskfIntegrationTest.runShortIntegration("ins15");
            checkpointIndex = [1, 51, 101, 201];
            actual = [
                result.PositionLlh(checkpointIndex, :, 1), ...
                result.VelocityEnu(checkpointIndex, :, 1), ...
                result.Euler(checkpointIndex, :, 1), ...
                result.GyroBias(checkpointIndex, :, 1), ...
                result.AccelBias(checkpointIndex, :, 1)
                ];
            expected = ConfigurableEskfIntegrationTest.legacySnapshot();

            testCase.verifyEqual(result.StateModel.Profile, "ins15");
            testCase.verifyEqual(result.StateModel.Dimension, 15);
            testCase.verifyTrue(all(isfinite(actual), "all"));
            testCase.verifyEqual(actual, expected, "AbsTol", 1.0e-12);
        end
    end

    methods (Static, Access = private)
        function result = runShortIntegration(profileName)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            cfg = setConfig();
            cfg.algorithm.stateModel.profile = profileName;
            cfg.sim.runs = 1;
            cfg.sim.duration = 2.0;
            cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input.mat");
            cfg.sensor.dvl.isEnabled = true;
            cfg.sensor.depth.isEnabled = false;
            cfg.sensor.gps.isEnabled = false;
            cfg.result.outputFolder = fullfile(projectRoot, "data", "output");

            rng(20260811, "twister");
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            results = FilterResults(cfg, meas);
            results = ESKF(cfg, meas, results);
            result = results.Data.ESKF;
        end

        function expected = legacySnapshot()
            expected = [
                0.57089737512141892, 2.0682151243785505, -49.863409680000224, 2.2532057113413635, 2.3723023291524266, -0.43361386388403322, -0.022285749244323649, 0.12663081990140843, 0.80451339804098476, 0, 0, 0, 0, 0, 0
                0.57089756457788399, 2.0682153356959656, -50.094431467136694, 2.2205127611186986, 2.4168507424977297, -0.45741417769533715, -0.01734954807455957, 0.12539151860156911, 0.81742555074551426, -6.7061092776486134e-11, -8.5315015587438488e-11, -1.1365699512249775e-10, -1.2973605688029933e-06, 1.2755900931653176e-06, -1.6991429845125155e-07
                0.570897758960483, 2.068215559033415, -50.371650578712348, 2.3117509434801295, 2.452718952376908, -0.4970257655144677, -0.0050652544632876668, 0.13286341438220117, 0.82347000849839913, -6.1713353167568194e-10, -2.0670149779263471e-10, -5.1346796972203758e-10, -6.8184213672019758e-06, 4.942449628225212e-06, 4.1816181314807841e-06
                0.57089814791768134, 2.0682159819554577, -50.714028047015027, 2.2527583245027318, 2.487328037037678, -0.40231100917469337, -0.0050981106515288876, 0.1276966374079859, 0.82140952834862246, -4.3563221417833132e-10, -1.4347488742058701e-09, -4.6878796245678226e-10, -2.0252339641969619e-06, 4.6912285720606653e-06, -1.59651761113489e-05
                ];
        end
    end
end
