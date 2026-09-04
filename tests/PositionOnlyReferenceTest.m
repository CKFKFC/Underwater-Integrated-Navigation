classdef PositionOnlyReferenceTest < matlab.unittest.TestCase
    %POSITIONONLYREFERENCETEST 验证只有 RTK 位置参考时仍可计算位置误差。

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, IncludingSubfolders=true);
            testCase.applyFixture(fixture);
        end
    end

    methods (Test)
        function computesPositionErrorWithoutVelocityOrAttitudeTruth(testCase)
            trajectoryCfg = setTrajectoryOptions();
            trajectoryCfg.Duration = 0.02;
            trajectoryCfg.DepthMotionType = "constant";
            trajectoryCfg.RollAmplitudeDeg = 0.0;
            trajectoryCfg.SaveToFile = false;
            inputData = genetraj(trajectoryCfg);
            inputData.truth = rmfield(inputData.truth, ...
                ["velocityEnu", "attitudeCbn", "attitudeEuler"]);
            inputFile = string(tempname) + ".mat";
            save(inputFile, "inputData");
            testCase.addTeardown(@() delete(inputFile));

            cfg = setConfig();
            cfg.data.file = inputFile;
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.algorithm.initialPerturbation.isEnabled = false;
            cfg.sim.duration = 0.02;
            cfg.sensor.dvl.isEnabled = false;
            cfg.sensor.depth.isEnabled = false;
            cfg.sensor.gps.isEnabled = false;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            results = FilterResults(cfg, meas);
            results = ESKF(cfg, meas, results);

            results.computeErrors(meas);

            testCase.verifyTrue(meas.hasTruth());
            testCase.verifyTrue(meas.hasPositionTruth());
            testCase.verifyFalse(meas.hasVelocityTruth());
            testCase.verifyFalse(meas.hasAttitudeTruth());
            testCase.verifyTrue(all(isfinite(results.Error.ESKF.PositionEnu), "all"));
            testCase.verifyTrue(all(isnan(results.Error.ESKF.VelocityEnu), "all"));
            testCase.verifyTrue( ...
                all(isnan(results.Error.ESKF.AttitudeMisalignment), "all"));
        end
    end
end
