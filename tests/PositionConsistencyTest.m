classdef PositionConsistencyTest < matlab.unittest.TestCase
    %POSITIONCONSISTENCYTEST 验证位置协方差记录、联合 NEES 与正负 3 sigma 绘图。

    properties (TestParameter)
        stateProfile = struct("ins9", "ins9", "ins15", "ins15");
        runCount = struct("single", 1, "multiple", 2);
    end

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture(projectRoot, IncludingSubfolders=true);
            testCase.applyFixture(fixture);
        end
    end

    methods (TestMethodSetup)
        function preserveSessionState(testCase)
            originalRng = rng;
            testCase.addTeardown(@() rng(originalRng));
            rng(20261008, "twister");
            originalVisibility = get(groot, "defaultFigureVisible");
            testCase.addTeardown(@() set(groot, "defaultFigureVisible", originalVisibility));
            set(groot, "defaultFigureVisible", "off");
        end
    end

    methods (Test)
        function correlatedCovarianceUsesEachRunBeforeAveraging(testCase)
            [~, meas, results] = PositionConsistencyTest.createResultCase(testCase);

            results.computeErrors(meas);
            errorData = results.Error.ESKF;
            numSamples = meas.getNumSamples();

            testCase.verifyEqual(errorData.PositionEnu(:, :, 1), ...
                repmat([2.0, 1.0, 3.0], numSamples, 1), AbsTol=1.0e-8);
            testCase.verifyEqual(errorData.PositionSigma(:, :, 2), ...
                repmat([4.0, 2.0*sqrt(2.0), 6.0], numSamples, 1), AbsTol=1.0e-12);
            % P 的东、北相关项使第一轮 NEES 为 15/7，而仅用对角项会得到 2.5。
            testCase.verifyEqual(errorData.PositionNees, ...
                repmat([15.0/7.0, 15.0/28.0], numSamples, 1), AbsTol=1.0e-8);
            % 两轮误差互为相反数；先平均误差会错误地得到零。
            testCase.verifyEqual(errorData.PositionMeanNees, ...
                repmat(75.0/56.0, numSamples, 1), AbsTol=1.0e-8);
            testCase.verifyTrue(all(isnan(errorData.VelocityEnu), "all"));
        end

        function singularAndMissingCovarianceLeaveNeesUnavailable(testCase)
            [~, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.Data.ESKF.PositionCovarianceEnu(:, :, 1, 1) = zeros(3);
            results.Data.ESKF.PositionCovarianceEnu(:, :, 2, 1) = nan(3);

            results.computeErrors(meas);

            testCase.verifyTrue(all(isnan(results.Error.ESKF.PositionNees(1:2, 1))));
            testCase.verifyEqual(results.Error.ESKF.PositionSigma(1, :, 1), zeros(1, 3), AbsTol=0.0);
            testCase.verifyTrue(all(isnan(results.Error.ESKF.PositionSigma(2, :, 1))));
            testCase.verifyEqual(results.Error.ESKF.PositionMeanNees(1:2), ...
                repmat(15.0/28.0, 2, 1), AbsTol=1.0e-8);
        end

        function neesPlotHasOnlyCurveAndTheoreticalExpectation(testCase, runCount)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase, runCount);
            results.computeErrors(meas);
            plotter = ResultPlotter(cfg, meas, results);

            plotFigure = plotter.plotPositionNees();
            testCase.addTeardown(@() delete(plotFigure));
            neesCurve = findobj(plotFigure, "Type", "line");
            expectationLine = findobj(plotFigure, "Type", "constantline");

            testCase.verifyNumElements(neesCurve, 1);
            testCase.verifyNumElements(expectationLine, 1);
            testCase.verifyEqual(expectationLine.Value, 3.0, AbsTol=0.0);
            testCase.verifyEqual(neesCurve.YData(:), results.Error.ESKF.PositionMeanNees, AbsTol=0.0);
            testCase.verifyEqual(neesCurve.XData(:), meas.getTime(), AbsTol=0.0);
        end

        function threeSigmaPlotUsesSelectedSignedRun(testCase)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.computeErrors(meas);
            plotter = ResultPlotter(cfg, meas, results);

            plotFigure = plotter.plotPositionError3Sigma(2);
            testCase.addTeardown(@() delete(plotFigure));
            eastAxes = PositionConsistencyTest.findEastAxes(plotFigure);
            errorCurve = findobj(eastAxes, "Type", "line", "DisplayName", "ESKF");
            upperCurve = findobj(eastAxes, "Type", "line", "DisplayName", "+3σ");
            lowerCurve = findobj(eastAxes, "Type", "line", "DisplayName", "-3σ");

            testCase.verifyNumElements(findobj(plotFigure, "Type", "axes"), 3);
            testCase.verifyEqual(errorCurve.YData(:), results.Error.ESKF.PositionEnu(:, 1, 2), AbsTol=0.0);
            testCase.verifyLessThan(errorCurve.YData, 0.0);
            testCase.verifyEqual(upperCurve.YData(:), repmat(12.0, meas.getNumSamples(), 1), AbsTol=0.0);
            testCase.verifyEqual(lowerCurve.YData(:), -upperCurve.YData(:), AbsTol=0.0);
        end

        function threeSigmaPlotDefaultsToFirstRun(testCase)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.computeErrors(meas);
            plotter = ResultPlotter(cfg, meas, results);

            plotFigure = plotter.plotPositionError3Sigma();
            testCase.addTeardown(@() delete(plotFigure));
            eastAxes = PositionConsistencyTest.findEastAxes(plotFigure);
            errorCurve = findobj(eastAxes, "Type", "line", "DisplayName", "ESKF");

            testCase.verifyEqual(errorCurve.YData(:), results.Error.ESKF.PositionEnu(:, 1, 1), AbsTol=0.0);
        end

        function invalidMonteCarloIndexIsRejected(testCase)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.computeErrors(meas);
            plotter = ResultPlotter(cfg, meas, results);

            testCase.verifyError(@() plotter.plotPositionError3Sigma(3), ...
                "ResultPlotter:InvalidMonteCarloIndex");
        end

        function legacyResultsSkipConsistencyPlots(testCase)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.Data.ESKF = rmfield(results.Data.ESKF, "PositionCovarianceEnu");
            results.computeErrors(meas);
            plotter = ResultPlotter(cfg, meas, results);

            testCase.verifyTrue(all(isnan(results.Error.ESKF.PositionNees), "all"));
            testCase.verifyWarning(@() plotter.plotPositionNees(), "ResultPlotter:MissingPositionCovariance");
            testCase.verifyWarning(@() plotter.plotPositionError3Sigma(), "ResultPlotter:MissingPositionCovariance");
        end

        function missingPositionTruthSkipsBothPlots(testCase)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase, 1, false);
            plotter = ResultPlotter(cfg, meas, results);

            testCase.verifyWarning(@() plotter.plotPositionNees(), "ResultPlotter:PositionTruthUnavailable");
            testCase.verifyWarning(@() plotter.plotPositionError3Sigma(), "ResultPlotter:PositionTruthUnavailable");
        end

        function eskfRecordsInitialAndUpdatedCovariance(testCase, stateProfile)
            [cfg, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            cfg.algorithm.stateModel.profile = stateProfile;

            results = ESKF(cfg, meas, results);
            results.computeErrors(meas);
            covariance = results.Data.ESKF.PositionCovarianceEnu;

            testCase.verifyEqual(size(covariance, [1, 2, 3, 4]), [3, 3, meas.getNumSamples(), 2]);
            testCase.verifyEqual(covariance(:, :, 1, 1), ...
                diag(cfg.algorithm.initialCovariance.positionStd.^2), AbsTol=0.0);
            testCase.verifyTrue(all(isfinite(covariance), "all"));
            testCase.verifyEqual(covariance, permute(covariance, [2, 1, 3, 4]), AbsTol=1.0e-15);
            testCase.verifyTrue(all(isfinite(results.Error.ESKF.PositionNees), "all"));
        end

        function delayedSamplesWithoutFilterAdvanceKeepMissingCovariance(testCase)
            [cfg, ~, results] = PositionConsistencyTest.createResultCase(testCase);
            cfg.sensorDelay.isEnabled = true;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();

            results = ESKF(cfg, meas, results);
            results.computeErrors(meas);
            covariance = results.Data.ESKF.PositionCovarianceEnu;

            testCase.verifyTrue(all(isnan(covariance(:, :, 2:3, :)), "all"));
            testCase.verifyTrue(all(isfinite(covariance(:, :, 4:end, :)), "all"));
            testCase.verifyTrue(all(isnan(results.Error.ESKF.PositionMeanNees(2:3))));
            testCase.verifyTrue(all(isnan(results.Error.ESKF.PositionSigma(2:3, :, :)), "all"));
        end

        function matExportIncludesCovarianceAndConsistency(testCase)
            [~, meas, results] = PositionConsistencyTest.createResultCase(testCase);
            results.computeErrors(meas);
            outputFixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            results.Cfg.result.outputFolder = outputFixture.Folder;

            results.saveToMat();
            saved = load(fullfile(outputFixture.Folder, results.Cfg.result.fileName));

            testCase.verifyEqual(saved.resultData.ESKF.PositionCovarianceEnu, ...
                results.Data.ESKF.PositionCovarianceEnu, AbsTol=0.0);
            testCase.verifyEqual(saved.errorData.ESKF.PositionMeanNees, ...
                results.Error.ESKF.PositionMeanNees, AbsTol=0.0);
        end
    end

    methods (Static, Access = private)
        function [cfg, meas, results] = createResultCase(testCase, runs, includePositionTruth)
            arguments
                testCase
                runs (1, 1) double = 2
                includePositionTruth (1, 1) logical = true
            end
            trajectoryCfg = setTrajectoryOptions();
            trajectoryCfg.Duration = 0.1;
            trajectoryCfg.DepthMotionType = "constant";
            trajectoryCfg.RollAmplitudeDeg = 0.0;
            trajectoryCfg.SaveToFile = false;
            inputData = genetraj(trajectoryCfg);
            if includePositionTruth
                inputData.truth = rmfield(inputData.truth, ["velocityEnu", "attitudeCbn", "attitudeEuler"]);
            else
                inputData = rmfield(inputData, "truth");
            end
            inputFile = string(tempname) + ".mat";
            save(inputFile, "inputData");
            testCase.addTeardown(@() delete(inputFile));

            cfg = setConfig();
            cfg.data.file = inputFile;
            cfg.data.isSensorNoiseMonteCarlo = false;
            cfg.algorithm.initialPerturbation.isEnabled = false;
            cfg.sim.duration = trajectoryCfg.Duration;
            cfg.sim.runs = runs;
            cfg.sensor.dvl.isEnabled = false;
            cfg.sensor.dvl.leverArmBody = zeros(3, 1);
            cfg.sensor.depth.isEnabled = false;
            cfg.sensor.gps.isEnabled = false;
            cfg.sensorDelay.isEnabled = false;
            meas = StateAndMeasurement(cfg);
            meas.loadInputData();
            meas.checkInputData();
            results = FilterResults(cfg, meas);
            results.registerAlgorithm("ESKF", runs, meas.getNumSamples());

            % 两轮误差相反，协方差尺度不同，检测先平均误差或错配 MC 的问题。
            covariance = [4.0, 1.0, 0.0; 1.0, 2.0, 0.0; 0.0, 0.0, 9.0];
            for mc = 1:runs
                errorEnu = (-1.0)^(mc-1) * [2.0; 1.0; 3.0];
                for sampleIndex = 1:meas.getNumSamples()
                    reference = inputData.initial.positionLlh;
                    if includePositionTruth
                        reference = inputData.truth.positionLlh(sampleIndex, :).';
                    end
                    results.Data.ESKF.PositionLlh(sampleIndex, :, mc) = ...
                        enuOffsetToLlh(reference, errorEnu).';
                    results.Data.ESKF.PositionCovarianceEnu(:, :, sampleIndex, mc) = mc^2 * covariance;
                end
            end
        end

        function axesHandle = findEastAxes(plotFigure)
            componentAxes = findobj(plotFigure, "Type", "axes");
            axesHandle = gobjects(0);
            for componentIndex = 1:numel(componentAxes)
                if string(componentAxes(componentIndex).YLabel.String) == "东向位置误差 (m)"
                    axesHandle = componentAxes(componentIndex);
                    return;
                end
            end
        end
    end
end
