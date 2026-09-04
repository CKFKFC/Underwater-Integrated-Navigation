classdef AttitudeConventionTest < matlab.unittest.TestCase
    %ATTITUDECONVENTIONTEST 验证 RFU 载体系和 ENU 导航系姿态约定。

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectRoot = fileparts(fileparts(mfilename("fullpath")));
            fixture = matlab.unittest.fixtures.PathFixture( ...
                projectRoot, IncludingSubfolders=true);
            testCase.applyFixture(fixture);
        end
    end

    methods (Test)
        function zeroEulerAlignsRfuWithEnu(testCase)
            actualCbn = dcmFromEuler(zeros(3, 1));

            testCase.verifyEqual(actualCbn, eye(3), AbsTol=1.0e-15);
        end

        function positiveYawTurnsForwardFromNorthToEast(testCase)
            actualCbn = dcmFromEuler([0.0; 0.0; pi / 2.0]);
            actualForwardEnu = actualCbn(:, 2);

            testCase.verifyEqual(actualForwardEnu, [1.0; 0.0; 0.0], AbsTol=1.0e-15);
        end

        function positivePitchRaisesForwardAxis(testCase)
            pitch = deg2rad(10.0);
            actualCbn = dcmFromEuler([0.0; pitch; 0.0]);
            actualForwardEnu = actualCbn(:, 2);
            expectedForwardEnu = [0.0; cos(pitch); sin(pitch)];

            testCase.verifyEqual(actualForwardEnu, expectedForwardEnu, AbsTol=1.0e-15);
        end

        function positiveRollLowersRightAxis(testCase)
            roll = deg2rad(10.0);
            actualCbn = dcmFromEuler([roll; 0.0; 0.0]);
            actualRightEnu = actualCbn(:, 1);
            expectedRightEnu = [cos(roll); 0.0; -sin(roll)];

            testCase.verifyEqual(actualRightEnu, expectedRightEnu, AbsTol=1.0e-15);
        end

        function eulerAndDcmRoundTrip(testCase)
            expectedEuler = deg2rad([12.0; -7.0; 35.0]);

            actualEuler = eulerFromDcm(dcmFromEuler(expectedEuler));

            testCase.verifyEqual(actualEuler, expectedEuler, AbsTol=1.0e-14);
        end

        function trajectoryAttitudeUsesNorthZeroClockwiseHeading(testCase)
            cfg = setTrajectoryOptions();
            cfg.TrajectoryType = "straight";
            cfg.RollAmplitudeDeg = 0.0;
            time = [0.0; 1.0];
            eastVelocity = repmat([1.0, 0.0, 0.0], 2, 1);
            northVelocity = repmat([0.0, 1.0, 0.0], 2, 1);

            eastEuler = createAttitudeEuler(time, eastVelocity, cfg);
            northEuler = createAttitudeEuler(time, northVelocity, cfg);

            testCase.verifyEqual(eastEuler(:, 3), (pi / 2.0) * ones(2, 1), AbsTol=1.0e-15);
            testCase.verifyEqual(northEuler, zeros(2, 3), AbsTol=1.0e-15);
        end

        function straightCourseUsesSameHeadingConvention(testCase)
            cfg = setTrajectoryOptions();
            cfg.TrajectoryType = "straight";
            cfg.StraightSpeed = 2.0;
            time = [0.0; 1.0];

            cfg.CourseDeg = 0.0;
            [~, northVelocity] = generateHorizontalMotion(time, cfg);
            cfg.CourseDeg = 90.0;
            [~, eastVelocity] = generateHorizontalMotion(time, cfg);

            testCase.verifyEqual( ...
                northVelocity, repmat([0.0, 2.0], 2, 1), AbsTol=1.0e-15);
            testCase.verifyEqual( ...
                eastVelocity, repmat([2.0, 0.0], 2, 1), AbsTol=1.0e-15);
        end

        function generatedInputRecordsFrameMetadata(testCase)
            cfg = setTrajectoryOptions();
            cfg.Duration = 0.02;
            cfg.SaveToFile = false;

            inputData = genetraj(cfg);

            testCase.verifyEqual(inputData.metadata.bodyFrame, "RFU");
            testCase.verifyEqual(inputData.metadata.navigationFrame, "ENU");
            testCase.verifyEqual( ...
                inputData.metadata.bodyAxisOrder, ["right", "forward", "up"]);
        end

        function localAndPublicConversionsMatch(testCase)
            euler = deg2rad([-4.0; 6.0; 80.0]);

            publicCbn = dcmFromEuler(euler);
            localCbn = dcmFromEulerLocal(euler);

            testCase.verifyEqual(localCbn, publicCbn, AbsTol=1.0e-15);
        end
    end
end
