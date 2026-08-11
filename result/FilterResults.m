classdef FilterResults < handle
    %FILTERRESULTS 保存 ESKF 输出并计算导航误差。
    % 作者: Kefan Chen
    % 日期: 2026-07-04
    % 功能: 管理结果数组、误差统计和 MAT 文件导出。

    properties
        Cfg
        Meas
        Data
        Error
    end

    methods
        function obj = FilterResults(cfg, meas)
            %FILTERRESULTS 构造结果管理器。
            arguments
                cfg struct
                meas StateAndMeasurement
            end

            obj.Cfg = cfg;
            obj.Meas = meas;
            obj.Data = struct();
            obj.Error = struct();
        end

        function registerAlgorithm(obj, algorithmName, runs, numSamples, stateModelMetadata)
            %REGISTERALGORITHM 为算法结果预分配数组。
            if nargin < 5
                stateModelMetadata = struct();
            end

            % 状态模型说明与导航数组保存在同一算法结果下，但不参与误差计算。
            % 即使未来不同算法使用不同维数，也能从各自结果中恢复其状态定义。
            name = char(algorithmName);
            result = struct();
            result.StateModel = stateModelMetadata;
            result.PositionLlh = nan(numSamples, 3, runs);
            result.VelocityEnu = nan(numSamples, 3, runs);
            result.Euler = nan(numSamples, 3, runs);
            result.GyroBias = nan(numSamples, 3, runs);
            result.AccelBias = nan(numSamples, 3, runs);
            obj.Data.(name) = result;
        end

        function storeNavigation(obj, algorithmName, mc, sampleIndex, navSol, imuBias)
            %STORENAVIGATION 保存一个导航解算结果。
            if nargin < 6
                imuBias = struct();
                imuBias.GyroBias = zeros(3, 1);
                imuBias.AccelBias = zeros(3, 1);
            end

            name = char(algorithmName);
            obj.Data.(name).PositionLlh(sampleIndex, :, mc) = navSol.PositionLlh(:).';
            obj.Data.(name).VelocityEnu(sampleIndex, :, mc) = navSol.VelocityEnu(:).';
            obj.Data.(name).Euler(sampleIndex, :, mc) = eulerFromDcm(navSol.Cbn).';
            obj.Data.(name).GyroBias(sampleIndex, :, mc) = imuBias.GyroBias(:).';
            obj.Data.(name).AccelBias(sampleIndex, :, mc) = imuBias.AccelBias(:).';
        end

        function computeErrors(obj, meas)
            %COMPUTEERRORS 在有真值时计算 ENU 误差和 RMSE。
            if ~meas.hasTruth()
                warning("FilterResults:TruthUnavailable", ...
                    "Truth trajectory is unavailable. Error and RMSE are not computed.");
                return;
            end

            truth = meas.getTruthArrays();
            algorithmNames = fieldnames(obj.Data);
            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                result = obj.Data.(name);
                [numSamples, ~, runs] = size(result.PositionLlh);

                positionError = nan(numSamples, 3, runs);
                velocityError = nan(numSamples, 3, runs);
                attitudeError = nan(numSamples, 3, runs);

                for mc = 1:runs
                    for sampleIndex = 1:numSamples
                        estimatePosition = result.PositionLlh(sampleIndex, :, mc).';
                        truthPosition = truth.PositionLlh(sampleIndex, :).';
                        positionError(sampleIndex, :, mc) = llh2enuError(estimatePosition, truthPosition).';

                        estimateVelocity = result.VelocityEnu(sampleIndex, :, mc).';
                        truthVelocity = truth.VelocityEnu(sampleIndex, :).';
                        velocityError(sampleIndex, :, mc) = (estimateVelocity - truthVelocity).';

                        estimateCbn = dcmFromEuler(result.Euler(sampleIndex, :, mc).');
                        truthCbn = truth.AttitudeCbn(:, :, sampleIndex);
                        attitudeError(sampleIndex, :, mc) = obj.computeAttitudeMisalignment( ...
                            estimateCbn, truthCbn).';
                    end
                end

                errorData = struct();
                errorData.PositionEnu = positionError;
                errorData.VelocityEnu = velocityError;
                errorData.AttitudeMisalignment = attitudeError;
                errorData.AttitudeEuler = attitudeError;
                errorData.PositionComponentRmse = obj.computeComponentRmse(positionError);
                errorData.VelocityComponentRmse = obj.computeComponentRmse(velocityError);
                errorData.AttitudeComponentRmse = obj.computeComponentRmse(attitudeError);
                errorData.PositionRmse = sqrt(mean(sum(positionError.^2, 2), 3, "omitnan"));
                errorData.VelocityRmse = sqrt(mean(sum(velocityError.^2, 2), 3, "omitnan"));
                errorData.AttitudeRmse = sqrt(mean(sum(attitudeError.^2, 2), 3, "omitnan"));
                obj.Error.(name) = errorData;
            end
        end

        function saveToMat(obj)
            %SAVETOMAT 保存结果和误差到 data/output。
            outputFolder = obj.Cfg.result.outputFolder;
            if ~isfolder(outputFolder)
                mkdir(outputFolder);
            end

            resultData = obj.Data;
            errorData = obj.Error;
            cfg = obj.Cfg;
            outputFile = fullfile(outputFolder, obj.Cfg.result.fileName);
            save(outputFile, "resultData", "errorData", "cfg");
        end
    end

    methods (Access = private)
        function attitudeError = computeAttitudeMisalignment(~, estimateCbn, truthCbn)
            %COMPUTEATTITUDEMISALIGNMENT 计算估计姿态相对真值姿态的小失准角。
            relativeDcm = estimateCbn * truthCbn.';
            attitudeError = FilterResults.rotationVectorFromDcm(relativeDcm);
        end

        function componentRmse = computeComponentRmse(~, errorData)
            %COMPUTECOMPONENTRMSE 沿 MC 维度计算每个分量的 RMSE。
            componentRmse = sqrt(mean(errorData.^2, 3, "omitnan"));
        end
    end

    methods (Static, Access = private)
        function rotationVector = rotationVectorFromDcm(rotationMatrix)
            %ROTATIONVECTORFROMDCM 从旋转矩阵提取旋转矢量。
            traceArgument = 0.5 * (trace(rotationMatrix) - 1.0);
            traceArgument = min(max(traceArgument, -1.0), 1.0);
            angle = acos(traceArgument);
            axisTimesSine = 0.5 * [
                rotationMatrix(3, 2) - rotationMatrix(2, 3)
                rotationMatrix(1, 3) - rotationMatrix(3, 1)
                rotationMatrix(2, 1) - rotationMatrix(1, 2)
                ];

            if angle > 1.0e-8
                rotationVector = axisTimesSine * angle / sin(angle);
            else
                rotationVector = axisTimesSine;
            end
        end
    end
end






