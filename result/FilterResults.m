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
        %% 构造结果管理器
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

        %% 注册算法并预分配结果数组
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
            result.PositionCovarianceEnu = nan(3, 3, numSamples, runs);
            obj.Data.(name) = result;
        end

        %% 保存单个时刻的导航结果
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

        %% 计算各算法的导航误差和 RMSE
        function computeErrors(obj, meas)
            %COMPUTEERRORS 按实际提供的参考量分别计算 ENU 误差和 RMSE。
            if ~meas.hasTruth()
                warning("FilterResults:TruthUnavailable", ...
                    "Reference trajectory is unavailable. Error and RMSE are not computed.");
                return;
            end

            truth = meas.getTruthArrays();
            hasPositionTruth = meas.hasPositionTruth();
            hasVelocityTruth = meas.hasVelocityTruth();
            hasAttitudeTruth = meas.hasAttitudeTruth();
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
                        if hasPositionTruth
                            estimatePosition = result.PositionLlh(sampleIndex, :, mc).';
                            truthPosition = truth.PositionLlh(sampleIndex, :).';
                            positionError(sampleIndex, :, mc) = ...
                                llh2enuError(estimatePosition, truthPosition).';
                        end

                        if hasVelocityTruth
                            estimateVelocity = result.VelocityEnu(sampleIndex, :, mc).';
                            truthVelocity = truth.VelocityEnu(sampleIndex, :).';
                            velocityError(sampleIndex, :, mc) = ...
                                (estimateVelocity - truthVelocity).';
                        end

                        if hasAttitudeTruth
                            estimateCbn = dcmFromEuler( ...
                                result.Euler(sampleIndex, :, mc).');
                            truthCbn = truth.AttitudeCbn(:, :, sampleIndex);
                            attitudeError(sampleIndex, :, mc) = ...
                                obj.computeAttitudeMisalignment(estimateCbn, truthCbn).';
                        end
                    end
                end

                errorData = struct();
                errorData.HasPositionTruth = hasPositionTruth;
                errorData.HasVelocityTruth = hasVelocityTruth;
                errorData.HasAttitudeTruth = hasAttitudeTruth;
                errorData.PositionEnu = positionError;
                errorData.VelocityEnu = velocityError;
                errorData.AttitudeMisalignment = attitudeError;
                errorData.AttitudeEuler = attitudeError;
                errorData.PositionComponentRmse = obj.computeComponentRmse(positionError);
                errorData.VelocityComponentRmse = obj.computeComponentRmse(velocityError);
                errorData.AttitudeComponentRmse = obj.computeComponentRmse(attitudeError);
                errorData.PositionRmse = sqrt(mean(sum(positionError.^2, 2), 3, "omitnan"));
                [errorData.PositionSigma, errorData.PositionNees] = ...
                    obj.computePositionConsistency(result, positionError);
                errorData.PositionMeanNees = mean(errorData.PositionNees, 2, "omitnan");
                errorData.VelocityRmse = sqrt(mean(sum(velocityError.^2, 2), 3, "omitnan"));
                errorData.AttitudeRmse = sqrt(mean(sum(attitudeError.^2, 2), 3, "omitnan"));
                obj.Error.(name) = errorData;
            end
        end

        %% 将结果、误差和配置保存到 MAT 文件
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
        %% 计算位置标准差与每次运行的三维联合 NEES
        function [positionSigma, positionNees] = computePositionConsistency(~, result, positionError)
            %COMPUTEPOSITIONCONSISTENCY 使用同一时刻、同一次运行的 ENU 位置协方差。
            %   Sigma 由非负对角方差计算；NEES 使用完整的正定 3-by-3 协方差。
            %   缺失、非有限或非正定协方差对应的 NEES 保留为 NaN。
            [numSamples, ~, runs] = size(positionError);
            positionSigma = nan(numSamples, 3, runs);
            positionNees = nan(numSamples, runs);
            if ~isfield(result, "PositionCovarianceEnu")
                return;
            end

            covarianceHistory = result.PositionCovarianceEnu;
            if ~isnumeric(covarianceHistory) || ~isreal(covarianceHistory) ...
                    || ndims(covarianceHistory) > 4 ...
                    || ~isequal(size(covarianceHistory, [1, 2, 3, 4]), [3, 3, numSamples, runs])
                error("FilterResults:InvalidPositionCovariance", ...
                    "PositionCovarianceEnu must be a real 3-by-3-by-N-by-runs numeric array.");
            end

            for mc = 1:runs
                for sampleIndex = 1:numSamples
                    covariance = covarianceHistory(:, :, sampleIndex, mc);
                    if any(~isfinite(covariance), "all")
                        continue;
                    end
                    covariance = 0.5 * (covariance + covariance.');
                    variance = diag(covariance);
                    if any(variance < 0.0)
                        continue;
                    end
                    positionSigma(sampleIndex, :, mc) = sqrt(variance).';

                    errorEnu = positionError(sampleIndex, :, mc).';
                    if any(~isfinite(errorEnu))
                        continue;
                    end
                    [lowerFactor, status] = chol(covariance, "lower");
                    if status == 0
                        % e'*(P\e) = ||L\e||^2，保留轴间相关性且不显式求逆。
                        normalizedError = lowerFactor\errorEnu;
                        positionNees(sampleIndex, mc) = sum(normalizedError.^2);
                    end
                end
            end
        end

        %% 计算估计姿态相对真值的失准角
        function attitudeError = computeAttitudeMisalignment(~, estimateCbn, truthCbn)
            %COMPUTEATTITUDEMISALIGNMENT 计算估计姿态相对真值姿态的小失准角。
            relativeDcm = estimateCbn * truthCbn.';
            attitudeError = FilterResults.rotationVectorFromDcm(relativeDcm);
        end

        %% 沿 Monte Carlo 维度计算分量 RMSE
        function componentRmse = computeComponentRmse(~, errorData)
            %COMPUTECOMPONENTRMSE 沿 MC 维度计算每个分量的 RMSE。
            componentRmse = sqrt(mean(errorData.^2, 3, "omitnan"));
        end
    end

    methods (Static, Access = private)
        %% 从方向余弦矩阵提取旋转矢量
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






