classdef ResultPlotter < handle
    %RESULTPLOTTER 显示水下导航结果。
    % 作者: Kefan Chen
    % 日期: 2026-07-04
    % 功能: 绘制轨迹、位置误差和 RMSE，打印位置统计。

    properties (Access = private)
        Cfg
        Meas
        Results
    end

    methods
        function obj = ResultPlotter(cfg, meas, results)
            %RESULTPLOTTER 构造结果绘图器。
            arguments
                cfg struct
                meas StateAndMeasurement
                results FilterResults
            end

            obj.Cfg = cfg;
            obj.Meas = meas;
            obj.Results = results;
        end

        function printPositionStatistics(obj)
            %PRINTPOSITIONSTATISTICS 打印各算法的位置平均 RMSE（m）和平均 NEES。
            %   PositionRmse 已在每个时刻跨 MC 求均方后开方；PositionMeanNees
            %   已在每个时刻跨 MC 求平均。这里再沿时间求算术平均，忽略 NaN。
            arguments
                obj ResultPlotter
            end
            algorithmNames = fieldnames(obj.Results.Error);
            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                errorData = obj.Results.Error.(name);
                if errorData.HasPositionTruth
                    averagePositionRmse = mean(errorData.PositionRmse, "omitnan");
                    averagePositionNees = mean(errorData.PositionMeanNees, "omitnan");
                    fprintf("%s：位置平均 RMSE = %.6f m，位置平均 NEES = %.6f\n", ...
                        name, averagePositionRmse, averagePositionNees);
                end
            end
        end

        function plotTrajectory(obj)
            %PLOTTRAJECTORY 绘制真实轨迹与算法轨迹的二维 ENU 水平投影。
            algorithmNames = fieldnames(obj.Results.Data);
            if isempty(algorithmNames)
                return;
            end

            if obj.Meas.hasPositionTruth()
                truth = obj.Meas.getTruthArrays();
                referenceLlh = truth.PositionLlh(1, :).';
            else
                firstResult = obj.Results.Data.(algorithmNames{1});
                referenceLlh = firstResult.PositionLlh(1, :, 1).';
            end

            figure("Name", "Horizontal Trajectory");
            hold on;
            grid on;

            if obj.Meas.hasPositionTruth()
                truthPositionEnu = llh2enuError(truth.PositionLlh, referenceLlh);
                plot(truthPositionEnu(:, 1), truthPositionEnu(:, 2), ...
                    "k-", "LineWidth", 1.5, "DisplayName", "Truth");
            end

            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                position = obj.Results.Data.(name).PositionLlh(:, :, 1);
                positionEnu = llh2enuError(position, referenceLlh);
                plot(positionEnu(:, 1), positionEnu(:, 2), ...
                    "LineWidth", 1.2, "DisplayName", name);
            end

            axis equal;
            xlabel("东向位置 (m)");
            ylabel("北向位置 (m)");
            legend("Location", "best");
            title("真实轨迹与算法轨迹二维对比");
        end

        function plotAttitude(obj)
            %PLOTATTITUDE 绘制第1次运行的滚转、俯仰和航向欧拉角，单位 deg。
            % 沿用 eulerFromDcm 约定：航向北零顺时针，抬头和右倾为正。
            arguments
                obj ResultPlotter
            end
            obj.plotNavigationComponents("Euler", ["滚转角", "俯仰角", "航向角"], ...
                "deg", "Estimated Euler Attitude");
        end

        function plotVelocity(obj)
            %PLOTVELOCITY 绘制第1次运行的 ENU 速度，单位 m/s。
            arguments
                obj ResultPlotter
            end
            obj.plotNavigationComponents("VelocityEnu", ["东向速度", "北向速度", "天向速度"], ...
                "m/s", "Estimated ENU Velocity");
        end

        function plotPositionError(obj)
            %PLOTPOSITIONERROR 绘制 ENU 位置误差的 MC 平均 RMSE。
            if ~obj.Meas.hasPositionTruth()
                return;
            end
            algorithmNames = fieldnames(obj.Results.Error);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            figure("Name", "Position Error RMSE");
            tiledlayout(3, 1);
            componentNames = ["东向位置误差", "北向位置误差", "天向位置误差"];

            for componentIndex = 1:3
                nexttile;
                hold on;
                grid on;
                for nameIndex = 1:numel(algorithmNames)
                    name = algorithmNames{nameIndex};
                    errorData = obj.Results.Error.(name).PositionComponentRmse(:, componentIndex);
                    plot(time, errorData, "LineWidth", 1.1, "DisplayName", name);
                end
                ylabel(componentNames(componentIndex) + " (m)");
                if componentIndex == 1
                    title("组合导航位置误差 RMSE");
                end
                if componentIndex == 3
                    xlabel("时间 (s)");
                end
                legend("Location", "best");
            end
        end

        function plotVelocityError(obj)
            %PLOTVELOCITYERROR 绘制 ENU 速度误差的 MC 平均 RMSE。
            if ~obj.Meas.hasVelocityTruth()
                return;
            end
            algorithmNames = fieldnames(obj.Results.Error);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            figure("Name", "Velocity Error RMSE");
            tiledlayout(3, 1);
            componentNames = ["东向速度误差", "北向速度误差", "天向速度误差"];

            for componentIndex = 1:3
                nexttile;
                hold on;
                grid on;
                for nameIndex = 1:numel(algorithmNames)
                    name = algorithmNames{nameIndex};
                    errorData = obj.Results.Error.(name).VelocityComponentRmse(:, componentIndex);
                    plot(time, errorData, "LineWidth", 1.1, "DisplayName", name);
                end
                ylabel(componentNames(componentIndex) + " (m/s)");
                if componentIndex == 1
                    title("组合导航速度误差 RMSE");
                end
                if componentIndex == 3
                    xlabel("时间 (s)");
                end
                legend("Location", "best");
            end
        end

        function plotFigure = plotPositionNees(obj)
            %PLOTPOSITIONNEES 绘制 ESKF 三维位置 NEES 和理论期望值 3。
            %   多次运行时绘制逐次 NEES 的 MC 平均，不对误差或协方差先求平均。
            arguments
                obj ResultPlotter
            end
            plotFigure = gobjects(0);
            errorData = obj.getEskfPositionErrors();
            if isempty(fieldnames(errorData))
                return;
            end
            if ~any(isfinite(errorData.PositionMeanNees))
                warning("ResultPlotter:PositionNeesUnavailable", ...
                    "No valid position NEES is available. Check position errors and positive definite covariance.");
                return;
            end

            curveName = "ESKF";
            if size(errorData.PositionNees, 2) > 1
                curveName = "ESKF (MC平均)";
            end
            time = obj.Meas.getTime();
            positionDimension = size(errorData.PositionEnu, 2);
            plotFigure = figure(Name="ESKF Position NEES");
            axesHandle = axes(plotFigure);
            hold(axesHandle, "on");
            grid(axesHandle, "on");
            plot(axesHandle, time, errorData.PositionMeanNees, LineWidth=1.2, DisplayName=curveName);
            yline(axesHandle, positionDimension, "k--", LineWidth=1.2, ...
                DisplayName="理论期望值 = " + string(positionDimension));
            xlabel(axesHandle, "时间 (s)");
            ylabel(axesHandle, "位置 NEES");
            title(axesHandle, "ESKF 三维位置联合 NEES");
            legend(axesHandle, "Location", "best", "Interpreter", "none");
        end

        function plotFigure = plotPositionError3Sigma(obj, mcIndex)
            %PLOTPOSITIONERROR3SIGMA 绘制指定运行的 ENU 位置误差与对应的正负 3 sigma。
            %   默认绘制第 1 次运行；mcIndex 可选择其他 Monte Carlo 运行。
            arguments
                obj ResultPlotter
                mcIndex (1, 1) double {mustBeFinite, mustBeInteger, mustBePositive} = 1
            end
            plotFigure = gobjects(0);
            errorData = obj.getEskfPositionErrors();
            if isempty(fieldnames(errorData))
                return;
            end
            runs = size(errorData.PositionEnu, 3);
            if mcIndex > runs
                error("ResultPlotter:InvalidMonteCarloIndex", ...
                    "mcIndex must be between 1 and %d, the number of stored ESKF runs.", runs);
            end

            positionError = errorData.PositionEnu(:, :, mcIndex);
            threeSigma = 3.0 * errorData.PositionSigma(:, :, mcIndex);
            if ~any(isfinite(positionError) & isfinite(threeSigma), "all")
                warning("ResultPlotter:PositionSigmaUnavailable", ...
                    "No matching position errors and standard deviations are available for run %d.", mcIndex);
                return;
            end

            time = obj.Meas.getTime();
            componentNames = ["东向", "北向", "天向"];
            plotFigure = figure(Name="ESKF Position Error and 3 Sigma");
            layout = tiledlayout(plotFigure, 3, 1);
            componentAxes = gobjects(3, 1);
            for componentIndex = 1:3
                componentAxes(componentIndex) = nexttile(layout);
                axesHandle = componentAxes(componentIndex);
                hold(axesHandle, "on");
                grid(axesHandle, "on");
                plot(axesHandle, time, positionError(:, componentIndex), ...
                    "b-", LineWidth=1.1, DisplayName="ESKF");
                plot(axesHandle, time, threeSigma(:, componentIndex), ...
                    "r--", LineWidth=1.1, DisplayName="+3σ");
                plot(axesHandle, time, -threeSigma(:, componentIndex), ...
                    "r--", LineWidth=1.1, DisplayName="-3σ");
                ylabel(axesHandle, componentNames(componentIndex) + "位置误差 (m)");
                legend(axesHandle, "Location", "best", "Interpreter", "none");
            end
            xlabel(componentAxes(3), "时间 (s)");
            linkaxes(componentAxes, "x");
            title(layout, "ESKF 位置误差与 ±3σ（第 " + string(mcIndex) + " 次运行）");
        end

        function plotAttitudeError(obj)
            %PLOTATTITUDEERROR 绘制姿态小失准角误差的 MC 平均 RMSE。
            if ~obj.Meas.hasAttitudeTruth()
                return;
            end
            algorithmNames = fieldnames(obj.Results.Error);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            figure("Name", "Attitude Error RMSE");
            tiledlayout(3, 1);
            componentNames = ["滚转失准角", "俯仰失准角", "航向失准角"];

            for componentIndex = 1:3
                nexttile;
                hold on;
                grid on;
                for nameIndex = 1:numel(algorithmNames)
                    name = algorithmNames{nameIndex};
                    errorData = rad2deg(obj.Results.Error.(name).AttitudeComponentRmse(:, componentIndex));
                    plot(time, errorData, "LineWidth", 1.1, "DisplayName", name);
                end
                ylabel(componentNames(componentIndex) + " (deg)");
                if componentIndex == 1
                    title("组合导航姿态误差 RMSE");
                end
                if componentIndex == 3
                    xlabel("时间 (s)");
                end
                legend("Location", "best");
            end
        end

        function plotRmse(obj)
            %PLOTRMSE 绘制位置 RMSE。
            if ~obj.Meas.hasPositionTruth()
                return;
            end
            algorithmNames = fieldnames(obj.Results.Error);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            figure("Name", "Position RMSE");
            hold on;
            grid on;

            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                plot(time, obj.Results.Error.(name).PositionRmse, ...
                    "LineWidth", 1.2, "DisplayName", name);
            end

            xlabel("Time (s)");
            ylabel("RMSE (m)");
            title("Position RMSE");
            legend("Location", "best");
        end

        function plotPositionComponents(obj)
            %PLOTPOSITIONCOMPONENTS 分别绘制算法和真值的 ENU 位置分量。
            if ~obj.Meas.hasPositionTruth()
                warning("ResultPlotter:PositionTruthUnavailable", ...
                    "Position truth is unavailable. ENU position component plots are not created.");
                return;
            end

            algorithmNames = fieldnames(obj.Results.Data);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            truth = obj.Meas.getTruthArrays();
            referenceLlh = truth.PositionLlh(1, :).';
            truthPositionEnu = llh2enuError(truth.PositionLlh, referenceLlh);

            estimatedPositionEnu = struct();
            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                positionLlh = obj.Results.Data.(name).PositionLlh(:, :, 1);
                estimatedPositionEnu.(name) = llh2enuError(positionLlh, referenceLlh);
            end

            componentNames = ["东向", "北向", "天向"];
            figure("Name", "Estimated ENU Position");
            tiledlayout(3, 1);
            for componentIndex = 1:3
                nexttile;
                hold on;
                grid on;
                for nameIndex = 1:numel(algorithmNames)
                    name = algorithmNames{nameIndex};
                    plot(time, estimatedPositionEnu.(name)(:, componentIndex), ...
                        "LineWidth", 1.1, "DisplayName", name);
                end
                ylabel(componentNames(componentIndex) + "位置 (m)");
                title(componentNames(componentIndex) + "位置");
                legend("Location", "best");
                if componentIndex == 3
                    xlabel("时间 (s)");
                end
            end
            sgtitle("算法解算 ENU 位置（第1次 Monte Carlo）");

            figure("Name", "Truth ENU Position");
            tiledlayout(3, 1);
            for componentIndex = 1:3
                nexttile;
                plot(time, truthPositionEnu(:, componentIndex), ...
                    "k-", "LineWidth", 1.1);
                grid on;
                ylabel(componentNames(componentIndex) + "位置 (m)");
                title(componentNames(componentIndex) + "位置");
                if componentIndex == 3
                    xlabel("时间 (s)");
                end
            end
            sgtitle("真实 ENU 位置");
        end
    end

    methods (Access = private)
        function errorData = getEskfPositionErrors(obj)
            %GETESKFPOSITIONERRORS 检查 ESKF 位置误差与一致性指标是否可用。
            errorData = struct();
            if ~isfield(obj.Results.Data, "ESKF")
                return;
            end
            if ~obj.Meas.hasPositionTruth()
                warning("ResultPlotter:PositionTruthUnavailable", ...
                    "Position truth is unavailable. ESKF position consistency plots are not created.");
                return;
            end
            if ~isfield(obj.Results.Data.ESKF, "PositionCovarianceEnu")
                warning("ResultPlotter:MissingPositionCovariance", ...
                    "ESKF position covariance history is missing. Rerun ESKF to create consistency plots.");
                return;
            end
            requiredFields = ["PositionEnu", "PositionSigma", "PositionNees", "PositionMeanNees"];
            if ~isfield(obj.Results.Error, "ESKF") ...
                    || ~all(isfield(obj.Results.Error.ESKF, requiredFields))
                warning("ResultPlotter:PositionConsistencyUnavailable", ...
                    "Run results.computeErrors(meas) before plotting ESKF position consistency.");
                return;
            end
            errorData = obj.Results.Error.ESKF;
        end

        function plotNavigationComponents(obj, fieldName, componentNames, unit, figureName)
            %PLOTNAVIGATIONCOMPONENTS 绘制解算值，不依赖真值是否存在。
            algorithmNames = fieldnames(obj.Results.Data);
            if isempty(algorithmNames)
                return;
            end

            time = obj.Meas.getTime();
            plotFigure = figure(Name=figureName);
            layout = tiledlayout(plotFigure, 3, 1);
            componentAxes = gobjects(3, 1);
            for componentIndex = 1:3
                componentAxes(componentIndex) = nexttile(layout);
                axesHandle = componentAxes(componentIndex);
                hold(axesHandle, "on");
                grid(axesHandle, "on");
                for nameIndex = 1:numel(algorithmNames)
                    name = algorithmNames{nameIndex};
                    values = obj.Results.Data.(name).(fieldName)(:, componentIndex, 1);
                    if fieldName == "Euler"
                        values = rad2deg(values);
                    end
                    plot(axesHandle, time, values, LineWidth=1.1, DisplayName=name);
                end
                ylabel(axesHandle, componentNames(componentIndex) + " (" + unit + ")");
                title(axesHandle, componentNames(componentIndex));
                legend(axesHandle, "Location", "best", "Interpreter", "none");
            end
            xlabel(componentAxes(3), "时间 (s)");
            linkaxes(componentAxes, "x");
            if fieldName == "Euler"
                title(layout, "算法解算欧拉角（第1次运行）");
            else
                title(layout, "算法解算 ENU 速度（第1次运行）");
            end
        end
    end
end


