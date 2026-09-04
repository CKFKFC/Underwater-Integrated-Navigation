classdef ResultPlotter < handle
    %RESULTPLOTTER 绘制水下导航结果。
    % 作者: Kefan Chen
    % 日期: 2026-07-04
    % 功能: 绘制轨迹、位置误差和 RMSE。

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
end


