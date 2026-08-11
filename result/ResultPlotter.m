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
            %PLOTTRAJECTORY 绘制经纬度水平轨迹。
            algorithmNames = fieldnames(obj.Results.Data);
            if isempty(algorithmNames)
                return;
            end

            figure("Name", "Trajectory");
            hold on;
            grid on;

            if obj.Meas.hasTruth()
                truth = obj.Meas.getTruthArrays();
                plot(rad2deg(truth.PositionLlh(:, 2)), rad2deg(truth.PositionLlh(:, 1)), ...
                    "k-", "LineWidth", 1.5, "DisplayName", "Truth");
            end

            for nameIndex = 1:numel(algorithmNames)
                name = algorithmNames{nameIndex};
                position = obj.Results.Data.(name).PositionLlh(:, :, 1);
                plot(rad2deg(position(:, 2)), rad2deg(position(:, 1)), ...
                    "LineWidth", 1.2, "DisplayName", name);
            end

            xlabel("Longitude (deg)");
            ylabel("Latitude (deg)");
            legend("Location", "best");
            title("Underwater Integrated Navigation Trajectory");
        end

        function plotPositionError(obj)
            %PLOTPOSITIONERROR 绘制 ENU 位置误差的 MC 平均 RMSE。
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
    end
end


