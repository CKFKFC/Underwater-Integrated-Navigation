classdef StateAndMeasurement < handle
    %STATEANDMEASUREMENT 管理输入数据和 MC 量测。
    % 作者: Kefan Chen
    % 日期: 2026-07-04
    % 功能: 读取导航输入数据，并按传感器生成 MC 量测。

    properties (SetAccess = private)
        Cfg
        % GtMeas 对应旧二维代码中的 gt_mes 主数据容器。
        GtMeas
        % CurMeas 保存当前 MC 的带噪量测。
        CurMeas
        Time
    end

    methods
        %% constructor
        function obj = StateAndMeasurement(cfg)
            arguments
                cfg struct
            end

            obj.Cfg = cfg;
            obj.GtMeas = struct();
            obj.CurMeas = struct();
            obj.Time = [];
        end

        %% load input data
        function loadInputData(obj)
            % 输入文件必须由外部提供，算法内不生成轨迹。
            inputFile = obj.Cfg.data.file;
            if ~isfile(inputFile)
                error("StateAndMeasurement:InputFileNotFound", ...
                    "Input data file was not found: %s", inputFile);
            end

            loadedData = load(inputFile);
            inputData = obj.selectInputStruct(loadedData);
            obj.GtMeas = obj.loadAllData(inputData);
            obj.GtMeas = obj.applySimulationDurationLimit(obj.GtMeas);
            obj.Time = obj.GtMeas.Time;
        end

        %% check input data
        function checkInputData(obj)
            % 主时间轴和 IMU 是惯导递推的必需输入。
            obj.checkFrameConfig();
            obj.checkInputFrameMetadata();
            obj.checkTimeData();
            obj.checkImuData();
            obj.checkInitialData();
            obj.checkTruthData();
            obj.checkSensorAvailabilityConfig();
            obj.checkSensorDelayConfig();
            obj.checkEnabledMeasurements();
        end

        %% check ideal measurements
        function generateIdealMeasurements(obj)
            % 本项目不在量测类中生成轨迹，只检查输入数据。
            obj.checkInputData();
        end

        %% prepare Monte Carlo data
        function prepareMonteCarloRun(obj, mc)
            arguments
                obj
                mc (1, 1) double {mustBePositive, mustBeInteger}
            end

            % 实测模式直接使用原始量测；仿真模式每次 MC 重新加噪。
            obj.CurMeas = struct();
            obj.CurMeas.MonteCarloIndex = mc;
            obj.CurMeas.Imu = obj.generateNoisyImu();
            obj.CurMeas.Dvl = obj.generateNoisyDvl();
            obj.CurMeas.Depth = obj.generateNoisyDepth();
            obj.CurMeas.Gps = obj.generateNoisyGps();
            obj.CurMeas.Index = obj.buildMeasurementIndexMap();
        end

        %% get number of samples
        function numSamples = getNumSamples(obj)
            numSamples = numel(obj.Time);
        end

        %% get time vector
        function time = getTime(obj)
            time = obj.Time;
        end

        %% get initial navigation solution
        function navSol = getInitialNavigation(obj)
            % 优先读取 initial；缺省时使用真值第一帧。
            initial = obj.GtMeas.Initial;
            truth = obj.GtMeas.Truth;

            positionLlh = initial.PositionLlh;
            if isempty(positionLlh) && ~isempty(truth.PositionLlh)
                positionLlh = truth.PositionLlh(1, :).';
            end

            velocityEnu = initial.VelocityEnu;
            if isempty(velocityEnu) && ~isempty(truth.VelocityEnu)
                velocityEnu = truth.VelocityEnu(1, :).';
            end

            Cbn = initial.Cbn;
            if isempty(Cbn) && ~isempty(truth.AttitudeCbn)
                Cbn = truth.AttitudeCbn(:, :, 1);
            end

            if isempty(positionLlh) || isempty(velocityEnu) || isempty(Cbn)
                error("StateAndMeasurement:IncompleteInitialNavigation", ...
                    "Initial navigation solution requires positionLlh, velocityEnu, and attitude.");
            end

            navSol = struct();
            navSol.PositionLlh = obj.firstVector(positionLlh);
            navSol.VelocityEnu = obj.firstVector(velocityEnu);
            navSol.Cbn = Cbn(:, :, 1);
        end

        %% get initial IMU bias estimate
        function imuBias = getInitialImuBias(obj)
            % 初始 IMU 零偏估计独立于导航解，缺省为输入文件中的零值。
            initial = obj.GtMeas.Initial;

            imuBias = struct();
            imuBias.GyroBias = obj.firstVector(initial.GyroBias);
            imuBias.AccelBias = obj.firstVector(initial.AccelBias);
        end

        %% get IMU data
        function imu = getImu(obj, index)
            imu = obj.emptyMeasurement();
            sampleIndex = obj.CurMeas.Index.Imu(index);
            if sampleIndex <= 0
                return;
            end

            imu.Time = obj.CurMeas.Imu.Time(sampleIndex);
            imu.SampleTime = imu.Time;
            imu.ArrivalTime = obj.Time(index);
            imu.DelaySteps = obj.getDelaySteps("imu");
            imu.DelaySeconds = imu.DelaySteps * obj.Cfg.sim.dt;
            imu.Gyro = obj.CurMeas.Imu.Gyro(sampleIndex, :).';
            imu.Accel = obj.CurMeas.Imu.Accel(sampleIndex, :).';
            imu.Valid = true;
        end

        %% get DVL measurement
        function measurement = getDvl(obj, index)
            measurement = obj.emptyMeasurement();
            if ~obj.Cfg.sensor.dvl.isEnabled || obj.isDvlMissing()
                return;
            end
            sampleIndex = obj.CurMeas.Index.Dvl(index);
            if sampleIndex <= 0
                return;
            end

            measurement = obj.buildDvlMeasurement(sampleIndex, index);
        end

        %% get depth measurement
        function measurement = getDepth(obj, index)
            measurement = obj.emptyMeasurement();
            if ~obj.Cfg.sensor.depth.isEnabled || isempty(obj.CurMeas.Depth.Depth)
                return;
            end

            sampleIndex = obj.CurMeas.Index.Depth(index);
            if sampleIndex <= 0
                return;
            end

            % ENU 高度向上为正，深度量测向下为正。
            measurement.Depth = obj.CurMeas.Depth.Depth(sampleIndex);
            measurement.R = obj.Cfg.noise.depth.depthStd^2;
            measurement.SampleTime = obj.CurMeas.Depth.Time(sampleIndex);
            measurement.ArrivalTime = obj.Time(index);
            measurement.DelaySteps = obj.getDelaySteps("depth");
            measurement.DelaySeconds = measurement.DelaySteps * obj.Cfg.sim.dt;
            measurement.Valid = true;
        end

        %% get GPS measurement
        function measurement = getGps(obj, index)
            measurement = obj.emptyMeasurement();
            if ~obj.Cfg.sensor.gps.isEnabled || isempty(obj.CurMeas.Gps.PositionLlh)
                return;
            end
            sampleIndex = obj.CurMeas.Index.Gps(index);
            if sampleIndex <= 0
                return;
            end

            measurement.PositionLlh = obj.CurMeas.Gps.PositionLlh(sampleIndex, :).';
            stdVector = obj.expandStd(obj.Cfg.noise.gps.positionStd, 3);
            measurement.R = diag(stdVector.^2);
            measurement.SampleTime = obj.CurMeas.Gps.Time(sampleIndex);
            measurement.ArrivalTime = obj.Time(index);
            measurement.DelaySteps = obj.getDelaySteps("gps");
            measurement.DelaySeconds = measurement.DelaySteps * obj.Cfg.sim.dt;
            measurement.Valid = true;
        end

        %% get one truth sample
        function truth = getTruth(obj, index)
            truth = struct();
            if ~obj.hasTruth()
                return;
            end

            if obj.hasPositionTruth()
                truth.PositionLlh = obj.GtMeas.Truth.PositionLlh(index, :).';
            end
            if obj.hasVelocityTruth()
                truth.VelocityEnu = obj.GtMeas.Truth.VelocityEnu(index, :).';
            end
            if obj.hasAttitudeTruth()
                truth.Cbn = obj.GtMeas.Truth.AttitudeCbn(:, :, index);
            end
        end

        %% get truth arrays
        function truth = getTruthArrays(obj)
            truth = obj.GtMeas.Truth;
        end

        %% check truth availability
        function tf = hasTruth(obj)
            tf = obj.hasPositionTruth() || obj.hasVelocityTruth() ...
                || obj.hasAttitudeTruth();
        end

        %% check position truth availability
        function tf = hasPositionTruth(obj)
            tf = isfield(obj.GtMeas, "Truth") ...
                && ~isempty(obj.GtMeas.Truth.PositionLlh);
        end

        %% check velocity truth availability
        function tf = hasVelocityTruth(obj)
            tf = isfield(obj.GtMeas, "Truth") ...
                && ~isempty(obj.GtMeas.Truth.VelocityEnu);
        end

        %% check attitude truth availability
        function tf = hasAttitudeTruth(obj)
            tf = isfield(obj.GtMeas, "Truth") ...
                && ~isempty(obj.GtMeas.Truth.AttitudeCbn);
        end
    end

    methods (Access = private)
        %% load all input blocks
        function gtMeas = loadAllData(obj, inputData)
            % 数据加载按类型分块，便于后续替换数据生成函数。
            gtMeas = struct();
            gtMeas.Metadata = obj.getField(inputData, "metadata", struct());
            gtMeas.Time = obj.loadTimeData(inputData);
            gtMeas.Initial = obj.loadInitialData(inputData);
            gtMeas.Truth = obj.loadTruthData(inputData, gtMeas.Time);
            gtMeas.Imu = obj.loadImuData(inputData, gtMeas.Time);
            gtMeas.Dvl = obj.loadDvlData(inputData, gtMeas.Time);
            gtMeas.Depth = obj.loadDepthData(inputData, gtMeas.Time);
            gtMeas.Gps = obj.loadGpsData(inputData, gtMeas.Time);
        end

        %% apply simulation duration limit
        function gtMeas = applySimulationDurationLimit(obj, gtMeas)
            % cfg.sim.duration 用于只截取输入轨迹前若干秒参与算法运行。
            duration = obj.getField(obj.Cfg.sim, "duration", inf);
            if isempty(duration)
                return;
            end
            if isnumeric(duration) && isscalar(duration) && isinf(duration)
                return;
            end
            if ~isnumeric(duration) || ~isscalar(duration) || ~isfinite(duration) || duration <= 0.0
                error("StateAndMeasurement:InvalidSimulationDuration", ...
                    "cfg.sim.duration must be a positive finite scalar, inf, or [].");
            end
            if isempty(gtMeas.Time)
                return;
            end

            startTime = gtMeas.Time(1);
            endTime = startTime + duration;
            timeTolerance = max(eps(endTime), 0.5 * obj.Cfg.sim.dt);
            if endTime >= gtMeas.Time(end) - timeTolerance
                return;
            end

            rootMask = gtMeas.Time <= endTime + timeTolerance;
            if nnz(rootMask) < 2
                error("StateAndMeasurement:SimulationDurationTooShort", ...
                    "cfg.sim.duration must keep at least two main time samples.");
            end

            % 主时间轴、真值和 IMU 必须严格同长，因此按主时间轴掩码裁剪。
            gtMeas.Time = gtMeas.Time(rootMask);
            gtMeas.Truth = obj.trimTruthByMainMask(gtMeas.Truth, rootMask);
            gtMeas.Imu = obj.trimStructRows(gtMeas.Imu, rootMask);

            % 低频传感器按各自时间戳裁剪到新的仿真结束时刻。
            limitedEndTime = gtMeas.Time(end);
            gtMeas.Dvl = obj.trimSensorByEndTime(gtMeas.Dvl, limitedEndTime, timeTolerance);
            gtMeas.Depth = obj.trimSensorByEndTime(gtMeas.Depth, limitedEndTime, timeTolerance);
            gtMeas.Gps = obj.trimSensorByEndTime(gtMeas.Gps, limitedEndTime, timeTolerance);
        end

        %% trim truth data by main time mask
        function truth = trimTruthByMainMask(~, truth, rootMask)
            if ~isempty(truth.PositionLlh)
                truth.PositionLlh = truth.PositionLlh(rootMask, :);
            end
            if ~isempty(truth.VelocityEnu)
                truth.VelocityEnu = truth.VelocityEnu(rootMask, :);
            end
            if ~isempty(truth.AttitudeCbn)
                truth.AttitudeCbn = truth.AttitudeCbn(:, :, rootMask);
            end
        end

        %% trim sensor data by end time
        function sensor = trimSensorByEndTime(obj, sensor, endTime, timeTolerance)
            if ~isfield(sensor, "Time") || isempty(sensor.Time)
                return;
            end

            sensorMask = sensor.Time <= endTime + timeTolerance;
            sensor = obj.trimStructRows(sensor, sensorMask);
        end

        %% trim struct fields whose first dimension matches mask length
        function data = trimStructRows(~, data, rowMask)
            rowCount = numel(rowMask);
            fieldNames = fieldnames(data);
            for fieldIndex = 1:numel(fieldNames)
                fieldName = fieldNames{fieldIndex};
                value = data.(fieldName);
                if ~(isnumeric(value) || islogical(value)) || isempty(value)
                    continue;
                end

                if isvector(value) && numel(value) == rowCount
                    data.(fieldName) = value(rowMask, :);
                elseif size(value, 1) == rowCount
                    data.(fieldName) = value(rowMask, :);
                end
            end
        end

        %% load time data
        function time = loadTimeData(obj, inputData)
            time = obj.asColumn(obj.getField(inputData, "time", []));
        end

        %% load initial data
        function initial = loadInitialData(obj, inputData)
            rawInitial = obj.getField(inputData, "initial", struct());
            initial = struct();
            initial.PositionLlh = obj.asVector(obj.getField(rawInitial, "positionLlh", []));
            initial.VelocityEnu = obj.asVector(obj.getField(rawInitial, "velocityEnu", []));
            initial.Cbn = obj.loadAttitude(rawInitial);
            initial.GyroBias = obj.asVector(obj.getField(rawInitial, "gyroBias", zeros(3, 1)));
            initial.AccelBias = obj.asVector(obj.getField(rawInitial, "accelBias", zeros(3, 1)));
        end

        %% load truth data
        function truth = loadTruthData(obj, inputData, time)
            rawTruth = obj.getField(inputData, "truth", struct());
            truth = struct();
            truth.PositionLlh = obj.asMatrix(obj.getField(rawTruth, "positionLlh", []), 3);
            truth.VelocityEnu = obj.asMatrix(obj.getField(rawTruth, "velocityEnu", []), 3);
            truth.AttitudeCbn = obj.loadAttitudeArray(rawTruth, numel(time));
        end

        %% load IMU data
        function imu = loadImuData(obj, inputData, time)
            rawImu = obj.getField(inputData, "imu", struct());
            imu = struct();
            imu.Gyro = obj.asMatrix(obj.getField(rawImu, "gyro", []), 3);
            imu.Accel = obj.asMatrix(obj.getField(rawImu, "accel", []), 3);
            imu.Time = obj.loadSensorTime(rawImu, time, size(imu.Gyro, 1), "imu");
            imu.Valid = true(size(imu.Gyro, 1), 1);
        end

        %% load DVL data
        function dvl = loadDvlData(obj, inputData, time)
            rawDvl = obj.getField(inputData, "dvl", struct());
            dvl = struct();
            dvl.VelocityBody = obj.asMatrix(obj.getField(rawDvl, "velocityBody", []), 3);
            dvl.Time = obj.loadSensorTime(rawDvl, time, size(dvl.VelocityBody, 1), "dvl");
            dvl.Valid = obj.loadValidVector(rawDvl, size(dvl.VelocityBody, 1));
        end

        %% load depth data
        function depth = loadDepthData(obj, inputData, time)
            rawDepth = obj.getField(inputData, "depth", struct());
            depth = struct();
            depth.Depth = obj.asColumn(obj.getField(rawDepth, "depth", []));
            depth.Time = obj.loadSensorTime(rawDepth, time, numel(depth.Depth), "depth");
            depth.Valid = obj.loadValidVector(rawDepth, numel(depth.Depth));
        end

        %% load GPS data
        function gps = loadGpsData(obj, inputData, time)
            rawGps = obj.getField(inputData, "gps", struct());
            gps = struct();
            gps.PositionLlh = obj.asMatrix(obj.getField(rawGps, "positionLlh", []), 3);
            gps.Time = obj.loadSensorTime(rawGps, time, size(gps.PositionLlh, 1), "gps");
            gps.Valid = obj.loadValidVector(rawGps, size(gps.PositionLlh, 1));
        end

        %% generate noisy IMU
        function imu = generateNoisyImu(obj)
            imu = obj.GtMeas.Imu;
            if ~obj.Cfg.data.isSensorNoiseMonteCarlo
                return;
            end

            gyroBias = zeros(3, 1);
            accelBias = zeros(3, 1);
            [~, stateBlocks] = createInertialStateProfile(obj.Cfg);
            blockNames = string({stateBlocks.Name});
            if any(blockNames == "GyroBias")
                gyroBias = obj.generateConstantBias( ...
                    obj.Cfg.noise.imu.gyroBiasStd, 3);
            end
            if any(blockNames == "AccelBias")
                accelBias = obj.generateConstantBias( ...
                    obj.Cfg.noise.imu.accelBiasStd, 3);
            end

            % Profile 包含相应零偏状态时才注入常值零偏；白噪声始终正常加入。
            imu.Gyro = obj.addConstantBias(obj.GtMeas.Imu.Gyro, gyroBias);
            imu.Accel = obj.addConstantBias(obj.GtMeas.Imu.Accel, accelBias);
            imu.Gyro = obj.addNoise(imu.Gyro, obj.Cfg.noise.imu.gyroStd, 3);
            imu.Accel = obj.addNoise(imu.Accel, obj.Cfg.noise.imu.accelStd, 3);
            imu.GyroBias = gyroBias;
            imu.AccelBias = accelBias;
        end

        %% generate noisy DVL
        function dvl = generateNoisyDvl(obj)
            dvl = obj.GtMeas.Dvl;
            if ~obj.Cfg.data.isSensorNoiseMonteCarlo
                return;
            end

            if ~isempty(obj.GtMeas.Dvl.VelocityBody)
                dvl.VelocityBody = obj.addNoise(obj.GtMeas.Dvl.VelocityBody, obj.Cfg.noise.dvl.velocityStd, 3);
            end
        end

        %% generate noisy depth
        function depth = generateNoisyDepth(obj)
            depth = obj.GtMeas.Depth;
            if obj.Cfg.data.isSensorNoiseMonteCarlo && ~isempty(obj.GtMeas.Depth.Depth)
                depth.Depth = obj.addNoise(obj.GtMeas.Depth.Depth, obj.Cfg.noise.depth.depthStd, 1);
            end
        end

        %% generate noisy GPS
        function gps = generateNoisyGps(obj)
            gps = obj.GtMeas.Gps;
            if obj.Cfg.data.isSensorNoiseMonteCarlo && ~isempty(obj.GtMeas.Gps.PositionLlh)
                gps.PositionLlh = obj.addPositionNoise(obj.GtMeas.Gps.PositionLlh, obj.Cfg.noise.gps.positionStd);
            end
        end

        %% build DVL measurement
        function measurement = buildDvlMeasurement(obj, sampleIndex, arrivalIndex)
            measurement = obj.emptyMeasurement();
            measurement.VelocityBody = obj.CurMeas.Dvl.VelocityBody(sampleIndex, :).';
            stdVector = obj.expandStd(obj.Cfg.noise.dvl.velocityStd, 3);
            measurement.R = diag(stdVector.^2);
            measurement.SampleTime = obj.CurMeas.Dvl.Time(sampleIndex);
            measurement.ArrivalTime = obj.Time(arrivalIndex);
            measurement.DelaySteps = obj.getDelaySteps("dvl");
            measurement.DelaySeconds = measurement.DelaySteps * obj.Cfg.sim.dt;
            measurement.Valid = true;
        end

        %% check configured coordinate frames
        function checkFrameConfig(obj)
            navigationFrame = upper(string(obj.getField( ...
                obj.Cfg.data, "coordinateFrame", "")));
            bodyFrame = upper(string(obj.getField(obj.Cfg.data, "bodyFrame", "")));
            if ~isscalar(navigationFrame) || navigationFrame ~= "ENU"
                error("StateAndMeasurement:UnsupportedNavigationFrame", ...
                    "This implementation requires cfg.data.coordinateFrame = ""ENU"".");
            end
            if ~isscalar(bodyFrame) || bodyFrame ~= "RFU"
                error("StateAndMeasurement:UnsupportedBodyFrame", ...
                    "This implementation requires cfg.data.bodyFrame = ""RFU"".");
            end
        end

        %% compare optional input metadata with configured frames
        function checkInputFrameMetadata(obj)
            metadata = obj.GtMeas.Metadata;
            inputNavigationFrame = upper(string(obj.getField( ...
                metadata, "navigationFrame", "ENU")));
            inputBodyFrame = upper(string(obj.getField(metadata, "bodyFrame", "RFU")));
            if ~isscalar(inputNavigationFrame) || inputNavigationFrame ~= "ENU"
                error("StateAndMeasurement:InputNavigationFrameMismatch", ...
                    "inputData.metadata.navigationFrame must be ""ENU"".");
            end
            if ~isscalar(inputBodyFrame) || inputBodyFrame ~= "RFU"
                error("StateAndMeasurement:InputBodyFrameMismatch", ...
                    "inputData.metadata.bodyFrame must be ""RFU"".");
            end
        end

        %% check time data
        function checkTimeData(obj)
            if isempty(obj.GtMeas) || isempty(obj.GtMeas.Time)
                error("StateAndMeasurement:MissingTime", ...
                    "Input data must contain inputData.time.");
            end
        end

        %% check IMU data
        function checkImuData(obj)
            obj.checkMatrix(obj.GtMeas.Imu.Gyro, 3, "imu.gyro");
            obj.checkMatrix(obj.GtMeas.Imu.Accel, 3, "imu.accel");
            obj.checkSameRows(obj.GtMeas.Time, obj.GtMeas.Imu.Gyro, "imu.gyro");
            obj.checkSameRows(obj.GtMeas.Time, obj.GtMeas.Imu.Accel, "imu.accel");
        end

        %% check initial data
        function checkInitialData(obj)
            try
                obj.getInitialNavigation();
            catch exception
                error("StateAndMeasurement:MissingInitialNavigation", ...
                    "Input data must provide initial or truth navigation solution. %s", exception.message);
            end
        end

        %% check truth data
        function checkTruthData(obj)
            truth = obj.GtMeas.Truth;
            if ~isempty(truth.PositionLlh)
                obj.checkMatrix(truth.PositionLlh, 3, "truth.positionLlh");
                obj.checkSameRows(obj.GtMeas.Time, truth.PositionLlh, "truth.positionLlh");
            end
            if ~isempty(truth.VelocityEnu)
                obj.checkMatrix(truth.VelocityEnu, 3, "truth.velocityEnu");
                obj.checkSameRows(obj.GtMeas.Time, truth.VelocityEnu, "truth.velocityEnu");
            end
            if ~isempty(truth.AttitudeCbn) && size(truth.AttitudeCbn, 3) ~= numel(obj.GtMeas.Time)
                error("StateAndMeasurement:TruthAttitudeLengthMismatch", ...
                    "truth attitude must have the same sample count as inputData.time.");
            end
        end

        %% check enabled measurements
        function checkEnabledMeasurements(obj)
            if obj.Cfg.sensor.dvl.isEnabled && obj.isDvlMissing()
                error("StateAndMeasurement:MissingDvl", ...
                    "DVL is enabled, but dvl.velocityBody is missing.");
            end
            if obj.Cfg.sensor.depth.isEnabled && isempty(obj.GtMeas.Depth.Depth)
                error("StateAndMeasurement:MissingDepth", ...
                    "Depth sensor is enabled, but depth.depth is missing.");
            end
            if obj.Cfg.sensor.gps.isEnabled && isempty(obj.GtMeas.Gps.PositionLlh)
                error("StateAndMeasurement:MissingGps", ...
                    "GPS is enabled, but gps.positionLlh is missing.");
            end
        end

        %% check sensor availability config
        function checkSensorAvailabilityConfig(obj)
            obj.checkTimeIntervalMatrix("dvl", obj.getAvailableTime("dvl"));
            obj.checkTimeIntervalMatrix("gps", obj.getAvailableTime("gps"));
        end

        %% check sensor delay config
        function checkSensorDelayConfig(obj)
            delayCfg = obj.getField(obj.Cfg, "sensorDelay", struct());
            isEnabled = obj.getField(delayCfg, "isEnabled", false);
            if ~(islogical(isEnabled) && isscalar(isEnabled))
                error("StateAndMeasurement:InvalidDelaySwitch", ...
                    "cfg.sensorDelay.isEnabled must be a logical scalar.");
            end

            sensorNames = ["imu", "dvl", "depth", "gps"];
            for sensorIndex = 1:numel(sensorNames)
                sensorName = sensorNames(sensorIndex);
                sensorCfg = obj.getField(delayCfg, sensorName, struct());
                delaySteps = obj.getField(sensorCfg, "delaySteps", 0);
                isValid = isnumeric(delaySteps) && isscalar(delaySteps) ...
                    && isfinite(delaySteps) && delaySteps >= 0 ...
                    && delaySteps <= 3 && delaySteps == floor(delaySteps);
                if ~isValid
                    error("StateAndMeasurement:InvalidDelaySteps", ...
                        "%s delaySteps must be an integer from 0 to 3.", sensorName);
                end
            end
        end

        %% build measurement index maps
        function indexMap = buildMeasurementIndexMap(obj)
            %BUILDMEASUREMENTINDEXMAP 预先建立主时间轴到异步量测的索引映射。
            numSamples = numel(obj.Time);
            indexMap = struct();
            indexMap.Imu = zeros(numSamples, 1);
            indexMap.Dvl = zeros(numSamples, 1);
            indexMap.Depth = zeros(numSamples, 1);
            indexMap.Gps = zeros(numSamples, 1);

            indexMap.Imu = obj.buildSensorIndexMap( ...
                obj.CurMeas.Imu.Time, obj.CurMeas.Imu.Valid, "");
            indexMap.Imu = obj.applySensorDelay(indexMap.Imu, "imu");

            if obj.Cfg.sensor.dvl.isEnabled && ~obj.isDvlMissing()
                indexMap.Dvl = obj.buildSensorIndexMap( ...
                    obj.CurMeas.Dvl.Time, obj.CurMeas.Dvl.Valid, "dvl");
                indexMap.Dvl = obj.applySensorDelay(indexMap.Dvl, "dvl");
            end

            if obj.Cfg.sensor.depth.isEnabled && ~isempty(obj.CurMeas.Depth.Depth)
                indexMap.Depth = obj.buildSensorIndexMap( ...
                    obj.CurMeas.Depth.Time, obj.CurMeas.Depth.Valid, "");
                indexMap.Depth = obj.applySensorDelay(indexMap.Depth, "depth");
            end

            if obj.Cfg.sensor.gps.isEnabled && ~isempty(obj.CurMeas.Gps.PositionLlh)
                indexMap.Gps = obj.buildSensorIndexMap( ...
                    obj.CurMeas.Gps.Time, obj.CurMeas.Gps.Valid, "gps");
                indexMap.Gps = obj.applySensorDelay(indexMap.Gps, "gps");
            end
        end

        %% 将采样时刻索引映射平移到总线到达时刻
        function arrivalMap = applySensorDelay(obj, sampleMap, sensorName)
            delaySteps = obj.getDelaySteps(sensorName);
            arrivalMap = zeros(size(sampleMap));
            if delaySteps == 0
                arrivalMap = sampleMap;
                return;
            end

            numSamples = numel(sampleMap);
            if delaySteps < numSamples
                arrivalMap((delaySteps + 1):numSamples) = ...
                    sampleMap(1:(numSamples - delaySteps));
            end
        end

        %% build one sensor index map
        function indexMap = buildSensorIndexMap(obj, sensorTime, valid, sensorName)
            %BUILDSENSORINDEXMAP 使用单调双指针匹配异步量测，避免逐点全量搜索。
            numSamples = numel(obj.Time);
            indexMap = zeros(numSamples, 1);
            if isempty(sensorTime)
                return;
            end

            sensorTime = sensorTime(:);
            if isempty(valid)
                valid = true(numel(sensorTime), 1);
            else
                valid = logical(valid(:));
            end
            validIndex = find(valid);
            if isempty(validIndex)
                return;
            end

            validTime = sensorTime(validIndex);
            [validTime, sortIndex] = sort(validTime);
            validIndex = validIndex(sortIndex);

            availableMask = obj.buildAvailableMask(sensorName);
            tolerance = obj.Cfg.measurement.timeTolerance;
            validCount = numel(validTime);
            validPointer = 1;

            for sampleIndex = 1:numSamples
                if ~availableMask(sampleIndex)
                    continue;
                end

                currentTime = obj.Time(sampleIndex);
                lowerBound = currentTime - tolerance;
                while validPointer <= validCount && validTime(validPointer) < lowerBound
                    validPointer = validPointer + 1;
                end
                if validPointer > validCount
                    break;
                end

                if abs(validTime(validPointer) - currentTime) <= tolerance
                    indexMap(sampleIndex) = validIndex(validPointer);
                end
            end
        end

        %% build sensor availability mask
        function availableMask = buildAvailableMask(obj, sensorName)
            %BUILDAVAILABLEMASK 将可用时间段展开到主时间轴。
            availableMask = true(numel(obj.Time), 1);
            sensorName = string(sensorName);
            if strlength(sensorName) == 0
                return;
            end

            intervals = obj.getAvailableTime(sensorName);
            if isempty(intervals)
                return;
            end

            availableMask = false(numel(obj.Time), 1);
            for intervalIndex = 1:size(intervals, 1)
                availableMask = availableMask ...
                    | (obj.Time >= intervals(intervalIndex, 1) ...
                    & obj.Time <= intervals(intervalIndex, 2));
            end
        end

        %% check DVL availability
        function tf = isDvlMissing(obj)
            tf = isempty(obj.GtMeas.Dvl.VelocityBody);
        end

        %% get configured sensor delay steps
        function delaySteps = getDelaySteps(obj, sensorName)
            delaySteps = 0;
            delayCfg = obj.getField(obj.Cfg, "sensorDelay", struct());
            isEnabled = logical(obj.getField(delayCfg, "isEnabled", false));
            if ~isEnabled
                return;
            end

            sensorCfg = obj.getField(delayCfg, string(sensorName), struct());
            delaySteps = double(obj.getField(sensorCfg, "delaySteps", 0));
        end

        %% get configured available time
        function intervals = getAvailableTime(obj, sensorName)
            sensorName = lower(string(sensorName));
            switch sensorName
                case "dvl"
                    sensorCfg = obj.Cfg.sensor.dvl;
                case "gps"
                    sensorCfg = obj.Cfg.sensor.gps;
                otherwise
                    error("StateAndMeasurement:InvalidSensorName", ...
                        "sensorName must be either ""dvl"" or ""gps"".");
            end

            intervals = obj.getField(sensorCfg, "availableTime", [0.0, inf]);
            intervals = double(intervals);
        end

        %% check available time matrix
        function checkTimeIntervalMatrix(~, sensorName, intervals)
            if isempty(intervals)
                return;
            end
            if size(intervals, 2) ~= 2
                error("StateAndMeasurement:InvalidAvailableTime", ...
                    "%s availableTime must be an N-by-2 matrix.", sensorName);
            end
            if any(isnan(intervals(:))) || any(intervals(:, 2) < intervals(:, 1))
                error("StateAndMeasurement:InvalidAvailableTime", ...
                    "%s availableTime must satisfy startTime <= endTime.", sensorName);
            end
        end

        %% check whether sensor is available now
        function tf = isSensorAvailableAt(obj, sensorName, queryTime)
            intervals = obj.getAvailableTime(sensorName);
            if isempty(intervals)
                tf = true;
                return;
            end

            tf = any(queryTime >= intervals(:, 1) & queryTime <= intervals(:, 2));
        end

        %% select root input structure
        function inputData = selectInputStruct(~, loadedData)
            if isfield(loadedData, "inputData") && isstruct(loadedData.inputData)
                inputData = loadedData.inputData;
            elseif isfield(loadedData, "gt_mes") && isstruct(loadedData.gt_mes)
                inputData = loadedData.gt_mes;
            elseif isfield(loadedData, "gtMeas") && isstruct(loadedData.gtMeas)
                inputData = loadedData.gtMeas;
            elseif isfield(loadedData, "data") && isstruct(loadedData.data)
                inputData = loadedData.data;
            else
                inputData = loadedData;
            end
        end

        %% get fixed field
        function value = getField(~, source, fieldName, defaultValue)
            value = defaultValue;
            if isstruct(source) && isfield(source, fieldName)
                value = source.(fieldName);
            end
        end

        %% load attitude matrix
        function Cbn = loadAttitude(obj, source)
            Cbn = obj.getField(source, "attitudeCbn", []);
            euler = obj.getField(source, "attitudeEuler", []);
            if isempty(Cbn) && ~isempty(euler)
                Cbn = dcmFromEuler(obj.firstVector(euler));
            end
        end

        %% load attitude array
        function attitudeCbn = loadAttitudeArray(obj, source, numSamples)
            attitudeCbn = obj.getField(source, "attitudeCbn", []);
            euler = obj.asMatrix(obj.getField(source, "attitudeEuler", []), 3);

            if isempty(attitudeCbn) && ~isempty(euler)
                attitudeCbn = zeros(3, 3, size(euler, 1));
                for rowIndex = 1:size(euler, 1)
                    attitudeCbn(:, :, rowIndex) = dcmFromEuler(euler(rowIndex, :).');
                end
            end

            if isempty(attitudeCbn)
                return;
            end
            if isequal(size(attitudeCbn), [3, 3])
                attitudeCbn = repmat(attitudeCbn, 1, 1, numSamples);
            end
            if size(attitudeCbn, 1) ~= 3 || size(attitudeCbn, 2) ~= 3
                error("StateAndMeasurement:InvalidAttitude", ...
                    "attitudeCbn must be a 3-by-3-by-N array.");
            end
            if size(attitudeCbn, 3) ~= numSamples
                error("StateAndMeasurement:AttitudeLengthMismatch", ...
                    "attitude sample count must match inputData.time.");
            end
        end

        %% load sensor time
        function sensorTime = loadSensorTime(obj, source, rootTime, rowCount, sensorName)
            sensorTime = obj.asColumn(obj.getField(source, "time", []));
            if isempty(sensorTime) && rowCount > 0
                if rowCount == numel(rootTime)
                    sensorTime = rootTime;
                else
                    error("StateAndMeasurement:MissingSensorTime", ...
                        "%s measurement length differs from time. Provide %s.time.", sensorName, sensorName);
                end
            end
            if ~isempty(sensorTime) && rowCount > 0 && numel(sensorTime) ~= rowCount
                error("StateAndMeasurement:SensorTimeLengthMismatch", ...
                    "%s.time must have the same row count as %s measurements.", sensorName, sensorName);
            end
        end

        %% load valid vector
        function valid = loadValidVector(obj, source, rowCount)
            valid = obj.getField(source, "valid", []);
            if isempty(valid)
                valid = true(rowCount, 1);
            else
                valid = logical(valid(:));
                if numel(valid) ~= rowCount
                    error("StateAndMeasurement:ValidLengthMismatch", ...
                        "valid vector must have the same row count as the measurement.");
                end
            end
        end

        %% convert to column
        function value = asColumn(~, value)
            if isempty(value)
                value = [];
            else
                value = double(value(:));
            end
        end

        %% convert to vector
        function value = asVector(~, value)
            if isempty(value)
                value = [];
            else
                value = double(value(:));
            end
        end

        %% convert to N-by-columnCount matrix
        function value = asMatrix(~, value, columnCount)
            if isempty(value)
                value = [];
                return;
            end

            value = double(value);
            if isvector(value)
                value = reshape(value, 1, []);
            end
            if size(value, 2) ~= columnCount && size(value, 1) == columnCount
                value = value.';
            end
        end

        %% get first vector sample
        function vector = firstVector(~, value)
            if isempty(value)
                vector = [];
            elseif isvector(value)
                vector = double(value(:));
            else
                vector = double(value(1, :).');
            end
        end

        %% get first nonempty row count
        function count = getFirstRowCount(~, varargin)
            count = 0;
            for inputIndex = 1:nargin - 1
                value = varargin{inputIndex};
                if ~isempty(value)
                    count = size(value, 1);
                    return;
                end
            end
        end

        %% add Gaussian noise
        function noisyData = addNoise(obj, data, stdValue, columnCount)
            if isempty(data)
                noisyData = data;
                return;
            end

            stdVector = obj.expandStd(stdValue, columnCount);
            noisyData = data + randn(size(data)) .* reshape(stdVector, 1, []);
        end

        %% generate constant bias
        function bias = generateConstantBias(obj, stdValue, columnCount)
            stdVector = obj.expandStd(stdValue, columnCount);
            bias = stdVector .* randn(columnCount, 1);
        end

        %% add constant bias
        function biasedData = addConstantBias(~, data, bias)
            if isempty(data)
                biasedData = data;
                return;
            end

            biasedData = data + reshape(bias, 1, []);
        end

        %% add GPS position noise
        function noisyPosition = addPositionNoise(obj, positionLlh, stdValue)
            noisyPosition = positionLlh;
            stdVector = obj.expandStd(stdValue, 3);
            for rowIndex = 1:size(positionLlh, 1)
                enuNoise = stdVector .* randn(3, 1);
                noisyPosition(rowIndex, :) = enuOffsetToLlh(positionLlh(rowIndex, :).', enuNoise).';
            end
        end

        %% expand noise standard deviation
        function stdVector = expandStd(~, stdValue, columnCount)
            stdVector = double(stdValue(:));
            if isscalar(stdVector)
                stdVector = repmat(stdVector, columnCount, 1);
            end
            if numel(stdVector) ~= columnCount
                error("StateAndMeasurement:InvalidNoiseStd", ...
                    "Noise standard deviation must be scalar or %d-by-1.", columnCount);
            end
        end

        %% find nearest measurement index
        function index = findMeasurementIndex(obj, time, currentTime, valid)
            index = [];
            if isempty(time)
                return;
            end

            timeError = abs(time(:) - currentTime);
            if ~isempty(valid)
                timeError(~valid(:)) = inf;
            end

            [minimumError, candidateIndex] = min(timeError);
            if minimumError <= obj.Cfg.measurement.timeTolerance
                index = candidateIndex;
            end
        end

        %% empty measurement template
        function measurement = emptyMeasurement(~)
            measurement = struct();
            measurement.Valid = false;
        end

        %% check matrix size
        function checkMatrix(~, value, columnCount, fieldName)
            if isempty(value) || size(value, 2) ~= columnCount
                error("StateAndMeasurement:InvalidField", ...
                    "%s must be a nonempty N-by-%d matrix.", fieldName, columnCount);
            end
        end

        %% check row count
        function checkSameRows(~, time, value, fieldName)
            if size(value, 1) ~= numel(time)
                error("StateAndMeasurement:LengthMismatch", ...
                    "%s must have the same row count as inputData.time.", fieldName);
            end
        end
    end
end





