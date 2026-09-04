function conversionInfo = convertTrjMeasuredToInput(sourceFile, outputFile)
%CONVERTTRJMEASUREDTOINPUT Convert a PSINS trj MAT file to project inputData.
%   conversionInfo = convertTrjMeasuredToInput(sourceFile, outputFile)
%   preserves the source trj variable and creates inputData for
%   StateAndMeasurement. PSINS IMU increments are converted to the angular
%   rates and specific forces required by this project's mechanization.

arguments
    sourceFile (1, 1) string
    outputFile (1, 1) string
end

if ~isfile(sourceFile)
    error("convertTrjMeasuredToInput:SourceNotFound", ...
        "Source MAT file was not found: %s", sourceFile);
end

loadedData = load(sourceFile, "trj");
if ~isfield(loadedData, "trj") || ~isstruct(loadedData.trj)
    error("convertTrjMeasuredToInput:MissingTrj", ...
        "Source MAT file must contain a scalar struct named trj.");
end

trj = loadedData.trj;
validateSourceTrajectory(trj);

imuIncrement = double(trj.imu);
avp = double(trj.avp);
avp0 = double(trj.avp0(:));
sampleInterval = double(trj.ts);
sourceTime = imuIncrement(:, 7);
initialTime = sourceTime(1) - sampleInterval;
time = [initialTime; sourceTime];
imuDt = [sampleInterval; diff(sourceTime)];

% PSINS att is [pitch, roll, yaw] with west-positive yaw. This project uses
% [roll, pitch, yaw] with east-positive yaw. Both use ENU navigation and
% RFU body axes, so the IMU axes remain unchanged.
sourceAttitude = [avp0(1:3).'; avp(:, 1:3)];
attitudeEuler = [sourceAttitude(:, 2), sourceAttitude(:, 1), ...
    -sourceAttitude(:, 3)];
numOutputSamples = numel(time);
attitudeCbn = zeros(3, 3, numOutputSamples);
for sampleIndex = 1:numOutputSamples
    attitudeCbn(:, :, sampleIndex) = ...
        dcmFromEuler(attitudeEuler(sampleIndex, :).');
end

gyroRate = imuIncrement(:, 1:3) ./ imuDt;
specificForce = imuIncrement(:, 4:6) ./ imuDt;

inputData = struct();
inputData.time = time;
inputData.metadata = createMetadata();

inputData.initial = struct();
inputData.initial.positionLlh = avp0(7:9);
inputData.initial.velocityEnu = avp0(4:6);
inputData.initial.attitudeCbn = attitudeCbn(:, :, 1);
inputData.initial.attitudeEuler = attitudeEuler(1, :).';
inputData.initial.gyroBias = zeros(3, 1);
inputData.initial.accelBias = zeros(3, 1);

inputData.truth = struct();
inputData.truth.positionLlh = [avp0(7:9).'; avp(:, 7:9)];
inputData.truth.velocityEnu = [avp0(4:6).'; avp(:, 4:6)];
inputData.truth.attitudeCbn = attitudeCbn;
inputData.truth.attitudeEuler = attitudeEuler;

% The first project sample is the initial state and its IMU row is unused.
% Duplicate the first rate as a harmless placeholder, then map every source
% increment to the following propagation interval without dropping samples.
inputData.imu = struct();
inputData.imu.time = time;
inputData.imu.gyro = [gyroRate(1, :); gyroRate];
inputData.imu.accel = [specificForce(1, :); specificForce];
inputData.imu.deltaAngle = imuIncrement(:, 1:3);
inputData.imu.deltaVelocity = imuIncrement(:, 4:6);
inputData.imu.sourceTime = sourceTime;
inputData.imu.sourceSampleIndex = [0; (1:size(imuIncrement, 1)).'];

inputData.dvl = emptyVectorMeasurement("velocityBody");
inputData.depth = emptyScalarMeasurement("depth");
inputData.gps = emptyVectorMeasurement("positionLlh");

conversionInfo = struct();
conversionInfo.SourceFile = sourceFile;
conversionInfo.OutputFile = outputFile;
conversionInfo.SourceSampleCount = size(imuIncrement, 1);
conversionInfo.OutputSampleCount = numOutputSamples;
conversionInfo.SampleInterval = sampleInterval;
conversionInfo.SourceImuRepresentation = "delta angle [rad], delta velocity [m/s]";
conversionInfo.ProjectImuRepresentation = "angular rate [rad/s], specific force [m/s^2]";
conversionInfo.SourceAttitudeConvention = ...
    "[pitch, roll, yaw], north-zero west-positive yaw";
conversionInfo.ProjectAttitudeConvention = ...
    "[roll, pitch, yaw], north-zero east-positive yaw";
conversionInfo.Note = ...
    "The original trj variable is saved unchanged beside inputData.";

outputFolder = fileparts(outputFile);
if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
    mkdir(outputFolder);
end
save(outputFile, "inputData", "conversionInfo", "trj");

end

function validateSourceTrajectory(trj)
%VALIDATESOURCETRAJECTORY Validate the PSINS fields used by the converter.

requiredFields = ["imu", "avp", "avp0", "ts"];
for fieldIndex = 1:numel(requiredFields)
    fieldName = requiredFields(fieldIndex);
    if ~isfield(trj, fieldName)
        error("convertTrjMeasuredToInput:MissingField", ...
            "trj.%s is required.", fieldName);
    end
end

if ~isnumeric(trj.imu) || size(trj.imu, 2) ~= 7
    error("convertTrjMeasuredToInput:InvalidImu", ...
        "trj.imu must be an N-by-7 numeric matrix [wm, vm, time].");
end
if ~isnumeric(trj.avp) || size(trj.avp, 2) ~= 10
    error("convertTrjMeasuredToInput:InvalidAvp", ...
        "trj.avp must be an N-by-10 numeric matrix [att, vn, pos, time].");
end
if size(trj.imu, 1) ~= size(trj.avp, 1) || isempty(trj.imu)
    error("convertTrjMeasuredToInput:SampleCountMismatch", ...
        "trj.imu and trj.avp must have the same nonzero row count.");
end
if ~isnumeric(trj.avp0) || numel(trj.avp0) ~= 9
    error("convertTrjMeasuredToInput:InvalidAvp0", ...
        "trj.avp0 must contain nine numeric values [att; vn; pos].");
end
if ~isnumeric(trj.ts) || ~isscalar(trj.ts) ...
        || ~isfinite(trj.ts) || trj.ts <= 0.0
    error("convertTrjMeasuredToInput:InvalidSampleInterval", ...
        "trj.ts must be a finite positive scalar.");
end

numericValues = [trj.imu(:); trj.avp(:); trj.avp0(:)];
if any(~isfinite(numericValues))
    error("convertTrjMeasuredToInput:NonfiniteData", ...
        "trj.imu, trj.avp, and trj.avp0 must contain only finite values.");
end

imuTime = double(trj.imu(:, 7));
avpTime = double(trj.avp(:, 10));
if any(diff(imuTime) <= 0.0)
    error("convertTrjMeasuredToInput:InvalidTime", ...
        "trj.imu timestamps must be strictly increasing.");
end
timeTolerance = max(1.0e-9, 1.0e-6*double(trj.ts));
if any(abs(imuTime - avpTime) > timeTolerance)
    error("convertTrjMeasuredToInput:TimeMismatch", ...
        "trj.imu and trj.avp timestamps must match.");
end

end

function metadata = createMetadata()
%CREATEMETADATA Describe frame, attitude, and IMU conventions.

metadata = struct();
metadata.bodyFrame = "RFU";
metadata.navigationFrame = "ENU";
metadata.bodyAxisOrder = ["right", "forward", "up"];
metadata.eulerOrder = ["roll", "pitch", "yaw"];
metadata.eulerConvention = ...
    "positive right-bank, nose-up, north-zero clockwise heading";
metadata.sourceFormat = "PSINS trj";
metadata.sourceImuRepresentation = "increment";
metadata.imuRepresentation = "rate";

end

function measurement = emptyVectorMeasurement(valueField)
%EMPTYVECTORMEASUREMENT Create a disabled three-axis measurement struct.

measurement = struct();
measurement.time = zeros(0, 1);
measurement.(valueField) = zeros(0, 3);
measurement.valid = false(0, 1);

end

function measurement = emptyScalarMeasurement(valueField)
%EMPTYSCALARMEASUREMENT Create a disabled scalar measurement struct.

measurement = struct();
measurement.time = zeros(0, 1);
measurement.(valueField) = zeros(0, 1);
measurement.valid = false(0, 1);

end
