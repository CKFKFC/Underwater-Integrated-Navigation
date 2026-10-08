# Underwater-Integrated-Navigation

## 水下组合导航代码阅读与使用说明

基于 MATLAB 的水下惯性组合导航程序，包含捷联惯导、误差状态卡尔曼滤波（ESKF）、多传感器融合及固定延时补偿。本文说明代码阅读顺序、数据准备、运行配置和结果解读，程序入口为 [main.m](main.m)。

> **首次运行：** 仓库不附带 MAT 数据文件。请先按第 2 节生成 **3600 s 直线轨迹**，再运行主程序。

### 目录

- [1. 从哪里开始读](#reading)
- [2. 运行一次程序](#running)
- [3. 坐标系、单位与输入格式](#input)
- [4. 常用配置如何影响运行](#configuration)
- [5. 主程序与数据流](#workflow)
- [6. 一个无延时时间步做了什么](#filter-step)
- [7. 异步量测与固定延时](#sensor-delay)
- [8. 读取结果与理解图形](#results)
- [9. 常见使用问题](#faq)

<a id="reading"></a>

## 1. 从哪里开始读

建议按下表顺序阅读。第一次只想运行程序时，先看第 2～4 节；想理解算法时，再看第 5～7 节。

| 文件 | 阅读重点 |
| --- | --- |
| [main.m](main.m) | 一次完整运行如何串起配置、量测、滤波、保存与绘图 |
| [setConfig.m](config/setConfig.m) | 输入文件、运行时长、传感器开关、噪声与状态模型 |
| [StateAndMeasurement.m](measurement/StateAndMeasurement.m) | MAT 字段读取、初值获取、加噪、异步量测匹配 |
| [ESKF.m](algorithm/ESKF.m) | Monte Carlo 外循环与时间内循环，以及量测更新顺序 |
| [insUpdateENU.m](tools/insUpdateENU.m) | IMU 如何递推姿态、速度和位置 |
| [InertialErrorStateModel.m](state_model/InertialErrorStateModel.m) | 误差状态含义、动力学、量测残差和反馈符号 |
| [ErrorStateKF.m](algorithm/ErrorStateKF.m) | 卡尔曼预测、串行量测更新、协方差与误差状态清零 |
| [SensorDelayCompensator.m](sensor_delay/SensorDelayCompensator.m) | 开启固定延时后，状态如何前推或回推 |
| [FilterResults.m](result/FilterResults.m) | 结果数组、误差定义和 RMSE |
| [ResultPlotter.m](result/ResultPlotter.m) | 轨迹、误差和位置分量图 |

辅助文件中，`createInertialStateProfile.m` 选择状态块组合，`createInertialStateBlock.m` 读取各块的初始标准差，`buildStateLayout.m` 分配索引。它们共同决定实际使用的状态向量。

<a id="running"></a>

## 2. 运行一次程序

GitHub 仓库不附带任何 MAT 文件，包括仿真轨迹、实测输入和计算结果。首次使用时需要先在本地生成轨迹，再运行导航程序；直接运行 `main.m` 会因默认输入文件不存在而报错。下面给出从无数据开始的完整步骤。

### 2.1 第一步：在本地生成仿真轨迹

在 MATLAB 中将当前文件夹切换到含 `main.m` 的项目主目录，然后在命令窗口执行：

```matlab
addpath(genpath(pwd));
genetraj(TrajectoryType="straight", Duration=3600.0, SampleInterval=0.01, ...
    OutputFile="data/input/navigation_input_straight_auv_3600s.mat");
```

该命令生成 3600 s 的直线轨迹，并保存到项目下的 `data/input/navigation_input_straight_auv_3600s.mat`，与主程序默认输入路径一致。输出文件夹不存在时会自动创建。文件包含导航初值、真值以及理想 IMU、DVL、深度和 GPS 数据，可直接作为本项目的仿真输入。

可在命令窗口确认文件已经生成：

```matlab
isfile(fullfile("data", "input", "navigation_input_straight_auv_3600s.mat"))
```

返回 `1` 后继续下一步。生成文件不会自动切换主程序的输入路径。本文显式指定 `OutputFile`，请保持生成路径与后面的 `cfg.data.file` 一致。

### 2.2 第二步：选择刚生成的文件并运行

打开 `config/setConfig.m`，将对应配置项修改为下面的值。请修改原有赋值行，使输入文件只有一个生效的赋值：

```matlab
cfg.data.mode = "simulation";
cfg.data.file = fullfile(cfg.path.inputFolder, "navigation_input_straight_auv_3600s.mat");
cfg.sim.dt = 0.01;
cfg.sim.runs = 1;
cfg.sim.duration = inf;
```

`cfg.sim.duration = inf` 表示使用完整的 3600 s 输入。本例设 `cfg.sim.runs = 1`，即运行一次完整轨迹；需要 Monte Carlo 统计时，再按实验要求增加次数。

保存配置文件，保持 MATLAB 当前文件夹为项目主目录，在命令窗口执行：

```matlab
main
```

本例先使用默认传感器开关，观察纯惯导结果。若要运行 IMU + DVL + 深度组合导航，可在 `setConfig.m` 中将 `cfg.sensor.dvl.isEnabled` 和 `cfg.sensor.depth.isEnabled` 都改为 `true` 后再次运行；上一步生成的文件已经包含这两类量测。GPS 是否开启按实验需要选择。

`main.m` 开始时会清空工作区变量、关闭已有图窗，并将当前目录及子目录加入 MATLAB 搜索路径。配置在脚本内由 `setConfig()` 重新创建，所以在命令窗口预先修改一个 `cfg` 再运行 `main`，不会保留这些修改。使用主脚本时，应在 `setConfig.m` 中设置参数。

当前配置默认值如下；实际运行以本地 `setConfig.m` 为准。

| 配置 | 当前默认值 | 含义 |
| --- | --- | --- |
| `cfg.data.mode` | `"simulation"` | 对输入量测进行仿真加噪 |
| `cfg.data.file` | 第 2.1 节生成的直线轨迹文件 | 路径见上方示例，需自行生成 |
| `cfg.sim.dt` | `0.01` s | 主采样周期，影响噪声换算、时间容差和固定延时 |
| `cfg.sim.runs` | `20` | Monte Carlo 次数 |
| `cfg.sim.duration` | `inf` | 使用完整输入时段 |
| `cfg.algorithm.stateModel.profile` | `"ins9"` | 9 维误差状态 |
| DVL / 深度计 / GPS | 全部关闭 | 不进行外部量测校正 |
| `cfg.sensorDelay.isEnabled` | `false` | 量测不附加总线延时 |
| `cfg.result.fileName` | `"eskf_results.mat"` | 保存文件名 |

> **注意：当前默认运行的是纯惯导传播。** 虽然进入 `ESKF` 流程并预测协方差，但没有外部量测更新，不会自动获得组合导航校正。需要融合时，开启相应传感器并提供对应输入数据。IMU 是必需输入，不能靠将 `cfg.sensor.imu.isEnabled` 改为 `false` 来运行无 IMU 的模式。

运行结束后，工作区保留 `cfg`、`meas`、`results` 和 `plotter` 等变量。结果默认写入 `data/output/eskf_results.mat`；再次使用同一文件名运行会覆盖旧结果。

### 2.3 生成其他轨迹

完成上述直线轨迹运行后，也可以生成同为 3600 s 的割草机测线轨迹。在已执行 `addpath(genpath(pwd))` 的项目主目录中运行：

```matlab
genetraj(TrajectoryType="lawnmower", Duration=3600.0, SampleInterval=0.01, ...
    OutputFile="data/input/navigation_input_lawnmower_auv_3600s.mat");
```

生成后同样需要在 `setConfig.m` 中切换输入文件。更换轨迹参数后，要重新调用 `genetraj` 才会更新 MAT 内容；仅修改轨迹配置不会改变已经生成的数据。

也可以生成定深长方形轨迹：

```matlab
genetraj(TrajectoryType="rectangle", Duration=3600.0, ...
    RectangleLength=450.0, RectangleWidth=200.0, RectangleTurnDuration=5.0, ...
    StraightSpeed=1.0, CourseDeg=0.0, ...
    OutputFile="data/input/navigation_input_rectangle_3600s.mat");
```

`RectangleLength`、`RectangleWidth` 为完整直边的长度和宽度（m），均须为正数。
从原点沿 `CourseDeg` 指定的航向出发，走完一条直边后在顶点停车、原地右转 90°，再走下一条边。
`RectangleTurnDuration` 为每次原地转向的持续时间（s），须为正数；转向时位置不变、平移速度为零。
该类型固定深度为 `InitialDepth`，关闭升沉和滚转激励，使直边只有 RFU 体坐标的前向速度。
原地转向由独立的航向真值驱动，IMU 输出对应角速度；无需设置 DVL 杆臂。
`trajectoryInfo.horizontalInfo.IsTurning` 标记各主采样时刻是否处于原地转向阶段。

轨迹入口为 `data/generate-traj/genetraj.m`，默认选项在 `data/generate-traj/Tools/setTrajectoryOptions.m`。水平轨迹支持 `straight`、`lawnmower`、`rectangle`、`figure8`、`circle`、`sCurve`；深度运动支持 `auvHeave`、`sine`、`constant`、`diveClimb`（`rectangle` 始终定深）。主采样周期和水面参考高度应分别与导航配置中的 `cfg.sim.dt`、`cfg.reference.surfaceAltitude` 一致。

生成器按“时间轴 → 真值轨迹 → 理想传感器 → `inputData`”组织数据。`main.m` 不负责生成轨迹；`cfg.data.generateIdealMeasurement = true` 只会再次检查输入完整性，不会补造缺失量测。

### 2.4 使用自行准备的实测数据

实测 MAT 文件也不随仓库提供，需自行采集或取得数据，并按第 3 节的字段和单位整理。

在 `setConfig.m` 中设置 `cfg.data.mode = "real"`，指定整理好格式的 MAT 文件，并按数据内容选择传感器开关。该模式会在配置函数末尾自动将运行次数设为 `1`，关闭人工传感器加噪、名义初值随机扰动及理想量测检查开关。

实测模式仍使用 `cfg.noise` 构造滤波器的 `Q/R`，仍使用 `initialCovariance` 构造 `P0`。这些参数应描述传感器和初值的不确定度，不能因为“不加人工噪声”就全部清零。

若只有 RTK 位置参考，可只提供 `truth.positionLlh`；但初始速度和姿态仍须通过 `initial` 提供。`truth` 用于初值回退和结果评估，不会自动作为 GPS 量测参与融合；要融合位置数据，应另外提供 `gps.positionLlh` 并开启 GPS。

<a id="input"></a>

## 3. 坐标系、单位与输入格式

### 3.1 全部模块共用的约定

| 量 | 列顺序 / 定义 | 单位 |
| --- | --- | --- |
| 导航系向量 | ENU：东、北、天 | 按物理量确定 |
| 载体系向量 | RFU：右、前、上 | 按物理量确定 |
| `positionLlh` | 纬度、经度、高度 | rad、rad、m |
| `velocityEnu` | 东、北、天速度 | m/s |
| `attitudeEuler` | roll、pitch、yaw | rad |
| `imu.gyro` | RFU 角速度 | rad/s |
| `imu.accel` | RFU 比力，包含静止时对重力的响应 | m/s² |
| `dvl.velocityBody` | RFU 体坐标速度 | m/s |
| `depth.depth` | 深度向下为正 | m |
| 时间 | 所有传感器使用相同时间基准 | s |

`Cbn` 把载体系向量变换到导航系：`vEnu = Cbn * vBody`。反向变换使用转置。欧拉角约定为右侧下沉时滚转为正、抬头时俯仰为正；航向以北为零，顺时针朝东为正。零欧拉角对应单位阵，此时右、前、上分别对齐东、北、天。

输入 IMU 必须是角速度和比力，不能直接填入角增量、速度增量，也不能填入已去除重力的导航系线加速度。输入经纬度和姿态使用弧度；轨迹生成器中名称带 `Deg` 的选项则使用度。

### 3.2 推荐的 MAT 结构

推荐在 MAT 文件中保存名为 `inputData` 的结构体，字段名区分大小写。设主时间轴长度为 `N`，某个外部传感器的样本数为 `M`。

| 字段 | 推荐尺寸 | 是否必需 / 用途 |
| --- | --- | --- |
| `inputData.time` | `N×1` | 必需，主时间轴 |
| `inputData.imu.gyro` | `N×3` | 必需，RFU 角速度 |
| `inputData.imu.accel` | `N×3` | 必需，RFU 比力 |
| `inputData.imu.time` | `N×1` | 可省略；省略时采用主时间轴，建议与主时间轴一致 |
| `inputData.initial.positionLlh` | `3×1` | 初始纬经高，缺省时取位置真值第一帧 |
| `inputData.initial.velocityEnu` | `3×1` | 初始速度，缺省时取速度真值第一帧 |
| `inputData.initial.attitudeCbn` | `3×3` | 初始姿态，可用 `initial.attitudeEuler`（`3×1`）替代 |
| `inputData.initial.gyroBias` | `3×1` | 初始陀螺零偏估计，rad/s；默认零 |
| `inputData.initial.accelBias` | `3×1` | 初始加速度计零偏估计，m/s²；默认零 |
| `inputData.truth.positionLlh` | `N×3` | 可选，位置参考 |
| `inputData.truth.velocityEnu` | `N×3` | 可选，速度参考 |
| `inputData.truth.attitudeCbn` | `3×3×N` | 可选，姿态参考；也可提供 `truth.attitudeEuler`（`N×3`） |
| `inputData.dvl.velocityBody` | `M×3` | 开启 DVL 时必需 |
| `inputData.depth.depth` | `M×1` | 开启深度计时必需 |
| `inputData.gps.positionLlh` | `M×3` | 开启 GPS 时必需，纬经高 |
| 各外部传感器的 `time` | `M×1` | 当量测行数不同于 `N` 时必须提供 |
| 各外部传感器的 `valid` | `M×1` logical | 可选，默认全有效；`false` 表示该行不参与匹配 |

不同传感器可以有不同的 `M`。提供的真值必须已经对齐主时间轴，代码不对真值做插值。主时间轴应递增，IMU 应连续覆盖；为使噪声和固定延时含义一致，宜采用与 `cfg.sim.dt` 一致的均匀采样。

初始位置、速度、姿态缺项时分别从对应真值第一帧回退，最终三者都必须可用。姿态同时提供矩阵和欧拉角时优先使用矩阵；初始姿态均未提供时，尝试使用姿态真值第一帧。单个 `3×3` 的真值姿态矩阵会被扩展到全部时刻，表示固定姿态。

可选元数据 `inputData.metadata.navigationFrame` 和 `bodyFrame` 应分别为 `"ENU"`、`"RFU"`。代码会检查声明的坐标系，但不会自动把其他坐标系的数据转换过来。当前 IMU 读取器不使用输入的 `imu.valid` 屏蔽数据；应在导入前处理 IMU 缺测。

读取器也兼容以 `gt_mes`、`gtMeas`、`data` 包装的结构体，以及直接保存上述字段的 MAT 文件；包装名称兼容不代表内部任意字段格式都兼容。

### 3.3 PSINS 轨迹转换入口

`data/convertTrjMeasuredToInput.m` 接收源文件和输出文件两个路径。源 MAT 必须含有符合该函数要求的 `trj`，包括 `imu`、`avp`、`avp0`、`ts`。

该函数把 PSINS IMU 增量除以采样间隔，转换为本程序使用的角速度和比力，并把源姿态 `[pitch, roll, yaw]` 转为本项目的 `[roll, pitch, -yaw]`。它在时间轴前补入初始状态，并保留源 `trj`。输出中的 DVL、深度和 GPS 块为空，不能仅开启开关就获得这些量测。

<a id="configuration"></a>

## 4. 常用配置如何影响运行

### 4.1 时段、传感器与随机数

| 参数 | 使用说明 |
| --- | --- |
| `cfg.sim.duration` | 正数表示从输入首时刻起保留指定秒数，截断有约半个主采样周期的容差；`inf` 或 `[]` 表示完整输入。至少需要保留两个主时间样本 |
| `cfg.sensor.dvl.isEnabled` | 开启后融合 RFU 速度 |
| `cfg.sensor.dvl.leverArmBody` | INS 安装点指向 DVL 安装点的矢量，在 INS RFU 三轴下表达，`[右; 前; 上]`，单位 m；默认零 |
| `cfg.sensor.depth.isEnabled` | 开启后融合深度，预测值为水面参考高度减当前高度 |
| `cfg.sensor.gps.isEnabled` | 开启后融合位置；输入虽为纬经高，残差与噪声协方差均以 ENU 米制表示 |
| `cfg.sensor.dvl.availableTime` | 每行一个闭区间 `[开始, 结束]`，仅允许这些时段的量测 |
| `cfg.sensor.gps.availableTime` | 同上，例如 `[0, 60; 300, 360]` 表示两段可用时段 |
| `cfg.reference.surfaceAltitude` | 水面参考高度，必须与输入高度采用同一基准 |
| `cfg.measurement.timeTolerance` | 主时间轴与传感器采样时刻的匹配容差，默认 `0.5 * cfg.sim.dt` |

`availableTime` 使用输入时间轴本身的数值，不会自动减去首时刻。空矩阵表示不限制时段，不能用它表示“全程不可用”；全程关闭应使用传感器开关。深度计不使用上述时间窗配置，可用 `depth.valid` 筛选。

随机数由 `main.m` 中的 `randomselect` 控制。`'random'` 生成新随机状态并保存到根目录 `randstates.mat`；`'repeatlast'` 恢复该文件中的状态。若文件不存在，会警告并新建状态。复现还需要保持输入、参数及随机数调用流程一致。

### 4.2 噪声、初始扰动与初始协方差

这三组参数的作用不同：

- `cfg.noise` 描述传感器误差。仿真模式用它向理想量测加噪，滤波器也用其中的连续噪声密度和量测标准差构造 `Q/R`。
- `cfg.algorithm.initialPerturbation` 随机改变每次运行的名义初始导航解和可选零偏估计。
- `cfg.algorithm.initialCovariance` 描述滤波器对初始误差的认识，按状态顺序形成 `P0 = diag(std.^2)`，不会直接改动输入初值。

配置中的标准差是 1σ。初始姿态扰动和协方差描述 ENU 三轴小失准角，不是 roll、pitch、yaw 的逐项扰动。

IMU 工程参数在 `setConfig.m` 中转换为 SI 单位：陀螺常值零偏用 deg/h，加速度计常值零偏用 micro-g；陀螺白噪声用 deg/√h，加速度计噪声密度用 micro-g/√Hz。逐点加噪标准差等于连续噪声密度除以 `sqrt(dt)`。修改上游工程参数后应重新执行配置函数，避免只修改上游字段而遗漏已计算的派生字段。

### 4.3 选择 9 维或 15 维状态

在 `cfg.algorithm.stateModel.profile` 中选择 `"ins9"` 或 `"ins15"`。

| 状态块 | 索引 | 坐标系与单位 | 所属模型 |
| --- | --- | --- | --- |
| `Attitude` | 1～3 | ENU 小失准角，rad | 两者都有 |
| `Velocity` | 4～6 | ENU 速度修正量，m/s | 两者都有 |
| `Position` | 7～9 | ENU 位置修正量，m | 两者都有 |
| `GyroBias` | 10～12 | RFU 陀螺零偏修正量，rad/s | `ins15` |
| `AccelBias` | 13～15 | RFU 加速度计零偏修正量，m/s² | `ins15` |

`ins9` 仍会减去输入的初始零偏估计，但不会继续估计或更新零偏。`ins15` 会把零偏修正反馈到后续 IMU 补偿。

仿真加噪也受模型选择影响：只有包含相应零偏状态块时，才注入该传感器的随机常值零偏；白噪声仍照常加入。因此当前代码中切换 `ins9` / `ins15` 同时改变了滤波状态和仿真零偏注入条件。常值零偏每次 Monte Carlo 重新抽取，单次运行内保持不变；当前过程噪声没有额外的零偏随机游走驱动项。

<a id="workflow"></a>

## 5. 主程序与数据流

```text
main.m
  ├─ ControlRandomNumber：建立或恢复随机状态
  ├─ setConfig：读取配置
  ├─ StateAndMeasurement：加载 MAT → 截取时段 → 检查输入
  ├─ FilterResults：创建结果容器
  ├─ ESKF：每次 MC 加噪与初始化 → 逐时刻传播、更新、反馈
  ├─ computeErrors：按可用真值计算误差与 RMSE
  ├─ saveToMat：保存结果、误差与配置
  └─ ResultPlotter：绘制轨迹、误差与位置分量
```

阅读变量时，可以用下面的对应关系区分原始数据与本次解算数据：

| 变量 / 属性 | 内容 |
| --- | --- |
| `cfg` | 本次运行的完整配置 |
| `meas.GtMeas` | 导入并规范化、截取后的数据；实测模式下也使用此名称，不意味着全是理想真值 |
| `meas.CurMeas` | 当前 MC 的量测、有效标记与时间索引；下一次 MC 会覆盖 |
| `navSol` | 当前名义导航解：`PositionLlh`、`VelocityEnu`、`Cbn` |
| `imuBias` | 当前名义陀螺和加速度计零偏估计 |
| `fil` | 误差状态均值、协方差及其卡尔曼递推 |
| `results.Data.ESKF` | 各时刻、各次 MC 的导航解与零偏估计 |
| `results.Error.ESKF` | 对应的评估误差和跨 MC 的 RMSE |

输入字段通常以小写开头，例如 `positionLlh`；读入后的对象字段通常以大写开头，例如 `PositionLlh`。查找某个量时需要区分这两套命名。

<a id="filter-step"></a>

## 6. 一个无延时时间步做了什么

`ESKF.m` 外层循环遍历 MC，内层从第 2 个主时间样本开始。第 1 帧保存初始导航解，无延时分支不会用第 1 行 IMU 做递推。

1. **建立本次初值。** `prepareMonteCarloRun` 准备量测，`CreateInitialNavigation` 读取初值、按配置扰动，并生成 `P0`。
2. **保存上一时刻滤波量。** `Preparation` 把当前误差状态和协方差作为新一步的预测起点。
3. **递推名义导航解。** `PropagateNavigation` 先从 IMU 减去当前零偏，再调用 `insUpdateENU`。后者考虑地球自转、运输角速度、重力与曲率半径，更新姿态、速度和纬经高，并将姿态矩阵重新正交化。
4. **预测误差状态和协方差。** `BuildErrorDynamics` 提供 `F`、`G`、`Qc`；`Predict` 使用一阶离散化 `Phi = I + F*dt`、`Qd = G*Qc*G'*dt`。
5. **串行融合有效量测。** 按 DVL → 深度 → GPS 的顺序更新。同一时间步允许多种量测参与，后一个更新继续使用前一个更新后的状态和协方差。
6. **闭环反馈。** `FeedbackNavigation` 把估计修正量加到名义速度、位置和可选零偏中，以左乘小角度矩阵修正姿态。
7. **清零并保存。** `ResetErrorState` 清零误差状态均值，保留协方差；当前导航解写入历史数组。

### 6.1 三种量测的残差

| 量测 | 预测量 / 残差 | 直接进入量测矩阵的状态 |
| --- | --- | --- |
| DVL | 预测 RFU 速度为 `Cbn' * VelocityEnu + cross(correctedImu.Gyro, leverArmBody)`；残差为实测减预测 | 姿态、速度；非零杆臂时还包括 `ins15` 的陀螺零偏 |
| 深度 | 预测深度为 `surfaceAltitude - height`；残差为实测减预测 | 位置的天向分量，系数为 `-1` |
| GPS | `llh2enuError(实测位置, 当前导航位置)` 得到米制 ENU 残差 | ENU 位置 |

误差状态代表“真值相对当前导航解的修正量”，所以反馈采用加性修正。评估误差则是“估计减真值”，两者符号含义不同。

### 6.2 DVL 已知安装杆臂

在 `config/setConfig.m` 中填写 `cfg.sensor.dvl.leverArmBody`，并开启 DVL。例如
`[0.2; 1.0; -0.3]` 表示 DVL 位于 INS 右侧 0.2 m、前方 1 m、下方 0.3 m。
安装角仍假设为零，杆臂是已知常量，不新增滤波状态；导航输出仍属于 INS 安装点。
输入 `dvl.velocityBody` 应为 DVL 安装点的对地速度，以 INS RFU 表达。
若设备已经把速度补偿到 INS 安装点，应设置零杆臂，避免重复补偿。

杆臂速度使用零偏补偿后的 `correctedImu.Gyro`，即 `omega_ib^b`，近似代替
`omega_eb^b`。仅此处忽略地球自转；1 m 杆臂引入的最大速度近似误差约为
0.073 mm/s。机械编排和误差状态传播仍保留地球自转。该近似下，DVL 雅可比为
`H_attitude = Cbn' * skew(VelocityEnu)`、`H_velocity = Cbn'`，位置和加速度计零偏块为零；
`ins15` 另有 `H_gyroBias = skew(leverArmBody)`。

非零杆臂时，`BuildDvlMeasurement` / `UpdateDvl` 需要额外的 `correctedImu` 输入。
量测协方差加入 `skew(l) * diag(gyroStd.^2) * skew(l)'`，使用离散角速度白噪声标准差；
当前标准 KF 仍忽略共用 IMU 导致的过程/量测噪声相关性。零杆臂保留原残差、H、R 和调用方式。

仿真时，在 `data/generate-traj/Tools/setTrajectoryOptions.m` 中设置真值
`DvlLeverArmBody` 并重新运行 `genetraj`；旧 MAT 数据不会自动获得杆臂速度。
生成器采用真实对地角速度 `omega_eb^b` 生成 DVL 安装点速度，因而也可以检验滤波中忽略
地球自转的近似。真值杆臂保存在 `inputData.metadata.dvlLeverArmBody`；滤波杆臂保存在
结果 `StateModel.DvlLeverArmBody`。两者分别配置，正常补偿试验应取相同值。

### 6.3 对比补偿与不补偿

独立实验入口不会修改当前配置或已有输入文件：

```matlab
addpath(genpath(pwd));
report = compareDvlLeverArm(Duration=120, Runs=5);
% 也可明确指定试验杆臂，例如：
% report = compareDvlLeverArm(LeverArmBody=[1; 0; 0], Duration=120, Runs=5);
```

默认采用当前 `setConfig()` 的杆臂、状态 profile 和噪声参数；试验统一关闭总线延时、
GPS 和深度量测，只融合 DVL。生成 2 m/s 的平直航行与割草机轨迹，后者直线段 40 m、
转弯半径 10 m。每种轨迹比较 A：零杆臂数据/零补偿，B：非零杆臂数据/零补偿，
C：与 B 相同的数据/正确杆臂补偿。无噪声试验用精确初值，含噪声试验重置相同种子进行配对 MC。
因此 B/C 的传感器数据与初值扰动相同，不应通过分别更改生成器杆臂来比较“补偿开关”。

每次运行在 `data/output/dvl_lever_arm_时间戳/` 保存 4 张对比图、`summary.csv`、
完整 `comparison.mat` 及两种轨迹的零/非零杆臂输入。表格 RMSE 的定义是先对三轴误差平方求和，
再对时间和 MC 平均后开方；图中曲线是在每个时刻跨 MC 的三维 RMSE。轨迹图只展示第 1 轮。
同时绘制 DVL 杆臂速度及真值状态下补偿后的量测残差；后者不等于闭环滤波新息，
可用于检查模型是否遗漏了转动速度，避免滤波器把模型错误吸收到速度、姿态或零偏中后掩盖问题。
补偿后的残差通常保留忽略地球自转造成的小量，不应要求严格为零。

仅改变滤波杆臂不会给旧 MAT 数据添加真实杆臂效应。应先检查数据生成配置和
`inputData.metadata.dvlLeverArmBody`；对已有理想数据，也可检查
`dvl.velocityBody - Cbn' * truth.velocityEnu` 是否包含预期的角速度叉乘项。

`KalmanUpdate` 使用 `innovation = residual - H*CurX`，扣除本时刻前序量测已经估计出的部分；协方差采用 Joseph 形式更新并对称化。清零误差均值是为了避免下一步重复反馈，不表示不确定度归零。

<a id="sensor-delay"></a>

## 7. 异步量测与固定延时

### 7.1 量测何时进入滤波

`StateAndMeasurement` 为各传感器建立“主时间样本 → 传感器样本”的索引。匹配受 `valid`、DVL/GPS 可用时间窗和 `timeTolerance` 控制。没有匹配量测时返回 `Valid = false`，该种量测更新被跳过。

这套机制不是对低频数据做插值或持续保持；每个主时间步、每种传感器只对应一个索引。输入时间戳应与主时间轴合理对齐，不宜通过任意增大容差来代替时间同步。

### 7.2 打开总线延时后

设置 `cfg.sensorDelay.isEnabled = true` 后，采样索引会向后平移 `delaySteps` 个主时间步。各传感器的步数当前只允许 `0～3` 的整数，实际补偿时间按 `delaySteps * cfg.sim.dt` 计算。默认步数为 IMU 3 步、其他传感器 2 步，在 0.01 s 周期下对应 30 ms 和 20 ms。

量测对象通过 `SampleTime`、`ArrivalTime`、`DelaySeconds` 区分采样与到达。平移后超出输入末尾的量测不会再参与运行。延时分支开头若 IMU 尚未到达，会保持导航解并跳过该步滤波更新。

`ESKF` 维护一个滞后导航解 `laggedNavSol`，收到 IMU 后按其采样间隔传播，再由 `forwardImu` 一阶外推位置、速度到当前计算时刻。DVL 更新回推速度，深度更新回推高度，GPS 更新回推位置和速度；这些临时状态用于构造对应采样时刻的残差。

非零 DVL 杆臂时，临时 DVL 姿态按 `dvl.SampleTime - imu.SampleTime` 从滞后姿态推算，
姿态传播仍考虑地球自转和运输角速度。角速度使用最新已到达的 IMU 值作短时保持，
不访问尚未到达的数据；默认 IMU 延时 30 ms、DVL 延时 20 ms 时，需要约 10 ms 的外推。
零杆臂保留原有不推算姿态的行为。上述处理未重放历史协方差，也未对临时状态额外映射
完整延时雅可比，是短固定延时的一阶近似，不能等同于任意大延时、快速角加速度或乱序量测处理。
反馈时同一导航修正还会应用到滞后状态，使后续传播保留本步修正。

<a id="results"></a>

## 8. 读取结果与理解图形

### 8.1 保存了哪些变量

默认输出 MAT 包含三个变量：`resultData`、`errorData`、`cfg`，分别对应 `results.Data`、`results.Error` 和运行配置。它不保存 `meas`、原始输入、主时间向量、完整协方差历史或逐时刻新息。

`resultData.ESKF` 的主要数组尺寸为 `N×3×runs`：第 1 维是时间，第 2 维是分量，第 3 维是 MC 次数。

| 字段 | 内容与单位 |
| --- | --- |
| `PositionLlh` | 纬度、经度、高度，rad、rad、m |
| `VelocityEnu` | ENU 速度，m/s |
| `Euler` | roll、pitch、yaw，rad |
| `GyroBias` | RFU 名义陀螺零偏估计，rad/s |
| `AccelBias` | RFU 名义加速度计零偏估计，m/s² |
| `PositionCovarianceEnu` | ENU 位置协方差，m²，`3×3×N×runs`，保留轴间相关项 |
| `StateModel` | 实际 profile、维数、状态块名称、索引及初始标准差等元数据 |

`GyroBias` 和 `AccelBias` 是用于补偿的估计值，并非仿真注入的真实零偏。`ins9` 也保存这两个数组，但值保持为其输入的零偏估计。

位置协方差第一帧取 `P0` 的位置状态块，后续与完成量测更新和闭环反馈的导航结果配对。延时模式下，尚未收到有效 IMU、未执行滤波推进的时刻保留为 `NaN`。

运行 `main` 后，可直接查看第一轮结果与主时间轴：

```matlab
time = meas.getTime();
positionLlh = results.Data.ESKF.PositionLlh(:, :, 1);
velocityEnu = results.Data.ESKF.VelocityEnu(:, :, 1);
attitudeDeg = rad2deg(results.Data.ESKF.Euler(:, :, 1));
```

重新打开输出文件时：

```matlab
savedResults = load(fullfile("data", "output", "eskf_results.mat"));
positionLlh = savedResults.resultData.ESKF.PositionLlh(:, :, 1);
stateModelInfo = savedResults.resultData.ESKF.StateModel;
```

若需要准确的时间轴，应保留本次 `meas.getTime()` 或用保存的配置重新加载同一份输入并执行相同的时段截取，不要默认用 `0:dt:duration` 替代非零起始时间的输入。

### 8.2 误差与 RMSE 的含义

| `errorData.ESKF` 字段 | 含义 |
| --- | --- |
| `HasPositionTruth` | 是否存在位置参考 |
| `HasVelocityTruth` | 是否存在速度参考 |
| `HasAttitudeTruth` | 是否存在姿态参考 |
| `PositionEnu` | 估计位置相对真值的 ENU 误差，m，`N×3×runs` |
| `VelocityEnu` | 估计速度减真值，m/s，`N×3×runs` |
| `AttitudeMisalignment` | 由 `C_est * C_truth'` 提取的 ENU 旋转矢量，rad，`N×3×runs` |
| `AttitudeEuler` | 与 `AttitudeMisalignment` 保存相同数据，名称不代表欧拉角相减 |
| `PositionComponentRmse` 等 | 每个时刻、每个分量跨 MC 的 RMSE，`N×3` |
| `PositionRmse` 等 | 每个时刻三轴误差平方和跨 MC 平均后开方，`N×1` |
| `PositionSigma` | ENU 位置标准差，m，`N×3×runs`，由协方差对角项开方得到 |
| `PositionNees` | 各次运行的三维位置联合 NEES，无量纲，`N×runs` |
| `PositionMeanNees` | 每个时刻跨有效 MC 的平均位置 NEES，`N×1` |

例如位置总 RMSE 为 `sqrt(mean(eEast.^2 + eNorth.^2 + eUp.^2, MC维))`。这是随时间变化的统计量，不是对整段时间求平均得到的单个数。只有一次运行时，分量 RMSE 等于该分量误差的绝对值，总 RMSE 等于三维误差模长。

有哪类真值，就计算哪类误差。只有位置真值时，位置指标可用，速度和姿态对应数组为 `NaN`；完全没有真值时，会警告并跳过误差计算，导航结果仍可保存。

位置 NEES 使用每次运行的带符号 ENU 误差与对应的完整位置协方差计算 `e'*(P\e)`，再跨 MC 求平均。其理论期望值为 3，与完整状态是 9 维还是 15 维无关。缺失、非有限或非正定协方差对应的 NEES 为 `NaN`，不会以伪逆或额外正则项替代。

`main` 最后通过 `plotter.printPositionStatistics()` 打印各算法的位置平均 RMSE（m）和位置平均 NEES（无量纲）。`PositionRmse` 已在每个时刻跨 MC 求均方后开方，`PositionMeanNees` 已在每个时刻跨 MC 求平均；打印方法分别对两条曲线沿时间求算术平均，并忽略 `NaN` 样本。没有位置参考的算法跳过打印。

### 8.3 图窗显示什么

- 水平轨迹图：真值与第 1 次 MC 导航解的东—北投影；无位置真值时仍可画导航轨迹。
- 位置、速度、姿态误差图：分别展示跨 MC 的三轴分量 RMSE，不是某一次的带符号误差曲线。
- 姿态图：绘图时转换为度；虽然标签写滚转、俯仰、航向失准角，底层数据仍是 ENU 旋转矢量分量。
- `Position RMSE`：三维位置总 RMSE。
- `ESKF Position NEES`：单次位置 NEES 或多次 MC 的平均 NEES，以及理论期望值 3。
- `ESKF Position Error and 3 Sigma`：第 1 次运行的东、北、天带符号位置误差及同一次运行的 ±3σ。
- 位置分量图：第 1 次 MC 解算位置和真值各自的 ENU 分量，以真值首位置为参考原点；无位置真值时跳过。

这些绘图调用只创建图窗，主程序没有自动保存 PNG 或 FIG 的步骤。需要检查带符号的单轮误差时，应读取 `results.Error.ESKF.PositionEnu(:, :, 1)` 等原始误差数组。

两个新增图窗由 `main` 自动调用，也可单独调用；指定其他运行时，误差和标准差会一起切换：

```matlab
plotter.plotPositionNees();
plotter.plotPositionError3Sigma();       % 默认第 1 次运行
plotter.plotPositionError3Sigma(2);      % 至少有 2 次 MC 时查看第 2 次
```

新方法支持返回图窗句柄。无位置参考时会警告并跳过；旧 MAT 结果缺少位置协方差历史时，需要重新运行 ESKF。

<a id="faq"></a>

## 9. 常见使用问题

| 现象 | 对照检查 |
| --- | --- |
| 首次运行提示找不到输入文件 | 仓库不附带 MAT 文件；先按第 2.1 节调用 `genetraj`，再按第 2.2 节设置 `cfg.data.file` |
| 启动后提示缺少 DVL、深度或 GPS | 已开启的传感器必须有对应量测；检查字段名称和数组是否为空 |
| 提示缺少初始导航解 | 确保位置、速度、姿态均能从 `initial` 或对应真值首帧获得 |
| 量测长度与时间不一致 | IMU 和真值应与主时间轴同长；低频传感器需提供自身 `time` |
| 开启传感器但没有明显校正 | 检查有效标记、时间匹配、可用时间窗及是否真的进入了有效量测更新 |
| 纬经高、航向或深度明显异常 | 检查弧度/角度、RFU/ENU、航向正方向以及高度/深度符号 |
| 在命令窗口改了配置却未生效 | `main` 会清空变量并重新调用 `setConfig()` |
| `ins9` 没出现设置的 IMU 常值零偏 | 当前仿真只对包含对应零偏状态块的 profile 注入常值零偏 |
| 无速度或姿态误差图 | 检查是否提供了该类真值；位置参考不等于完整导航真值 |
| 运行时间或结果体积过大 | 先缩短 `cfg.sim.duration`、减少 `cfg.sim.runs`；结果数组随样本数和运行次数增长 |
| 重复运行结果不同 | 检查 `randomselect`、`randstates.mat`、输入和配置是否保持一致 |
