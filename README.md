# scan_match

面向 BK7258/toflib 的纯 C99 ToF 扫描匹配器，附电脑日志回放和可视化。
输入是**预测位姿、栅格地图、扫描点云**；输出匹配位姿、修正量、评分及图片。
Python 负责日志读取和绘图，C 核心负责匹配。只使用 CMake 构建；只保留 Point2D/四层原生实现，不依赖 ROS。

## 1. 文件结构

```text
scan_match/
├── CMakeLists.txt           # 唯一构建入口
├── README.md                # 编译、使用、算法和阅读说明
├── requirements.txt         # Python 依赖：numpy、matplotlib
├── .gitignore               # 排除构建文件、输出数据和缓存
├── cli/
│   └── main.c               # PC 程序入口：读输入、调用核心、打印 JSON
├── core/
│   ├── scan_match.h         # 输入输出结构体、单位、接口
│   └── scan_match.c         # 评分场、搜索、精修和结果判断
├── adapters/
│   ├── toflib_adapter.h     # 固件桥接接口、缓冲区和同步约定
│   └── toflib_adapter.c     # 地图零拷贝视图、逐点坐标转换和匹配包装
├── tools/
│   ├── log_data.py          # 日志解析、地图/点云导出、坐标变换模块
│   ├── simulate.py          # 唯一日常脚本入口：提取、画图、可选运行 C 匹配
│   └── report.html          # 人工查看的结果页模板（由脚本自动填入数据）
└── tests/                   # 可选开发验证，不参与默认编译
    ├── test_core.c          # 搜索正确性、精修、歧义及内存边界测试
    ├── test_toflib_adapter.c # 转置/偏移/采样位姿、拒绝结果、锁释放测试
    ├── check_toflib_replays.py # 日志经真实toflib结构体后的匹配一致性
    ├── check_native_regression.py # 全量日志优化回归和人工报告
    ├── test_pipeline.py     # 日志关联、坐标变换和 CLI 输入测试
    └── check_replays.py     # 已导出日志样本的枚举/BnB 一致性检查
```

`build*/` 是编译产物，`output/` 是生成的数据和图片，都不属于源代码。
已有样本保留在 `output/review/`，可打开 `index.html` 查看。

### 本次整理

- 删除根目录 Makefile 和 quickstart.sh，统一用 CMake 编译，再手动运行 Python 脚本。
- CODE_REVIEW.md、VALIDATION.md、REQUIREMENTS.md 的必要说明合并到本 README。
- 将日志一致性验证脚本从 tools/ 移入 tests/。
- 保留两个 Python 文件，避免把数据解析与绘图混成一个大文件；日常只调用 simulate.py。
- 测试从运行角度可以删除，但建议保留：后续修改剪枝、坐标或精修时可检测回归，默认不构建，也不增加板端内存。
- requirements.txt 不是运行时代码，但便于安装依赖；.gitignore 避免将大量产物提交到仓库。

## 2. CMake 编译

环境：C99 编译器、CMake 3.16+、make 或 Ninja。**只编译 C 程序不需要 Python。**

按熟悉的 `cmake ..` 方式，在项目根目录执行：

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF
cmake --build . --parallel
cd ..
```

输出 `build/scan_match` 和 `build/libscan_match_core.a`。
Debug 适合单步调试；测性能时将 Debug 换成 RelWithDebInfo 再配置、编译。
以后只改 C 代码，执行 `cmake --build build --parallel` 即可。

IDE 可使用构建目录的 `compile_commands.json`。只编译核心库时增加：

```bash
cmake -S . -B build-core -DSCAN_MATCH_BUILD_CLI=OFF -DBUILD_TESTING=OFF
cmake --build build-core --parallel
```

## 3. 从日志提取地图与点云，不运行匹配

Python 3.8+；若缺依赖，可在自己的 Python 环境中执行：

```bash
python3 -m pip install -r requirements.txt
```

以下命令均在项目根目录执行：

```bash
# 列出日志匹配记录，编号从 0 开始；一次扫描重试可能产生多条记录
python3 tools/simulate.py ../ReceivedTofile-COM21-2026_9_25_15-17-14.DAT --list

# 只提取指定场次，输出地图、点云、预测位姿和输入叠加图，无需 C 可执行程序
python3 tools/simulate.py ../ReceivedTofile-COM21-2026_9_25_15-17-14.DAT \
  --matches 13,19 --extract-only --out output/extracted
```

默认打开 **`output/extracted/index.html`**，或者单场目录内的 **`index.html`**，不必阅读 JSON/CSV。

页面上方对比原日志和新算法的状态、得分/阈值、预测位姿、修正量和候选位姿；统一使用 **cm 和 °**。
下方两张地图支持同步滚轮缩放、拖动平移、鼠标坐标查看，以及地图、原始/匹配点云和位姿开关。
右图可切换“新算法候选”与“原日志候选（估算）”。纯提取时显示新算法未运行。
原日志修正后的点云由输入点云与日志修正重建，不冒充固件实际输出或实际已应用的位置。

默认只显示结果表和地图点云。源行、警告、各项诊断、评分面和原始文件链接都放在“详细调试信息”，**默认折叠**。
页面数据内嵌，不访问网络，浏览器直接打开即可。`tools/report.html` 是模板，不是结果入口。
如需静态图片，可在详细信息中打开 `raw_map.png` 或 `overlay.png`。

| 输出 | 含义 |
|---|---|
| `map.npy` | 原始地图数组，按 `[y,x]` 排列，uint8 |
| `map_cells.csv` | 地图 i/j、数值、日志源行，可直接人工检查 |
| `observed.npy` | 哪些格子确实被打印，区别缺失记录与显式未知 |
| `scan.csv` | 参考机器人坐标系的端点 x/y、射线原点 x/y，单位米 |
| `raw_sp.csv` | 日志扫描字段及源行，转换为米和弧度 |
| `bundle.json` | 预测位姿、旧匹配结果、地图信息、来源与警告 |
| `damaged_map_rows.json` | 无法完整解析的疑似地图记录，不猜补 |
| `scene.txt` | C 程序可直接读取的完整地图、预测位姿和点云 |
| `raw_map.png` | 原始地图与输入点云叠加图 |

不指定 --out 时，纯提取默认写到 output/extracted，匹配模式写到 output/review。
重复运行会覆盖同名输入/结果文件；更换实验配置时建议使用新的输出目录。

## 4. 运行匹配与画前后对比图

```bash
# 编译后，一次完成提取、匹配和画图
python3 tools/simulate.py ../ReceivedTofile-COM21-2026_9_25_15-17-14.DAT \
  --matches 1,4,13,19,35,36 --out output/review

# 其他构建目录用 --executable 指定程序
python3 tools/simulate.py ../ReceivedTofile-COM21-2026_9_25_15-17-14.DAT \
  --matches 13 --executable ./build-debug/scan_match --method bnb --out output/bnb
```

匹配模式另外输出：

- `result.json`：候选位姿、修正量、评分、接受状态、原因、内存及 PC 耗时。
- `overlay.png`：匹配前后点云，橙色为输入，蓝色为最佳候选；拒绝时也画候选。
- `scores.csv` / `landscape.png`：候选评分及平移热力图、角度曲线；BnB 模式仅显示访问过的叶子。
- `run.json`：实际程序参数与退出状态。
- 输出目录根部的 `index.html` / `summary.csv`：多个场次的图表与结果汇总。

只运行 C，不重新提取或画图：

```bash
./build/scan_match output/extracted/match_013/scene.txt \
  --method exhaustive --refine 0
```

日常脚本默认使用BnB；需要完整评分热力图时显式传`--method exhaustive`。

常用参数：`--window 1.2`（平移半范围，米）、`--angle-deg 15`、`--angle-step-deg 0.5`、
`--sigma 0.18`（评分容差，米）、`--wall-threshold 151`、`--refine 25`。
Python `--sensor-offset 0.142` 调整重建射线原点的前置外参；它不再次给端点加外参。

## 5. 怎么读 C 代码

1. **core/scan_match.h**：原生版 SmPoint 是 toflib 的 Point2D；SmPose 保留 double 保证精修精度。再看地图、配置和结果。
2. **cli/main.c 的 main()**：读文件 → 分配内存 → sm_build_field → sm_match → 打印结果。
3. **core/scan_match.c**：先读 sm_build_field、sample/sm_score、sm_match、search_pass、leaf。
4. 再读 visit（分支定界），最后读 refine/solve3（LM 精修）和 diagnostics（诊断）。

第一遍运行 `--method exhaustive --refine 0`，先理解一个候选如何产生、如何评分。
可在 sm_match、leaf、refine 设置断点，观察 prediction、score、best_score、candidates 和 reasons。
leaf 会执行多次，适合使用条件断点。

单个点的变换链：**机器人参考坐标 → 候选角度旋转 → 候选平移 → 地图坐标 → 栅格坐标 → 查评分场**。
核心单位为米/弧度，地图数组为 `cells[y*width+x]`，origin 表示格子中心。

## 6. 算法与参数边界

- 已知且原地图值 >151 视为障碍，128 为未知。原值不直接当作 `value/255` 的概率。
- 最近障碍物距离 d 转为 `round(255*exp(-d²/(2σ²)))`，默认 σ=0.18 m、截断 0.60 m。
- 双线性插值后对全部点平均；未知、越界点仍计入分母。未知格中心零分，插值边界可含相邻已知格贡献。
- 默认完整搜索 x/y ±1.2 m、角度 ±15°；不乘平移/角度惩罚，不因达到阈值提前返回，不锁死坐标轴。
- 平移步长等于地图分辨率，实际半范围为 `floor(window/resolution)*resolution`；角度均分并包含端点和零修正。
- 枚举与 BnB 使用同一离散候选集合。四层滑动窗口 max 给出安全上界；等分按固定字典序选择。
- 最多保留 3 个不同候选，排除前面候选邻域后重新搜索；默认区别为平移至少 0.36 m 或角度至少 5°。
- LM 精修使用 `sqrt(1-score)` 残差，只接受实际分数提高的步骤，不越过搜索范围或已排除邻域。
- 分差只对保留候选有效，不是全局统计置信度；连续精修不保证连续空间全局最优。
- 射线穿障比例只作诊断，不参与默认接受判定。

默认接受条件：score≥0.55、已知比例≥0.50、近障碍比例≥0.50（对应距离0.18 m）、
有备选时分差≥0.02、至少20点、最佳结果不在搜索边界。阈值尚未标定；没有备选时 margin=null，不证明无歧义。

`accepted` 是启发式判断，不是定位真值。看图时检查不同方向的墙是否同时对齐，再看评分面是否多峰、呈长带或贴边。

## 7. 日志约定与已验证样本

- 地图每格 12 cm：`x=(i-128)*0.12`、`y=(j-128)*0.12`，尺寸256×256。
- sp 的 x/y/distance 为 cm，theta 为百分之一弧度，这是旧工具提供的字段约定。
- 端点按 `[x,y]+distance*[cos(theta),sin(theta)]` 重建；distance 按已含前置外参解释。
- 各点使用自己的采样位姿重建后，逆变换到统一参考机器人坐标系。
- 预测位姿取扫描结束前800个逻辑行内最后一条 g_pose_20ms，是代理值。
- sp_g 分段；匹配结果关联前3000行内最后结束的段，是行距启发式而非严格时间同步。
- 使用最后一次地图 dump，完整记录才入图；缺失和未知在核心中都当未知，原始掩码单独保留。

已回放全部38次记录中的37次；场次27有效测距不足3个，无法回放。10次失败来自2个扫描片段的重试，不是10个独立样本。失败段只有约87°，并非完整一圈；1、35接近整圈。
枚举/BnB 的离散最佳值、精修候选及判定一致；没有位置真值，因此不能将通过数量当作准确率。
旧程序分数与新分数定义不同，不能直接比较绝对值。原始数据、已有报告均保留在 output/。

## 8. 可选测试与后续移植

```bash
# 独立测试目录，日常 build 不需要 Python
cmake -S . -B build-test -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build-test --parallel
(cd build-test && ctest --output-on-failure)

# 已有样本的两种搜索一致性；默认使用 build/scan_match
python3 tests/check_replays.py output/review

# 需要内存检查时配置新目录，再编译并运行 ctest
cmake -S . -B build-sanitize -DBUILD_TESTING=ON -DSCAN_MATCH_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
```

测试包含已知位姿恢复、非格点精修、随机地图枚举/BnB一致性、长墙歧义、未知/越界、坐标往返及日志关联。

## 9. 作为 toflib 第四种匹配方法

本次接入只替换 `tryMatchEx()` 中的搜索部分。保留原有 `GridPoint`、
`tof_response_t`、`g_finalCorrected`、`fillMap()` 和 `applyMatchResult()` 接口。
`defer_apply`、未知区域强制插图保护和原25点门槛沿用旧流程。
完整重定位等其他入口不受这个选择器影响。

```c
#include "tof_match_fourth.h"

/* 匹配任务空闲时设置一次，之后原tryMatch/tryMatchEx调用保持不变。 */
setMatchAlgorithm(4);  /* 使用新算法；默认0为原有搜索 */
/* setMatchAlgorithm(0); 切回原有搜索 */
```

当前状态：搜索分支和组件CMake已接好，但正式业务代码尚未调用
`setMatchAlgorithm(4)`，只有集成测试显式启用；静态默认值为0，因此默认仍走旧搜索。
需要在运行匹配任务的处理器一侧、任务空闲的初始化阶段设置一次；不能只在另一核设置局部变量。
PC的`cli/main.c`直接调用核心，用于连续结果仿真，不代表固件第四方法已启用。

这里0表示原有入口，4表示新增方法，没有给原有算法虚构1/2/3编号。
需要调参数时，调用 `setMatchFourthConfig(&config)`；NULL恢复默认。
默认完整窗口为预测位置周围±1.2m、±15°。第四种方法不采用旧轴向约束、
角度/距离惩罚和逐圈扩大策略；原调用的min_range/max_range不控制新搜索窗口。
上层应每帧调用一次新搜索；若仍在外层循环扩大旧范围，会重复搜索同一配置窗口。
接受使用新评分的阈值、覆盖率、近障比例、峰间分差及边界检查，不能沿用旧评分阈值。

调用链：

```text
tryMatchEx → 原generateScan / 未知区保护
           → tofMatchFourthSearch → sm_tof_match_scan → sm_match
           → 原成功/失败处理 → 原累计修正和fillMap
```

### 输出和精度边界

旧接口x/y修正量的单位是整数格，每格12cm；theta是float弧度。
第四种方法开启 `SmConfig.grid_output`：连续候选按实际接口量化，重新评分、
排序、诊断和接受判断；保留原离散候选作为评分下限，避免连续精修后取整造成退化。
`getMatchFourthResult()` 返回最近一次搜索诊断，候选分数对应实际GridPoint位置。
少于25点时旧流程在搜索前返回NOT_ENOUGH_POINTS，应以返回状态为准，不读取上次诊断。

目前仍采用“精修后取整，与原离散候选比较”的实现；讨论过的邻近整数格逐点优化角度方案尚未实现。

这保持接口兼容，但不能保留连续输出的全部精度。PC独立CLI仍默认输出连续位姿。
不要把“第四种方法被接受”解释为“与连续结果位置完全一样”或“定位一定正确”。

### 文件阅读顺序

1. `../tof_match_tool/toflib/inc/tof_match_fourth.h`：算法选择、配置、诊断接口。
2. `../tof_match_tool/toflib/src/tof_match_fourth.c`：工作区、停止/让出、旧输出适配。
3. `../tof_match_tool/toflib/src/tof_gridmap.c` 的 `tryMatchEx()`：实际接入位置。
4. `core/scan_match.c` 的 `sm_match()`：全窗口搜索、备选峰、精修及接受判断。
5. `adapters/toflib_adapter.c`：真实概率图视图及已有SingleFrameScan复用。

只保留一份四层核心；旧double/五层编译分支已移除。
`tests/fixtures/reference_results.json` 是冻结的历史结果，不是另一套实现。

### 编译和回归

在scan_match目录执行：

```bash
mkdir -p build-fourth
cd build-fourth
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build . --parallel
ctest --output-on-failure
cd ..
python3 tests/check_native_regression.py ../ReceivedTofile-COM21-2026_9_25_15-17-14.DAT --native build-fourth/scan_match --adapter-test build-fourth/test_toflib_adapter --out output/fourth_core_validation --repeats 1 --plots ''
python3 tests/check_fourth_replays.py --scenes output/fourth_core_validation
```

`output/fourth_core_validation/comparison.html`：连续核心与冻结历史基准比较。
参考耗时来自历史结果，不用于新旧同机实时性比较。
`output/fourth_validation/comparison.html`：真实tryMatchEx第四种方法的分数、
实际修正、接受状态和选定场次的地图/点云对比；每个场次附完整运行日志。
测试覆盖实际GridPoint位置的独立评分、延迟应用、原applyMatchResult插图、
停止返回、旧评分缓存未被覆盖。另有ASan/UBSan配置 `-DSCAN_MATCH_SANITIZE=ON`。

37次可回放记录中，连续核心与冻结基准接受状态一致；第27次有效点不足不能回放。
第四种方法在原日志失败组接受10/10次（仅2个独立扫描片段）；
原日志成功组接受24/27次：第2次歧义、第32次仅12点；第0次因整数输出后的候选分差新增歧义拒绝。
第6次相对连续候选位置变化约16.4cm，评分从0.967885降到0.924117，需重点人工看图。
第13次实际修正约(-24cm,+72cm,+3.990°)，评分约0.996354。
没有通过放宽阈值维持接受率；这些记录缺少真实位姿真值，不能计算定位正确率。

### 内存、实时性和移植

第四方法借用已有 `SingleFrameScan`，不申请第二份点云或原地图副本。
地图明确读取 `g_proGridMap->map_data[x][y]`，robot只提供位姿、活动尺寸和偏移。
为了保留切回旧方法的能力，评分层独立保存，不覆盖旧level_map：

| 新增主要数组（256×256、512点、32位指针） | 大小 |
|---|---:|
| 四层评分 | 256 KiB |
| 行指针 | 4 KiB |
| 旋转工作区 | 4 KiB |
| 合计 | 264 KiB |

另加少量状态、分配器开销及栈。调用toflib的tof_malloc/tof_free，首次使用申请、
之后复用，空闲时可用releaseMatchFourthWorkspace释放。
默认不采用先前约199KiB的借用L0方案：目前不能保证所有旧缓存使用者都重建，
独立缓存可以可靠切换算法。核心适配器仍支持由调用方提供安全的共享缓存。

BK7258是否够用，必须结合整机剩余堆/PSRAM和任务栈确认，不能仅凭芯片型号保证。
PC回放不是板端实时性结论。核心仍使用double做位姿、插值累加与精修，
新入口接入停止标志及计算资源让出；可通过配置poll添加时限，返回0表示取消。
地图/点云/预测位姿必须由外层保持稳定并互斥；实例不支持并发或回调重入。

仓库内toflib组件CMake已加入core、adapter和tof_match_fourth.c，并定义TOF_ENABLE_SCAN_MATCH4。
默认不切换算法，需要上层调用setMatchAlgorithm(4)。固件复制代码时保留core/与adapters/，
通过TOF_SCAN_MATCH_ROOT指定其根目录；PC可用SCAN_MATCH_TOFLIB_DIR指定真实toflib目录。
这里只完成PC实际源码编译及日志验证，尚未执行BK7258固件交叉编译和上板测试。

### 清理后的工程

编译目录build/build-*、output回放报告和Python缓存均为可重新生成的文件，已清理。
保留源码、CMake、README、工具和测试脚本，以及tests/fixtures/reference_results.json冻结基准。
该JSON是回归输入，不是缓存；删除会导致历史对比无法运行。原始DAT日志也保留在上级目录。
需要查看图或运行程序时，按上面的编译和回归命令重新生成。
