/* 头文件防重复包含：同一编译单元只声明一次这些类型和接口。 */
#ifndef SCAN_MATCH_H
/* 记录本头文件已经进入，防止间接包含形成重复定义。 */
#define SCAN_MATCH_H

/* 独立 C99 接口：无文件读写、无 RTOS、无 toflib 全局变量、无内部 malloc。
 * 全部长度单位为米，角度为弧度；地图默认按 cells[y * width + x] 排列，也支持显式字节步长。
 * origin 是索引 (0,0) 的格子中心，不是格子左下角。
 * 点及每条射线原点必须已经统一到同一个参考机器人坐标系。
 * 端点使用Point2D和四层评分场；内部精修位姿保留double。 */
/* 提供size_t，用于数组容量和字节数，避免用int表示内存大小。 */
#include <stddef.h>
/* 提供uint8_t、uint64_t等明确位宽的整数类型。 */
#include <stdint.h>

/* 引用真实toflib结构定义，确保Point2D布局与固件一致。 */
#include "tof_gridmap.h"
/* 扫描点只存局部x/y端点，单位米；每束射线原点通过回调按需获取。 */
typedef Point2D SmPoint;
/* 评分场固定四层，滑动max窗口边长依次1、2、4、8格。 */
#define SM_LEVELS 4
/* 最多保留三个不同峰，用于查看备选与判断歧义。 */
#define SM_CANDIDATES 3
/* 精修内部保留double位姿，避免逐次写回Point(float)改变迭代路径。
 * 仅最终与toflib交换时转换为Point；这几个标量不值得为复用而牺牲精度。 */
/* 连续地图位姿：x/y单位米，theta单位弧度；不是整数格修正。 */
typedef struct { double x, y, theta; } SmPose;

typedef struct {
    /* 有效地图宽、高，单位格子；不是底层数组的字节跨度。 */
    int width, height;
    /* 每格米数，以及索引(0,0)格子中心在世界坐标中的x/y。 */
    double resolution, origin_x, origin_y;
    /* 只读原始占据数据，内存由外部拥有；不是高斯评分场数据。 */
    const uint8_t *cells;             /* 未知/缺失均以 128 交给核心；原始缺失掩码由工具保存 */
    /* 同为0表示默认行优先；否则必须同时为正。调用方保证数组覆盖最大偏移。
     * toflib occ_map[x][y] 可设 stride_x=MAX_MAP_SIZE, stride_y=1，无需转置副本。
     * 新建此结构必须零初始化或显式设置这两个字段。连续评分存储自身为行优先；行指针评分模式使用[level][x][y]。 */
    /* 索引x/y各加1时跨过的字节数；都为0采用cells[y*width+x]。 */
    size_t stride_x, stride_y;
} SmMap;
typedef struct {
    /* 平移单侧半窗（米）、角度单侧半窗（弧度）、期望角度步长（弧度）。 */
    double window_xy, window_angle, angle_step;
    /* sigma控制高斯评分随距离衰减的宽度；cutoff控制构场影响范围，均为米。 */
    double sigma, cutoff;            /* 评分容差及距离场截断距离 */
    /* 两个峰的平移距离或周期角差任一达到阈值即认为不同，单位米/弧度。 */
    double distinct_xy, distinct_angle;
    /* 依次为最低贴合分、最低近障比例、最低已知覆盖比例及前两候选最小分差。 */
    double min_score, min_near_ratio, min_known_ratio, min_margin;
    /* 近障诊断的距离容差（米），会转换为相应高斯评分阈值。 */
    double hit_distance;
    /* 原图已知占据值严格大于此阈值才作为障碍；SmField中此值记录实际构场参数。 */
    int occupied_threshold;          /* 已知且 value > threshold 视为障碍 */
    /* 核心接受所需最少点数，以及LM精修最多迭代次数；0次表示关闭精修。 */
    int min_points, refine_iterations;
    /* 非0使用分支定界剪枝，0使用全枚举；两者离散候选集合相同。 */
    int use_bnb;
    /* 返回0表示停止/超时；每32个搜索节点及每轮精修检查，构场也检查。
     * 回调可接现有RTOS让出机制；不可修改输入或重入同一匹配实例。 */
    /* 可选停止/让出回调，返回1继续、0取消；核心不直接依赖RTOS。 */
    int (*poll)(void *user);
    /* 传给poll的调用者上下文，核心不拥有该对象。 */
    void *poll_user;
    /* 可选：按点号返回参考机器人坐标系中的射线原点，只在候选诊断调用。
     * NULL表示不做射线穿障诊断；不会影响端点评分和接受阈值。 */
    /* 可选按索引取射线原点的回调，成功写origin并返回非0，否则跳过该束诊断。 */
    int (*ray_origin)(void *user, size_t index, Point2D *origin);
    /* 传给ray_origin的上下文，需要在整个匹配期间有效。 */
    void *ray_user;
    /* 旧GridPoint接口：平移修正量取整数格、角度修正取float，再重新评分/诊断。
     * 默认0保留连续输出；第四种方法强制1。 */
    /* 非0量化为旧GridPoint可表达的修正，并按实际量化位置重评。 */
    int grid_output;
} SmConfig;
typedef struct {
    /* 评分场借用的原始地图视图；其结构及cells均须活到查询结束。 */
    const SmMap *map;
    /* 连续存储模式的只读查询指针；构场时借用外部可写内存生成它。 */
    const uint8_t *scores;            /* level 0..SM_LEVELS-1，每层 width*height 字节 */
    /* 当前场实际使用的高斯sigma，用于将近障距离换算为评分阈值。 */
    double sigma;
    /* 原图已知占据值严格大于此阈值才作为障碍；SmField中此值记录实际构场参数。 */
    int occupied_threshold;
    /* 非NULL时直接读写toflib的[x][y]行指针缓存；scores为NULL。
     * 行指针和每行可用长度由sm_build_field_rows检查。 */
    /* 行指针模式，索引为[level][x][y]；与连续scores二选一。 */
    uint8_t **rows[SM_LEVELS];
} SmField;
typedef struct {
    /* 该候选的地图绝对位姿，不是相对预测位姿的修正量。 */
    SmPose pose;
    /* 平均贴合分、已知端点比例、近障端点比例、有效射线中穿障比例；最后一项只作诊断。 */
    double score, known_ratio, near_ratio, ray_conflict_ratio;
    /* 可判定的射线数，以及候选是否贴近搜索边界的布尔标志。 */
    int ray_valid_count, boundary;
} SmCandidate;
typedef struct {
    /* 最终按分数从高到低排列的候选；只访问candidate_count个元素。 */
    SmCandidate candidates[SM_CANDIDATES];
    /* 第一轮未排除任何峰时的离散最优位姿，便于核对精修与量化效果。 */
    SmPose discrete_best;
    /* 初始离散最优分、预测位姿分、最终前两峰分差；margin=-1表示缺少备选。 */
    double discrete_score, predicted_score, margin;
    /* 实际执行的平移半窗（米）和均分角度步长（弧度），供报告重建搜索坐标。 */
    double actual_window_xy, actual_angle_step;
    /* 访问的合格离散候选数与被剪枝块数；后者不是被跳过点数。 */
    uint64_t evaluated, pruned;
    /* 有效候选数、是否允许采用结果、拒绝原因位掩码；返回码0不等于accepted=1。 */
    int candidate_count, accepted, reasons;
} SmResult;

/* 每个原因占一个二进制位，可以同时出现多个原因，通过按位或组合。 */
enum {
    /* 1表示低分，2表示已知覆盖不足，4表示近障比例不足。 */
    SM_LOW_SCORE=1, SM_LOW_COVERAGE=2, SM_LOW_NEAR=4,
    /* 8表示峰间歧义，16表示边界，32表示点太少，64表示停止请求。 */
    SM_AMBIGUOUS=8, SM_BOUNDARY=16, SM_FEW_POINTS=32, SM_INTERRUPTED=64
};
/* trace 仅报告第一轮“未排除其他峰”的离散候选；BnB 下被剪枝处没有记录。
 * 需要完整热力图时选择枚举模式。回调不得修改输入、工作区或上下文。 */
/* 跟踪函数类型：user为外部上下文，dx/dy米、dtheta弧度、score归一化贴合分。 */
typedef void (*SmTrace)(void *user, double dx, double dy, double dtheta, double score);

/* 创建默认配置值；调用者可修改自己的副本，不会影响其他匹配实例。 */
SmConfig sm_default_config(void);
/* 连续四层评分数组所需字节数；无效地图返回0，不包含原图和旋转缓存。 */
size_t sm_field_bytes(const SmMap *map);
/* 旋转缓存所需字节数；无效点数返回0，缓存由调用方管理。 */
size_t sm_workspace_bytes(size_t count);
/* 使用调用方的连续storage构建评分场；bytes是容量，field为输出视图。 */
int sm_build_field(const SmMap *map, const SmConfig *config,
                   /* storage在构场及后续匹配期间保持有效；失败时field会被清空。 */
                   void *storage, size_t bytes, SmField *field);
/* 复用调用方独占的评分缓存；每层行数>=地图宽，每行字节数>=地图高。
 * 只写有效地图范围，padding保持不动。失败/中断后缓存视为无效，必须重建。 */
/* 使用已有分层行指针构场，适合toflib不连续分配的数组。 */
int sm_build_field_rows(const SmMap *map, const SmConfig *config,
                        /* dimensions给出每层正方形缓存的可用边长；不包括外部指针对象的生命周期管理。 */
                        uint8_t **const rows[SM_LEVELS], const size_t dimensions[SM_LEVELS],
                        SmField *field);
/* -3表示poll取消/超时；此时accepted=0，禁止应用部分搜索候选。 */
/* 执行搜索：points是预测参考系点云，prediction是地图位姿，config定义窗口及判据。 */
int sm_match(const SmField *field, const SmPoint *points, size_t count,
             /* 预测只确定搜索中心，不会自动增加距离/角度惩罚项。 */
             SmPose prediction, const SmConfig *config,
             /* 外部工作区、可选跟踪回调及结果输出；核心不会自动应用或插图。 */
             void *workspace, size_t bytes, SmTrace trace, void *user, SmResult *result);
/* 连续评分与搜索使用相同双线性插值目标，未知/越界保留在分母中。
 * 输出是贴合分，不是“位置正确的概率”。 */
/* 仅对一个位姿求平均分，用于匹配前后比较和实际输出复核。 */
double sm_score(const SmField *field, const SmPoint *points, size_t count, SmPose pose);
/* 结束头文件包含保护。 */
#endif
