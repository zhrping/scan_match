/* 防止适配层头文件被重复声明。 */
#ifndef SM_TOFLIB_ADAPTER_H
/* 定义本头文件的包含保护标志。 */
#define SM_TOFLIB_ADAPTER_H
/* 适配层依赖核心类型；核心头文件再引用真实toflib结构。 */
#include "scan_match.h"

/* 原生端点/最终输出使用Point2D/Point，内部SmPose保持double。每帧端点转换只需8*N字节，
 * 旋转工作区另需8*N；不再复制RobotData或保存double射线数组。 */
typedef struct {
    /* 原始RobotData转换后的局部端点缓存；传已有SingleFrameScan时不使用。 */
    Point2D *points;
    /* points能够存放的端点数量，不是字节数。 */
    size_t point_capacity;
    /* 核心每个角度复用的旋转缓存，不得与输入点云重叠。 */
    Point2D *rotated;
    /* rotated可容纳的点数，至少等于本帧点数。 */
    size_t rotated_capacity;
    /* 接口允许levels[0]借用robot->level_map[0]；当前第四方法实际使用独立四层缓存。
     * 只有已确认全部旧使用者在读之前会重建且互斥时，才可借用全部四层。
     * dimensions[l]表示行指针个数及每行字节容量（正方形）。 */
    /* 四层评分行指针；调用期间必须独占且可写，适配器不申请内存。 */
    uint8_t **levels[SM_LEVELS];
    /* 每层可访问的行数和每行字节数，按正方形容量描述。 */
    size_t dimensions[SM_LEVELS];
} SmTofMemory;
typedef struct {
    /* 可选加锁函数，返回0才成功；非0时匹配返回-4且不会调用release。 */
    int (*acquire)(void *user); /* 0成功；覆盖地图/预测位姿/点云和借用的评分缓存 */
    /* 与acquire成对提供的解锁函数，成功获取锁后的所有退出路径都会调用。 */
    void (*release)(void *user);
    /* 传给锁函数的外部上下文，例如互斥锁对象，不由适配器释放。 */
    void *user;
} SmTofLock;
typedef struct {
    /* 完整核心诊断，即使正常完成但拒绝也能查看候选及原因。 */
    SmResult match;
    /* 成功完成且接受时为最终地图位姿；完成但拒绝时保留预测位姿，错误返回不可使用。 */
    Point pose;               /* 连续位姿；拒绝时保留预测位置 */
    /* 接受时为米/弧度修正；不是GridPoint格数，第四方法另做单位转换。 */
    Point correction;         /* 米/弧度，绝不能强转为GridPoint；拒绝时为0 */
} SmTofResult;

/* 普通MATCH明确使用g_proGridMap；robot只提供活动尺寸、原点偏移及预测位置。
 * resolution显式提供；概率地图不得传入旧D0评分数组。 */
/* 创建g_proGridMap的零拷贝视图，robot提供位姿相关地图偏移和活动尺寸。 */
int sm_tof_map_view(const Robot *robot,const GridValueMap *probability,
                    /* 分辨率显式传入米/格，map是调用方提供的输出结构。 */
                    double resolution,SmMap *map);
/* 将世界坐标RobotData端点统一到prediction参考系，写入调用方points。 */
int sm_tof_convert_scan(const DynamicArray *raw,Point prediction,
                        /* capacity为端点容量；scan输出一个借用points的视图，没有新增内存。 */
                        Point2D *points,size_t capacity,SingleFrameScan *scan);
/* -1参数错误，-2无候选，-3取消/超时，-4锁失败。返回0仍需检查accepted。
 * 每次重建评分，失败/停止后借用缓存也不能再当旧评分使用。
 * cfg=NULL开启BnB；poll由cfg接入原RTOS停止/让出机制。
 * 不调用malloc、不修改原始地图和位姿，不触发插图/应用状态机。
 * 所有输入/缓存必须保持有效、互斥、不重叠；大缓冲放堆/PSRAM而非任务栈。
 * 不支持共享实例并发匹配。ray_origin由本适配层从raw按需计算。
 */
/* 已经调用generateScan时用此入口：直接借用sc.clouds，memory.points不使用，
 * 不再转换/分配第二份端点。raw可为NULL（不做射线诊断）；非NULL时必须与sc逐点对应。
 * scan及其坐标参考必须对应robot->position，调用期间保持不变。 */
/* 已有generateScan结果时使用此入口，避免重复转换和第二份点云缓存。 */
int sm_tof_match_scan(const Robot *robot,const GridValueMap *probability,
                      /* scan为待匹配端点；可选raw必须逐点对应，只用于恢复射线起点。 */
                      const SingleFrameScan *scan,const DynamicArray *raw,
                      /* 地图分辨率及传感器安装x/y外参，单位均为米。 */
                      double resolution,double sensor_x,double sensor_y,
                      /* 配置可为空表示默认；memory提供可写缓存；lock可为空表示外部已保证互斥。 */
                      const SmConfig *cfg,const SmTofMemory *memory,const SmTofLock *lock,
                      /* 调用方结果对象；返回0后还要检查out->match.accepted。 */
                      SmTofResult *out);
/* 只有DynamicArray原始数据时使用此入口，内部先转换再调用共同匹配流程。 */
int sm_tof_match(const Robot *robot,const GridValueMap *probability,const DynamicArray *raw,
                 /* 地图分辨率及传感器安装x/y外参，单位均为米。 */
                 double resolution,double sensor_x,double sensor_y,
                 /* 配置可为空表示默认；memory提供可写缓存；lock可为空表示外部已保证互斥。 */
                 const SmConfig *cfg,const SmTofMemory *memory,const SmTofLock *lock,
                 /* 调用方结果对象；返回0后还要检查out->match.accepted。 */
                 SmTofResult *out);
/* 结束适配层头文件保护。 */
#endif
