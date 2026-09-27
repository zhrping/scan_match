/* 导入适配器接口及真实toflib结构，确保与固件共享类型布局。 */
#include "toflib_adapter.h"
/* 坐标变换、有限值检查及角差归一化所需数学函数。 */
#include <math.h>
/* 用memset清空视图和输出，避免失败时留下旧结果。 */
#include <string.h>

/* 把真实概率地图包装为核心地图视图，整张地图不复制、不转置。 */
int sm_tof_map_view(const Robot *robot,const GridValueMap *probability,
                    /* resolution单位米/格；map由调用方提供，只写元信息和借用指针。 */
                    double resolution,SmMap *map) {
    /* 没有输出结构时立即报参数错误。 */
    if(!map) return -1;
    /* 初始化全部元信息，特别是自定义步长字段，避免使用残留值。 */
    memset(map,0,sizeof(*map));
    /* 确认机器人、地图、活动尺寸和分辨率有效，再访问底层数据。 */
    if(!robot || !robot->map_data || !probability || !robot->size || robot->size>MAX_MAP_SIZE ||
       /* 拒绝非正或非有限分辨率，防止后续除零或坐标变换异常。 */
       !isfinite(resolution) || resolution<=0) return -1;
    /* 当前固件活动地图为正方形；有效边长来自robot->size。 */
    map->width=map->height=robot->size;map->resolution=resolution;
    /* 索引0中心的世界x为-(半边长+x偏移)×分辨率，单位米。 */
    map->origin_x=-(robot->size/2+robot->map_data->x_offset)*resolution;
    /* y方向采用相同偏移约定，符号必须与旧库的世界转格子规则一致。 */
    map->origin_y=-(robot->size/2+robot->map_data->y_offset)*resolution;
    /* 显式选择概率图probability，不误用占据共享图或已加工的D0评分缓存。 */
    map->cells=(const uint8_t *)probability->map_data;
    /* 原数组是map_data[x][y]：x增1跨MAX_MAP_SIZE字节，y增1跨1字节。 */
    map->stride_x=MAX_MAP_SIZE;map->stride_y=1;
    /* 借用核心几何校验再次检查视图，合法返回0，非法返回-1。 */
    return sm_field_bytes(map) ? 0 : -1;
}
/* 把raw中的世界端点转成同一预测机器人参考系；仅转换，不滤点。 */
int sm_tof_convert_scan(const DynamicArray *raw,Point prediction,
                        /* points是外部缓存，capacity为点数；scan输出借用该缓存的单帧视图。 */
                        Point2D *points,size_t capacity,SingleFrameScan *scan) {
    /* 必须有单帧输出视图。 */
    if(!scan) return -1;
    /* 先置空输出，即使中途失败也不会暴露部分有效点云。 */
    scan->clouds=NULL;scan->size=0;
    /* 检查原始数组、目标缓存、点数范围及预测位姿。 */
    if(!raw || !points || !raw->size || raw->size>MAX_SCAN_SIZE || raw->size>capacity ||
       /* 三个预测分量都必须是有限数，才能作为统一坐标参考。 */
       !isfinite(prediction.x)||!isfinite(prediction.y)||!isfinite(prediction.theta)) return -1;
    /* 每帧只转换一次，double中间值避免大世界坐标减法多一次舍入；存储仍为float。 */
    /* 缓存预测角的正余弦；逆变换使用旋转矩阵的转置。 */
    double c=cos(prediction.theta),s=sin(prediction.theta);
    /* 逐点转换，保留原数组顺序以便与射线原点一一对应。 */
    for(size_t i=0;i<raw->size;i++) {
        /* 借用第i条原始记录，不复制整份RobotData数组。 */
        const RobotData *p=&raw->data[i];
        /* 检查端点和采样时机器人位姿，任一非法则整个转换失败。 */
        if(!isfinite(p->point_cloud_x)||!isfinite(p->point_cloud_y)||!isfinite(p->robot_x)||
           /* 补充采样y与角度的有效性检查。 */
           !isfinite(p->robot_y)||!isfinite(p->robot_theta)) return -1;
        /* 先转double再减预测平移，减少float中间减法误差。 */
        double dx=(double)p->point_cloud_x-prediction.x,dy=(double)p->point_cloud_y-prediction.y;
        /* 用R转置乘(world-prediction)得到局部端点，再存入float缓存。 */
        points[i]=(Point2D){c*dx+s*dy,-s*dx+c*dy};
    }
    /* 全部转换成功后才发布指针与点数，返回0；没有内存所有权转移。 */
    scan->clouds=points;scan->size=raw->size;return 0;
}
/* 射线回调的临时上下文，只在当前match_impl调用期间有效。 */
typedef struct {
    /* 每束射线对应的原始机器人采样位姿，数组顺序必须与端点一致。 */
    const DynamicArray *raw;
    /* 把所有射线起点统一到的参考机器人位姿。 */
    Point prediction;
    /* 传感器相对机器人原点的安装位置，单位米。 */
    double sensor_x,sensor_y;
} Rays;
/* 按需还原第i束射线的局部起点，成功返回1并写out。 */
static int ray_origin(void *user,size_t i,Point2D *out) {
    /* 从void上下文恢复类型，并取得对应RobotData记录。 */
    const Rays *r=user;const RobotData *p=&r->raw->data[i];
    /* 参考位姿的逆旋转参数，供世界起点转局部。 */
    double c=cos(r->prediction.theta),s=sin(r->prediction.theta);
    /* 采样当时的机器人角度，用于把安装外参旋转到世界坐标。 */
    double ct=cos(p->robot_theta),st=sin(p->robot_theta);
    /* 采样机器人世界x加上旋转后的传感器偏移，再减参考机器人x。 */
    double dx=p->robot_x+ct*r->sensor_x-st*r->sensor_y-r->prediction.x;
    /* 同样计算世界y差；外参只用于起点，不能再给已含外参的端点加一次。 */
    double dy=p->robot_y+st*r->sensor_x+ct*r->sensor_y-r->prediction.y;
    /* 应用参考位姿的逆旋转，把射线起点写为局部float坐标。 */
    *out=(Point2D){c*dx+s*dy,-s*dx+c*dy};return 1;
}
/* 两个公开匹配入口的共同实现：加锁、准备视图、构场、搜索、解锁。 */
static int match_impl(const Robot *robot,const GridValueMap *probability,const DynamicArray *raw,
                 /* 非NULL表示借用已有局部点云；NULL表示需要从raw转换。 */
                 const SingleFrameScan *existing,
                 /* 几何单位都是米/弧度；cfg为空时使用核心默认配置。 */
                 double resolution,double sensor_x,double sensor_y,const SmConfig *cfg,
                 /* memory只描述外部工作区；lock负责输入/缓存互斥；out接收结果。 */
                 const SmTofMemory *memory,const SmTofLock *lock,SmTofResult *out) {
    /* 必须提供结果对象，才能报告候选或失败状态。 */
    if(!out) return -1;
    /* 清除上次结果，默认接受标志为0，防止失败时误用旧修正。 */
    memset(out,0,sizeof(*out));
    /* 检查旋转缓存和外参；只有需要转换raw时才要求points缓存。 */
    if(!memory || (!existing && !memory->points) || !memory->rotated || !isfinite(sensor_x)||!isfinite(sensor_y) ||
       /* 检查float缓存对齐，避免某些芯片因未对齐访问异常。 */
       (!existing && (uintptr_t)memory->points%sizeof(float)) || (uintptr_t)memory->rotated%sizeof(float) ||
       /* 锁函数必须成对提供；不能只加锁而不解锁，或只解锁。 */
       (lock && (!!lock->acquire != !!lock->release))) return -1;
    /* 获取锁失败返回-4，此时没有获得锁，不执行release。 */
    if(lock && lock->acquire && lock->acquire(lock->user)!=0) return -4;
    /* 默认状态设为参数错误，准备栈上的地图视图和点云视图。 */
    int rc=-1;SmMap map;SingleFrameScan scan;
    /* 创建概率图零拷贝视图，失败走统一解锁出口。 */
    if(sm_tof_map_view(robot,probability,resolution,&map)) goto done;
    /* 已有单帧点云时直接借用其端点，不做第二次坐标转换。 */
    if(existing) {
        /* 只复制指针和点数，底层端点仍由原调用方拥有。 */
        scan=*existing;
        /* 已有点云必须非空，点数必须在核心支持范围内。 */
        if(!scan.clouds || !sm_workspace_bytes(scan.size)) goto done;
        /* 提供raw时启用射线诊断，因此需核实其与端点的对应关系。 */
        if(raw) {
            /* 原始位姿条数与端点数不同就拒绝，避免射线与端点错配。 */
            if(raw->size!=scan.size) goto done;
            /* 遍历对应采样位姿，检查射线回调依赖的数据。 */
            for(size_t i=0;i<scan.size;i++)
                /* 采样位置和角度均要有限，否则直接退出。 */
                if(!isfinite(raw->data[i].robot_x)||!isfinite(raw->data[i].robot_y)||
                   /* 补齐角度检查；goto done确保加锁后的失败也会解锁。 */
                   !isfinite(raw->data[i].robot_theta)) goto done;
        }
    /* 没有现成单帧时，使用调用方points缓存从raw转换，失败同样走统一出口。 */
    } else if(sm_tof_convert_scan(raw,robot->position,memory->points,memory->point_capacity,&scan)) goto done;
    /* 旋转缓存的点容量不能小于当前帧点数。 */
    if(memory->rotated_capacity<scan.size) goto done;
    /* 复制配置到栈，后面替换射线回调不会修改调用方原配置。 */
    SmConfig config=cfg ? *cfg : sm_default_config();
    /* 建立本帧射线上下文，供搜索结束前的候选诊断按需访问。 */
    Rays rays={raw,robot->position,sensor_x,sensor_y};
    /* 有raw则接本适配器射线回调；没有raw明确关闭射线诊断。 */
    config.ray_origin=raw ? ray_origin : NULL;config.ray_user=&rays;
    /* 评分场视图本身很小，底层各层数据由memory提供。 */
    SmField field;
    /* 每次按当前地图重建评分，不把旧D0或上帧缓存当作当前结果。 */
    rc=sm_build_field_rows(&map,&config,memory->levels,memory->dimensions,&field);
    /* 构场或搜索失败时进入统一出口，不生成可应用修正。 */
    if(rc) goto done;
    /* 把toflib的float预测值提升到核心double位姿，不改动robot位置。 */
    SmPose prediction={robot->position.x,robot->position.y,robot->position.theta};
    /* 执行核心搜索，使用已有端点、配置和外部旋转缓存。 */
    rc=sm_match(&field,scan.clouds,scan.size,prediction,&config,
                /* 传入准确工作区字节数；此包装不输出trace，完整诊断写out->match。 */
                memory->rotated,scan.size*sizeof(Point2D),NULL,NULL,&out->match);
    /* 构场或搜索失败时进入统一出口，不生成可应用修正。 */
    if(rc) goto done;
    /* 正常完成时先以预测位姿为输出，拒绝候选时保持这个值。 */
    out->pose=robot->position;
    /* 只有核心接受才产生可应用的位姿与修正。 */
    if(out->match.accepted) {
        /* 取最终按分数排序的第一候选。 */
        SmPose best=out->match.candidates[0].pose;
        /* 边界处把double候选转换成toflib的float地图位姿。 */
        out->pose=(Point){best.x,best.y,best.theta};
        /* 从最终float位姿减去原预测位置，得到米制平移修正。 */
        out->correction=(Point){out->pose.x-robot->position.x,out->pose.y-robot->position.y,
            /* 角度修正归一化为一圈内的短角差，单位弧度。 */
            remainder((double)out->pose.theta-robot->position.theta,2.0*acos(-1.0))};
    }
/* 所有成功加锁后的退出路径汇合到这里，统一处理取消与解锁。 */
done:
    /* 取消时明确清除accepted并记录中断位，不允许应用部分搜索结果。 */
    if(rc==-3) { out->match.accepted=0;out->match.reasons|=SM_INTERRUPTED; }
    /* 有配对解锁函数则释放已获取的锁。 */
    if(lock && lock->release) lock->release(lock->user);
    /* 返回过程状态；0仍需要调用方检查match.accepted。 */
    return rc;
}

/* 公开原始记录入口：内部先将raw转换为SingleFrameScan。 */
int sm_tof_match(const Robot *robot,const GridValueMap *probability,const DynamicArray *raw,
                 /* 几何单位都是米/弧度；cfg为空时使用核心默认配置。 */
                 double resolution,double sensor_x,double sensor_y,const SmConfig *cfg,
                 /* memory只描述外部工作区；lock负责输入/缓存互斥；out接收结果。 */
                 const SmTofMemory *memory,const SmTofLock *lock,SmTofResult *out) {
    /* existing传NULL，选择转换原始端点的路径。 */
    return match_impl(robot,probability,raw,NULL,resolution,sensor_x,sensor_y,cfg,memory,lock,out);
}
/* 公开借用单帧入口：适合直接接在旧generateScan之后。 */
int sm_tof_match_scan(const Robot *robot,const GridValueMap *probability,
                      /* scan是已转换端点，raw仅用于可选射线诊断。 */
                      const SingleFrameScan *scan,const DynamicArray *raw,
                      /* 几何单位都是米/弧度；cfg为空时使用核心默认配置。 */
                      double resolution,double sensor_x,double sensor_y,const SmConfig *cfg,
                      /* memory只描述外部工作区；lock负责输入/缓存互斥；out接收结果。 */
                      const SmTofMemory *memory,const SmTofLock *lock,SmTofResult *out) {
    /* 没有单帧对象就拒绝，并在可写out存在时清空它。 */
    if(!scan) { if(out) memset(out,0,sizeof(*out));return -1; }
    /* 传入已有scan，选择零复制端点的路径。 */
    return match_impl(robot,probability,raw,scan,resolution,sensor_x,sensor_y,cfg,memory,lock,out);
}
