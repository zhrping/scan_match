/* 导入公开的地图、点云、配置和结果结构；本文件实现这些接口。 */
#include "scan_match.h"
/* 提供三角函数、指数、开方、取整及有限数检查，所有内部角度使用弧度。 */
#include <math.h>
/* 提供memset，用于清空结构体状态，不负责申请内存。 */
#include <string.h>
/* 浮点类型限制的标准头文件；当前实现的容差主要在计算处显式指定。 */
#include <float.h>

/* 圆周率常量，用于角度转弧度和周期归一化。 */
#define PI 3.14159265358979323846

/* 直接复用toflib的Point2D。缓存与输入为float；位姿、评分累加/LM小矩阵保留double，
 * 避免原日志的窄峰在float精修下发生明显漂移。没有每点double射线副本。 */
/* 给旋转缓存取语义名称；实际仍是两个float，通常每点8字节。 */
typedef Point2D Rotated;
/* 一次sm_match调用的内部搜索上下文；借用输入和工作区，函数结束后不保留。 */
typedef struct {
    /* 只读评分场；底层可能是连续数组，也可能是toflib的多层行指针。 */
    const SmField *field;
    /* 本次调用的搜索窗口、评分阈值、精修次数和取消回调。 */
    const SmConfig *cfg;
    /* 参考机器人坐标系中的扫描端点，单位米；不在这里复制点云。 */
    const SmPoint *points;
    /* 参与评分的端点总数，包含落在未知或地图外的端点，避免缩小分母抬高分数。 */
    size_t count;
    /* 搜索中心，即预测的地图坐标位姿；x/y为米，theta为弧度。 */
    SmPose prediction;
    /* 当前角度下点云对应的浮点栅格坐标；已含预测平移和地图原点，便于叠加整数格偏移。 */
    Rotated *rotated;
    /* 已经选择的峰位置；后续搜索排除其邻域，寻找不同的备选峰。 */
    SmPose excluded[SM_CANDIDATES];
    /* 依次为已排除峰数、单侧平移格数、当前角度编号、单侧角度步数。 */
    int excluded_count, nx, angle_index, angle_half;
    /* 实际角度间隔，以及本轮尚未排除区域内找到的最高离散评分。 */
    double angle_step, best_score;
    /* 与best_score对应的候选位姿；不一定等于最终精修、排序后的第一候选。 */
    SmPose best;
    /* 可选调试回调，记录第一轮访问到的离散候选，不参与评分。 */
    SmTrace trace;
    /* 原样交给trace的调用者上下文；核心不解释它的内容。 */
    void *user;
    /* 调用者提供的输出对象；用于记录候选、计数及接受状态。 */
    SmResult *result;
    /* 累计检查次数，用于降低停止回调的调用频率。 */
    unsigned poll_count;
    /* 一旦收到停止请求就置1；递归和后续阶段均应尽快退出。 */
    int cancelled;
} Search;

/* 把v限制到闭区间[lo,hi]；精修步长和搜索边界共用此工具。 */
static double clamp(double v, double lo, double hi) { return fmax(lo, fmin(hi, v)); }
/* 取a-b的最短有符号周期角差，避免跨越±π时把相近朝向当作相差一整圈。 */
static double angle_diff(double a, double b) { return remainder(a-b, 2.0*PI); }
/* 只改变原始占据地图的访问方式，评分场布局和搜索目标保持一致。 */
/* 读取原始占据值；调用者应先保证x/y有效，此函数仅处理内存布局。 */
static uint8_t map_cell(const SmMap *m, int x, int y) {
    /* x增加1对应的字节数；没有显式步长时采用行优先布局，x相邻差1字节。 */
    size_t sx=m->stride_x ? m->stride_x : 1;
    /* y增加1对应的字节数；默认跨过一整行width个字节。 */
    size_t sy=m->stride_y ? m->stride_y : (size_t)m->width;
    /* 按两个方向的步长访问；支持C常见的[y][x]和toflib的[x][y]，不需要转置地图。 */
    return m->cells[(size_t)x*sx+(size_t)y*sy];
}
/* 检查地图元信息及偏移计算是否可能溢出；无法代替调用方检查真实分配容量。 */
static int valid_map(const SmMap *m) {
    /* 短路判断先检查指针，再限制地图尺寸；后续才允许访问地图成员。 */
    return m && m->cells && m->width>0 && m->height>0 && m->width<=2048 && m->height<=2048 &&
           /* 分辨率必须为正的有限米数，地图原点不能是NaN或无穷。 */
           isfinite(m->resolution) && m->resolution>0 && isfinite(m->origin_x) && isfinite(m->origin_y) &&
           /* 两个步长同时为0表示默认布局；否则必须两个都为正。 */
           ((!m->stride_x && !m->stride_y) ||
            /* 显式布局需同时提供x/y步长，不能只指定一个方向。 */
            (m->stride_x && m->stride_y &&
             /* 除法形式先检查x方向乘法上限，防止size_t无符号溢出。 */
             m->stride_x <= (SIZE_MAX-1)/(size_t)m->width &&
             /* 同样检查y方向乘法上限。 */
             m->stride_y <= (SIZE_MAX-1)/(size_t)m->height &&
             /* 再检查两个方向的最大偏移相加是否溢出，保留末尾一个字节的空间。 */
             (size_t)(m->width-1)*m->stride_x <=
                 /* 从size_t可表示范围减去y偏移，得到允许的最大x偏移。 */
                 SIZE_MAX-1-(size_t)(m->height-1)*m->stride_y));
}
/* 返回一份配置值，调用者可复制后修改；不使用可变全局配置。 */
SmConfig sm_default_config(void) {
    /* 依次设平移半窗1.2m、角度半窗15°、角度步长0.5°、高斯sigma=0.18m、截断0.60m。 */
    SmConfig c = {1.2, 15*PI/180, 0.5*PI/180, .18, .60,
                  /* 依次设峰间距离0.36m/5°、最低分0.55、近障/已知比例各0.50、分差0.02、近障距离0.18m、障碍阈值151、最少20点、精修25次、暂置BnB关闭。 */
                  .36, 5*PI/180, .55, .50, .50, .02, .18, 151, 20, 25, 0,
                  /* 停止回调及其上下文均为空，表示默认不主动中断。 */
                  NULL, NULL
                  /* 射线原点回调及其上下文为空；grid_output=0表示保留连续位姿输出。 */
                  , NULL, NULL, 0
    };
    /* 默认启用分支定界；与枚举使用相同离散候选定义，通过上界减少实际计算。 */
    c.use_bnb=1;
    /* 按值返回配置，不要求调用者释放内存。 */
    return c;
}
/* 计算连续存储四层评分所需字节数；每层保持原地图尺寸，每格uint8_t。 */
size_t sm_field_bytes(const SmMap *m) {
    /* 有效地图返回width×height×层数；非法地图返回0，调用方据此拒绝申请和构场。 */
    return valid_map(m) ? (size_t)m->width*(size_t)m->height*SM_LEVELS : 0;
}
/* 计算旋转缓存容量；点数为0或超过toflib的MAX_SCAN_SIZE时返回0。 */
size_t sm_workspace_bytes(size_t count) {
    /* 合法点数乘每个Rotated的大小；此函数只报价，不执行malloc。 */
    return count && count<=MAX_SCAN_SIZE ? count*sizeof(Rotated) : 0;
}

/* 原始地图与评分场分别访问，不能把旧D0评分当作概率地图。 */
/* 返回评分格子的可写地址；与map_cell不同，它访问派生评分而不是原始占据地图。 */
static uint8_t *field_cell(SmField *f,int x,int y,int level) {
    /* 行指针模式直接访问[level][x][y]，每行可独立分配并带有额外padding。 */
    if(f->rows[0]) return &f->rows[level][x][y];
    /* 连续模式按层偏移、y行偏移、x列偏移定位；构场阶段底层存储必须可写。 */
    return (uint8_t *)f->scores+(size_t)level*f->map->width*f->map->height+
           /* 加上当前层内的行优先偏移。 */
           (size_t)y*f->map->width+x;
}
/* 确认评分场至少绑定有效地图以及一种可读存储；调用前必须已成功构场。 */
static int field_valid(const SmField *f) {
    /* 先检查场对象及其地图；无效返回假，避免后续空指针访问。 */
    if(!f || !valid_map(f->map)) return 0;
    /* 允许连续数组或行指针两种模式之一存在。 */
    return f->scores || f->rows[0];
}
/* 内部构场：先建立L0障碍高斯评分，再构建L1～L3滑动最大值上界。 */
static int build_field(SmField *f,const SmConfig *c) {
    /* 借用评分场关联的原始地图，缩短后续成员访问；没有拷贝地图数据。 */
    const SmMap *m=f->map;
    /* 把米制截断距离向上取整为格数；后面用真实距离再次剔除圆形范围之外的格子。 */
    int radius=(int)ceil(c->cutoff/m->resolution);
    /* 高斯核可分离：仅计算radius+1次exp，最多129个double（1032字节栈）。
     * 地图中每个障碍仍只散布局部窗口，不反复扫描整图；不增加常驻数组。 */
    /* 局部一维高斯权重表；参数校验保证radius≤128，所以索引0..radius均有效。 */
    double weight[129];
    /* 只计算非负距离的权重，负方向查询时取绝对值复用。 */
    for(int i=0;i<=radius;i++)
        /* 高斯exp(-距离平方/(2sigma平方))；二维权重通过两个方向相乘得到。 */
        weight[i]=exp(-(i*i*m->resolution*m->resolution)/(2*c->sigma*c->sigma));
    /* 逐行处理有效地图区域；在较长循环中给取消回调检查机会。 */
    for(int y=0;y<m->height;y++) {
        /* 如果调用者提供回调且返回0，立即以-3报告中断；不能使用部分构建的评分场。 */
        if(c->poll && !c->poll(c->poll_user)) return -3;
        /* 先将L0每格清零，避免上一次匹配的地图评分残留。 */
        for(int x=0;x<m->width;x++) *field_cell(f,x,y,0)=0;
    }
    /* 逐行处理有效地图区域；在较长循环中给取消回调检查机会。 */
    for(int y=0;y<m->height;y++) {
        /* 如果调用者提供回调且返回0，立即以-3报告中断；不能使用部分构建的评分场。 */
        if(c->poll && !c->poll(c->poll_user)) return -3;
        /* 遍历该行每个地图格子。 */
        for(int x=0;x<m->width;x++) {
            /* 读取原始占据字节；128有特殊的未知语义，不能直接当作障碍。 */
            uint8_t occ=map_cell(m,x,y);
            /* 未知或不超过障碍阈值的格子不作为高斯核中心。 */
            if(occ==128 || occ<=c->occupied_threshold) continue;
            /* 以当前障碍为中心遍历邻域；dx/dy均是整数格偏移。 */
            for(int dy=-radius;dy<=radius;dy++) for(int dx=-radius;dx<=radius;dx++) {
                /* 计算邻域目标格子的绝对索引。 */
                int xx=x+dx,yy=y+dy;
                /* 越界位置不写入，防止访问评分场之外的内存。 */
                if(xx<0 || yy<0 || xx>=m->width || yy>=m->height) continue;
                /* 将格子偏移平方转换为米平方，用于圆形截断判断。 */
                double d2=(dx*dx+dy*dy)*m->resolution*m->resolution;
                /* 超过cutoff半径的邻域点不贡献评分，避免无限扩散。 */
                if(d2>c->cutoff*c->cutoff) continue;
                /* 组合x/y高斯权重并量化为0..255；中心为255，离障碍越远数值越低。 */
                uint8_t score=(uint8_t)lround(255.0*weight[dx<0?-dx:dx]*weight[dy<0?-dy:dy]);
                /* 多个障碍影响同一格时只保留最大评分，不把重叠高斯相加抬高分数。 */
                uint8_t *v=field_cell(f,xx,yy,0);if(score>*v) *v=score;
            }
        }
    }
    /* 遍历每格，将原图未知区域的L0评分重新清零。 */
    for(int y=0;y<m->height;y++) for(int x=0;x<m->width;x++)
        /* 即使未知格附近有障碍，也不让高斯扩散使未知格中心获得正分。 */
        if(map_cell(m,x,y)==128) *field_cell(f,x,y,0)=0;
    /* 同尺寸滑动max；toflib外扩padding不参与新算法查询。 */
    /* 从上一层生成下一层；四层分别对应边长1、2、4、8格的滑动窗口。 */
    for(int l=1;l<SM_LEVELS;l++) {
        /* 上一层窗口边长为2^(l-1)，四个窗口按这个距离拼成更大的窗口。 */
        int step=1<<(l-1);
        /* 逐行处理有效地图区域；在较长循环中给取消回调检查机会。 */
        for(int y=0;y<m->height;y++) {
            /* 如果调用者提供回调且返回0，立即以-3报告中断；不能使用部分构建的评分场。 */
            if(c->poll && !c->poll(c->poll_user)) return -3;
            /* 遍历该行每个地图格子。 */
            for(int x=0;x<m->width;x++) {
                /* 本格上界从0开始，随后取四个子窗口的最大值。 */
                uint8_t best=0;
                /* a/b分别选择左右、上下两个子窗口，总共四个。 */
                for(int b=0;b<2;b++) for(int a=0;a<2;a++) {
                    /* 当前大窗口内对应子窗口的起始索引。 */
                    int xx=x+a*step,yy=y+b*step;
                    /* 只合并仍落在有效地图内的子窗口；地图外等价于零分。 */
                    if(xx<m->width && yy<m->height) {
                        /* 读上一层上界，并在更大窗口中保留最大的一个。 */
                        uint8_t v=*field_cell(f,xx,yy,l-1);if(v>best) best=v;
                    }
                }
                /* 写入当前层；这些高层值用于剪枝，不是降低分辨率后的地图。 */
                *field_cell(f,x,y,l)=best;
            }
        }
    }
    /* 正常完成；此处的0是函数状态码，不能据此认定匹配结果已被接受。 */
    return 0;
}
/* 验证构场参数；这些约束同时保证高斯临时数组和障碍阈值有效。 */
static int field_config_valid(const SmMap *m,const SmConfig *c) {
    /* 先检查地图与配置，再要求sigma为正有限数。 */
    return valid_map(m) && c && isfinite(c->sigma) && c->sigma>0 &&
           /* cutoff必须为正，最多覆盖128格半径，以限制构场时间与栈数组下标。 */
           isfinite(c->cutoff) && c->cutoff>0 && c->cutoff/m->resolution<=128 &&
           /* 阈值至少为128且小于255；实际障碍判断使用严格大于，未知128另行排除。 */
           c->occupied_threshold>=128 && c->occupied_threshold<255;
}
/* 连续数组构场入口：storage由调用方申请，bytes为真实可用字节数，f输出场视图。 */
int sm_build_field(const SmMap *m,const SmConfig *c,void *storage,size_t bytes,SmField *f) {
    /* 输出对象、工作存储、参数或容量任一无效，返回-1而不开始构场。 */
    if(!f || !storage || !field_config_valid(m,c) || bytes<sm_field_bytes(m)) return -1;
    /* 先清空场结构，防止两种存储模式的旧指针混用；若同一行还有赋值，则立即绑定元信息。 */
    memset(f,0,sizeof(*f));
    /* 绑定连续评分存储及构场参数；sigma/障碍阈值供后续诊断使用。 */
    f->map=m;f->scores=storage;f->sigma=c->sigma;f->occupied_threshold=c->occupied_threshold;
    /* 执行真正的L0高斯与高层max构建，并保留返回状态。 */
    int rc=build_field(f,c);
    /* 构场失败或取消时清空视图，避免后续把未完成的缓存当作有效评分场。 */
    if(rc) memset(f,0,sizeof(*f));
    /* 将内部成功、参数错误或取消状态传回调用者。 */
    return rc;
}
/* 行指针构场入口：rows/dims由调用者提供，本函数不申请或释放各行。 */
int sm_build_field_rows(const SmMap *m,const SmConfig *c,
                        /* 每层提供行指针数组及正方形可用边长；f接收最终只读查询视图。 */
                        uint8_t **const rows[SM_LEVELS],const size_t dims[SM_LEVELS],SmField *f) {
    /* 先验证输出、各层指针描述与构场参数。 */
    if(!f || !rows || !dims || !field_config_valid(m,c)) return -1;
    /* 逐层核实或绑定四层缓存；循环范围不包含旧库额外层。 */
    for(int l=0;l<SM_LEVELS;l++) {
        /* 每层行数与每行长度都必须至少容纳当前地图宽、高。 */
        if(!rows[l] || dims[l]<(size_t)m->width || dims[l]<(size_t)m->height) return -1;
        /* 检查将被访问的每一个x行指针，避免不连续缓存中的空行。 */
        for(int x=0;x<m->width;x++) if(!rows[l][x]) return -1;
    }
    /* 先清空场结构，防止两种存储模式的旧指针混用；若同一行还有赋值，则立即绑定元信息。 */
    memset(f,0,sizeof(*f));f->map=m;f->sigma=c->sigma;f->occupied_threshold=c->occupied_threshold;
    /* 只复制各层行指针，不复制评分内容；缓存寿命必须覆盖匹配全过程。 */
    for(int l=0;l<SM_LEVELS;l++) f->rows[l]=rows[l];
    /* 执行真正的L0高斯与高层max构建，并保留返回状态。 */
    int rc=build_field(f,c);
    /* 构场失败或取消时清空视图，避免后续把未完成的缓存当作有效评分场。 */
    if(rc) memset(f,0,sizeof(*f));
    /* 将内部成功、参数错误或取消状态传回调用者。 */
    return rc;
}

/* 读取某层单个评分并归一化到0..1；高层返回窗口上界，L0返回实际节点评分。 */
static double cell(const SmField *f, int x, int y, int level) {
    /* 借用评分场关联的原始地图，缩短后续成员访问；没有拷贝地图数据。 */
    const SmMap *m=f->map;
    /* 当前层窗口边长2^level；四层对应1、2、4、8格。 */
    int width=1<<level;
    /* 窗口整体不与地图相交时返回零；负坐标要检查窗口右端而非只看起点。 */
    if(x>=m->width || y>=m->height || x+width<=0 || y+width<=0) return 0;
    /* 窗口从负坐标开始时向内平移，取得覆盖原有效区域的更宽松上界。
     * level=0 的负坐标已被上一条件排除，仍是严格的越界零分。 */
    /* 负x但窗口仍相交时将起点移到0，使高层查询保持宽松上界；L0负坐标已返回。 */
    if(x<0) x=0;
    /* y方向同样处理，宁可少剪枝也不能把可行高分错误剪掉。 */
    if(y<0) y=0;
    /* 行指针模式直接取值；除以255.0保证浮点归一化而非整数除法。 */
    if(f->rows[0]) return f->rows[level][x][y]/255.0;
    /* 连续模式的地址为层偏移+行偏移+列偏移，同样归一化到0..1。 */
    return f->scores[(size_t)level*m->width*m->height+(size_t)y*m->width+x]/255.0;
}
/* 双线性插值及对栅格坐标的导数；连续精修与离散搜索共用同一目标。 */
/* 对浮点栅格坐标插值；gx/gy非NULL时额外返回评分对栅格x/y的导数。 */
static double sample(const SmField *f, double x, double y, int level, double *gx, double *gy) {
    /* 拒绝NaN、无穷和过大坐标，避免floor后转int溢出。 */
    if(!isfinite(x) || !isfinite(y) || x < -100000 || x > 100000 || y < -100000 || y > 100000) {
        /* 只在调用者需要x导数时写回；无效采样按零梯度处理。 */
        if(gx) *gx=0;
        /* 同样清零y导数；NULL表示调用者只需要分数。 */
        if(gy) *gy=0;
        /* 无效坐标采样贡献零分；这里返回的是评分值，不是过程状态码。 */
        return 0;
    }
    /* 向下取整得到左下节点；负坐标也必须floor，不能用向零截断代替。 */
    int ix=(int)floor(x), iy=(int)floor(y);
    /* a/b是点相对左下节点的格内比例，正常范围均为[0,1)。 */
    double a=x-ix, b=y-iy;
    /* 读取下边两个节点：s00为左下、s10为右下；下标表示x/y偏移。 */
    double s00=cell(f,ix,iy,level), s10=cell(f,ix+1,iy,level);
    /* 读取上边两个节点：s01为左上、s11为右上。 */
    double s01=cell(f,ix,iy+1,level), s11=cell(f,ix+1,iy+1,level);
    /* x导数是上下两条边的评分差，再按y位置加权；单位为每格的评分变化。 */
    if(gx) *gx=(1-b)*(s10-s00)+b*(s11-s01);
    /* y导数是左右两条边的评分差，再按x位置加权。 */
    if(gy) *gy=(1-a)*(s01-s00)+a*(s11-s10);
    /* 先在每条水平边按a插值，再在上下边之间按b插值；同格内的不同点也可得不同分数。 */
    return (1-b)*((1-a)*s00+a*s10)+b*((1-a)*s01+a*s11);
}

/* 计算给定位姿的平均贴合分；不搜索、不加预测距离或角度惩罚。 */
double sm_score(const SmField *f, const SmPoint *p, size_t count, SmPose pose) {
    /* 评分输入必须有有效场、非空点云和正点数；此接口对非法输入返回零分。 */
    if(!field_valid(f) || !p || !count ||
       /* 位姿三个分量必须是有限数，否则不能进行刚体变换。 */
       !isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.theta)) return 0;
    /* 借用评分场关联的原始地图，缩短后续成员访问；没有拷贝地图数据。 */
    const SmMap *m=f->map;
    /* 预先计算当前朝向的正余弦，并初始化double累加器，避免每点重复求三角函数。 */
    double c=cos(pose.theta), s=sin(pose.theta), sum=0;
    /* 依次检查或处理全部输入端点，不在评分阶段挑选有利点。 */
    for(size_t i=0;i<count;i++) {
        /* 端点先按theta旋转并加机器人地图x，再减地图原点、除分辨率，得到浮点栅格x。 */
        double x=(pose.x+c*p[i].x-s*p[i].y-m->origin_x)/m->resolution;
        /* 按同一个二维刚体变换计算浮点栅格y；点云和地图必须使用一致的轴方向。 */
        double y=(pose.y+s*p[i].x+c*p[i].y-m->origin_y)/m->resolution;
        /* L0才是最终评分目标；两个NULL表示只算分数，不需要梯度。 */
        sum+=sample(f,x,y,0,NULL,NULL);
    }
    /* 以所有端点数量求平均，未知和越界贡献零分但仍在分母中。 */
    return sum/count;
}
/* 判断是否属于不同候选峰：距离或角差任一达到阈值就视为不同。 */
static int distinct(SmPose a, SmPose b, const SmConfig *c) {
    /* 平移使用欧氏距离，角度使用周期差；不是要求两个条件同时成立。 */
    return hypot(a.x-b.x,a.y-b.y)>=c->distinct_xy || fabs(angle_diff(a.theta,b.theta))>=c->distinct_angle;
}
/* 判断候选是否落在已选峰排除区之外，确保备选不是同一峰重复收敛。 */
static int eligible(const Search *s, SmPose p) {
    /* 只要与任一已选峰不够分离就拒绝；所有已选峰都不同才允许参与比较。 */
    for(int i=0;i<s->excluded_count;i++) if(!distinct(p,s->excluded[i],s->cfg)) return 0;
    /* 当前检查或搜索阶段成功；这里是布尔返回，不是最终匹配分数。 */
    return 1;
}
/* 把整数格平移索引与当前角度索引转换为真实地图位姿。 */
static SmPose offset_pose(const Search *s,int x,int y) {
    /* 先复制预测位姿，后续只在局部副本上加修正，不改变调用方状态。 */
    SmPose p=s->prediction;
    /* x/y偏移乘分辨率从格数变成米，再累加到预测位置。 */
    p.x+=x*s->field->map->resolution; p.y+=y*s->field->map->resolution;
    /* 角度索引乘实际角度步长得到弧度修正。 */
    p.theta+=s->angle_index*s->angle_step;
    /* 返回转换后的候选位姿。 */
    return p;
}
/* 等分时用固定字典序选取，与遍历顺序无关；不暗中加入预测惩罚。 */
/* 同分候选按theta、y、x升序确定唯一顺序，确保枚举和BnB结果可比较。 */
static int before(SmPose a,SmPose b) {
    /* 先比较角度，角度不同时较小者优先。 */
    if(a.theta!=b.theta) return a.theta<b.theta;
    /* 角度相同再比较y。 */
    if(a.y!=b.y) return a.y<b.y;
    /* 角度和y都相同时最终比较x。 */
    return a.x<b.x;
}
/* 计算平移块的上界；level=0时退化为一个离散候选的评分。 */
static double block_score(Search *s,int x,int y,int level) {
    /* 使用double求和，减少大量float缓存采样累加的舍入误差。 */
    double sum=0;
    /* 先将缓存坐标提升到double再加整数偏移；逐点读取当前层插值并累加。 */
    for(size_t i=0;i<s->count;i++) sum+=sample(s->field,(double)s->rotated[i].x+x,(double)s->rotated[i].y+y,level,NULL,NULL);
    /* 对上界或候选分数求平均，与最终评分保持同一分母。 */
    return sum/s->count;
}
/* 统一处理取消状态；force表示本次必须检查回调，否则每32次检查一次。 */
static int keep_running(Search *s,int force) {
    /* 取消标志有记忆性，后续不能因为回调再次返回真而恢复部分搜索。 */
    if(s->cancelled) return 0;
    /* 有回调并到检查时机才调用；返回0表示用户停止、超时等终止条件。 */
    if(s->cfg->poll && (force || (++s->poll_count%32)==0) && !s->cfg->poll(s->cfg->poll_user))
        /* 记住取消，让外层循环和递归共同退出。 */
        s->cancelled=1;
    /* 返回1表示可继续，0表示必须停止。 */
    return !s->cancelled;
}
/* 处理一个具体整数平移候选，统计访问量、输出trace并更新本轮最优。 */
static void leaf(Search *s,int x,int y) {
    /* 按节流策略检查取消；已取消时不再计算当前候选或子块。 */
    if(!keep_running(s,0)) return;
    /* 将离散索引还原为候选位姿，用于排除判断和保存结果。 */
    SmPose p=offset_pose(s,x,y);
    /* 排除已选峰邻域，不把同一匹配峰作为备选。 */
    if(!eligible(s,p)) return;
    /* 使用L0计算当前候选实际评分，高层上界不能作为最终得分。 */
    double score=block_score(s,x,y,0);
    /* 记录真正算过的、未被排除的离散候选数量。 */
    s->result->evaluated++;
    /* 只记录第一轮无排除区的候选；回调参数为米制dx/dy、弧度dtheta和分数。 */
    if(s->trace && !s->excluded_count) s->trace(s->user,x*s->field->map->resolution,
                                              /* 补齐trace中的y修正、角度修正和当前评分。 */
                                              y*s->field->map->resolution,s->angle_index*s->angle_step,score);
    /* 严格更高分时更新；数值近似相等时用固定字典序消除遍历顺序影响。 */
    if(score>s->best_score+1e-12 || (fabs(score-s->best_score)<=1e-12 && before(p,s->best))) {
        /* 同时保存最优分数和对应位姿，二者必须保持一致。 */
        s->best_score=score; s->best=p;
    }
}
/* 递归访问一个平移块，利用max上界跳过不可能超过当前最优的区域。 */
static void visit(Search *s,int x,int y,int level) {
    /* 超出正向搜索边界或收到取消时不访问该块；负向起点由外层保证。 */
    if(!keep_running(s,0) || x>s->nx || y>s->nx) return;
    /* L0块只有一个候选，交给leaf计算后结束递归。 */
    if(!level) { leaf(s,x,y); return; }
    /* 用该层滑动最大评分估计块内最佳可能值，而不是对块中心打分。 */
    double bound=block_score(s,x,y,level);
    /* 严格小于才剪枝，保留等分候选的统一 tie-break；留出浮点累加裕量。 */
    /* 只有上界仍低于已知最优才剪枝；小容差用于避免浮点误差导致误剪。 */
    if(bound+1e-10<s->best_score) { s->result->pruned++; return; }
    /* 子块边长为父块一半，随后访问四个象限。 */
    int half=1<<(level-1);
    /* 两个方向各二分，递归层数减少1；循环条件同时响应取消。 */
    for(int b=0;b<2 && !s->cancelled;b++) for(int a=0;a<2 && !s->cancelled;a++) visit(s,x+a*half,y+b*half,level-1);
}
/* 在完整配置窗口内找一个未被排除的离散最优峰；每个备选重新执行一轮。 */
static int search_pass(Search *s,SmCandidate *out) {
    /* 合法评分非负，-1表示本轮尚未找到任何候选。 */
    s->best_score=-1;
    /* 读取本次搜索所用地图尺寸、原点和分辨率。 */
    const SmMap *m=s->field->map;
    /* 从负半窗到正半窗遍历全部角度，包含两端和预测角度。 */
    for(int a=-s->angle_half;a<=s->angle_half;a++) {
        /* 每个新角度强制检查取消，返回0让调用方停止寻找更多候选。 */
        if(!keep_running(s,1)) return 0;
        /* 保存角度编号，offset_pose和trace会共用这个索引。 */
        s->angle_index=a;
        /* 用预测角加角度修正；每个角度只计算一次cos/sin。 */
        double theta=s->prediction.theta+a*s->angle_step, c=cos(theta), sn=sin(theta);
        /* 逐点更新当前阶段需要的缓存、残差或诊断；点数保持不变。 */
        for(size_t i=0;i<s->count;i++) {
            /* 当前角度下将端点变换到预测平移处的栅格x；后续平移搜索只需加整数格。 */
            s->rotated[i].x=(s->prediction.x+c*s->points[i].x-sn*s->points[i].y-m->origin_x)/m->resolution;
            /* 同样缓存栅格y；结果存float以降低每点工作区占用。 */
            s->rotated[i].y=(s->prediction.y+sn*s->points[i].x+c*s->points[i].y-m->origin_y)/m->resolution;
        }
        /* 启用分支定界时，从最大层块开始逐级分裂。 */
        if(s->cfg->use_bnb) {
            /* 最大层为3，块边长8格；这是搜索分组大小，不改变地图分辨率。 */
            int l=SM_LEVELS-1, block=1<<l;
            /* 按整块铺满平移窗口，最右/最上块的多余候选由visit边界检查剔除。 */
            for(int y=-s->nx;y<=s->nx && !s->cancelled;y+=block) for(int x=-s->nx;x<=s->nx && !s->cancelled;x+=block) visit(s,x,y,l);
        /* 进入另一种执行路径；与对应if互斥，不会重复执行前一分支。 */
        } else {
            /* 枚举模式逐个访问所有整数x/y偏移，用作BnB一致性基准。 */
            for(int y=-s->nx;y<=s->nx && !s->cancelled;y++) for(int x=-s->nx;x<=s->nx && !s->cancelled;x++) leaf(s,x,y);
        }
    }
    /* 取消或没有合法候选都返回未找到；调用方会进一步区分取消状态。 */
    if(s->cancelled || s->best_score<0) return 0;
    /* 清空候选诊断字段，再写入本轮最优位姿和评分，诊断留到后面统一计算。 */
    memset(out,0,sizeof(*out)); out->pose=s->best; out->score=s->best_score;
    /* 当前检查或搜索阶段成功；这里是布尔返回，不是最终匹配分数。 */
    return 1;
}

/* 带部分主元的 3x3 求解。病态或失败时放弃本次精修步，不影响离散结果。 */
/* 解3×3线性方程A*x=b；a/b被消元过程就地修改，x为输出步长。 */
static int solve3(double a[3][3],double b[3],double x[3]) {
    /* 逐列进行高斯消元，共三个未知量dx、dy、dtheta。 */
    for(int i=0;i<3;i++) {
        /* 在当前列剩余行中选绝对值最大的主元，提高数值稳定性。 */
        int k=i; for(int j=i+1;j<3;j++) if(fabs(a[j][i])>fabs(a[k][i])) k=j;
        /* 主元太接近0说明方程退化，返回失败并放弃这一步精修。 */
        if(fabs(a[k][i])<1e-14) return 0;
        /* 交换矩阵的主元行；已经消掉的左侧列无需重复处理。 */
        for(int j=i;j<3;j++) { double t=a[i][j];a[i][j]=a[k][j];a[k][j]=t; }
        /* 右端向量必须与矩阵做相同换行，保持方程等价。 */
        double t=b[i];b[i]=b[k];b[k]=t;
        /* 消去主元下面各行的当前列。 */
        for(int j=i+1;j<3;j++) {
            /* 计算当前行需要减去主元行的倍数。 */
            double v=a[j][i]/a[i][i];
            /* 对当前列及右侧所有列执行行消元。 */
            for(int z=i;z<3;z++) a[j][z]-=v*a[i][z];
            /* 对右端向量执行同样的消元操作。 */
            b[j]-=v*b[i];
        }
    }
    /* 从最后一行开始回代，减去已知项，再除以对角主元得到当前未知量。 */
    for(int i=2;i>=0;i--) { double v=b[i]; for(int j=i+1;j<3;j++) v-=a[i][j]*x[j]; x[i]=v/a[i][i]; }
    /* 只有三个步长都是有限数才算求解成功。 */
    return isfinite(x[0]) && isfinite(x[1]) && isfinite(x[2]);
}
/* LM 残差 r=sqrt(1-score)：sum(r^2) 正好对应最大化平均评分。
 * 不换用另一套精修目标。每步只接受真实评分上升，并限制在原搜索范围内。
 * 双线性图在格子边界不可微，因此精修只是局部改善，不作连续全局最优承诺。 */
/* 使用LM阻尼最小二乘局部精修；不保证连续空间全局最优。 */
static void refine(Search *s, SmCandidate *candidate) {
    /* 读取本次搜索所用地图尺寸、原点和分辨率。 */
    const SmMap *m=s->field->map;
    /* lambda为初始阻尼；xy为整数格搜索半窗换算成米后的真实边界。 */
    double lambda=.01, xy=s->nx*m->resolution;
    /* 最多执行配置的精修次数，0表示直接保留离散结果。 */
    for(int it=0;it<s->cfg->refine_iterations;it++) {
        /* 每轮精修强制检查停止，避免取消后仍持续迭代。 */
        if(!keep_running(s,1)) return;
        /* 以当前已接受的候选位姿作为本轮线性化中心。 */
        SmPose p=candidate->pose;
        /* 计算当前朝向；h初始化为3×3的J转置J，b初始化为负J转置r。 */
        double c=cos(p.theta), sn=sin(p.theta), h[3][3]={{0}}, b[3]={0};
        /* 逐点更新当前阶段需要的缓存、残差或诊断；点数保持不变。 */
        for(size_t i=0;i<s->count;i++) {
            /* 先只旋转端点，rx/ry也是角度导数计算需要的中间量，单位米。 */
            double rx=c*s->points[i].x-sn*s->points[i].y, ry=sn*s->points[i].x+c*s->points[i].y;
            /* 在当前世界位置查询L0评分和两个栅格方向导数。 */
            double gx,gy,v=sample(s->field,(p.x+rx-m->origin_x)/m->resolution,
                                  /* 补充栅格y及两个梯度输出地址；精修和最终评分使用同一插值函数。 */
                                  (p.y+ry-m->origin_y)/m->resolution,0,&gx,&gy);
            /* 将每格梯度转换为每米梯度，才能与米制dx/dy步长配合。 */
            gx/=m->resolution; gy/=m->resolution;
            /* 残差取sqrt(1-score)，平方和等价于最大化总评分；下限避免导数除零。 */
            double r=sqrt(fmax(1-v,1e-8));
            /* 链式法则得到残差对x、y、theta的雅可比；角度项利用旋转导数(-ry,rx)。 */
            double j[3]={-gx/(2*r),-gy/(2*r),(gx*ry-gy*rx)/(2*r)};
            /* 累加负J转置r和J转置J，把全部端点贡献合并到小型线性系统。 */
            for(int a=0;a<3;a++) { b[a]-=j[a]*r; for(int z=0;z<3;z++) h[a][z]+=j[a]*j[z]; }
        }
        /* 在对角线上加入阻尼；即使原对角为0，额外的1仍提供正的正则项。 */
        for(int a=0;a<3;a++) h[a][a]+=lambda*(h[a][a]+1);
        /* 初始化步长并解方程；求解退化则退出精修，保留此前候选。 */
        double step[3]={0}; if(!solve3(h,b,step)) break;
        /* 限制本轮x最多移动一格，防止局部线性近似产生过大跳跃。 */
        step[0]=clamp(step[0],-m->resolution,m->resolution);
        /* 同样限制y步长为一格以内。 */
        step[1]=clamp(step[1],-m->resolution,m->resolution);
        /* 单次角度更新最多为配置角度步长，单位弧度。 */
        step[2]=clamp(step[2],-s->cfg->angle_step,s->cfg->angle_step);
        /* 尝试新x，并裁剪到以预测位置为中心的实际搜索半窗。 */
        SmPose q={clamp(p.x+step[0],s->prediction.x-xy,s->prediction.x+xy),
                  /* 裁剪尝试的新y，不允许精修偷偷扩大平移搜索范围。 */
                  clamp(p.y+step[1],s->prediction.y-xy,s->prediction.y+xy),
                  /* 裁剪尝试的新角度，不允许超过配置角度半窗。 */
                  clamp(p.theta+step[2],s->prediction.theta-s->cfg->window_angle,s->prediction.theta+s->cfg->window_angle)};
        /* 用完整点云重新计算真实目标值，不只相信线性方程预测的提升。 */
        double score=sm_score(s->field,s->points,s->count,q);
        /* 试探位置必须不进入已排除峰邻域，而且实际分数要明显提升才接受。 */
        if(eligible(s,q) && score>candidate->score+1e-10) {
            /* 接受更好位姿和分数，同时降低阻尼，让后续步骤更接近高斯牛顿更新。 */
            candidate->pose=q; candidate->score=score; lambda=fmax(lambda*.3,1e-8);
        /* 试探没有改善时保留旧候选并增大阻尼；阻尼过大说明继续尝试收益有限，结束精修。 */
        } else { lambda*=10; if(lambda>1e8) break; }
    }
}
/* 将世界米制坐标映射到最近格子中心，主要供覆盖率及射线诊断使用。 */
static int grid_index(const SmMap *m,double x,double y,int *ix,int *iy) {
    /* 减格子0中心并除分辨率，转换为浮点栅格索引。 */
    double gx=(x-m->origin_x)/m->resolution, gy=(y-m->origin_y)/m->resolution;
    /* 有效格子的几何范围包含中心两侧半格；范围外不能访问原始占据地图。 */
    if(!isfinite(gx)||!isfinite(gy)||gx<-.5||gy<-.5||gx>=m->width-.5||gy>=m->height-.5) return 0;
    /* 按最近中心取整并写入两个输出索引；这里和sample的连续插值用途不同。 */
    *ix=(int)floor(gx+.5); *iy=(int)floor(gy+.5); return 1;
}
/* 为候选计算已知覆盖率、近障比例、射线穿障比例和搜索边界标志。 */
static void diagnostics(Search *s,SmCandidate *v) {
    /* 读取本次搜索所用地图尺寸、原点和分辨率。 */
    const SmMap *m=s->field->map;
    /* 缓存候选朝向，known/near分别统计满足条件的端点数量。 */
    double c=cos(v->pose.theta),sn=sin(v->pose.theta),known=0,near=0;
    /* 把配置的近障米制距离换算为对应高斯评分阈值，供近障比例判断。 */
    double threshold=exp(-s->cfg->hit_distance*s->cfg->hit_distance/(2*s->field->sigma*s->field->sigma));
    /* 分别统计具有可判定已知区域的射线数，以及其中穿过旧障碍的射线数。 */
    int rays=0,conflicts=0;
    /* 逐点更新当前阶段需要的缓存、残差或诊断；点数保持不变。 */
    for(size_t i=0;i<s->count;i++) {
        /* 按节流策略检查取消；已取消时不再计算当前候选或子块。 */
        if(!keep_running(s,0)) return;
        /* 取当前局部端点副本，不修改输入点云。 */
        SmPoint p=s->points[i];
        /* 把当前端点按候选位姿变换到世界坐标，用于原始地图及评分查询。 */
        double x=v->pose.x+c*p.x-sn*p.y,y=v->pose.y+sn*p.x+c*p.y;
        /* 存放离散格子索引；只有grid_index返回成功才允许使用。 */
        int ix,iy;
        /* 端点在地图内且占据值不是128，才计入已知覆盖；空闲格也属于已知。 */
        if(grid_index(m,x,y,&ix,&iy) && map_cell(m,ix,iy)!=128) known++;
        /* 端点评分达到近障阈值才计数，不要求端点恰好落在障碍中心。 */
        if(sample(s->field,(x-m->origin_x)/m->resolution,(y-m->origin_y)/m->resolution,0,NULL,NULL)>=threshold) near++;
        /* 按需接收这一束射线的局部起点，避免长期保存每点double射线数组。 */
        Point2D origin;
        /* 没有射线信息或回调取不到当前起点时，跳过穿障诊断，不影响端点评分。 */
        if(!s->cfg->ray_origin || !s->cfg->ray_origin(s->cfg->ray_user,i,&origin)) continue;
        /* 无效或过远的射线起点不参与诊断，避免异常采样长度。 */
        if(!isfinite(origin.x) || !isfinite(origin.y) || hypot(origin.x,origin.y)>1000) continue;
        /* 把射线起点也用同一候选刚体变换映射到世界坐标。 */
        double ox=v->pose.x+c*origin.x-sn*origin.y,oy=v->pose.y+sn*origin.x+c*origin.y;
        /* 计算起点到端点的射线长度，单位米。 */
        double length=hypot(x-ox,y-oy);
        /* 短射线剔除两端邻域后几乎没有内部区域，不做穿障统计。 */
        if(length<=3*m->resolution) continue;
        /* 按半格间距划分射线；valid/conflict是本条射线的布尔标记。 */
        int n=(int)ceil(length/(m->resolution*.5)),valid=0,conflict=0;
        /* 仅诊断：沿每条真实采样射线检查旧障碍，忽略端点前两格及原点附近。
         * 不参与默认接受判定，避免晚期地图/外参代理导致误拒绝。 */
        /* 只采样射线内部，端点k=n和起点k=0不直接查询。 */
        for(int k=1;k<n;k++) {
            /* 计算沿射线从0到1的插值比例，显式转换避免整数除法。 */
            double t=(double)k/n;
            /* 忽略起点附近一格及端点附近两格，降低传感器自身和目标墙面的误报。 */
            if(t*length<m->resolution || (1-t)*length<2*m->resolution) continue;
            /* 把射线采样位置映射到原图索引；越界样本不参与判断。 */
            if(grid_index(m,ox+t*(x-ox),oy+t*(y-oy),&ix,&iy)) {
                /* 读取射线路径上的原始占据值，不把高斯晕圈误当作真实障碍。 */
                uint8_t occ=map_cell(m,ix,iy);
                /* 路径上出现已知格则此射线有效；任一已知格超过障碍阈值就标记穿障。 */
                if(occ!=128) { valid=1; if(occ>s->field->occupied_threshold) conflict=1; }
            }
        }
        /* 每条有效射线只计一次，穿过多个障碍格也只增加一次冲突计数。 */
        if(valid) { rays++; conflicts+=conflict; }
    }
    /* 将端点计数除以总点数，得到两个0..1比例。 */
    v->known_ratio=known/s->count; v->near_ratio=near/s->count;
    /* 记录有效射线数；没有有效射线时冲突比例为0，但不能解释为已证实无冲突。 */
    v->ray_valid_count=rays; v->ray_conflict_ratio=rays ? (double)conflicts/rays : 0;
    /* 计算搜索半窗的实际米数，可能略小于配置值。 */
    double xy=s->nx*m->resolution;
    /* 只要最佳位置贴近平移窗口任一边界，就标记可能需要更大搜索范围。 */
    v->boundary=(s->nx>0 && (fabs(v->pose.x-s->prediction.x)>=xy-1e-6 || fabs(v->pose.y-s->prediction.y)>=xy-1e-6)) ||
                /* 角度贴近半窗端点同样属于边界；零角度窗口不因固定角度自动触发。 */
                (s->cfg->window_angle>0 && fabs(v->pose.theta-s->prediction.theta)>=s->cfg->window_angle-1e-6);
}
/* 验证搜索、精修与接受参数；构场参数另由field_config_valid检查。 */
static int valid_config(const SmConfig *c,const SmMap *m) {
    /* 先检查配置指针，再读取各个阈值。 */
    if(!c) return 0;
    /* 收集需要有限数检查的连续配置值，避免NaN使比较结果失真。 */
    const double nums[]={c->window_xy,c->window_angle,c->angle_step,c->distinct_xy,c->distinct_angle,
        /* 补齐接受阈值和近障距离，所有这些数都需要isfinite。 */
        c->min_score,c->min_near_ratio,c->min_known_ratio,c->min_margin,c->hit_distance};
    /* 遍历配置数组，任一NaN/无穷即判无效。 */
    for(size_t i=0;i<sizeof(nums)/sizeof(nums[0]);i++) if(!isfinite(nums[i])) return 0;
    /* 平移半窗非负且至多512格，角度半窗限制在0..π。 */
    return c->window_xy>=0 && c->window_xy/m->resolution<=512 && c->window_angle>=0 && c->window_angle<=PI &&
           /* 角度步长为正且单侧至多3600步；峰分离距离和角度必须为正。 */
           c->angle_step>0 && c->window_angle/c->angle_step<=3600 && c->distinct_xy>0 && c->distinct_angle>0 &&
           /* 最低分及两个端点比例都必须位于概率式的0..1数值区间，但得分不是概率。 */
           c->min_score>=0 && c->min_score<=1 && c->min_near_ratio>=0 && c->min_near_ratio<=1 &&
           /* 继续验证覆盖阈值与峰间分差阈值。 */
           c->min_known_ratio>=0 && c->min_known_ratio<=1 && c->min_margin>=0 && c->min_margin<=1 &&
           /* 近障距离非负，点数门槛为正，精修轮数在0..100以内。 */
           c->hit_distance>=0 && c->min_points>0 && c->refine_iterations>=0 && c->refine_iterations<=100;
}
/* 主入口：输入评分场、局部点云、预测位姿和配置；work由调用者提供，r接收结果。 */
int sm_match(const SmField *f,const SmPoint *p,size_t count,SmPose prediction,const SmConfig *c,
             /* bytes必须覆盖旋转缓存；trace/user为可选跟踪接口，不参与接受判断。 */
             void *work,size_t bytes,SmTrace trace,void *user,SmResult *r) {
    /* 短路检查先保证评分场及配置有效，再检查点云、输出和工作区指针。 */
    if(!field_valid(f) || !valid_config(c,f->map) || !p || !r || !work ||
       /* 同时验证点数上限和实际工作区容量；非法输入统一返回-1。 */
       !sm_workspace_bytes(count) || bytes<sm_workspace_bytes(count) ||
       /* 预测位姿不能含NaN或无穷。 */
       !isfinite(prediction.x)||!isfinite(prediction.y)||!isfinite(prediction.theta) ||
       /* 额外限制位置和角度绝对值，避免异常输入放大计算范围；这是数值保护而非搜索半径。 */
       fabs(prediction.x)>10000 || fabs(prediction.y)>10000 || fabs(prediction.theta)>10000) return -1;
    /* 依次检查或处理全部输入端点，不在评分阶段挑选有利点。 */
    for(size_t i=0;i<count;i++) {
        /* 逐点检查端点为有限数且距离参考原点不超过1000m。 */
        if(!isfinite(p[i].x)||!isfinite(p[i].y)||hypot(p[i].x,p[i].y)>1000) return -1;
    }
    /* 清空输出，候选数、统计量、拒绝原因和接受标志从零开始。 */
    memset(r,0,sizeof(*r));
    /* 创建栈上的搜索上下文并清零，尤其要初始化排除计数和取消标志。 */
    Search s; memset(&s,0,sizeof(s));
    /* 绑定借用的输入及旋转工作区，复制预测位姿；不复制地图和整份点云。 */
    s.field=f;s.cfg=c;s.points=p;s.count=count;s.prediction=prediction;s.rotated=work;
    /* 半窗除分辨率得到单侧格数并向下取整；1e-9抵消整倍数附近的浮点误差。 */
    s.nx=(int)floor(c->window_xy/f->map->resolution+1e-9);
    /* 向上取整单侧角度步数，保证实际步长不超过配置步长且覆盖两端。 */
    s.angle_half=(int)ceil(c->window_angle/c->angle_step);
    /* 用半窗重新均分得到实际角度步长；半窗为0时步长置0，避免除零。 */
    s.angle_step=s.angle_half ? c->window_angle/s.angle_half : 0;
    /* 保存跟踪回调、用户上下文和结果指针，内部各阶段通过s共享它们。 */
    s.trace=trace;s.user=user;s.result=r;
    /* 记录真正执行的平移半窗与角度步长，便于报告正确还原搜索坐标。 */
    r->actual_window_xy=s.nx*f->map->resolution;r->actual_angle_step=s.angle_step;
    /* 先按预测位姿评分，供最后比较匹配前后贴合程度；不会改变搜索目标。 */
    r->predicted_score=sm_score(f,p,count,prediction);
    /* 每找到一个候选就先精修，再排除其邻域重新搜索。
     * 备选精修也不能进入已选候选邻域，避免三个“第二峰”实际全收敛回第一峰。
     * 每轮保证排除区域之外的离散最优；不宣称连续空间的全局第二峰置信度。 */
    /* 最多寻找三个不同候选峰，每轮先离散搜索再连续精修。 */
    for(int k=0;k<SM_CANDIDATES;k++) {
        /* 没有找到更多候选就退出；取消状态会在循环后单独处理。 */
        if(!search_pass(&s,&r->candidates[k])) break;
        /* 只在第一轮保存未排除任何区域的离散最优，作为精修效果和回退的参考。 */
        if(!k) { r->discrete_best=r->candidates[k].pose; r->discrete_score=r->candidates[k].score; }
        /* 保留本轮离散候选副本，整数格输出模式需要防止取整后的结果反而更差。 */
        SmCandidate discrete_candidate=r->candidates[k];
        /* 在原窗口和排除约束内进行连续局部精修，只有真实评分上升才更新。 */
        refine(&s,&r->candidates[k]);
        /* 旧GridPoint兼容模式才量化修正；PC连续仿真默认不进入此分支。 */
        if(c->grid_output) {
            /* 按真正送给旧接口的修正量评分，不能把连续位置的分数用于整数格位置。
             * 量化发生在排除邻域之前，备选峰、排序和接受判断都使用实际输出。 */
            /* 取得当前候选可写引用，后续量化直接更新这个候选。 */
            SmCandidate *v=&r->candidates[k];
            /* 缓存每格米数，用于米制修正与整数格之间的转换。 */
            double res=f->map->resolution;
            /* 先将x修正除分辨率取整，再乘回米数；量化的是相对预测修正，不是绝对地图位置。 */
            v->pose.x=prediction.x+round((v->pose.x-prediction.x)/res)*res;
            /* 对y做同样的整数格修正量化。 */
            v->pose.y=prediction.y+round((v->pose.y-prediction.y)/res)*res;
            /* 角度修正转float以匹配GridPoint.theta，再加回预测角度用于实际评分。 */
            v->pose.theta=prediction.theta+(float)(v->pose.theta-prediction.theta);
            /* 按量化后的位姿重新评分，绝不能沿用量化前更高的连续分数。 */
            v->score=sm_score(f,p,count,v->pose);
            /* 连续精修后直接取整可能比原离散最优更差，甚至错误切换到次峰。
             * 保留可直接表达的离散候选作为下限；角度也按GridPoint的float重评。 */
            /* 离散回退候选也按旧接口可表达的角度精度处理。 */
            discrete_candidate.pose.theta=prediction.theta+
                /* 只降低角度修正的存储精度，不将整个绝对角度先转换为float。 */
                (float)(discrete_candidate.pose.theta-prediction.theta);
            /* 重新评分实际可表达的离散候选，确保比较目标一致。 */
            discrete_candidate.score=sm_score(f,p,count,discrete_candidate.pose);
            /* 若回退候选更好就完整替换当前候选；这里可能改变位置及角度，而不只是四舍五入。 */
            if(discrete_candidate.score>v->score) *v=discrete_candidate;
        }
        /* 把当前最终候选加入排除列表，并增加有效候选数，下一轮寻找其他峰。 */
        s.excluded[s.excluded_count++]=r->candidates[k].pose;r->candidate_count++;
    }
    /* 搜索或诊断取消时强制拒绝并记录SM_INTERRUPTED，返回-3，不应用部分候选。 */
    if(s.cancelled) { r->accepted=0;r->reasons|=SM_INTERRUPTED;return -3; }
    /* 没有任何候选返回-2，与分数低而拒绝的正常完成情况区分。 */
    if(!r->candidate_count) return -2;
    /* 对少量候选做两两比较排序，保证第0个是最终分数最高者。 */
    for(int k=0;k<r->candidate_count;k++) for(int j=k+1;j<r->candidate_count;j++)
        /* 后面的候选分数更高时需要交换，不能假设先找到的峰精修后仍是第一名。 */
        if(r->candidates[j].score>r->candidates[k].score) {
            /* 用临时结构体交换两个候选，包括位置、评分及诊断字段。 */
            SmCandidate t=r->candidates[k];r->candidates[k]=r->candidates[j];r->candidates[j]=t;
        }
    /* 数值容差下再去重，避免边界浮点误差产生重合候选。 */
    /* 记录去重后已写入数组前部的候选数量。 */
    int kept=0;
    /* 逐个检查排序后的候选，将独立峰压缩到数组前部。 */
    for(int k=0;k<r->candidate_count;k++) {
        /* 先假定当前候选独立，随后与已保留的所有候选比较。 */
        int unique=1;
        /* 只要落入某个已保留峰的邻域，就不再作为独立备选保留。 */
        for(int j=0;j<kept;j++) if(!distinct(r->candidates[k].pose,r->candidates[j].pose,c)) unique=0;
        /* 原位保留独立候选并增加数量，无需第二份候选数组。 */
        if(unique) r->candidates[kept++]=r->candidates[k];
    }
    /* 更新去重后的真实候选数，后续诊断和输出只访问这些元素。 */
    r->candidate_count=kept;
    /* 统一按最终位姿计算覆盖、近障、射线与边界诊断，避免使用精修前的旧统计。 */
    for(int k=0;k<kept;k++) diagnostics(&s,&r->candidates[k]);
    /* 搜索或诊断取消时强制拒绝并记录SM_INTERRUPTED，返回-3，不应用部分候选。 */
    if(s.cancelled) { r->accepted=0;r->reasons|=SM_INTERRUPTED;return -3; }
    /* 两峰以上取前两名分差；没有第二峰时用-1表示分差不可用，而非无限置信度。 */
    r->margin=kept>1 ? r->candidates[0].score-r->candidates[1].score : -1;
    /* 排序后的第0个候选用于最终接受判断。 */
    const SmCandidate *best=&r->candidates[0];
    /* 贴合分不足时设置低分位；按位或允许同时记录多个拒绝原因。 */
    if(best->score<c->min_score) r->reasons|=SM_LOW_SCORE;
    /* 太多端点落在未知或地图外时设置覆盖不足位。 */
    if(best->known_ratio<c->min_known_ratio) r->reasons|=SM_LOW_COVERAGE;
    /* 靠近障碍的端点比例不足时设置近障不足位。 */
    if(best->near_ratio<c->min_near_ratio) r->reasons|=SM_LOW_NEAR;
    /* 只有分差可用才检查歧义；最高分与第二峰太接近时拒绝。 */
    if(r->margin>=0 && r->margin<c->min_margin) r->reasons|=SM_AMBIGUOUS;
    /* 最佳候选触及搜索边界，可能仍存在窗口外更好位置，因此拒绝。 */
    if(best->boundary) r->reasons|=SM_BOUNDARY;
    /* 输入点数低于核心配置门槛时拒绝；旧tryMatchEx还可能有更早的25点门槛。 */
    if(count<(size_t)c->min_points) r->reasons|=SM_FEW_POINTS;
    /* 没有任何拒绝位才接受；函数返回0只表示流程完成，调用者仍必须检查accepted。 */
    r->accepted=r->reasons==0;
    /* 正常完成；此处的0是函数状态码，不能据此认定匹配结果已被接受。 */
    return 0;
}
