/* 导入真实适配器和toflib类型，验证布局与转换约定。 */
#include "toflib_adapter.h"
/* Release通常会定义NDEBUG，下面主动恢复assert以免测试被优化成空操作。 */
#ifdef NDEBUG
/* 启用断言；本测试的部分断言内包含实际被测调用，必须执行。 */
#undef NDEBUG
/* 结束条件编译，后续测试在Debug和Release下都运行。 */
#endif
/* 断言条件不成立时立即中止测试，向CTest返回失败。 */
#include <assert.h>
/* 读取场景及输出测试摘要。 */
#include <stdio.h>
/* 测试侧申请和释放缓存，核心自身仍无malloc。 */
#include <stdlib.h>
/* 初始化测试地图、哨兵和结构体。 */
#include <string.h>

/* 使用真实固件头文件；测试用大数组放静态区，避免占任务栈。 */
/* 测试用真实共享地图对象，大数组放静态区，避免过大栈占用。 */
static map_data_t raw;
/* 保存原始RobotData测试采样，布局与固件一致。 */
static DynamicArray scan;
/* 独立概率图对象，故意与共享占据图分开以验证输入选择。 */
static GridValueMap probability;
/* 独立行优先地图副本，作为适配器[x][y]视图的参照。 */
static uint8_t row_map[MAX_MAP_SIZE*MAX_MAP_SIZE];
/* converted由适配器生成，reference由测试独立变换生成。 */
static SmPoint converted[MAX_SCAN_SIZE], reference[MAX_SCAN_SIZE];
/* 旋转工作区与射线起点参照数组，均采用真实Point2D布局。 */
static Point2D rotated[MAX_SCAN_SIZE], origins[MAX_SCAN_SIZE];
/* 从测试独立计算的起点数组取值，验证适配层的按需射线变换。 */
static int reference_ray(void *u,size_t i,Point2D *out) {(void)u;*out=origins[i];return 1;}
/* 控制测试回调第几次触发取消，分别覆盖构场和搜索阶段。 */
static int cancel_after;
/* 每次减少剩余次数，到0即请求停止。 */
static int cancel_poll(void *u) {(void)u;return --cancel_after>0;}
/* 记录加解锁次数；deny用于模拟加锁失败。 */
static int acquired, released, deny;
/* 验证锁上下文，累计获取次数，按deny控制是否成功。 */
static int acquire(void *u) { assert(u==&raw); acquired++; return deny; }
/* 验证释放的是同一个上下文，并记录释放次数。 */
static void release(void *u) { assert(u==&raw); released++; }

/* 逐项对比两种路径的核心结果，包括次峰和诊断，不只比总分。 */
static void compare(const SmResult *a, const SmResult *b) {
    /* 接受状态和拒绝位必须完全一致。 */
    assert(a->accepted==b->accepted && a->reasons==b->reasons);
    /* 候选数量必须一致，否则可能漏峰或重复峰。 */
    assert(a->candidate_count==b->candidate_count);
    /* 对照匹配前评分，检查地图或预测坐标是否接错。 */
    assert(fabs(a->predicted_score-b->predicted_score)<1e-11);
    /* 对照第一轮离散评分，检查搜索输入布局一致性。 */
    assert(fabs(a->discrete_score-b->discrete_score)<1e-11);
    /* 检查次峰分差一致，避免适配后歧义判定悄悄变化。 */
    assert(fabs(a->margin-b->margin)<1e-11);
    /* 逐排名核对全部保留候选。 */
    for(int i=0;i<a->candidate_count;i++) {
        /* 分别取适配路径与参照路径的同名次候选。 */
        const SmCandidate *x=&a->candidates[i], *y=&b->candidates[i];
        /* 两条路径的候选评分应只允许微小数值容差。 */
        assert(fabs(x->score-y->score)<1e-11);
        /* 候选x坐标应一致，防止地图原点符号出错。 */
        assert(fabs(x->pose.x-y->pose.x)<1e-10);
        /* 候选y坐标应一致，防止转置或stride错误。 */
        assert(fabs(x->pose.y-y->pose.y)<1e-10);
        /* 候选角度应一致，防止正逆旋转混淆。 */
        assert(fabs(x->pose.theta-y->pose.theta)<1e-10);
        /* 检查覆盖与近障统计也完全一致。 */
        assert(x->known_ratio==y->known_ratio && x->near_ratio==y->near_ratio);
        /* 射线冲突比例及有效条数一致，验证外参只作用于正确位置。 */
        assert(x->ray_conflict_ratio==y->ray_conflict_ratio && x->ray_valid_count==y->ray_valid_count);
        /* 边界判定一致，保证适配不会改变接受规则。 */
        assert(x->boundary==y->boundary);
    }
}

/* 共同测试流程：建立两种地图布局，分别匹配并检查结果、padding及失败路径。 */
static void check(Robot *robot, double resolution, SmConfig cfg, int invalid_tests) {
    /* 接收适配器输出的零拷贝地图视图。 */
    SmMap view;
    /* 实际调用地图适配器并检查成功。 */
    assert(sm_tof_map_view(robot,&probability,resolution,&view)==0);
    /* 确认读取的是概率图而不是robot原始占据图。 */
    assert(view.cells==(const uint8_t *)probability.map_data);
    /* 验证[x][y]布局：x跨度为整行最大边长，y跨度为1。 */
    assert(view.stride_x==MAX_MAP_SIZE && view.stride_y==1);
    /* 根据固件地图偏移独立核对世界x原点。 */
    assert(view.origin_x==-(robot->size/2+raw.x_offset)*resolution);
    /* 同样核对世界y原点。 */
    assert(view.origin_y==-(robot->size/2+raw.y_offset)*resolution);
    /* 复制几何元信息，只更换底层布局用于独立对照。 */
    SmMap ordinary=view;
    /* 把对照地图设为连续行优先布局，stride同时为0。 */
    ordinary.cells=row_map;ordinary.stride_x=ordinary.stride_y=0;
    /* 逐格转置复制原始概率值，仅测试基准需要这份副本。 */
    for(int y=0;y<view.height;y++) for(int x=0;x<view.width;x++)
        /* 将toflib[x][y]值写入标准[y][x]偏移。 */
        row_map[y*view.width+x]=probability.map_data[x][y];
    /* 分别计算四层场和当前点数旋转缓存容量。 */
    size_t fb=sm_field_bytes(&view),wb=sm_workspace_bytes(scan.size);
    /* 测试侧分配各组缓存，避免适配结果与参考结果共享评分数组。 */
    void *fs=malloc(fb),*baseline_fs=malloc(fb),*work=malloc(wb);
    /* 检查测试内存都申请成功，否则中止避免把分配失败误判算法错误。 */
    assert(fs && baseline_fs && work);
    /* 先清空适配工作区描述，随后逐项绑定容量与指针。 */
    SmTofMemory memory={0};
    /* 绑定转换端点缓存并明确容量以点为单位。 */
    memory.points=converted;memory.point_capacity=MAX_SCAN_SIZE;
    /* 绑定另一份旋转缓存，不能与输入点云重叠。 */
    memory.rotated=rotated;memory.rotated_capacity=MAX_SCAN_SIZE;
    /* 模拟旧库不连续、带padding的层，外扩部分用哨兵检查不被覆盖。 */
    /* 逐层分配或释放独立行缓存，模拟真实toflib非连续存储。 */
    for(int l=0;l<SM_LEVELS;l++) {
        /* 按层窗口边长额外分配padding，验证核心只写活动区域。 */
        size_t dim=view.width+(1u<<l)-1;
        /* 设置可用边长并分配行指针表，每行实际数据随后独立申请。 */
        memory.dimensions[l]=dim;memory.levels[l]=malloc(dim*sizeof(uint8_t *));assert(memory.levels[l]);
        /* 每行填0xA5哨兵，构场后padding若被改写就能被断言发现。 */
        for(size_t x=0;x<dim;x++) { memory.levels[l][x]=malloc(dim);assert(memory.levels[l][x]);memset(memory.levels[l][x],0xA5,dim); }
    }
    /* 绑定测试计数回调和固定上下文，验证异常路径正确解锁。 */
    SmTofLock lock={acquire,release,&raw};
    /* 以同一份float预测输入提升到double，避免把不同输入精度误当适配误差。 */
    SmPose pred={robot->position.x,robot->position.y,robot->position.theta};
    /* 独立使用世界坐标→参考系的旋转角差计算射线原点，检查外参未加到端点。 */
    /* 逐点计算参照变换或对照转换输出。 */
    for(size_t i=0;i<scan.size;i++) {
        /* 取采样记录副本，原始输入不被测试计算修改。 */
        RobotData p=scan.data[i];
        /* 计算预测参考系的逆旋转参数。 */
        double c=cos(pred.theta),s=sin(pred.theta);
        /* 独立计算端点相对预测位置的double差值。 */
        double dx=(double)p.point_cloud_x-pred.x,dy=(double)p.point_cloud_y-pred.y;
        /* 将端点逆旋转得到参照局部点，和适配器输出逐值对照。 */
        reference[i].x=c*dx+s*dy;reference[i].y=c*dy-s*dx;
        /* 改用采样机器人位置构造射线起点参考。 */
        dx=(double)p.robot_x-pred.x;dy=(double)p.robot_y-pred.y;
        /* 采样角减参考角得到安装外参在参考系中的方向。 */
        double a=p.robot_theta-pred.theta;
        /* 机器人位移逆旋转后加上旋转的安装x/y外参，计算射线起点x。 */
        origins[i].x=c*dx+s*dy+.142*cos(a)-.031*sin(a);
        /* 按同一几何关系计算射线起点y。 */
        origins[i].y=c*dy-s*dx+.142*sin(a)+.031*cos(a);
    }
    /* 接收适配器完整结果。 */
    SmTofResult out;
    /* 执行raw转换路径，并按当前测试配置校验返回码。 */
    assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==0);
    /* 确认成功或失败后都没有漏锁；同一行可能恢复被临时修改的容量。 */
    assert(acquired==released);
    /* 现有generateScan产物可直接借用；不要第二次转换或另申请端点数组。 */
    /* 用已转换端点构造可借用视图，检查已有generateScan路径。 */
    SingleFrameScan existing={converted,scan.size};
    /* 复制工作区描述并故意清空points，证明借用路径不依赖第二份端点缓存。 */
    SmTofResult borrowed;SmTofMemory borrowed_memory=memory;borrowed_memory.points=NULL;
    /* 执行借用单帧路径，要求正常完成。 */
    assert(sm_tof_match_scan(robot,&probability,&existing,&scan,resolution,.142,.031,&cfg,
                            &borrowed_memory,&lock,&borrowed)==0);
    /* 两种适配入口输入相同，应得到完全相同候选和诊断。 */
    compare(&out.match,&borrowed.match);
    /* 逐点计算参照变换或对照转换输出。 */
    for(size_t i=0;i<scan.size;i++) {
        /* 核对每个局部x端点，不能只依赖最终分数间接判断转换正确。 */
        assert(reference[i].x==converted[i].x);
        /* 核对每个局部y端点。 */
        assert(reference[i].y==converted[i].y);

    }
    /* 独立连续布局评分场作为逐字节与结果对照。 */
    SmField baseline;
    /* 使用相同参数为转置后的普通地图构场。 */
    assert(sm_build_field(&ordinary,&cfg,baseline_fs,fb,&baseline)==0);
    /* 遍历所有层和padding，分别检查有效数据与未写区域。 */
    for(int l=0;l<SM_LEVELS;l++) for(size_t x=0;x<memory.dimensions[l];x++) for(size_t y=0;y<memory.dimensions[l];y++) {
        /* 有效区域需要与连续评分场逐字节一致。 */
        if(x<(size_t)view.width && y<(size_t)view.height)
            /* 同一层同一逻辑坐标评分必须一致，直接排查层偏移或转置错误。 */
            assert(memory.levels[l][x][y]==((uint8_t *)baseline_fs)[l*view.width*view.height+y*view.width+x]);
        /* 地图外padding必须保持哨兵值，避免损坏旧库额外缓存。 */
        else assert(memory.levels[l][x][y]==0xA5);
    } /* 四层逐字节一致，padding保持原值 */
    /* 给独立核心绑定测试计算的射线起点，以核对适配回调。 */
    cfg.ray_origin=reference_ray;cfg.ray_user=NULL;
    /* 存放独立核心的预期结果。 */
    SmResult expected;
    /* 使用相同端点与配置运行独立核心，并检查正常或取消返回码。 */
    assert(sm_match(&baseline,reference,scan.size,pred,&cfg,work,wb,NULL,NULL,&expected)==0);
    /* 比较适配路径和独立核心的全部候选及诊断。 */
    compare(&out.match,&expected);
    /* 接受和拒绝的公开输出有不同语义，分别验证。 */
    if(out.match.accepted) {
        /* 接受时公开float位置应来自最佳double候选的最终转换。 */
        assert(out.pose.x==(float)out.match.candidates[0].pose.x);
        /* 对y同样检查边界类型转换。 */
        assert(out.pose.y==(float)out.match.candidates[0].pose.y);
        /* 米制修正应等于最终候选减预测位置，允许float出口误差。 */
        assert(fabs(out.correction.x-(out.match.candidates[0].pose.x-pred.x))<1e-6);
    /* 进入相反状态的验证分支，不能把拒绝候选当作可应用修正。 */
    } else {
        /* 正常完成但拒绝时保留预测位姿，不能泄漏未接受的位置。 */
        assert(out.pose.x==robot->position.x && out.pose.y==robot->position.y && out.pose.theta==robot->position.theta);
        /* 拒绝时修正必须全零。 */
        assert(out.correction.x==0 && out.correction.y==0 && out.correction.theta==0);
    }
    /* 输出当前输入的点数、接受状态、分数和修正供日志回归记录。 */
    printf("PASS adapter: points=%u accepted=%d score=%.9f dx=%.6f dy=%.6f\n",
           scan.size,out.match.accepted,out.match.candidates[0].score,
           out.match.candidates[0].pose.x-pred.x,out.match.candidates[0].pose.y-pred.y);
    /* 只有内置用例执行额外异常注入，真实日志批量回放避免重复这些检查。 */
    if(invalid_tests) {
        /* 保存加锁次数，随后模拟一次获取失败。 */
        int n=acquired;
        /* 让锁获取回调返回失败，要求适配器返回-4。 */
        deny=1;assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==-4);
        /* 锁失败只增加获取尝试，不应执行解锁；随后恢复计数和允许状态。 */
        assert(acquired==n+1 && released==n);deny=0;acquired--;
        /* 故意提供不足旋转容量，适配器必须拒绝。 */
        memory.rotated_capacity=0;
        /* 执行raw转换路径，并按当前测试配置校验返回码。 */
        assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==-1);
        /* 确认成功或失败后都没有漏锁；同一行可能恢复被临时修改的容量。 */
        assert(acquired==released);memory.rotated_capacity=MAX_SCAN_SIZE;
        /* 原始转换路径故意少给一个端点容量，检查边界。 */
        memory.point_capacity=scan.size-1;
        /* 执行raw转换路径，并按当前测试配置校验返回码。 */
        assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==-1);
        /* 恢复正常容量，避免影响下一项独立异常测试。 */
        memory.point_capacity=MAX_SCAN_SIZE;
        /* 保存首点机器人角度后注入NaN，验证射线相关输入校验。 */
        float saved=scan.data[0].robot_theta;scan.data[0].robot_theta=NAN;
        /* 执行raw转换路径，并按当前测试配置校验返回码。 */
        assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==-1);
        /* 恢复原采样，并再次确认加解锁平衡。 */
        scan.data[0].robot_theta=saved;assert(acquired==released);
        /* 故意只清一个stride，验证不完整自定义布局被拒绝。 */
        SmMap bad=view;bad.stride_x=0;assert(sm_field_bytes(&bad)==0);
        /* 注入会导致地址乘法溢出的巨大步长，报价函数应返回0。 */
        bad=view;bad.stride_x=SIZE_MAX;assert(sm_field_bytes(&bad)==0);
        /* 构场及搜索取消均须拒绝；已获取的锁必须归还。 */
        /* 复制配置并接计数取消回调，第一次构场检查就请求停止。 */
        SmConfig cancelled=cfg;cancelled.poll=cancel_poll;cancelled.poll_user=NULL;cancel_after=1;
        /* 执行raw转换路径，并按当前测试配置校验返回码。 */
        assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cancelled,&memory,&lock,&out)==-3);
        /* 取消必须拒绝、带中断位并正确释放锁。 */
        assert(!out.match.accepted && (out.match.reasons&SM_INTERRUPTED) && acquired==released);
        /* 让独立核心在后续检查时取消，覆盖搜索中断路径。 */
        cancel_after=2;
        /* 使用相同端点与配置运行独立核心，并检查正常或取消返回码。 */
        assert(sm_match(&baseline,reference,scan.size,pred,&cancelled,work,wb,NULL,NULL,&expected)==-3);
        /* 独立核心取消也不能返回可应用结果。 */
        assert(!expected.accepted && (expected.reasons&SM_INTERRUPTED));
        /* 故意只保留加锁不提供解锁，检查成对接口验证。 */
        lock.release=NULL;
        /* 执行raw转换路径，并按当前测试配置校验返回码。 */
        assert(sm_tof_match(robot,&probability,&scan,resolution,.142,.031,&cfg,&memory,&lock,&out)==-1);
    }
    /* 逐层分配或释放独立行缓存，模拟真实toflib非连续存储。 */
    for(int l=0;l<SM_LEVELS;l++) {
        /* 释放每一行，再由下一句释放该层指针表。 */
        for(size_t x=0;x<memory.dimensions[l];x++) free(memory.levels[l][x]);
        /* 释放本层行指针数组，防止测试反复运行泄漏。 */
        free(memory.levels[l]);
    }
    /* 释放其余测试缓存，不改变被测试的原始数据对象。 */
    free(fs);free(baseline_fs);free(work);
}

/* 根据是否传scene选择真实日志回放或内置构造用例。 */
int main(int argc,char **argv) {
    /* 建立最小机器人视图并绑定测试共享地图对象。 */
    Robot robot={0};robot.map_data=&raw;
    /* 以默认配置启用BnB进行适配一致性验证。 */
    SmConfig cfg=sm_default_config();cfg.use_bnb=1;
    /* 给了scene路径则从真实日志导出的文件构造测试输入。 */
    if(argc==2) {
        /* 日志scene→真实RobotData(float)→适配器；独立行优先核心作为对照。
         * 比较对象使用相同float输入，不把类型量化差异误报为转置/变换错误。 */
        /* 只读打开场景并断言成功，格式后续逐项校验。 */
        FILE *f=fopen(argv[1],"r");assert(f);
        /* 准备格式标记、地图尺寸/原点、预测位姿和点数变量。 */
        char magic[32];int w,h;double res,ox,oy;struct {double x,y,theta;} p;size_t count;
        /* 严格校验场景版本标记，防止按错误协议解释数据。 */
        assert(fscanf(f,"%31s",magic)==1 && !strcmp(magic,"SM_SCENE_V1"));
        /* 必须读满地图宽高、分辨率和原点五项。 */
        assert(fscanf(f,"%d%d%lf%lf%lf",&w,&h,&res,&ox,&oy)==5);
        /* 当前toflib集成回放只接受256正方形地图；若有分辨率判断则也需与固件一致。 */
        assert(w==MAX_MAP_SIZE && h==w);
        /* 读取预测位姿和点数，并确保不超过固件最大点数。 */
        assert(fscanf(f,"%lf%lf%lf%zu",&p.x,&p.y,&p.theta,&count)==4 && count<=MAX_SCAN_SIZE);
        /* 设置活动地图边长，并模拟固件float预测位姿的存储精度。 */
        robot.size=w;robot.position=(Point){p.x,p.y,p.theta};
        /* 从世界原点反解固件x偏移，检查地图零拷贝视图的约定。 */
        raw.x_offset=(int16_t)lround(-ox/res-w/2);
        /* 同样反解固件y偏移。 */
        raw.y_offset=(int16_t)lround(-oy/res-h/2);
        /* 场景文件按[y][x]排列，读入时显式转为toflib的[x][y]。 */
        for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
            /* 逐格读取占据值并检查输入没有提前结束。 */
            int v;assert(fscanf(f,"%d",&v)==1);probability.map_data[x][y]=(uint8_t)v;
        }
        /* 设置真实原始数组的有效条数。 */
        scan.size=count;
        /* 逐点把场景局部端点和射线起点还原为RobotData世界坐标。 */
        for(size_t i=0;i<count;i++) {
            /* 读取四列局部端点/起点，使用double暂存避免文件解析额外误差。 */
            struct {double x,y,ox,oy;} q;assert(fscanf(f,"%lf%lf%lf%lf",&q.x,&q.y,&q.ox,&q.oy)==4);
            /* 预测角用于正变换；a给每个采样不同朝向以验证外参旋转。 */
            double c=cos(p.theta),s=sin(p.theta),a=.17*i;
            /* 将局部射线起点还原为世界坐标。 */
            double wx=p.x+c*q.ox-s*q.oy,wy=p.y+s*q.ox+c*q.oy;
            /* 从射线起点减去世界安装偏移得到机器人采样位置；端点另用正变换恢复。 */
            scan.data[i]=(RobotData){wx-.142*cos(a)+.031*sin(a),
                wy-.142*sin(a)-.031*cos(a),a,p.x+c*q.x-s*q.y,p.y+s*q.x+c*q.y};
        }
        /* 关闭输入，运行公共适配对照流程，日志模式正常后直接退出。 */
        fclose(f);check(&robot,res,cfg,0);return 0;
    }
    /* 非对称地图+非零offset+非256活动尺寸+移动的采样位置，覆盖易错坐标边界。 */
    /* 将未使用地图区域设为未知；若同句调用check则是在验证全未知地图拒绝。 */
    memset(probability.map_data,128,sizeof(probability.map_data));
    /* 使用非256活动边长和非零偏移，检查代码没有写死中心128。 */
    robot.size=48;raw.x_offset=3;raw.y_offset=-5;
    /* 设置非零平移和角度的预测位姿，并指定测试点数。 */
    robot.position=(Point){.12,-.24,.13};scan.size=32;
    /* 将活动区设置为空闲，活动区外保持未知。 */
    for(int x=0;x<48;x++) for(int y=0;y<48;y++) probability.map_data[x][y]=25;
    /* 构造32个非对称地标与移动的采样机器人位置。 */
    for(int i=0;i<32;i++) {
        /* 确定性非对称格点坐标，避免地图自身存在镜像对称峰。 */
        int x=5+(i*7)%37,y=4+(i*i*3)%39;
        /* 标记强障碍，但不写共享原图，检验适配读取了正确地图。 */
        probability.map_data[x][y]=245;
        /* 模拟每束采样时机器人位置/角度变化，端点对应带偏移的地图格中心。 */
        scan.data[i]=(RobotData){.01*i,-.015*i,.12*i,
            (x-24-3)*.12,(y-24+5)*.12};
    }
    /* 额外插入未知格，覆盖特殊值处理。 */
    probability.map_data[7][18]=128;
    /* 缩小测试窗口以快速运行，同时保留多角度搜索。 */
    cfg.window_xy=.48;cfg.window_angle=.1;cfg.angle_step=.05;
    /* 这个几何适配用例降低接受判据干扰，只比较两种实现是否一致。 */
    cfg.min_margin=0;cfg.min_score=.5;cfg.min_near_ratio=0;cfg.min_known_ratio=0;
    /* 运行正常路径和完整错误注入测试。 */
    check(&robot,.12,cfg,1);
    /* 再用枚举验证适配路径，排除只在BnB下恰好一致。 */
    cfg.use_bnb=0;check(&robot,.12,cfg,0);
    /* 将未使用地图区域设为未知；若同句调用check则是在验证全未知地图拒绝。 */
    memset(probability.map_data,128,sizeof(probability.map_data));check(&robot,.12,cfg,0);
    /* 全部断言通过后以0退出，CTest据此标记成功。 */
    return 0;
}
