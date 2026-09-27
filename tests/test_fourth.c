/* 测试第四方法的真实搜索入口与旧应用流程。 */
#include "tof_match_fourth.h"
/* Release通常会定义NDEBUG，下面主动恢复assert以免测试被优化成空操作。 */
#ifdef NDEBUG
/* 启用断言；本测试的部分断言内包含实际被测调用，必须执行。 */
#undef NDEBUG
/* 结束条件编译，后续测试在Debug和Release下都运行。 */
#endif
/* 断言条件不成立时立即中止测试，向CTest返回失败。 */
#include <assert.h>
/* 初始化测试地图、哨兵和结构体。 */
#include <string.h>
/* 测量PC集成入口CPU耗时，不代表BK7258实际耗时。 */
#include <time.h>
/* 测试用真实共享地图对象，大数组放静态区，避免过大栈占用。 */
static map_data_t shared;
/* 独立概率图对象，故意与共享占据图分开以验证输入选择。 */
static GridValueMap probability;
/* 提供旧插图流程需要的局部导航地图对象。 */
static DynamicMap local;
/* 模拟旧系统累计修正共享对象，检查defer_apply不会提前改变它。 */
static GridPoint accumulated;
/* 独立保存旧float坐标变换产生的端点，复核最终应用位置分数。 */
static Point2D points[MAX_SCAN_SIZE];
/* 供独立行优先评分场使用的原图数组。 */
static uint8_t cells[MAX_MAP_SIZE*MAX_MAP_SIZE];

/* 根据是否传scene选择真实日志回放或内置构造用例。 */
int main(int argc,char **argv) {
    /* 设置旧共享地图协议要求的头尾标记。 */
    shared.head_magic=MAP_DATA_HEAD_MAGIC;shared.tail_magic=MAP_DATA_TAIL_MAGIC;
    /* 把旧库全局地图指针绑定到测试对象。 */
    g_matchSharedMapData=&shared;g_proGridMap=&probability;
    /* 绑定累计修正和局部导航对象，供实际应用路径使用。 */
    g_finalCorrected=&accumulated;g_localNavMap=&local;
    /* 先分配和复位旧库，再加载测试数据，避免复位擦掉输入。 */
    initMatchMap();resetMapData();probability.size=MAX_MAP_SIZE;
    /* 确认新增方法默认关闭，不会未经选择改变旧行为。 */
    assert(getMatchAlgorithm()==0);
    /* 未知编号应被拒绝，且不能改变当前算法选择。 */
    assert(setMatchAlgorithm(3)==-1 && getMatchAlgorithm()==0);
    /* 显式启用第四种方法，这一步对应正式固件初始化需要做的选择。 */
    assert(setMatchAlgorithm(4)==0);
    /* 本集成测试使用完整256地图。 */
    g_robot.size=MAX_MAP_SIZE;
    /* 以固件分辨率为默认，读scene后检查两者相符。 */
    double res=RESOLUTION;
    /* 给了scene路径则从真实日志导出的文件构造测试输入。 */
    if(argc==2) {
        /* 只读打开场景并断言成功，格式后续逐项校验。 */
        FILE *f=fopen(argv[1],"r");assert(f);
        /* 准备格式标记、地图尺寸/原点、预测位姿和点数变量。 */
        char magic[32];int w,h;double ox,oy,px,py,pt;size_t n;
        /* 严格校验场景版本标记，防止按错误协议解释数据。 */
        assert(fscanf(f,"%31s",magic)==1 && !strcmp(magic,"SM_SCENE_V1"));
        /* 必须读满地图宽高、分辨率和原点五项。 */
        assert(fscanf(f,"%d%d%lf%lf%lf",&w,&h,&res,&ox,&oy)==5);
        /* 当前toflib集成回放只接受256正方形地图；若有分辨率判断则也需与固件一致。 */
        assert(w==MAX_MAP_SIZE && h==w && fabs(res-RESOLUTION)<1e-8);
        /* 读取预测位姿和点数，并确保不超过固件最大点数。 */
        assert(fscanf(f,"%lf%lf%lf%zu",&px,&py,&pt,&n)==4 && n<=MAX_SCAN_SIZE);
        /* 模拟真实接口对日志预测位姿的float转换。 */
        g_robot.position=(Point){px,py,pt};
        /* 由scene原点反算旧地图x偏移，保持坐标几何一致。 */
        g_robot.map_data->x_offset=lround(-ox/res-w/2);
        /* 由scene原点反算旧地图y偏移。 */
        g_robot.map_data->y_offset=lround(-oy/res-h/2);
        /* 场景文件按[y][x]排列，读入时显式转为toflib的[x][y]。 */
        for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
            /* 逐格读取占据值并检查输入没有提前结束。 */
            int v;assert(fscanf(f,"%d",&v)==1);
            /* 把日志值同时填入概率与共享地图，供实际旧流程运行。 */
            probability.map_data[x][y]=v;g_robot.map_data->occ_map[x][y]=v;
        }
        /* 设置实际有效点数及原始点数，本回放不额外模拟旧滤点步骤。 */
        g_robotAndPointPose.size=g_robotAndPointPose.num_points_before_filtering=n;
        /* 逐条将scene局部数据重建成旧库所需世界坐标记录。 */
        for(size_t i=0;i<n;i++) {
            /* 读取局部端点和起点四列，解析失败立即中止测试。 */
            double x,y,rx,ry;assert(fscanf(f,"%lf%lf%lf%lf",&x,&y,&rx,&ry)==4);
            /* 以预测位姿正变换重建采样位置和端点，构造真实入口可读取的数据。 */
            g_robotAndPointPose.data[i]=(RobotData){px+cos(pt)*rx-sin(pt)*ry,
                py+sin(pt)*rx+cos(pt)*ry,pt,px+cos(pt)*x-sin(pt)*y,py+sin(pt)*x+cos(pt)*y};
        }
        /* 读完场景后关闭文件，后续验证只使用内存对象。 */
        fclose(f);
    /* 进入相反状态的验证分支，不能把拒绝候选当作可应用修正。 */
    } else {
        /* 内置用例使用非格点预测位姿，专门覆盖整数格修正相对量化。 */
        g_robot.position=(Point){.023,.041,.037};
        /* 默认活动概率图为空闲，再逐点设置障碍。 */
        memset(probability.map_data,25,sizeof(probability.map_data));
        /* 内置用例提供40点，超过旧流程25点门槛。 */
        g_robotAndPointPose.size=g_robotAndPointPose.num_points_before_filtering=40;
        /* 逐点构造非对称地标或长墙样本，避免把单一场景当成通用验证。 */
        for(int i=0;i<40;i++) {
            /* 构造不对称地标索引，避免接受判定被简单几何对称干扰。 */
            int x=108+i%15,y=110+(i*i*7)%31;
            /* 将选中格子设为障碍中心。 */
            probability.map_data[x][y]=255;
            /* 给端点加非整数格小偏移，测试量化与实际位置重新评分。 */
            g_robotAndPointPose.data[i]=(RobotData){0,0,0,(x-128)*RESOLUTION-.03f,(y-128)*RESOLUTION+.02f};
        }
    }
    /* 使用旧流程真正使用的float变换，独立重算实际GridPoint输出评分。 */
    /* 独立按照旧generateScan的float公式转换全部端点。 */
    for(size_t i=0;i<g_robotAndPointPose.size;i++) {
        /* 借用原始世界端点记录，不调用待验证的适配转换函数。 */
        RobotData *p=&g_robotAndPointPose.data[i];
        /* 先做旧库相同的float平移差，不能用double参照冒充同一输入。 */
        float dx=p->point_cloud_x-g_robot.position.x,dy=p->point_cloud_y-g_robot.position.y;
        /* 使用旧流程的float三角函数，复核真实送入搜索的点云。 */
        float c=cosf(g_robot.position.theta),sn=sinf(g_robot.position.theta);
        /* 用预测角的逆旋转构造局部端点，作为独立评分输入。 */
        points[i]=(Point2D){dx*c+dy*sn,-dx*sn+dy*c};
    }
    /* 在旧缓存放哨兵，第四方法使用独立缓存后不应改动它。 */
    for(int l=0;l<SM_LEVELS;l++) g_robot.level_map[l][0][0]=0xA5;
    /* 准备旧接口输出，平移单位为格数而非米。 */
    GridPoint result={0};float score=0;
    /* 记录实际旧入口调用前的PC CPU时钟。 */
    clock_t begin=clock();
    /* 调用真实tryMatchEx并设defer_apply=true；旧min/max范围不控制第四方法窗口。 */
    MatchResultStatus_t status=tryMatchEx(&result,0,0,1,false,&score,0,true);
    /* 把CPU时间差换算毫秒，包含构场等入口开销。 */
    double ms=1000.0*(clock()-begin)/CLOCKS_PER_SEC;
    /* 检查旧评分缓存哨兵未改变，确保可安全切回旧方法。 */
    for(int l=0;l<SM_LEVELS;l++) assert(g_robot.level_map[l][0][0]==0xA5);
    /* 取得第四方法最近一次核心诊断，后续按入口状态判断是否可读。 */
    const SmResult *r=getMatchFourthResult();
    /* 延迟应用模式不能提前累加任何修正。 */
    assert(accumulated.x==0 && accumulated.y==0 && accumulated.theta==0);
    /* 只有通过旧点数门槛才执行过搜索，此时才能核对本次候选诊断。 */
    if(status!=NOT_ENOUGH_POINTS) {
        /* 旧返回状态应与新核心accepted一致，不能只用旧分数阈值误判。 */
        assert(status==(r->accepted ? MATCH_SUCCESSFUL : MATCH_FAILED));
        /* 正常执行搜索必须至少输出一个候选。 */
        assert(r->candidate_count>0);
        /* 独立取得同一地图几何视图，为重新评分准备。 */
        SmMap map;assert(sm_tof_map_view(&g_robot,&probability,RESOLUTION,&map)==0);
        /* 逐格复制为普通行优先地图，独立于第四方法的层缓存。 */
        for(int y=0;y<MAX_MAP_SIZE;y++) for(int x=0;x<MAX_MAP_SIZE;x++)
            /* 明确从原[x][y]布局转换成[y][x]连续下标。 */
            cells[y*MAX_MAP_SIZE+x]=probability.map_data[x][y];
        /* 绑定参照地图并恢复默认连续步长。 */
        map.cells=cells;map.stride_x=map.stride_y=0;
        /* 使用同一高斯构场参数建立独立评分场。 */
        SmConfig cfg=sm_default_config();SmField field;
        /* 为独立评分场申请测试内存并确保成功。 */
        void *buf=malloc(sm_field_bytes(&map));assert(buf);
        /* 重新构建参照场，不复用第四方法内部计算结果。 */
        assert(sm_build_field(&map,&cfg,buf,sm_field_bytes(&map),&field)==0);
        /* 把实际GridPoint修正乘分辨率后加预测位姿，得到真正可应用的位置。 */
        SmPose actual={g_robot.position.x+result.x*(double)RESOLUTION,
                      g_robot.position.y+result.y*(double)RESOLUTION,
                      (double)g_robot.position.theta+result.theta};
        /* 在实际应用位姿对同一端点重算平均评分。 */
        double applied_score=sm_score(&field,points,g_robotAndPointPose.size,actual);
        /* 新核心报告分数必须等于实际整数输出的位置分数。 */
        assert(fabs(applied_score-r->candidates[0].score)<1e-11);
        /* 旧float评分出口只允许浮点舍入误差。 */
        assert(fabs(applied_score-score)<1e-7);
        /* 释放这次独立评分验证的临时场。 */
        free(buf);
    }
    /* 用稳定前缀输出机器可读摘要，Python可从旧库调试输出中准确提取。 */
    printf("FOURTH_JSON {\"status\":%d,\"accepted\":%d,\"score\":%.12g,\"dx_cm\":%.8g,\"dy_cm\":%.8g,\"dtheta_deg\":%.8g,\"reasons\":%d,\"points\":%u,\"total_ms\":%.6g}\n",
           status,status==MATCH_SUCCESSFUL,score,result.x*100.0*RESOLUTION,
           result.y*100.0*RESOLUTION,result.theta*180.0/3.141592653589793,
           r->reasons,g_robotAndPointPose.size,ms);
    /* 内置构造场景额外验证原应用/插图路径，真实日志回放保持defer不修改地图。 */
    if(argc==1) {
        /* 该非对称内置用例应被接受，才能继续测试应用路径。 */
        assert(status==MATCH_SUCCESSFUL);
        /* 调用旧应用函数，将真实GridPoint修正累加并插图。 */
        applyMatchResult(&result,0);
        /* 检查三个累计修正分量恰好等于本次实际应用量。 */
        assert(accumulated.x==result.x && accumulated.y==result.y && accumulated.theta==result.theta);
    }
    /* 取消不得泄漏部分输出，也不能通过strong unknown的失败强插图路径。 */
    /* 借用刚才独立计算的局部端点来测试停止入口。 */
    SingleFrameScan scan={points,g_robotAndPointPose.size};
    /* 模拟旧系统下发停止请求。 */
    setMatchStopFlag(true);
    /* 第四方法应返回-3表示取消，不应返回正常候选。 */
    assert(tofMatchFourthSearch(&g_robot,&probability,&scan,&result,&score)==-3);
    /* 取消后接受标志为假，公开score和修正保持零，防止泄漏部分结果。 */
    assert(!getMatchFourthResult()->accepted && score==0 && result.x==0 && result.y==0);
    /* 恢复全局停止标志，以免污染其他测试。 */
    setMatchStopFlag(false);
    /* 释放第四方法按需申请的长期缓存。 */
    releaseMatchFourthWorkspace();
    /* 验证可以显式切回旧搜索。 */
    assert(setMatchAlgorithm(0)==0);
    /* 释放旧库初始化时申请的分层缓存。 */
    freeRobotMaps(&g_robot);
    /* 全部断言通过后以0退出，CTest据此标记成功。 */
    return 0;
}
