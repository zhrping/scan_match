/* 导入待测试核心接口；测试不模拟核心算法实现。 */
#include "scan_match.h"
/* Release 构建也必须执行断言；这些断言中包含实际测试调用。 */
/* Release通常会定义NDEBUG，下面主动恢复assert以免测试被优化成空操作。 */
#ifdef NDEBUG
/* 启用断言；本测试的部分断言内包含实际被测调用，必须执行。 */
#undef NDEBUG
/* 结束条件编译，后续测试在Debug和Release下都运行。 */
#endif
/* 断言条件不成立时立即中止测试，向CTest返回失败。 */
#include <assert.h>
/* 构造已知旋转、计算误差并检查浮点数。 */
#include <math.h>
/* 读取场景及输出测试摘要。 */
#include <stdio.h>
/* 测试侧申请和释放缓存，核心自身仍无malloc。 */
#include <stdlib.h>
/* 初始化测试地图、哨兵和结构体。 */
#include <string.h>

/* 单一原生核心：已知真值及遍历策略回归。 */
/* 已知位姿恢复容差，允许Point2D量化产生微小浮点误差。 */
#define TRUTH_EPS 1e-5
/* 构造测试端点；原点参数保留在辅助接口中但端点结构只保存x/y。 */
static SmPoint point(double x,double y,double ox,double oy) {
    /* 显式忽略起点参数，避免误以为每个SmPoint还保存射线原点。 */
    (void)ox;(void)oy;return (SmPoint){x,y};
}
/* 固定伪随机种子，随机地图测试可重复定位失败。 */
static unsigned rng=123;
/* 线性同余产生确定序列，仅用于造测试数据，不用于安全随机用途。 */
static unsigned next(void) {rng=1664525u*rng+1013904223u;return rng;}
/* 测试辅助：申请旋转工作区，调用核心，校验过程成功后按值返回诊断。 */
static SmResult run(const SmField *f,SmPoint *p,size_t n,SmPose pred,SmConfig c) {
    /* 按核心报价申请工作区，同时准备结果对象。 */
    void *w=malloc(sm_workspace_bytes(n));SmResult r;
    /* 先确认内存存在，再要求核心正常完成；接受状态由各用例另行判断。 */
    assert(w && sm_match(f,p,n,pred,&c,w,sm_workspace_bytes(n),NULL,NULL,&r)==0);
    /* 释放测试工作区并返回完整结果，不让后续用例依赖该缓存。 */
    free(w);return r;
}
/* 根据是否传scene选择真实日志回放或内置构造用例。 */
int main(void) {
    /* 创建已知大小的测试原图，初值为空闲；不会占用固件任务栈。 */
    uint8_t cells[64*64];memset(cells,0,sizeof(cells));
    /* 构造64×64、12cm分辨率地图，(0,0)中心为(-3.84,-3.84)m，使用默认配置。 */
    SmMap m={64,64,.12,-3.84,-3.84,cells,0,0};SmConfig c=sm_default_config();
    /* 准备40个端点、已知真值和预测原点，后续检查能否恢复变换。 */
    SmPoint points[40];SmPose truth={.24,-.36,.10},pred={0,0,0};
    /* 非对称孤立地标，构造已知刚体变换真值，防止测试本身存在对称歧义。 */
    /* 逐点构造非对称地标或长墙样本，避免把单一场景当成通用验证。 */
    for(int i=0;i<40;i++) {
        /* 选地图内部随机格子放障碍，保留边界空间用于平移测试。 */
        int x=8+(int)(next()%48),y=8+(int)(next()%48);cells[y*64+x]=245;
        /* 先从地图障碍中心减去真实机器人平移，再做逆旋转生成局部端点。 */
        double wx=m.origin_x+x*m.resolution-truth.x,wy=m.origin_y+y*m.resolution-truth.y;
        /* 用真值逆变换生成扫描，匹配应该恢复真值而不是记住输入地图坐标。 */
        points[i]=point(cos(truth.theta)*wx+sin(truth.theta)*wy,
                           -sin(truth.theta)*wx+cos(truth.theta)*wy,0,0);
    }
    /* 先关闭剪枝与精修，固定搜索窗口和步长，隔离离散搜索的正确性。 */
    c.use_bnb=0;c.window_xy=.60;c.window_angle=.20;c.angle_step=.05;c.refine_iterations=0;
    /* 按地图尺寸申请评分场，准备场视图。 */
    void *storage=malloc(sm_field_bytes(&m));SmField field;
    /* 确保构场成功，否则后续分数比较没有意义。 */
    assert(storage && sm_build_field(&m,&c,storage,sm_field_bytes(&m),&field)==0);
    /* 同一输入先枚举再BnB，比较的是搜索策略而不是不同评分实现。 */
    SmResult a=run(&field,points,40,pred,c);c.use_bnb=1;SmResult b=run(&field,points,40,pred,c);
    /* 障碍中心构造应接近满分，误差仅允许在float端点容差内。 */
    assert(fabs(a.discrete_score-1)<TRUTH_EPS);
    /* 检查恢复的x/y与已知平移一致。 */
    assert(fabs(a.discrete_best.x-truth.x)<TRUTH_EPS && fabs(a.discrete_best.y-truth.y)<TRUTH_EPS);
    /* 检查恢复角度与已知真值一致。 */
    assert(fabs(a.discrete_best.theta-truth.theta)<TRUTH_EPS);
    /* 两个搜索策略应保留相同候选数；附加状态判断时也要求接受/原因一致。 */
    assert(a.candidate_count==b.candidate_count);
    /* 逐个候选比较，不能只验证第一名而漏掉错误的次峰。 */
    for(int i=0;i<a.candidate_count;i++) {
        /* 核对枚举和BnB对应候选的评分一致。 */
        assert(fabs(a.candidates[i].score-b.candidates[i].score)<1e-10);
        /* 核对候选x一致，排除剪枝漏掉同分决胜位置。 */
        assert(fabs(a.candidates[i].pose.x-b.candidates[i].pose.x)<1e-10);
        /* 核对候选y一致。 */
        assert(fabs(a.candidates[i].pose.y-b.candidates[i].pose.y)<1e-10);
        /* 核对候选角度一致。 */
        assert(fabs(a.candidates[i].pose.theta-b.candidates[i].pose.theta)<1e-10);
    }
    /* 非离散格点真值：精修必须提高分数，并向已知真值靠近。 */
    /* 构造不落在离散平移或角度格点上的真值，专门测试连续精修。 */
    SmPose fine_truth={.277,-.319,.113};
    /* 逐点构造非对称地标或长墙样本，避免把单一场景当成通用验证。 */
    for(int i=0;i<40;i++) {
        /* 将此前局部点恢复成固定世界地标，保持地图不变。 */
        double wx=truth.x+cos(truth.theta)*points[i].x-sin(truth.theta)*points[i].y;
        /* 同样恢复世界y坐标。 */
        double wy=truth.y+sin(truth.theta)*points[i].x+cos(truth.theta)*points[i].y;
        /* 减去新的非格点机器人平移，为新的逆变换准备坐标。 */
        wx-=fine_truth.x;wy-=fine_truth.y;
        /* 根据新真值逆旋转得到局部x端点。 */
        points[i].x=cos(fine_truth.theta)*wx+sin(fine_truth.theta)*wy;
        /* 根据新真值逆旋转得到局部y端点。 */
        points[i].y=-sin(fine_truth.theta)*wx+cos(fine_truth.theta)*wy;
    }
    /* 运行当前配置并保存结果；同一行如启用精修，再执行一次作对照。 */
    a=run(&field,points,40,pred,c);c.refine_iterations=25;b=run(&field,points,40,pred,c);
    /* 非格点样本上精修应实际改善评分，不能只返回不同位置。 */
    assert(b.candidates[0].score>a.candidates[0].score+1e-5);
    /* 精修位置必须靠近已知非格点真值，避免提高分数却跑到错误峰。 */
    assert(hypot(b.candidates[0].pose.x-fine_truth.x,b.candidates[0].pose.y-fine_truth.y)<.04);
    /* 精修角度应接近真值。 */
    assert(fabs(b.candidates[0].pose.theta-fine_truth.theta)<.02);
    /* 第四方法的整数输出也必须保持BnB/枚举一致，且不低于可表达的离散候选。 */
    /* 切换到第四方法使用的整数输出模式，检查取整与回退行为。 */
    c.grid_output=1;c.use_bnb=0;a=run(&field,points,40,pred,c);
    /* 用相同输入运行剪枝模式，与枚举结果比较。 */
    c.use_bnb=1;b=run(&field,points,40,pred,c);
    /* 两个搜索策略应保留相同候选数；附加状态判断时也要求接受/原因一致。 */
    assert(a.candidate_count==b.candidate_count && a.accepted==b.accepted && a.reasons==b.reasons);
    /* 逐候选检查整数输出的评分和格点约束。 */
    for(int i=0;i<b.candidate_count;i++) {
        /* 复制当前候选，便于分别核查位置量化与重评分。 */
        SmCandidate v=b.candidates[i];
        /* 整数输出模式下枚举/BnB仍须一致。 */
        assert(fabs(v.score-a.candidates[i].score)<1e-12);
        /* 检查x修正除分辨率后确实是整数，量化针对相对修正而非绝对位姿。 */
        assert(fabs((v.pose.x-pred.x)/m.resolution-round((v.pose.x-pred.x)/m.resolution))<1e-10);
        /* 对y修正做相同整数格约束检查。 */
        assert(fabs((v.pose.y-pred.y)/m.resolution-round((v.pose.y-pred.y)/m.resolution))<1e-10);
        /* 独立重算该候选实际位置评分，确保没有沿用取整前的高分。 */
        assert(fabs(v.score-sm_score(&field,points,40,v.pose))<1e-12);
    }
    /* 离散基准也按GridPoint角度float精度转换，以便公平比较。 */
    SmPose seed=b.discrete_best;seed.theta=pred.theta+(float)(seed.theta-pred.theta);
    /* 回退保护必须保证实际输出分数不低于可表达的初始离散候选。 */
    assert(b.candidates[0].score+1e-12>=sm_score(&field,points,40,seed));
    /* 恢复连续输出且关闭精修，开始下一类独立测试。 */
    c.grid_output=0;c.refine_iterations=0;
    /* 随机地图、未知格、越界点、非整数栅格坐标：检查父界剪枝不会漏最优。 */
    /* 用12组确定随机地图验证上界剪枝，涵盖未知和越界端点。 */
    for(int trial=0;trial<12;trial++) {
        /* 按固定比例混合障碍、未知和空闲格，增加评分边界情况。 */
        for(size_t i=0;i<sizeof(cells);i++) {unsigned r=next()%100;cells[i]=r<9?245:r<25?128:25;}
        /* 地图变化后重新构场，避免测试误用上一个地图的缓存。 */
        assert(sm_build_field(&m,&c,storage,sm_field_bytes(&m),&field)==0);
        /* 构造可落在地图之外的随机局部点，检查越界零分约定。 */
        for(int i=0;i<40;i++) points[i]=point(((int)(next()%1000)-500)*.013,
                                                             ((int)(next()%1000)-500)*.013,0,0);
        /* 使用非格点预测坐标，确保不是仅在整数世界原点才正确。 */
        pred=(SmPose){.071,-.053,.027};c.use_bnb=0;a=run(&field,points,40,pred,c);
        /* 用相同输入运行剪枝模式，与枚举结果比较。 */
        c.use_bnb=1;b=run(&field,points,40,pred,c);
        /* 两个搜索策略应保留相同候选数；附加状态判断时也要求接受/原因一致。 */
        assert(a.candidate_count==b.candidate_count);
        /* 逐个候选比较，不能只验证第一名而漏掉错误的次峰。 */
        for(int i=0;i<a.candidate_count;i++) {
            /* 核对枚举和BnB对应候选的评分一致。 */
            assert(fabs(a.candidates[i].score-b.candidates[i].score)<1e-10);
            /* 核对候选x一致，排除剪枝漏掉同分决胜位置。 */
            assert(fabs(a.candidates[i].pose.x-b.candidates[i].pose.x)<1e-10);
            /* 核对候选y一致。 */
            assert(fabs(a.candidates[i].pose.y-b.candidates[i].pose.y)<1e-10);
            /* 核对候选角度一致。 */
            assert(fabs(a.candidates[i].pose.theta-b.candidates[i].pose.theta)<1e-10);
        }
        /* 对当前地图另跑连续精修，再恢复配置；比较精修前后的真实目标值。 */
        c.refine_iterations=25;b=run(&field,points,40,pred,c);c.refine_iterations=0;
        /* 精修只能保留或提高分数，允许极小浮点比较裕量。 */
        assert(b.candidates[0].score+1e-10>=a.candidates[0].score);
        /* 检查精修x仍位于原0.60m半窗内。 */
        assert(fabs(b.candidates[0].pose.x-pred.x)<=.600001);
        /* 检查精修y未越过原搜索窗口。 */
        assert(fabs(b.candidates[0].pose.y-pred.y)<=.600001);
    }
    /* 单一长墙缺乏沿墙位置约束，分数高也必须识别等价的备选峰。 */
    /* 清空上一张随机地图，准备单独长墙场景。 */
    memset(cells,0,sizeof(cells));
    /* 在同一y行铺一面长墙，故意制造沿墙方向的定位歧义。 */
    for(int x=0;x<64;x++) cells[32*64+x]=245;
    /* 构造可落在地图之外的随机局部点，检查越界零分约定。 */
    for(int i=0;i<40;i++) points[i]=point(-1.8+i*.09,0,0,-1);
    /* 地图变化后重新构场，避免测试误用上一个地图的缓存。 */
    assert(sm_build_field(&m,&c,storage,sm_field_bytes(&m),&field)==0);
    /* 使用非格点预测坐标，确保不是仅在整数世界原点才正确。 */
    pred=(SmPose){0,0,0};a=run(&field,points,40,pred,c);
    /* 虽然墙上贴合接近满分，但多个位置等价，必须因歧义拒绝。 */
    assert(a.discrete_score>.999 && (a.reasons&SM_AMBIGUOUS) && !a.accepted);
    /* 一个满分已知点 + 一个越界点，应为 0.5，不能偷偷改用命中点数量归一化。 */
    /* 构造一个障碍上端点和一个远越界端点，检查分母不偷减。 */
    SmPoint pair[2]={point(0,0,0,0),point(100,100,0,0)};
    /* 两个点只有一个得满分，平均必须为0.5而不是1。 */
    assert(fabs(sm_score(&field,pair,2,(SmPose){0,0,0})-.5)<1e-10);
    /* 空地图必须零分并拒绝；绝不能通过少量已知格缩小分母“抬高”分数。 */
    /* 把整图设为未知并重建评分，验证无地图证据时拒绝。 */
    memset(cells,128,sizeof(cells));assert(sm_build_field(&m,&c,storage,sm_field_bytes(&m),&field)==0);
    /* 运行当前配置并保存结果；同一行如启用精修，再执行一次作对照。 */
    a=run(&field,points,40,pred,c);assert(a.discrete_score==0 && !a.accepted);
    /* 为非法输入测试准备正常容量工作区。 */
    SmResult r;void *w=malloc(sm_workspace_bytes(40));
    /* 故意只报1字节工作区，核心应返回-1而不是越界写。 */
    assert(sm_match(&field,points,40,pred,&c,w,1,NULL,NULL,&r)==-1);
    /* 注入非法端点，确认输入检查拒绝，不产生可应用结果。 */
    points[0].x=NAN;assert(sm_match(&field,points,40,pred,&c,w,sm_workspace_bytes(40),NULL,NULL,&r)==-1);
    /* 释放测试申请的工作区和评分场。 */
    free(w);free(storage);
    /* 打印已通过的测试范围；没有真值的日志回放不在这里冒充准确率。 */
    puts("PASS: known transform; exhaustive/BnB best and alternate peaks; unknown/out-of-map; refinement monotonicity; input validation");
    /* 全部断言通过后以0退出，CTest据此标记成功。 */
    return 0;
}
