/* 启用POSIX 2008接口声明，以便使用单调时钟clock_gettime。 */
#define _POSIX_C_SOURCE 200809L
/* PC程序通过公开核心接口运行匹配，不包含固件插图流程。 */
#include "scan_match.h"
/* 文件读写、JSON输出与错误信息输出。 */
#include <stdio.h>
/* malloc/free、数值字符串转换等PC运行时功能。 */
#include <stdlib.h>
/* 字符串比较及场景文件标记检查。 */
#include <string.h>
/* 有限值验证、距离计算和角度转换。 */
#include <math.h>
/* 单调时钟和timespec，用于测量构场及搜索耗时。 */
#include <time.h>
/* 检测strtod解析溢出等错误。 */
#include <errno.h>
/* 角度输入以度表示时，需要用圆周率转成核心的弧度。 */
#define PI 3.14159265358979323846

/* PC 外壳拥有文件和 malloc；移植时只带 core/，由固件提供数组和工作区。 */
/* 读取单调时间并合成为秒，避免系统校时影响耗时差值。 */
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9; }
/* 把一个被访问离散候选写入CSV，列为dx米、dy米、角度弧度、分数。 */
static void trace_row(void *f,double x,double y,double a,double score) {
    /* 输出候选数值；启用trace会增加I/O耗时，不能当成纯算法耗时。 */
    fprintf(f,"%.9g,%.9g,%.9g,%.12g\n",x,y,a,score);
}
/* 按点号从PC场景读入的起点数组中取射线原点。 */
static int cli_ray(void *user,size_t i,Point2D *out) {
    /* 将通用上下文还原成Point2D数组，复制第i个起点并报告成功。 */
    *out=((Point2D *)user)[i];return 1;
}
/* 按[x米,y米,theta弧度]格式输出一个JSON位姿数组。 */
static void pose_json(SmPose p) { printf("[%.12g,%.12g,%.12g]",p.x,p.y,p.theta); }
/* 严格解析一个有限浮点数，不允许数值后夹带额外字符。 */
static int number(const char *s,double *v) {
    /* 清errno，调用strtod，再检查确实读到数字、无溢出且整个字符串被消费。 */
    char *end;errno=0;*v=strtod(s,&end);return !errno && end!=s && *end=='\0' && isfinite(*v);
}
/* 将命令帮助写stderr，避免污染stdout上的机器可读JSON。 */
static void usage(void) {
    /* 第一项为scene路径；支持枚举/BnB以及可选trace CSV。 */
    fprintf(stderr,"Usage: scan_match SCENE.txt [--method exhaustive|bnb] [--trace scores.csv]\n"
        /* 平移和高斯参数使用米；角度参数在命令行使用度。 */
        "  [--window M] [--angle-deg DEG] [--angle-step-deg DEG] [--sigma M]\n"
        /* 障碍阈值为原始字节值，refine控制精修轮数，min-score控制接受阈值。 */
        "  [--cutoff M] [--wall-threshold N] [--refine 0..100] [--min-score S]\n"
        /* 备选峰分离与歧义阈值，可用于观察对称地图的候选。 */
        "  [--distinct-xy M] [--distinct-angle-deg DEG] [--min-margin S]\n"
        /* 只扰动预测搜索中心，输入点云仍保持文件中定义的参考坐标。 */
        "  [--pose-dx M] [--pose-dy M] [--pose-dtheta-deg DEG]\n");
}
/* PC命令行入口；成功执行返回0，文件/配置/格式错误返回非0。 */
int main(int argc,char **argv) {
    /* 缺少输入时显示用法并失败；显式--help显示用法并成功退出。 */
    if(argc<2 || !strcmp(argv[1],"--help")) { usage();return argc<2 ? 1 : 0; }
    /* 复制核心默认配置，trace_path为空表示不记录逐候选CSV。 */
    SmConfig c=sm_default_config();const char *trace_path=NULL;
    /* 三个命令行预测扰动初值为0，最终加到场景预测位姿。 */
    double dx=0,dy=0,da=0;
    /* 从scene路径后的参数开始解析，每个选项必须跟一个值。 */
    for(int i=2;i<argc;i++) {
        /* 保存选项名称并前进到值；缺值则输出帮助并退出。 */
        const char *key=argv[i]; if(++i>=argc) {usage();return 1;}
        /* 取选项值字符串，v接收数值解析结果。 */
        const char *value=argv[i];double v;
        /* 搜索方法是字符串选项，单独处理，不送进数值转换。 */
        if(!strcmp(key,"--method")) {
            /* 只接受两个已支持方法名，拼写错误不能默默回退。 */
            if(strcmp(value,"exhaustive") && strcmp(value,"bnb")) {usage();return 1;}
            /* 值为bnb时启用剪枝，否则枚举；continue跳过后面的数值选项处理。 */
            c.use_bnb=!strcmp(value,"bnb");continue;
        }
        /* 保存CSV路径，真正打开文件放到场景与构场检查通过后。 */
        if(!strcmp(key,"--trace")) {trace_path=value;continue;}
        /* 其他选项必须是有限数，失败时输出原字符串便于定位输入错误。 */
        if(!number(value,&v)) {fprintf(stderr,"Invalid number: %s\n",value);return 1;}
        /* 设置以预测位置为中心的平移半窗，单位米。 */
        if(!strcmp(key,"--window")) c.window_xy=v;
        /* 把角度半窗从度转成弧度。 */
        else if(!strcmp(key,"--angle-deg")) c.window_angle=v*PI/180;
        /* 把角度采样步长从度转成弧度。 */
        else if(!strcmp(key,"--angle-step-deg")) c.angle_step=v*PI/180;
        /* 设置高斯评分宽度sigma，单位米。 */
        else if(!strcmp(key,"--sigma")) c.sigma=v;
        /* 设置高斯影响半径cutoff，单位米。 */
        else if(!strcmp(key,"--cutoff")) c.cutoff=v;
        /* 设置不同候选峰的平移分离阈值，单位米。 */
        else if(!strcmp(key,"--distinct-xy")) c.distinct_xy=v;
        /* 设置不同候选峰的角度分离阈值，转成弧度保存。 */
        else if(!strcmp(key,"--distinct-angle-deg")) c.distinct_angle=v*PI/180;
        /* 设置最低可接受贴合分，不是定位正确概率。 */
        else if(!strcmp(key,"--min-score")) c.min_score=v;
        /* 设置第一、第二候选的最低分差。 */
        else if(!strcmp(key,"--min-margin")) c.min_margin=v;
        /* 记录预测x扰动，单位米。 */
        else if(!strcmp(key,"--pose-dx")) dx=v;
        /* 记录预测y扰动，单位米。 */
        else if(!strcmp(key,"--pose-dy")) dy=v;
        /* 记录预测角度扰动并转为弧度。 */
        else if(!strcmp(key,"--pose-dtheta-deg")) da=v*PI/180;
        /* 障碍阈值必须是128..254的整数；不合法时进入最后的参数错误分支。 */
        else if(!strcmp(key,"--wall-threshold") && v>=128 && v<255 && floor(v)==v) c.occupied_threshold=(int)v;
        /* 精修次数必须是0..100的整数，0表示关闭连续精修。 */
        else if(!strcmp(key,"--refine") && v>=0 && v<=100 && floor(v)==v) c.refine_iterations=(int)v;
        /* 未知选项或整数参数越界时明确失败，避免以错误默认值继续。 */
        else {fprintf(stderr,"Unknown option or invalid value: %s %s\n",key,value);return 1;}
    }
    /* 以只读模式打开场景文件，打开失败通过perror输出系统错误。 */
    FILE *input=fopen(argv[1],"r");if(!input) {perror(argv[1]);return 1;}
    /* 默认执行失败，所有需清理资源先设NULL，便于统一出口安全free。 */
    int rc=1;uint8_t *cells=NULL;SmPoint *points=NULL;void *field_data=NULL,*work=NULL;FILE *trace=NULL;
    /* PC额外保存射线起点，固件路径可通过回调避免这份数组。 */
    Point2D *origins=NULL;
    /* 用double暂存场景预测位姿，不先截断到float。 */
    double px=0,py=0,pt=0;
    /* 地图结构零初始化以选默认行优先布局；同时准备预测位姿、魔数和点数。 */
    SmMap m={0};SmPose pose;char magic[32];size_t count=0;
    /* 校验格式标记SM_SCENE_V1，限制字符串长度防止写越界。 */
    if(fscanf(input,"%31s",magic)!=1 || strcmp(magic,"SM_SCENE_V1") ||
       /* 读取地图宽高、分辨率及原点，必须正好得到5项。 */
       fscanf(input,"%d%d%lf%lf%lf",&m.width,&m.height,&m.resolution,&m.origin_x,&m.origin_y)!=5 ||
       /* 尺寸限定在1..2048，先限制后乘算容量。 */
       m.width<=0 || m.height<=0 || m.width>2048 || m.height>2048 ||
       /* 读取预测位姿与端点数，并用工作区报价函数验证点数范围。 */
       fscanf(input,"%lf%lf%lf%zu",&px,&py,&pt,&count)!=4 || !sm_workspace_bytes(count)) {
        /* 格式不符时去统一清理出口，不继续解释剩余文件。 */
        fprintf(stderr,"Invalid scene header\n");goto done;
    }
    /* 将已验证读入的数值组合成预测位姿。 */
    pose=(SmPose){px,py,pt};
    /* 计算格子总数，分别申请原图字节数组和float端点数组。 */
    size_t n=(size_t)m.width*m.height;cells=malloc(n);points=malloc(count*sizeof(*points));
    /* 任一内存申请失败，跳到统一出口释放已经分配的资源。 */
    if(!cells || !points) goto done;
    /* 逐格读取0..255整数；不接受截断或超范围占据值。 */
    for(size_t i=0;i<n;i++) {int v;if(fscanf(input,"%d",&v)!=1 || v<0 || v>255) {fprintf(stderr,"Invalid map cells\n");goto done;}cells[i]=(uint8_t)v;}
    /* 申请射线起点数组，申请失败终止本次PC运行。 */
    origins=malloc(count*sizeof(*origins));if(!origins) goto done;
    /* 把场景中的起点数组通过回调交给核心诊断。 */
    c.ray_origin=cli_ray;c.ray_user=origins;
    /* 逐行读取每点的局部端点及局部射线起点。 */
    for(size_t i=0;i<count;i++) {
        /* 四列单位均为米，前两个为端点，后两个为该束射线起点。 */
        double x,y,ox,oy;
        /* 每点必须读满四列，否则判定场景损坏。 */
        if(fscanf(input,"%lf%lf%lf%lf",&x,&y,&ox,&oy)!=4 ||
           /* 拒绝NaN、无穷及异常过远起点；端点距离还由核心校验。 */
           !isfinite(x)||!isfinite(y)||!isfinite(ox)||!isfinite(oy)||hypot(ox,oy)>1000) {
            /* 点云格式无效时停止，避免把后续数据错位读取。 */
            fprintf(stderr,"Invalid scan rows\n");goto done;
        }
        /* 端点与起点分别存为Point2D，保留行号对应关系。 */
        points[i]=(SmPoint){x,y};origins[i]=(Point2D){ox,oy};
    }
    /* 所有约定字段读完后不允许还有非空白字符，防止误读其他版本协议。 */
    char extra;if(fscanf(input," %c",&extra)==1) {fprintf(stderr,"Trailing scene data\n");goto done;}
    /* 绑定原图指针，分别计算评分场和旋转工作区所需字节数。 */
    m.cells=cells;size_t field_bytes=sm_field_bytes(&m),work_bytes=sm_workspace_bytes(count);
    /* 核心地图几何校验不通过时输出错误。 */
    if(!field_bytes) {fprintf(stderr,"Invalid map geometry\n");goto done;}
    /* 由PC外壳分配评分场和工作区；核心内部不申请堆内存。 */
    field_data=malloc(field_bytes);work=malloc(work_bytes);if(!field_data || !work) goto done;
    /* 准备评分场视图并记录构场开始时间。 */
    SmField field;double start=now();
    /* 按当前地图与高斯参数构场，失败时不进入搜索。 */
    if(sm_build_field(&m,&c,field_data,field_bytes,&field)) {fprintf(stderr,"Invalid field configuration\n");goto done;}
    /* 将构场耗时从秒转换为毫秒。 */
    double build_ms=(now()-start)*1000;
    /* 显式要求trace才创建CSV并写表头，文件创建失败则清理退出。 */
    if(trace_path) {trace=fopen(trace_path,"w");if(!trace){perror(trace_path);goto done;}fprintf(trace,"dx_m,dy_m,dtheta_rad,score\n");}
    /* 把命令行扰动加到预测搜索中心，便于评估恢复能力。 */
    pose.x+=dx;pose.y+=dy;pose.theta+=da;
    /* 准备结果对象并记录搜索开始时间。 */
    SmResult r;start=now();
    /* 运行核心匹配，按是否打开CSV选择跟踪回调。 */
    if(sm_match(&field,points,count,pose,&c,work,work_bytes,trace?trace_row:NULL,trace,&r)) {
        /* 核心过程错误输出到stderr，不输出一份看似有效的结果JSON。 */
        fprintf(stderr,"Invalid search input or configuration\n");goto done;
    }
    /* 记录搜索耗时；如有trace，包含回调内的CSV写入开销。 */
    double search_ms=(now()-start)*1000;
    /* 开始结果JSON，记录schema、搜索方法与真实使用的预测位姿。 */
    printf("{\n\"schema\":1,\"method\":\"%s\",\"prediction\":",c.use_bnb?"bnb":"exhaustive");pose_json(pose);
    /* 输出接受状态、位掩码和可读原因数组开头。 */
    printf(",\n\"accepted\":%s,\"reason_bits\":%d,\"reasons\":[",r.accepted?"true":"false",r.reasons);
    /* 数组顺序与核心原因位一一对应，用于将位掩码转为JSON字符串。 */
    const char *names[]={"low_score","low_known_coverage","low_near_ratio","ambiguous","search_boundary","too_few_points"};
    /* 逐位输出已设置原因，sep控制逗号，避免产生无效JSON。 */
    int sep=0;for(int i=0;i<6;i++) if(r.reasons&(1<<i)) printf("%s\"%s\"",sep++?",":"",names[i]);
    /* 输出匹配前分数、第一轮离散分数及离散最优位姿。 */
    printf("],\n\"predicted_score\":%.12g,\"discrete_score\":%.12g,\"discrete_best\":",r.predicted_score,r.discrete_score);pose_json(r.discrete_best);
    /* 无第二候选时把内部-1写成JSON null，避免误解成负置信度。 */
    printf(",\n\"margin\":");if(r.margin<0) printf("null");else printf("%.12g",r.margin);
    /* 明确分差只是保留峰之间的诊断，不是全局统计置信度。 */
    printf(",\"margin_note\":\"diagnostic among retained refined peaks, not certified global confidence\"");
    /* 输出实际搜索窗口、角度步长及总点数，供Python正确画评分面。 */
    printf(",\n\"actual_window_xy_m\":%.9g,\"actual_angle_step_rad\":%.9g,\"point_count\":%zu,",r.actual_window_xy,r.actual_angle_step,count);
    /* 输出访问/剪枝统计，以及主要数组的字节预算。 */
    printf("\n\"evaluated\":%llu,\"pruned\":%llu,\"field_bytes\":%zu,\"workspace_bytes\":%zu,",
           /* 匹配printf格式的整数类型转换；这两项只统计核心访问量。 */
           (unsigned long long)r.evaluated,(unsigned long long)r.pruned,field_bytes,work_bytes);
    /* 输出构场/搜索计时，并标明是否含trace I/O。 */
    printf("\n\"build_ms\":%.6f,\"search_ms\":%.6f,\"timing_includes_trace_io\":%s,",build_ms,search_ms,trace?"true":"false");
    /* 输出影响本次运行的主要参数，便于保存结果后复现。 */
    printf("\n\"config\":{\"window_xy_m\":%.9g,\"window_angle_rad\":%.9g,\"angle_step_rad\":%.9g,\"sigma_m\":%.9g,\"cutoff_m\":%.9g,\"occupied_threshold\":%d,\"refine_iterations\":%d,\"distinct_xy_m\":%.9g,\"distinct_angle_rad\":%.9g,\"min_score\":%.9g,\"min_near_ratio\":%.9g,\"min_known_ratio\":%.9g,\"min_margin\":%.9g},",
           /* 参数顺序必须与上方JSON格式占位符一致。 */
           c.window_xy,c.window_angle,c.angle_step,c.sigma,c.cutoff,c.occupied_threshold,c.refine_iterations,c.distinct_xy,c.distinct_angle,c.min_score,c.min_near_ratio,c.min_known_ratio,c.min_margin);
    /* 开始输出最终候选列表，候选数可能小于三。 */
    printf("\n\"candidates\":[");
    /* 只输出核心实际保留的候选，按分数排序。 */
    for(int i=0;i<r.candidate_count;i++) {
        /* 取候选，按是否第一项决定逗号，并输出绝对位姿。 */
        SmCandidate *v=&r.candidates[i];printf("%s{\"pose\":",i?",":"");pose_json(v->pose);
        /* 输出相对预测修正，以及评分、覆盖、近障、穿障、边界诊断。 */
        printf(",\"correction\":[%.12g,%.12g,%.12g],\"score\":%.12g,\"known_ratio\":%.9g,\"near_ratio\":%.9g,\"ray_conflict_ratio\":%.9g,\"ray_valid_count\":%d,\"boundary\":%s}",
               /* 修正由候选绝对位姿减预测位姿得到，单位为米/弧度。 */
               v->pose.x-pose.x,v->pose.y-pose.y,v->pose.theta-pose.theta,v->score,v->known_ratio,v->near_ratio,v->ray_conflict_ratio,v->ray_valid_count,v->boundary?"true":"false");
    }
    /* 关闭JSON数组和对象，并将退出状态设为成功。 */
    printf("]\n}\n");rc=0;
 /* 集中清理出口，前面任何失败都不会绕过资源释放。 */
 done:
    /* 关闭CSV时也可能写入失败，若发生则让整个命令返回非0。 */
    if(trace && fclose(trace)) rc=1;
    /* 释放PC专用射线起点数组，free(NULL)也是安全操作。 */
    free(origins);
    /* 关闭输入文件并依次释放原图、端点、评分场及工作区，最后返回状态码。 */
    fclose(input);free(cells);free(points);free(field_data);free(work);return rc;
}
