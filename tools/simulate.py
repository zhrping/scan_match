#!/usr/bin/env python3
"""统一入口：列出匹配场次、提取地图/点云并画图，或调用 C 核心运行仿真。"""
# 解析命令行参数并生成帮助信息。
import argparse
# 复制颜色映射后再修改缺失颜色，避免污染matplotlib全局色图。
import copy
# 输出机器可读汇总表。
import csv
# 读取C程序结果、保存执行参数并嵌入报告数据。
import json
# 角度转换、旋转方向和有限数检查。
import math
# 以Path操作源文件、可执行程序和每场输出路径。
from pathlib import Path
# 启动C匹配程序并分离stdout结果与stderr错误。
import subprocess
# 错误消息输出到stderr，失败时返回非0退出码。
import sys

# 载入地图与点云数组，执行坐标转换和统计。
import numpy as np
# 导入绘图库，在导入pyplot之前设置离线后端。
import matplotlib
# 使用无需显示器的后端，终端和自动回归环境也能生成PNG。
matplotlib.use('Agg')
# 创建并保存静态地图、点云和评分图。
import matplotlib.pyplot as plt

# 复用统一日志解析与刚体变换，避免报告使用另一套坐标约定。
from log_data import LogData, transform

# 从脚本所在位置定位项目根目录，不依赖用户启动时的工作目录。
ROOT = Path(__file__).resolve().parents[1]
# 避免部分字体不支持Unicode负号，图中文字采用英文以减少缺字。
plt.rcParams['axes.unicode_minus'] = False
# 英文绘图文字避免无中文字体机器产生方框；说明和 HTML 使用中文。


# 绘制原图或阈值分类图；注意origin是格子中心，imshow需要外边界。
def background(ax, grid, origin, resolution, observed=None, raw=False, wall_threshold=151):
    # 从第0格中心减半格，到最后格中心加半格，得到真实米制图像范围。
    extent = [origin[0]-.5*resolution, origin[0]+(grid.shape[1]-.5)*resolution,
              origin[1]-.5*resolution, origin[1]+(grid.shape[0]-.5)*resolution]
    # raw保留原始字节；否则先保留未知128，再按障碍阈值分成黑/白。
    view = grid if raw else np.where(grid==128,128,np.where(grid>wall_threshold,255,0))
    # 观测掩码为假时隐藏该格并显示缺失专用色，不能与未知混为一谈。
    masked = np.ma.masked_where(~observed, view) if observed is not None else view
    # 复制灰度反向色图，单独将缺失格设为浅蓝色。
    cmap = copy.copy(plt.get_cmap('gray_r'));cmap.set_bad('#d9e6ed')
    # 按y向上显示像素，不对地图做平滑插值，保留真实格子边界。
    artist = ax.imshow(masked, origin='lower',extent=extent,cmap=cmap,vmin=0,vmax=255,interpolation='nearest')
    # 保持x/y物理比例一致，并标明坐标单位米。
    ax.set_aspect('equal');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
    # 把图像对象交给调用者，例如添加颜色条。
    return artist


# 在预测或候选位置画中心标记与朝向箭头。
def arrow(ax, pose, color, label):
    # 画机器人位置，zorder较高避免被点云遮住。
    ax.scatter(pose[0],pose[1],marker='x',s=65,c=color,label=label,zorder=5)
    # 沿theta方向绘制0.3m箭头，显示位姿角度，不是运动轨迹。
    ax.arrow(pose[0],pose[1],.3*math.cos(pose[2]),.3*math.sin(pose[2]),
             width=.008,head_width=.08,length_includes_head=True,color=color,zorder=5)


# 只用导出的原始数据画输入图，不要求已经编译或运行C算法。
def save_input_plot(bundle, meta):
    """只依赖日志提取结果；无需编译或运行 C 程序即可检查地图与原始点云。"""
    # 读取原始地图与实际打印掩码，二者必须保持相同shape。
    grid=np.load(bundle/'map.npy');observed=np.load(bundle/'observed.npy')
    # 跳过CSV表头并保证即使单行也返回二维数组。
    cloud=np.loadtxt(bundle/'scan.csv',delimiter=',',skiprows=1,ndmin=2)
    # 将日志代理预测位姿转换成数组，单位米/弧度。
    prediction=np.asarray(meta['prediction'])
    # 对同一份点云分别施加预测位姿和最佳候选位姿，不修改点云自身。
    predicted=transform(cloud[:,:2],prediction)
    # 读取地图原点与分辨率，不能硬编码图片坐标范围。
    origin=meta['map']['origin_center_m'];resolution=meta['map']['resolution_m']
    # 输入原始地图独立输出，不把阈值化后的图片冒充原始地图。
    # 创建原始地图图像，raw=True避免用阈值图冒充原始占据值。
    fig,ax=plt.subplots(figsize=(10,8));im=background(ax,grid,origin,resolution,observed,raw=True)
    # 叠加输入端点并画预测位姿，颜色为橙色。
    ax.scatter(*predicted.T,s=8,c='#eb7810',label='Input cloud');arrow(ax,prediction,'#e88315','Prediction')
    # 定位实际打印区域的索引范围，裁图时保留少量边缘。
    ys,xs=np.where(observed)
    # 把已观测x索引范围换算成世界坐标，并各加3格留白。
    ax.set_xlim(origin[0]+(xs.min()-3)*resolution,origin[0]+(xs.max()+3)*resolution)
    # 同样按已观测y范围设置显示边界。
    ax.set_ylim(origin[1]+(ys.min()-3)*resolution,origin[1]+(ys.max()+3)*resolution)
    # 标题说明原始值与缺失颜色，图例区分输入点云和预测位置。
    ax.set_title('Raw map values | pale blue = missing dump cells');ax.legend()
    # 颜色条标明原始字节值不是已校准的占据概率。
    fig.colorbar(im,ax=ax,label='Raw value (not calibrated probability)')
    # 自动排版、保存原始输入图并关闭对象，避免批量回放积累绘图内存。
    fig.tight_layout();fig.savefig(bundle/'raw_map.png',dpi=140);plt.close(fig)


# 生成匹配前后叠加图；有trace时再生成平移评分面和角度曲线。
def save_plots(bundle, meta, result):
    # 读取原始地图与实际打印掩码，二者必须保持相同shape。
    grid=np.load(bundle/'map.npy');observed=np.load(bundle/'observed.npy')
    # 跳过CSV表头并保证即使单行也返回二维数组。
    cloud=np.loadtxt(bundle/'scan.csv',delimiter=',',skiprows=1,ndmin=2)
    # 采用本次C程序实际使用的预测位姿，取最高分候选作为对比。
    prediction=np.array(result['prediction']);best=result['candidates'][0]
    # 对同一份点云分别施加预测位姿和最佳候选位姿，不修改点云自身。
    predicted=transform(cloud[:,:2],prediction);matched=transform(cloud[:,:2],best['pose'])
    # 取得地图元数据，用于所有子图共享坐标范围。
    m=meta['map'];origin=m['origin_center_m'];resolution=m['resolution_m']
    # 使用本次算法真正使用的障碍阈值绘图，避免图与评分解释不一致。
    threshold=result['config']['occupied_threshold']
    # 左右并排显示匹配前和匹配后，两图用相同米制坐标。
    fig,axes=plt.subplots(1,2,figsize=(14,7))
    # 对子图应用相同背景或显示范围，方便肉眼比较。
    for ax in axes:
        # 绘制原始占据值的分类图，浅蓝代表日志缺失。
        background(ax,grid,origin,resolution,observed,wall_threshold=threshold)
    # 左图只叠加预测位置下的输入端点。
    axes[0].scatter(*predicted.T,c='#e88315',s=12,label='Input cloud')
    # 左图标出预测机器人位置和方向。
    arrow(axes[0],prediction,'#e88315','Prediction')
    # 左标题显示匹配前评分。
    axes[0].set_title(f"Input | score={result['predicted_score']:.3f}")
    # 右图用空心橙点保留原输入，方便看修正前后的位移。
    axes[1].scatter(*predicted.T,facecolors='none',edgecolors='#e88315',s=20,label='Input cloud')
    # 右图用蓝色实点显示最佳候选下的端点，即使拒绝也仍可查看。
    axes[1].scatter(*matched.T,c='#0479bf',s=12,label='Best candidate cloud')
    # 在右图保留预测位姿作参照。
    arrow(axes[1],prediction,'#e88315','Prediction')
    # 叠加最佳候选位置和朝向。
    arrow(axes[1],best['pose'],'#0479bf','Best candidate')
    # 标题区分启发式接受与拒绝原因，不把最高分候选自动叫成功定位。
    status='ACCEPTED (heuristic)' if result['accepted'] else 'REJECTED: '+', '.join(result['reasons'])
    # 把接受状态和候选分数写入右图标题。
    axes[1].set_title(f"{status}\nscore={best['score']:.3f}",fontsize=10)
    # 合并两份世界点云及两个机器人位置，用它们共同决定显示范围。
    combined=np.vstack([predicted,matched,prediction[:2],np.asarray(best['pose'])[:2]])
    # 计算包围盒并向四边各留0.6m空白。
    lo=combined.min(axis=0)-.6;hi=combined.max(axis=0)+.6
    # 对子图应用相同背景或显示范围，方便肉眼比较。
    for ax in axes:
        # 左右图设置完全相同的x/y范围并显示图例。
        ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.legend(fontsize=8)
    # 读取最佳候选相对实际预测位姿的修正量。
    corr=best['correction']
    # 总标题同时标原日志状态、无真值说明及米/度修正。
    fig.suptitle(f"Log attempt #{meta['match']['index']} ({meta['match']['logged_status']}; no ground truth) | "
                 f"correction {corr[0]:+.3f}m, {corr[1]:+.3f}m, {math.degrees(corr[2]):+.2f}deg")
    # 保存匹配对比图并关闭，供默认人工查看。
    fig.tight_layout();fig.savefig(bundle/'overlay.png',dpi=150);plt.close(fig)
    # 定位可选逐候选评分文件。
    trace=bundle/'scores.csv'
    # 只有运行时启用了trace并留下文件，才画评分面。
    if trace.exists():
        # 按dx、dy、dtheta、score四列载入离散访问记录。
        rows=np.loadtxt(trace,delimiter=',',skiprows=1,ndmin=2)
        # BnB 会跳过整列/整段角度，必须保留完整坐标轴并用 NaN 留白，
        # 不能把实际访问的坐标压缩后 imshow，否则热力图的位置会失真。
        # 由实际半窗与地图分辨率还原平移索引范围，而不是只看访问过的点。
        half=int(round(result['actual_window_xy_m']/resolution))
        # 生成完整x/y平移格点坐标，单位米。
        xs=np.arange(-half,half+1)*resolution;ys=xs.copy()
        # 读取实际均分角度步长，可能略小于命令配置值。
        angle_step=result['actual_angle_step_rad']
        # 由半窗还原角度索引数；零角度窗口单独处理避免除零。
        angle_half=int(round(result['config']['window_angle_rad']/angle_step)) if angle_step else 0
        # 生成包含两端及零修正的完整角度轴。
        angles=np.arange(-angle_half,angle_half+1)*angle_step
        # 二维平移评分面初始全NaN，被BnB剪掉的区域保持留白。
        heat=np.full((len(ys),len(xs)),np.nan)
        # 每个角度的最高已访问分同样初始化为缺失，不能补成0。
        angle_scores=np.full(len(angles),np.nan)
        # 逐条把trace映射回完整搜索网格。
        for x,y,a,score in rows:
            # 把米制平移转成整数格索引，并加half移成非负数组下标。
            xi=int(round(x/resolution))+half;yi=int(round(y/resolution))+half
            # 把角度转索引；固定角度模式始终写第0项。
            ai=int(round(a/angle_step))+angle_half if angle_step else 0
            # 同一平移上保留各已访问角度中的最高分，得到对角度取max的投影。
            heat[yi,xi]=score if np.isnan(heat[yi,xi]) else max(heat[yi,xi],score)
            # 同一角度保留各已访问平移中的最高分，得到对x/y取max的曲线。
            angle_scores[ai]=score if np.isnan(angle_scores[ai]) else max(angle_scores[ai],score)
        # 创建平移评分面和角度评分曲线的两个子图。
        fig,axes=plt.subplots(1,2,figsize=(13,5))
        # 热图每个像素对应一个平移格，宽度等于地图分辨率。
        step=resolution
        # 用完整轴范围显示评分面，并为边界像素加半格范围。
        im=axes[0].imshow(heat,origin='lower',extent=[xs[0]-step/2,xs[-1]+step/2,ys[0]-step/2,ys[-1]+step/2],
                          cmap='viridis',vmin=0,vmax=1,interpolation='nearest',aspect='equal')
        # 按最终候选排名叠加标记，排名从1开始便于阅读。
        for rank,candidate in enumerate(result['candidates'],1):
            # 读取该候选的真实修正，不用离散索引代替精修位置。
            delta=candidate['correction']
            # 用红叉标候选平移位置。
            axes[0].scatter(delta[0],delta[1],marker='x',c='red')
            # 在红叉旁写排名，便于对应候选JSON。
            axes[0].annotate(str(rank),(delta[0],delta[1]),color='red')
        # 添加仅用于图例的红叉，不改变实际评分数据。
        axes[0].plot([],[],marker='x',ls='',c='red',label='Refined candidates (rank)')
        # 标出平移修正单位米，并显示候选图例。
        axes[0].set_xlabel('dx (m)');axes[0].set_ylabel('dy (m)');axes[0].legend(fontsize=8)
        # 左标题显示匹配前评分。
        axes[0].set_title('Max score over angle'+(' (visited leaves only)' if result['method']=='bnb' else ''))
        # 给评分面加0..1颜色条，代表贴合分。
        fig.colorbar(im,ax=axes[0],label='Map fit score')
        # 将角度轴由弧度换成度，再绘制最高访问分曲线。
        axes[1].plot(np.rad2deg(angles),angle_scores,'o-',ms=3)
        # 用虚线标出精修候选角度，它可能落在离散角度采样之间。
        axes[1].axvline(math.degrees(corr[2]),color='red',ls='--',label='Refined candidate')
        # 角度曲线注明横轴度、纵轴最高分，并增加辅助网格。
        axes[1].set_xlabel('dtheta (deg)');axes[1].set_ylabel('Max score over x,y');axes[1].grid(alpha=.3)
        # 固定评分显示范围并排版图例。
        axes[1].set_ylim(0,1.02);axes[1].legend();fig.tight_layout()
        # 保存评分面图并释放绘图对象。
        fig.savefig(bundle/'landscape.png',dpi=140);plt.close(fig)
    # 把最佳候选返回给可能需要后续处理的调用者。
    return best


# 把提取结果和可选C结果组织成可离线打开的HTML报告。
def report(out, entries, log):
    """面向人工查看的离线报告。原始文件仍保留，但只在默认折叠区提供链接。

    每个场次目录及汇总目录都生成 index.html；提取模式 result=None，
    明确标明日志候选由代理预测位姿估算，绝不冒充新算法或实际应用位置。
    """
    # 读取本地报告模板，算法数据只注入占位符位置。
    template=(ROOT/'tools/report.html').read_text()
    # 收集汇总页面需要的所有场次数据。
    cases=[]
    # 内部写页函数，单场页和总览页共用相同模板。
    def write_page(path, items):
        # 防止原日志文本中的 HTML 标签结束 JSON 脚本块。
        # 序列化时禁止NaN，并转义小于号，避免日志文本结束JSON脚本标签。
        payload=json.dumps({'cases':items},ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
        # 把唯一数据占位符换成JSON并保存页面，不依赖网络请求。
        path.write_text(template.replace('__REPORT_DATA__',payload))
    # 列出详细区可链接的原始文件和中文名称，默认界面无需逐个打开。
    labels=[('raw_map.png','原始输入图'),('overlay.png','匹配叠加图'),
            ('landscape.png','评分面'),('bundle.json','日志与输入 JSON'),
            ('result.json','新算法 JSON'),('map_cells.csv','原始地图格子'),
            ('scan.csv','点云 CSV'),('raw_sp.csv','原始测距 CSV'),
            ('damaged_map_rows.json','损坏地图记录'),('scene.txt','C 程序输入'),
            ('scores.csv','候选评分 CSV'),('run.json','执行参数')]
    # 每项包含场次目录、提取元数据、以及可能为空的新算法结果。
    for bundle,meta,result in entries:
        # 只链接真实存在的文件；提取模式隐藏以前残留的新算法图和JSON。
        files=[(f,label) for f,label in labels if (bundle/f).is_file() and
               (result is not None or f not in ('overlay.png','landscape.png','result.json','scores.csv','run.json'))]
        # 跳过CSV表头并保证即使单行也返回二维数组。
        cloud=np.loadtxt(bundle/'scan.csv',delimiter=',',skiprows=1,ndmin=2)
        # 打包原图、观测掩码、局部端点和文件链接，浏览器据此重画交互图。
        item={'meta':meta,'result':result,'prefix':bundle.name+'/',
              'grid':np.load(bundle/'map.npy').ravel().tolist(),
              'observed':np.load(bundle/'observed.npy').ravel().astype(int).tolist(),
              'points':cloud[:,:2].tolist(),'files':files}
        # 把当前场次加入总览数据。
        cases.append(item)
        # 单场页的文件就在当前目录，因此prefix置空。
        write_page(bundle/'index.html',[dict(item,prefix='')])
    # 总览页用场次目录作为前缀，包含所有已选场次。
    write_page(out/'index.html',cases)
    # 保留机器可读汇总；日常页面不要求用户打开它。
    # 另外生成机器可读的仿真摘要，不代替默认可读页面。
    summary=[]
    # 再次遍历已完成场次，只挑有新算法结果的项写汇总表。
    for _,meta,result in entries:
        # 仅提取输入的场次没有新算法分数，不能编造摘要。
        if result is None:
            # 跳过本项，继续下一场次或下一段处理。
            continue
        # 取最高分候选，不意味着该候选已通过接受阈值。
        best=result['candidates'][0]
        # 汇总原日志状态、新算法接受、评分、修正、原因，并标明缺少真值。
        summary.append({'match':meta['match']['index'],'logged_status':meta['match']['logged_status'],
                        'logged_score':meta['match']['logged_score'],'accepted':result['accepted'],
                        'predicted_score':result['predicted_score'],'score':best['score'],
                        'dx_m':best['correction'][0],'dy_m':best['correction'][1],
                        'dtheta_deg':math.degrees(best['correction'][2]),
                        'reasons':','.join(result['reasons']),'ground_truth':'unavailable'})
    # 至少有一条仿真结果时才创建summary.csv。
    if summary:
        # 用上下文管理器打开汇总CSV，保证写完关闭。
        with (out/'summary.csv').open('w',newline='') as f:
            # 按摘要键顺序写表头与全部场次，保持列一致。
            writer=csv.DictWriter(f,fieldnames=list(summary[0]));writer.writeheader();writer.writerows(summary)


# 统一命令入口：列场次、仅提取、或提取后运行C匹配。
def main():
    # 建立命令参数解析器，模块说明作为帮助文本。
    ap=argparse.ArgumentParser(description=__doc__)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('log',type=Path)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--list',action='store_true')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--matches',default='1,4,13,19,35,36',help='原始 MATCH_RES 场次编号，逗号分隔')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--out',type=Path,help='默认提取到 output/extracted，仿真到 output/review')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--extract-only',action='store_true',help='只提取地图和点云并画输入图，不调用 C 程序')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--executable',type=Path,default=ROOT/'build/scan_match',help='指定 CMake 生成的匹配程序')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--method',choices=['exhaustive','bnb'],default='bnb')
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--window',type=float,default=1.2)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--angle-deg',type=float,default=15)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--angle-step-deg',type=float,default=.5)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--sigma',type=float,default=.18)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--wall-threshold',type=int,default=151)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--refine',type=int,default=25)
    # 注册一个输入或调参选项；下方默认值决定不显式指定时的行为。
    ap.add_argument('--sensor-offset',type=float,default=.142)
    # 解析用户输入，格式错误由argparse打印帮助并退出。
    args=ap.parse_args()
    # 外参限定0..1m且为有限数，避免错误单位污染点云。
    if not math.isfinite(args.sensor_offset) or not 0<=args.sensor_offset<=1:
        # 报告命令行使用错误并退出，不继续生成半成品结果。
        ap.error('--sensor-offset 必须在 0..1 米')
    # 读取原始日志并建立场次、扫描与地图索引。
    ld=LogData(args.log)
    # 只列概要时不创建场景、不启动C程序、不画图。
    if args.list:
        # 按日志场次顺序打印状态、分数、点数与旧搜索范围。
        for row in ld.listing():
            # 打印单场可读摘要，编号可直接传给--matches。
            print(f"{row['index']:3d} line={row['line']:7d} {row['logged_status']:7s} "
                  f"score={row['logged_score']:.3f} points={row['samples']:3d} range={row.get('logged_search')}")
        # 打印总尝试数、扫描段数和有效地图格数。
        print(f'{len(ld.matches)} attempts, {len(ld.spins)} scan segments, {len(ld.map_cells)} map cells')
        # 当前入口任务已完成，直接返回，避免继续执行仿真。
        return
    # 解析实际C可执行程序绝对路径。
    exe=args.executable.resolve()
    # 只有运行匹配才要求已编译程序，纯输入可视化不依赖C。
    if not args.extract_only and not exe.is_file():
        # 报告命令行使用错误并退出，不继续生成半成品结果。
        ap.error('请先用 CMake 构建，或用 --executable 指定程序；只看输入可加 --extract-only')
    # 用户未指定目录时，提取与仿真分别使用不同默认输出位置。
    args.out=args.out or ROOT/('output/extracted' if args.extract_only else 'output/review')
    # 捕获预期的输入或运行错误，以便给出简明错误信息。
    try:
        # 把逗号分隔场次列表转换成整数索引。
        indices=[int(x) for x in args.matches.split(',')]
        # 拒绝空列表或重复场次，避免覆盖同一目录造成混淆。
        if not indices or len(set(indices))!=len(indices):
            # 把无效场次列表作为可读参数错误报告。
            raise ValueError('场次列表不能为空或重复')
    # 捕获场次列表格式和重复等问题。
    except ValueError as e:
        # 将解析异常内容交给argparse统一输出。
        ap.error(str(e))
    # 创建输出根目录，允许已有目录以便重复回放。
    args.out.mkdir(parents=True,exist_ok=True)
    # 收集本轮实际生成的场次，最终报告只包含这些项。
    entries=[]
    # 逐个处理用户指定场次，不把全部日志默认都画出来。
    for index in indices:
        # 每个场次放入独立match_编号目录，便于对比。
        bundle=args.out/f'match_{index:03d}'
        # 按当前外参提取地图、点云和C场景，并取得完整元数据。
        meta=ld.export(index,bundle,args.sensor_offset)
        # 先生成不依赖匹配结果的原始输入图。
        save_input_plot(bundle,meta)
        # 只提取模式到此完成，不调用匹配程序。
        if args.extract_only:
            # 用None明确表示尚未运行新算法，报告中不会冒充算法结果。
            entries.append((bundle,meta,None))
            # 立即报告当前导出目录，flush使长任务进度及时显示。
            print(f"已提取场次 {index}：{bundle.resolve()}",flush=True)
            # 跳过本项，继续下一场次或下一段处理。
            continue
        # 用参数列表启动程序，避免shell字符串插值和路径空格问题。
        command=[str(exe),str(bundle/'scene.txt'),'--method',args.method,
                 '--trace',str(bundle/'scores.csv'),'--window',str(args.window),
                 '--angle-deg',str(args.angle_deg),'--angle-step-deg',str(args.angle_step_deg),
                 '--sigma',str(args.sigma),'--wall-threshold',str(args.wall_threshold),'--refine',str(args.refine)]
        # 显示当前点数和日志状态，仅为进度提示，不是新算法判定。
        print(f'运行场次 {index}：{meta["scan"]["count"]} 点，日志 {meta["match"]["logged_status"]}',flush=True)
        # 同步运行C程序，分别捕获结果JSON和错误输出。
        proc=subprocess.run(command,text=True,capture_output=True)
        # 保存完整命令、返回码和stderr，便于复现失败。
        (bundle/'run.json').write_text(json.dumps({'command':command,'returncode':proc.returncode,
                                                  'stderr':proc.stderr},ensure_ascii=False,indent=2)+'\n')
        # C程序非0退出时不解析stdout为正常结果。
        if proc.returncode:
            # 把C端错误传给顶层处理，终止当前批次。
            raise RuntimeError(proc.stderr)
        # 解析C程序输出的单个JSON结果对象。
        result=json.loads(proc.stdout)
        # 保存完整候选与诊断，默认HTML只展示主要字段。
        (bundle/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        # 生成匹配叠加图及可选评分面。
        save_plots(bundle,meta,result)
        # 记录本次完成的真实结果，供最后生成报告。
        entries.append((bundle,meta,result))
        # 打印匹配前后分数和接受原因，不能只看分数提升就叫定位成功。
        print(f"  分数 {result['predicted_score']:.3f} → {result['candidates'][0]['score']:.3f}, "
              f"accepted={result['accepted']}, reasons={result['reasons']}",flush=True)
    # 为本批次生成单场与总览HTML及可选CSV摘要。
    report(args.out,entries,ld.path)
    # 输出默认应打开的人工可读报告路径。
    print(f'报告：{(args.out/"index.html").resolve()}')

# 直接运行脚本才进入命令入口，被测试或其他脚本导入时不自动执行。
if __name__=='__main__':
    # 捕获预期的输入或运行错误，以便给出简明错误信息。
    try:
        # 调用统一入口，预期的文件、解析、程序运行异常由外层捕获。
        main()
    # 文件读写、输入数据和C运行失败统一作为命令错误处理。
    except (OSError,ValueError,RuntimeError) as exc:
        # 把简明错误写stderr，与正常进度输出区分。
        print(f'错误：{exc}',file=sys.stderr)
        # 以非0退出码通知调用脚本或自动回归本次失败。
        sys.exit(1)
