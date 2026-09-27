#!/usr/bin/env python3
"""在已提取的scene上执行真实tryMatchEx第四种方法；算法接受不等于真实定位正确。"""
# 解析场景目录、实际集成程序路径、报告目录及绘图场次。
import argparse
# 输出逐场对照表，便于按变化或拒绝原因筛选。
import csv
# 将JSON摘要转义后嵌入HTML，避免数据被解释为标签。
import html
# 读取连续结果和实际入口结果，并保存差异摘要。
import json
# 计算厘米位置差和周期角差，避免跨一圈产生虚假大误差。
import math
# 统一构造工程、场景和生成文件的路径。
from pathlib import Path
# 实际运行test_fourth，验证的是旧入口接入路径而非只调用核心。
import subprocess

# 根据脚本路径定位项目根目录，不依赖当前工作目录。
ROOT = Path(__file__).resolve().parents[1]

# 为选定场次并排画预测、连续候选和第四方法实际候选，辅助人工判断取整影响。
def plot_case(src, out, native, result):
    # 绘图阶段才导入数组库，处理地图与CSV点云。
    import numpy as np
    # 准备离线绘图库。
    import matplotlib
    # 无显示器环境也能生成图片，测试运行不弹窗口。
    matplotlib.use('Agg')
    # 使用pyplot创建、保存并关闭图形。
    import matplotlib.pyplot as plt
    # 给障碍、未知和空闲指定离散颜色，不把原字节当概率渐变。
    from matplotlib.colors import ListedColormap
    # 读取当前场景原始占据地图，保持[y,x]布局。
    grid=np.load(src/'map.npy')
    # 读取局部端点两列；绘图不需要额外射线起点列。
    scan=np.loadtxt(src/'scan.csv',delimiter=',',skiprows=1,ndmin=2)[:,:2]
    # 从同一C输入文件取得地图几何，避免绘图使用另一套硬编码原点。
    with (src/'scene.txt').open() as f:
        # 跳过版本行，读取宽高、分辨率和索引0中心坐标。
        next(f);w,h,res,ox,oy=map(float,next(f).split())
    # 取连续CLI运行的预测位姿，单位米/弧度。
    prediction=np.asarray(native['prediction'])
    # 先模拟toflib的float预测，再加第四方法厘米/度修正转换成米/弧度后的值。
    actual=prediction.astype(np.float32).astype(float)+np.array([result['dx_cm']/100,result['dy_cm']/100,math.radians(result['dtheta_deg'])])
    # 三个子图分别使用预测、连续最高分和旧接口实际候选位姿。
    poses=[prediction,native['candidates'][0]['pose'],actual]
    # 标题明确第四候选是否接受，拒绝候选仍可用于诊断。
    titles=['Prediction','Continuous candidate',f"Fourth candidate (accepted={result['accepted']})"]
    # 创建三幅共享坐标轴的子图，避免缩放不同造成视觉误判。
    fig,axes=plt.subplots(1,3,figsize=(15,5),sharex=True,sharey=True)
    # 128优先标为未知，其余按本回放默认阈值151分障碍和空闲。
    shown=np.where(grid==128,1,np.where(grid>151,0,2))
    # 收集同一点云在三个位姿下的世界坐标。
    clouds=[]
    # 对每个位姿应用同一刚体变换，输入点云保持不变。
    for pose in poses:
        # 每个位姿计算一次旋转系数。
        c,sn=math.cos(pose[2]),math.sin(pose[2])
        # 行向量右乘旋转矩阵再加平移，得到世界端点。
        clouds.append(scan @ np.array([[c,sn],[-sn,c]])+np.array(pose[:2]))
    # 合并三份点云，用统一包围盒确定画图范围。
    all_points=np.concatenate(clouds)
    # 把三个坐标轴、位姿和点云逐一配对绘制。
    for ax,pose,cloud,title in zip(axes,poses,clouds,titles):
        # 以格子中心外扩半格构建范围，y轴向上，显示原图分类值。
        ax.imshow(shown,origin='lower',extent=(ox-res/2,ox+(w-.5)*res,oy-res/2,oy+(h-.5)*res),cmap=ListedColormap(['#333333','#bfc5cb','#ffffff']),vmin=0,vmax=2)
        # 红点画端点，蓝十字画机器人候选位置。
        ax.scatter(cloud[:,0],cloud[:,1],s=6,color='#e64a19');ax.plot(pose[0],pose[1],'b+')
        # 标标题、米制x轴及等比例坐标，避免图像拉伸。
        ax.set_title(title);ax.set_xlabel('x (m)');ax.set_aspect('equal')
        # 三图共用所有点的x范围，两端各留0.6m。
        ax.set_xlim(all_points[:,0].min()-.6,all_points[:,0].max()+.6)
        # 三图共用所有点的y范围，两端各留0.6m。
        ax.set_ylim(all_points[:,1].min()-.6,all_points[:,1].max()+.6)
    # 左图注明y坐标单位米，其余共享坐标。
    axes[0].set_ylabel('y (m)')
    # 标题给出场次及连续到整数输出的评分变化，不宣称真值误差。
    fig.suptitle(f"{src.name}: score {native['candidates'][0]['score']:.6f} -> {result['score']:.6f}")
    # 排版、保存PNG并及时关闭图形，批量运行不积累图像内存。
    fig.tight_layout();fig.savefig(out/f'{src.name}.png',dpi=130);plt.close(fig)


# 扫描已提取scene并运行真实tryMatchEx集成验证，最后生成对照报告。
def main():
    # 建立命令行解析器，以模块说明作为帮助文本。
    ap=argparse.ArgumentParser(description=__doc__)
    # 注册输入目录、程序或报告选项，默认值便于复现当前工程工作流。
    ap.add_argument('--scenes',type=Path,default=ROOT/'output/native_validation')
    # 注册输入目录、程序或报告选项，默认值便于复现当前工程工作流。
    ap.add_argument('--exe',type=Path,default=ROOT/'build-fourth/test_fourth')
    # 注册输入目录、程序或报告选项，默认值便于复现当前工程工作流。
    ap.add_argument('--out',type=Path,default=ROOT/'output/fourth_validation')
    # 注册输入目录、程序或报告选项，默认值便于复现当前工程工作流。
    ap.add_argument('--plots',default='0,6,13,19,31')
    # 解析参数，整理需要绘图的编号集合并创建输出目录。
    a=ap.parse_args();selected={int(x) for x in a.plots.split(',') if x};a.out.mkdir(parents=True,exist_ok=True)
    # 收集每次真实入口运行的对照记录。
    rows=[]
    # 按场次目录顺序处理所有已存在scene，不重新猜测日志关联。
    for scene in sorted(a.scenes.glob('match_*/scene.txt')):
        # 获取当前场次目录，并从match_编号解析原场次索引。
        src=scene.parent;index=int(src.name.split('_')[1])
        # 读取相同场景的连续核心结果作为对照，不把它当定位真值。
        native=json.loads((src/'result.json').read_text())
        # 读取原日志状态、扫描片段编号等元数据。
        meta=json.loads((src/'bundle.json').read_text())
        # 运行真实toflib集成测试并保留全部调试输出。
        p=subprocess.run([str(a.exe.resolve()),str(scene.resolve())],capture_output=True,text=True)
        # 把stdout和stderr保存为单场日志，失败时可追溯断言原因。
        (a.out/f'{src.name}.txt').write_text(p.stdout+p.stderr)
        # 任何集成测试失败都终止批次，不能把错误输出当成匹配拒绝。
        if p.returncode: raise RuntimeError(f'{src.name} 接入测试失败，查看输出日志')
        # 从旧库大量日志中找到明确前缀的机器可读摘要行。
        line=next(x for x in p.stdout.splitlines() if x.startswith('FOURTH_JSON '))
        # 去掉前缀后解析实际GridPoint输出和状态。
        r=json.loads(line[len('FOURTH_JSON '):])
        # 取连续实现最高分候选，作为位置和评分变化的参照。
        b=native['candidates'][0]
        # 合并场次/原状态，计算实际修正与连续修正的厘米距离及最短角差。
        r.update(index=index,logged_status=meta['match']['logged_status'],
                 spin_index=meta['match']['spin_index'],continuous_accepted=native['accepted'],
                 continuous_score=b['score'],score_delta=r['score']-b['score'],
                 position_delta_cm=math.hypot(r['dx_cm']-100*b['correction'][0],r['dy_cm']-100*b['correction'][1]),
                 angle_delta_deg=abs(math.remainder(r['dtheta_deg']-math.degrees(b['correction'][2]),360)))
        # 只给用户选择的场次画三图对比，避免全部绘图拖慢验证。
        if index in selected: plot_case(src,a.out,native,r)
        # 保存该场真实入口结果和变化量。
        rows.append(r)
        # 即时打印原日志、连续接受和第四方法接受，三种状态分开显示。
        print(f"{index:02d}: log={r['logged_status']} continuous={r['continuous_accepted']} fourth={r['accepted']} score={r['score']:.6f}",flush=True)
    # 没有找到scene则报错提示先提取，不能生成空通过报告。
    if not rows: raise RuntimeError('没有scene，请先运行check_native_regression.py')
    # 统计回放数量和接受状态变化的场次编号，不将接受等同定位正确。
    summary={'replayed':len(rows),'groups':{},'acceptance_changed':[r['index'] for r in rows if bool(r['accepted'])!=bool(r['continuous_accepted'])]}
    # 按原日志状态分组，分别观察成功组退化和失败组改善。
    for status in ('success','failed'):
        # 筛选当前状态组的记录。
        group=[r for r in rows if r['logged_status']==status]
        # 记录尝试数、接受数及独立扫描段数，防止重复尝试夸大样本数。
        summary['groups'][status]={'attempts':len(group),'accepted':sum(r['accepted'] for r in group),'scan_segments':len({r['spin_index'] for r in group})}
    # 保存完整结构化摘要及逐场记录。
    (a.out/'comparison.json').write_text(json.dumps({'summary':summary,'cases':rows},ensure_ascii=False,indent=2))
    # 写机器可读表格，with保证自动关闭。
    with (a.out/'comparison.csv').open('w',newline='') as f:
        # 按记录键写表头与全部数据，方便外部表格工具查看。
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    # 把主要字段组织成HTML表格，每场链接原运行日志和可选图片。
    body=''.join(f"<tr><td><a href='match_{r['index']:03d}.txt'>{r['index']}</a></td><td>{r['logged_status']} <a href='match_{r['index']:03d}.png'>图（所选场次）</a></td><td>{r['continuous_accepted']} → {r['accepted']}</td><td>{r['continuous_score']:.6f} → {r['score']:.6f}</td><td>{r['dx_cm']:.2f}, {r['dy_cm']:.2f}, {r['dtheta_deg']:.3f}°</td><td>{r['position_delta_cm']:.3f}</td><td>{r['reasons']}</td><td>{r['total_ms']:.2f}</td></tr>" for r in rows)
    # 逐场检查静态图片是否属于本轮选定绘图项。
    for r in rows:
        # 未画图的场次不能保留死链接。
        if r['index'] not in selected:
            # 仅移除该场图片链接，保留数值和完整日志链接。
            body=body.replace(f" <a href='match_{r['index']:03d}.png'>图（所选场次）</a>",'')
    # 定义报告页面及解释文本，区分旧25点门槛与算法搜索拒绝。
    page='''<!doctype html><meta charset="utf-8"><title>第四种方法实际入口验证</title>
<style>body{font:16px sans-serif;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #ccd;padding:8px}</style>
<h1>第四种方法：tryMatchEx实际整数格输出</h1>
<p>使用原generateScan、原25点门槛、原GridPoint协议；defer_apply=true。逐场独立重算实际输出位置评分。
取消测试确保不返回部分候选。没有真实位姿真值，接受状态不是定位正确率。计时是本机单次CPU时间，包含构场，不代表芯片耗时。</p>
<p>reason位：1低分、2已知覆盖低、4近障比例低、8歧义、16边界、32点数少、64取消。
状态2表示旧流程25点门槛拦截，尚未执行搜索，分数0。</p>'''
    # 将摘要JSON转义后以预格式文本展示，保留换行方便阅读。
    page+='<pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre>'
    # 补齐表头和逐场对照行。
    page+='<table><tr><th>场次/完整日志</th><th>原日志状态</th><th>连续→第四接受</th><th>连续→实际分数</th><th>实际修正cm/°</th><th>位置变化cm</th><th>reason</th><th>总耗时ms</th></tr>'+body+'</table>'
    # 写入默认人工查看的离线报告。
    (a.out/'comparison.html').write_text(page)
    # 终端同时输出摘要，自动运行时无需打开浏览器也能看到分组结果。
    print(json.dumps(summary,ensure_ascii=False,indent=2))

# 直接运行脚本才进入main，被导入时只提供函数。
if __name__=='__main__': main()
