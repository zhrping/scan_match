#!/usr/bin/env python3
"""全量日志回归：原生类型/四层实现与double参考比较，生成默认可读报告。

不能从评分推断真实定位准确率；同一扫描片段的重试不是独立成功样本。
"""
# 解析回归脚本命令参数，支持指定可执行程序和输出位置。
import argparse
# 校验输入日志SHA-256，避免不同日志误用冻结基准。
import hashlib
# 将每场回归结果另存为表格，方便筛查差异。
import csv
# 转义报告中的动态文本，防止日志内容变成HTML标签。
import html
# 读取和保存结构化算法结果及报告数据。
import json
# 计算位置欧氏差、周期角差和数值容差。
import math
# 统一定位脚本根目录、场景、程序和输出文件。
from pathlib import Path
# 多次执行时取耗时中位数，减少偶发调度扰动。
import statistics
# 启动实际C程序或集成测试，检查退出状态并捕获结果。
import subprocess
# 调整本项目工具导入路径以及处理命令退出。
import sys

# 以脚本所在目录定位项目根目录，不依赖启动位置。
ROOT = Path(__file__).resolve().parents[1]
# 优先导入当前项目tools，防止误用系统中同名模块。
sys.path.insert(0, str(ROOT / 'tools'))
# 复用生产日志解析与坐标变换，用真实导出协议测试C入口。
from log_data import LogData
# 复用生产绘图或报告生成函数，验证实际用户所见输出。
from simulate import report, save_input_plot, save_plots


# 多次运行指定C算法并收集结果，返回首份数值结果和耗时中位数。
def execute(exe, scene, method, repeats):
    # 保存本场多次运行输出，数值结果与计时分别处理。
    results = []
    # 重复测量，避免把一次偶然调度延迟当作算法性能。
    for _ in range(repeats):
        # 用参数数组启动实际可执行程序，分开捕获输出与错误。
        p = subprocess.run([str(exe), str(scene), '--method', method],
                           capture_output=True, text=True, check=True)
        # 解析C输出JSON并收集到本次重复运行列表。
        results.append(json.loads(p.stdout))
    # 候选数值以第一次结果为准，下面只替换计时统计。
    r = results[0]
    # 分别对构场和搜索耗时取中位数，不将两者混成单个时间。
    for name in ('build_ms', 'search_ms'):
        # 用多次运行的中位数减少离群值影响。
        r[name] = statistics.median(x[name] for x in results)
    # 返回结果对象，供基准比较和报告使用。
    return r


# 回归命令入口；只有直接执行脚本才运行。
def main():
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap = argparse.ArgumentParser(description=__doc__)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('log', type=Path)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--native', type=Path, default=ROOT/'build-native/scan_match')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--reference', type=Path, default=ROOT/'tests/fixtures/reference_results.json')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--adapter-test', type=Path, default=ROOT/'build-native/test_toflib_adapter')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--out', type=Path, default=ROOT/'output/native_validation')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--repeats', type=int, default=3)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--max-position-cm', type=float, default=1.0)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--max-angle-deg', type=float, default=0.1)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--max-score-delta', type=float, default=0.001)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--plots', default='1,2,13,19,32,35,36')
    # 解析命令行参数，非法选项由argparse直接报告。
    args = ap.parse_args()
    # 逐个检查新程序、冻结基准及适配测试文件是否存在。
    for name in ('native', 'reference', 'adapter_test'):
        # 将传入路径解析为绝对路径，避免子进程工作目录影响。
        path = getattr(args, name).resolve()
        # 缺少程序或基准时直接报错，不能生成空的通过报告。
        if not path.is_file():
            # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
            ap.error(f'缺少{name}: {path}，先以BUILD_TESTING=ON编译原生版本')
        # 保存已校验的绝对路径，后续所有调用使用同一路径。
        setattr(args, name, path)
    # 至少需要运行一次，防止空计时数组求中位数失败。
    if args.repeats<1:
        # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
        ap.error('repeats必须>=1')
    # 逐个检查回归位置、角度与评分容差。
    for value in (args.max_position_cm,args.max_angle_deg,args.max_score_delta):
        # 阈值必须有限且非负，不能用NaN绕过比较。
        if not math.isfinite(value) or value<0:
            # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
            ap.error('差异阈值必须为有限非负数')
    # 读取删除旧实现前保存的历史结果，回归不再编译第二份算法。
    frozen=json.loads(args.reference.read_text())
    # 确认日志字节完全对应基准记录，否则场次索引和结果不可比较。
    if hashlib.sha256(args.log.read_bytes()).hexdigest()!=frozen['log_sha256']:
        # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
        ap.error('日志与冻结基准不一致')
    # 读取真实日志并建立匹配尝试与扫描段索引。
    ld = LogData(args.log)
    # 创建本轮报告输出目录，允许重新生成已有文件。
    args.out.mkdir(parents=True, exist_ok=True)
    # 把逗号分隔场次列表转换成集合，只给指定场次画静态图。
    plots = {int(x) for x in args.plots.split(',') if x}
    # 分别收集数值对照、报告输入、不能回放记录和失败场次。
    records, entries, skipped, failures = [], [], [], []
    # 遍历日志中的每次尝试，重试也保留原编号。
    for index in range(len(ld.matches)):
        # 每个场次用独立目录保存scene和验证结果。
        bundle = args.out/f'match_{index:03d}'
        # 尝试导出场次，输入不足是可记录的跳过条件而非静默成功。
        try:
            # 生成真实地图、点云和scene，并保留源行号等元数据。
            meta = ld.export(index, bundle)
        except ValueError as exc:
            # 记录跳过编号、原日志状态及理由，防止把未验证项计为通过。
            skipped.append({'index':index, 'logged_status':ld.matches[index]['logged_status'], 'reason':str(exc)})
            # 即时输出跳过原因，长批次运行时能看到进度。
            print(f'SKIP {index}: {exc}', flush=True)
            # 跳过当前不能验证的记录，继续处理其余场次。
            continue
        # 定位当前场景文本，供两个搜索策略与适配器共同使用。
        scene = bundle/'scene.txt'
        # 按原日志场次编号读取冻结double参考结果。
        ref = frozen['cases'][str(index)]
        # 运行当前连续原生核心，并按指定重复数统计计时。
        native = execute(args.native, scene, 'bnb', args.repeats)
        # 用相同当前核心执行完整枚举，验证BnB没有改变结果。
        exhaustive = execute(args.native, scene, 'exhaustive', 1)
        # 相同原生实现的遍历策略必须给出完全相同的最终结果（含备选和诊断）。
        # 选择应一致的候选、离散最优、接受状态、原因和峰分差字段。
        keys = ('candidates','discrete_best','discrete_score','accepted','reasons','margin')
        # 逐项比较两种遍历策略的最终结果，不能只看最佳得分。
        search_equal = all(native[k]==exhaustive[k] for k in keys)
        # 调用真实toflib类型适配测试，独立验证地图布局与坐标转换。
        adapter = subprocess.run([str(args.adapter_test), str(scene)], capture_output=True, text=True)
        # 保存适配测试的stdout/stderr，失败时可以逐场追溯。
        (bundle/'adapter_check.txt').write_text(adapter.stdout+adapter.stderr)
        # 取得历史参考与当前实现的最高分候选。
        a,b = ref['candidates'][0], native['candidates'][0]
        # 计算两最佳绝对位置的欧氏距离，并由米换成厘米。
        dpos = 100*math.hypot(a['pose'][0]-b['pose'][0],a['pose'][1]-b['pose'][1])
        # 用周期角差计算最短方向差，再转为度。
        dangle = abs(math.degrees(math.remainder(a['pose'][2]-b['pose'][2],2*math.pi)))
        # 比较同一定义评分的绝对变化。
        dscore = abs(a['score']-b['score'])
        # 回归同时要求位置/角度/分数容差、判定一致、枚举一致以及适配测试通过。
        ok = (dpos<=args.max_position_cm and dangle<=args.max_angle_deg and
              dscore<=args.max_score_delta and ref['accepted']==native['accepted'] and
              ref['reason_bits']==native['reason_bits'] and search_equal and adapter.returncode==0)
        # 组织该场摘要，明确区分原日志状态、算法接受与实现一致性。
        r = dict(index=index, spin_index=meta['match']['spin_index'],
                 logged_status=meta['match']['logged_status'],logged_score=meta['match']['logged_score'],
                 points=native['point_count'],reference_accepted=ref['accepted'],native_accepted=native['accepted'],
                 reference_score=a['score'],native_score=b['score'],position_delta_cm=dpos,angle_delta_deg=dangle,
                 score_delta=dscore,dx_cm=b['correction'][0]*100,dy_cm=b['correction'][1]*100,
                 dtheta_deg=math.degrees(b['correction'][2]),reasons=','.join(native['reasons']),
                 reference_ms=ref['search_ms'],native_ms=native['search_ms'],build_ms=native['build_ms'],
                 search_equal=search_equal,adapter_passed=adapter.returncode==0,passed=ok)
        # 保留当前场记录，最后汇总成功/失败分组。
        records.append(r)
        # 任一门槛失败都记录场次，不能只显示平均误差掩盖个别退化。
        if not ok:
            # 把该场编号加入回归失败列表。
            failures.append(index)
        # 保存本场冻结基准，便于逐字段人工对照。
        (bundle/'reference.json').write_text(json.dumps(ref,indent=2))
        # 保存当前核心完整结果，作为默认报告的数据来源。
        (bundle/'result.json').write_text(json.dumps(native,indent=2))
        # 保存单场差异和通过状态。
        (bundle/'comparison.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
        # 仅指定场次生成耗时较高的静态图片。
        if index in plots:
            # 先画未匹配输入，便于确认日志解析与地图坐标。
            save_input_plot(bundle,meta)
            # 使用实际结果生成叠加图和可选评分面。
            save_plots(bundle,meta,native)
        # 把已验证场次加入交互报告输入。
        entries.append((bundle,meta,native))
        # 立即打印每场回归状态和误差，便于运行中发现异常。
        print(f"{'PASS' if ok else 'FAIL'} {index:02d} log={r['logged_status']} "
              f"accepted={native['accepted']} score={b['score']:.6f} "
              f"delta={dpos:.5f}cm/{dangle:.5f}deg",flush=True)
    # 一个可回放场次都没有时必须失败，不能输出空通过。
    if not records:
        # 输入不足、子进程失败或缺少结果时明确终止，错误不能算作通过。
        raise RuntimeError('没有可回放场次')
    # 汇总样本数、跳过项、阈值、最大差异和解释，不宣称定位真值准确率。
    summary = dict(log=str(args.log.resolve()),native=str(args.native),reference=str(args.reference),
                   total=len(ld.matches),replayed=len(records),skipped=skipped,failures=failures,
                   thresholds=dict(position_cm=args.max_position_cm,angle_deg=args.max_angle_deg,score=args.max_score_delta),
                   max_position_cm=max(r['position_delta_cm'] for r in records),
                   max_angle_deg=max(r['angle_delta_deg'] for r in records),
                   max_score_delta=max(r['score_delta'] for r in records),
                   timing_repeats=args.repeats,trace_io=False,
                   meaning='Algorithm acceptance and implementation regression only; no ground-truth position accuracy.',
                   groups={})
    # 按原日志成功/失败分组统计，和新算法接受状态分开。
    for status in ('success','failed'):
        # 筛选属于该原日志状态的回放记录。
        group=[r for r in records if r['logged_status']==status]
        # 统计尝试数、接受数与独立扫描段数，避免将多次重试当独立成功样本。
        summary['groups'][status]=dict(attempts=len(group),accepted=sum(r['native_accepted'] for r in group),
                                      unique_scan_segments=len({r['spin_index'] for r in group}))
    # 写完整回归摘要与逐场数值，供脚本或人工继续分析。
    (args.out/'regression.json').write_text(json.dumps({'summary':summary,'cases':records},ensure_ascii=False,indent=2))
    # 以CSV保存同一组逐场记录，文件退出上下文自动关闭。
    with (args.out/'regression.csv').open('w',newline='') as f:
        # 用记录键构造表头，再一次写入所有对照行。
        writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    # 生成可缩放地图和点云报告，路径与当前场次目录一致。
    report(args.out,entries,args.log)
    # 收集HTML表格行，页面只展示主要差异与链接。
    rows=[]
    # 逐场生成可读表格，而不是让用户直接读大JSON。
    for r in records:
        # 把编号、原日志状态、新结果、修正、差异、耗时及原因写进一行。
        rows.append(f"<tr><td><a href='match_{r['index']:03d}/index.html'>{r['index']}</a></td>"
                    f"<td>{r['spin_index']}</td><td>{r['logged_status']} / {r['logged_score']:.3f}</td>"
                    f"<td>{'接受' if r['native_accepted'] else '拒绝'}</td>"
                    f"<td>{r['reference_score']:.6f} → {r['native_score']:.6f}</td>"
                    f"<td>{r['dx_cm']:+.2f}, {r['dy_cm']:+.2f}, {r['dtheta_deg']:+.3f}°</td>"
                    f"<td>{r['position_delta_cm']:.5f} / {r['angle_delta_deg']:.5f}</td>"
                    f"<td>{r['reference_ms']:.2f} → {r['native_ms']:.2f}</td>"
                    f"<td>{html.escape(r['reasons']) or '—'}</td><td>{'PASS' if r['passed'] else 'FAIL'}</td></tr>")
    # 把不能回放的理由合成为报告说明。
    skip_text='；'.join(f"场次{x['index']}：{x['reason']}" for x in skipped) or '无'
    # 创建离线HTML报告内容；模板中的文本和数值在此格式化。
    page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>toflib原生版本回归</title>
<style>body{{font:16px sans-serif;margin:24px;color:#223}}table{{border-collapse:collapse;font-size:14px}}td,th{{padding:9px;border:1px solid #ccd}}th{{background:#eef}}a{{color:#1565c0}}</style>
<h1>toflib原生版本回归</h1><p>共{summary['total']}次记录，回放{summary['replayed']}次；回归不通过：{len(failures)}次。</p>
<p>日志失败组：{summary['groups']['failed']['accepted']}/{summary['groups']['failed']['attempts']}次被算法接受，涉及{summary['groups']['failed']['unique_scan_segments']}个扫描片段。
日志成功组：{summary['groups']['success']['accepted']}/{summary['groups']['success']['attempts']}次被算法接受。</p>
<p>相对double参考版最大差异：{summary['max_position_cm']:.5f} cm，{summary['max_angle_deg']:.5f}°，分数{summary['max_score_delta']:.8f}。
回归门槛：{args.max_position_cm} cm / {args.max_angle_deg}° / {args.max_score_delta}，接受状态和拒绝原因必须一致。</p>
<p>未回放：{html.escape(skip_text)}。</p><p><strong>接受不等于真实定位正确。</strong>同一扫描片段的重试不是独立样本；日志预测位置和地图时刻存在原有解析限制。
参考耗时来自历史基准；新版本计时为本机{args.repeats}次中位数，不含文件读取和逐候选CSV，不代表BK7258耗时。</p>
<p><a href="index.html">查看地图、原始点云和匹配后点云</a> · <a href="regression.csv">下载表格</a></p>
<table><thead><tr><th>场次/地图</th><th>扫描片段</th><th>日志状态/分数</th><th>新算法</th><th>参考→新分数</th><th>修正x,y(cm),角度</th><th>位置(cm)/角度(°)差</th><th>搜索耗时ms 参考→新</th><th>拒绝原因</th><th>回归</th></tr></thead><tbody>{''.join(rows)}</tbody></table></html>'''
    # 保存可直接打开的对比页面。
    (args.out/'comparison.html').write_text(page)
    # 在终端输出本次回归摘要，中文保留原样。
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    # 提示用户应打开的人工可读报告路径。
    print('报告：',args.out/'comparison.html')
    # 只要有回归失败场次，整个验证命令必须以非0退出。
    if failures:
        # 用非0退出码让自动化检查识别回归失败。
        raise SystemExit(1)

# 只有直接启动脚本时进入main/测试运行器，导入模块时不自动执行。
if __name__=='__main__':
    # 执行本脚本的命令入口。
    main()
