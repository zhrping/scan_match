#!/usr/bin/env python3
"""在已生成日志数据包上验证 BnB 与完整枚举一致；这不是位置准确率测试。

读取每场 run.json 的原命令，保留参数、去掉评分 CSV 输出，分别运行两种搜索。
保存双方法的真实 PC 时间与结果，并核对离散最优、精修后的所有候选及判定。
"""
# 解析回归脚本命令参数，支持指定可执行程序和输出位置。
import argparse
# 读取和保存结构化算法结果及报告数据。
import json
# 统一定位脚本根目录、场景、程序和输出文件。
from pathlib import Path
# 启动实际C程序或集成测试，检查退出状态并捕获结果。
import subprocess
# 计算位置欧氏差、周期角差和数值容差。
import math

# 以脚本所在目录定位项目根目录，不依赖启动位置。
ROOT=Path(__file__).resolve().parents[1]


# 递归比较JSON中的数组、字典和标量，携带字段路径便于定位差异。
def close(a,b,path='result'):
    # 数组先比较长度，再逐项比较，避免zip静默截断漏掉候选。
    if isinstance(a,list):
        # 候选或字段列表长度不同立即报告失败。
        if len(a)!=len(b): raise AssertionError(path+': length differs')
        # 对相同索引递归比较，错误信息包含具体数组位置。
        for i,(x,y) in enumerate(zip(a,b)): close(x,y,f'{path}[{i}]')
    # 字典按参考对象的键递归核对内容。
    elif isinstance(a,dict):
        # 将当前键名附加到错误路径，便于查到差异字段。
        for k in a: close(a[k],b[k],f'{path}.{k}')
    # 数值使用浮点容差，布尔值必须留给精确比较。
    elif isinstance(a,(int,float)) and not isinstance(a,bool):
        # 使用绝对和相对容差，避免微小舍入差异误报。
        if not math.isclose(a,b,rel_tol=1e-9,abs_tol=1e-9): raise AssertionError(f'{path}: {a} != {b}')
    # 字符串、布尔或空值等非数值字段必须完全一致。
    elif a!=b:
        # 将差异及字段路径抛出，回归不能继续报告通过。
        raise AssertionError(f'{path}: {a} != {b}')


# 回归命令入口；只有直接执行脚本才运行。
def main():
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap=argparse.ArgumentParser(description=__doc__)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('report',type=Path,nargs='?',default=ROOT/'output/review')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--executable',type=Path,default=ROOT/'build/scan_match')
    # 解析参数并初始化回归记录集合。
    args=ap.parse_args();records=[]
    # 按目录顺序遍历已生成场次数据包。
    for bundle in sorted(args.report.glob('match_*')):
        # 读取该场原执行命令，以保留当时的参数设置。
        run=json.loads((bundle/'run.json').read_text())
        # 复制原参数列表的遍历状态，前两项是程序和scene路径。
        argv=run['command'];extras=[];i=2
        # 按选项/值成对扫描原命令。
        while i<len(argv):
            # 去掉trace和method，保留其余影响评分和搜索的参数。
            if argv[i] not in ('--trace','--method'): extras.extend(argv[i:i+2])
            # 跳过当前选项及其值，进入下一参数对。
            i+=2
        # 分别保存枚举和BnB结果，供字段级对照。
        results={}
        # 用两种遍历策略运行完全相同场景和配置。
        for method in ('exhaustive','bnb'):
            # 构造无shell参数数组，路径中空格不会被拆开。
            command=[str(args.executable.resolve()),str(bundle/'scene.txt'),'--method',method]+extras
            # 运行实际程序并捕获结果；check=True时非0会直接抛异常。
            proc=subprocess.run(command,capture_output=True,text=True,check=True)
            # 保存该遍历策略的JSON诊断。
            results[method]=json.loads(proc.stdout)
        # 列出必须一致的实际结果，耗时与节点数不要求相同。
        for key in ('discrete_score','discrete_best','candidates','accepted','reasons','predicted_score','margin'):
            # 递归比较同场两个结果，错误路径包括场次和字段。
            close(results['exhaustive'][key],results['bnb'][key],f'{bundle.name}.{key}')
        # 汇总一致性与搜索开销；节点减少不等于板端耗时必然降低。
        record={'bundle':bundle.name,'consistent':True,
                'exhaustive_ms':results['exhaustive']['search_ms'],
                'bnb_ms':results['bnb']['search_ms'],
                'exhaustive_evaluated':results['exhaustive']['evaluated'],
                'bnb_evaluated':results['bnb']['evaluated'],'bnb_pruned':results['bnb']['pruned']}
        # 保留当前场记录，最后汇总成功/失败分组。
        records.append(record)
        # 保存两种完整结果，便于人工检查候选列表。
        (bundle/'method_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
        # 即时输出该场通过记录。
        print(record,flush=True)
    # 没有数据包时提醒先生成输入，不能静默成功。
    if not records: ap.error('没有找到报告数据包；先运行 tools/simulate.py 生成报告')
    # 保存本批验证结果，并明确没有位置真值且计时不含trace。
    (args.report/'verification.json').write_text(json.dumps({'checks':records,
        'meaning':'Search implementation consistency only; no ground truth position accuracy.',
        'trace_io':False},indent=2)+'\n')
    # 输出本次通过范围和边界，不把实现一致性称为定位准确率。
    print(f'PASS: {len(records)} real-log bundles, all candidate results agree; timing excludes trace I/O')

# 只有直接启动脚本时进入main/测试运行器，导入模块时不自动执行。
if __name__=='__main__':main()
