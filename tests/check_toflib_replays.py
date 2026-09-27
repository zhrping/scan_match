#!/usr/bin/env python3
"""真实日志的适配回归；比较相同float输入下的适配层与独立核心，不证明定位真值。"""
# 解析回归脚本命令参数，支持指定可执行程序和输出位置。
import argparse
# 统一定位脚本根目录、场景、程序和输出文件。
from pathlib import Path
# 启动实际C程序或集成测试，检查退出状态并捕获结果。
import subprocess
# 调整本项目工具导入路径以及处理命令退出。
import sys
# 创建自动清理的临时数据目录，不污染源码目录。
import tempfile

# 以脚本所在目录定位项目根目录，不依赖启动位置。
ROOT = Path(__file__).resolve().parents[1]
# 优先导入当前项目tools，防止误用系统中同名模块。
sys.path.insert(0, str(ROOT / 'tools'))
# 复用生产日志解析与坐标变换，用真实导出协议测试C入口。
from log_data import LogData


# 回归命令入口；只有直接执行脚本才运行。
def main():
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap = argparse.ArgumentParser(description=__doc__)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('log', type=Path)
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--matches', default='1,4,13,19,35,36')
    # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
    ap.add_argument('--executable', type=Path,
                    default=ROOT / 'build-native/test_toflib_adapter')
    # 解析命令行参数，非法选项由argparse直接报告。
    args = ap.parse_args()
    # 把测试可执行程序解析成绝对路径或从环境变量取得本次构建路径。
    exe = args.executable.resolve()
    # 测试程序不存在时给出编译提示。
    if not exe.is_file():
        # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
        ap.error('请先开启 BUILD_TESTING 编译')
    # 把用户指定的场次列表转换成整数。
    indices = [int(x) for x in args.matches.split(',')]
    # 空列表没有验证意义，作为参数错误拒绝。
    if not indices:
        # 建立或注册命令行选项；默认值用于未显式指定路径或阈值的情况。
        ap.error('场次不能为空')
    # 读取真实日志并建立匹配尝试与扫描段索引。
    ld = LogData(args.log)
    # 隔离测试临时输入和输出，退出时自动清理，不污染工程目录。
    with tempfile.TemporaryDirectory(prefix='sm_tof_replays_') as tmp:
        # 逐个提取并运行指定真实日志场次。
        for index in indices:
            # 每个场次用独立目录保存scene和验证结果。
            bundle = Path(tmp) / f'match_{index:03d}'
            # 生成真实地图、点云和scene，并保留源行号等元数据。
            meta = ld.export(index, bundle)
            # 运行实际程序并捕获结果；check=True时非0会直接抛异常。
            proc = subprocess.run([str(exe), str(bundle / 'scene.txt')],
                                  capture_output=True, text=True)
            # 子进程非0意味着测试或输入失败，必须报告出来。
            if proc.returncode:
                # 输入不足、子进程失败或缺少结果时明确终止，错误不能算作通过。
                raise RuntimeError(f'场次{index}: {proc.stdout}\n{proc.stderr}')
            # 输出该场原日志状态及适配测试摘要，便于区分原结果与回归结果。
            print(f"场次{index}, 日志{meta['match']['logged_status']}: {proc.stdout.strip()}")
    # 只宣称类型适配和结果一致性通过，不宣称实机定位真值正确。
    print(f'PASS: {len(indices)}场；验证地图布局、坐标转换和匹配结果一致性，不代表定位准确率。')


# 只有直接启动脚本时进入main/测试运行器，导入模块时不自动执行。
if __name__ == '__main__':
    # 执行本脚本的命令入口。
    main()
