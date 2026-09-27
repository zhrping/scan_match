"""只测试会影响结果可信度的边界：坐标、字段关联、缺损地图和 CLI 输入。"""
# 读取和保存结构化算法结果及报告数据。
import json
# 从CTest设置的环境变量读取本次构建程序路径。
import os
# 计算位置欧氏差、周期角差和数值容差。
import math
# 统一定位脚本根目录、场景、程序和输出文件。
from pathlib import Path
# 启动实际C程序或集成测试，检查退出状态并捕获结果。
import subprocess
# 调整本项目工具导入路径以及处理命令退出。
import sys
# 创建自动清理的临时数据目录，不污染源码目录。
import tempfile
# 使用标准单元测试框架管理断言和测试发现。
import unittest
# 构造地图/点云数组并进行数值对照。
import numpy as np
# 以脚本所在目录定位项目根目录，不依赖启动位置。
ROOT=Path(__file__).resolve().parents[1]
# 优先导入当前项目tools，防止误用系统中同名模块。
sys.path.insert(0,str(ROOT/'tools'))
# 复用生产日志解析与坐标变换，用真实导出协议测试C入口。
from log_data import LogData, inverse_transform, transform, write_scene

# 测试解析、坐标变换、字段关联与C输入边界这些会影响结果可信度的路径。
class PipelineTest(unittest.TestCase):
    # 验证世界→局部→世界的坐标往返，防止旋转矩阵方向写反。
    def test_rigid_transform_roundtrip(self):
        # 构造不同象限的点，避免只测原点或轴对齐情况。
        points=np.array([[1.,2.],[-3.,.5],[0.,0.]])
        # 使用非零平移与角度，覆盖完整刚体变换。
        pose=np.array([2.4,-1.2,.73])
        # 数值对照变换结果，atol为允许绝对误差，不是定位精度承诺。
        np.testing.assert_allclose(transform(inverse_transform(points,pose),pose),points,atol=1e-12)

    # 构造同一扫描两次重试及损坏地图，确认字段不串场、缺失不猜补。
    def test_association_and_missing_cells(self):
        # 这是刻意构造的原始日志字符串，内部损坏行不能被注释处理或拼接修复。
        text='''*#sp_g
 g_pose_20ms 100 200 50 0 0
*#sp 100 200 0 100
*#sp 100 200 50 200
*#sp 100 200 100 300
*#sp_g
[MATCH_RES] res=failed score=0.4 th=0.6
[t] match_failed2 score:0.4 (0,5,0.04)
[SLAM_BOW] task=7 far=1 range=0..5 score=0.4
[MATCH_RES] res=failed score=0.1 th=0.6
[t] match_failed2 score:0.1 (0,8,0.05)
[SLAM_BOW] task=7 far=1 range=6..8 score=0.1
start print map data
pborder minx 128 maxx 130 miny 128 maxy 130
i 128 j 129 mapdata 245
i 129 j 128 mapdata 25
i 130 j 130 mapd
noise
ata 245
'''
        # 隔离测试临时输入和输出，退出时自动清理，不污染工程目录。
        with tempfile.TemporaryDirectory() as tmp:
            # 把构造日志写到临时文件，再调用实际LogData解析器。
            path=Path(tmp)/'test.DAT';path.write_text(text);ld=LogData(path)
            # 检查第一场的修正或范围只来自它自己的日志窗口。
            self.assertEqual(ld.matches[0]['logged_best']['dy_cells'],5)
            # 检查后一次重试不会覆盖前一次，也能保留自己的修正。
            self.assertEqual(ld.matches[1]['logged_best']['dy_cells'],8)
            # 检查第一场的修正或范围只来自它自己的日志窗口。
            self.assertEqual(ld.matches[0]['logged_search']['max_cells'],5)
            # 导出首场的完整输入包，随后检查真实落盘文件。
            out=Path(tmp)/'bundle';meta=ld.export(0,out)
            # 加载导出原图和观测掩码，分别验证值与来源完整性。
            grid=np.load(out/'map.npy');obs=np.load(out/'observed.npy')
            # 确认日志i=x、j=y转换为数组[y,x]，没有转置错误。
            self.assertEqual(grid[129,128],245)
            # 损坏记录对应格必须标为未观测，且值保留128未知。
            self.assertFalse(obs[130,130]);self.assertEqual(grid[130,130],128)
            # 读取生产导出的局部点云，不直接复用解析器内部数组。
            cloud=np.loadtxt(out/'scan.csv',delimiter=',',skiprows=1)
            # 把导出点云按元数据预测位姿放回世界坐标，核查距离与外参解释。
            world=transform(cloud[:,:2],np.array(meta['prediction']))
            # 独立从原始采样角度和距离计算三个世界端点作为真值。
            expected=np.array([[2.,2.],[1+2*math.cos(.5),2+2*math.sin(.5)],
                                [1+3*math.cos(1),2+3*math.sin(1)]])
            # 数值对照变换结果，atol为允许绝对误差，不是定位精度承诺。
            np.testing.assert_allclose(world,expected,atol=1e-10)
            # 核对预测来源距离扫描结束的行数，防止使用未来位姿。
            self.assertEqual(meta['prediction_gap_lines'],3)

    # 验证全未知地图不会接受，非法数值及多余文件内容会被C入口拒绝。
    def test_cli_invalid_and_unknown(self):
        # 把测试可执行程序解析成绝对路径或从环境变量取得本次构建路径。
        exe=Path(os.environ.get('SCAN_MATCH_EXECUTABLE',str(ROOT/'build/scan_match')))
        # 隔离测试临时输入和输出，退出时自动清理，不污染工程目录。
        with tempfile.TemporaryDirectory() as tmp:
            # 为每个输入边界用例准备独立临时scene文件。
            path=Path(tmp)/'scene.txt'
            # 构造全未知小地图和少量合法端点，专门测试零分拒绝。
            grid=np.full((12,12),128,np.uint8);cloud=np.array([[.1,.1,0,0],[.2,.1,0,0],[.1,.2,0,0]])
            # 通过实际场景写入器生成C输入，保证协议检查覆盖真实路径。
            write_scene(path,grid,cloud,[0,0,0],origin=(-.72,-.72))
            # 运行实际程序并捕获结果；check=True时非0会直接抛异常。
            proc=subprocess.run([str(exe),str(path),'--window','0','--angle-deg','0'],capture_output=True,text=True)
            # 正常格式即使匹配被拒绝，程序过程仍应返回0。
            self.assertEqual(proc.returncode,0,proc.stderr)
            # 核对正常完成但不接受，且全未知地图离散得分必须为0。
            r=json.loads(proc.stdout);self.assertFalse(r['accepted']);self.assertEqual(r['discrete_score'],0)
            # 运行实际程序并捕获结果；check=True时非0会直接抛异常。
            proc=subprocess.run([str(exe),str(path),'--angle-step-deg','nan'],capture_output=True,text=True)
            # 非法命令数值或额外场景数据必须导致非0退出。
            self.assertNotEqual(proc.returncode,0)
            # 故意在合法场景末尾添加多余内容，验证解析器不会忽略格式错误。
            path.write_text(path.read_text()+'unexpected\n')
            # 运行实际程序并捕获结果；check=True时非0会直接抛异常。
            proc=subprocess.run([str(exe),str(path)],capture_output=True,text=True)
            # 非法命令数值或额外场景数据必须导致非0退出。
            self.assertNotEqual(proc.returncode,0)

# 只有直接启动脚本时进入main/测试运行器，导入模块时不自动执行。
if __name__=='__main__':unittest.main()
