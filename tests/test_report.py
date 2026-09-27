"""报告只嵌入当前运行结果，原日志字符串不能破坏脚本；调试区默认折叠。"""
# 读取和保存结构化算法结果及报告数据。
import json
# 统一定位脚本根目录、场景、程序和输出文件。
from pathlib import Path
# 从离线HTML中精确提取内嵌JSON并检查默认展示状态。
import re
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
# 复用生产绘图或报告生成函数，验证实际用户所见输出。
from simulate import report

# 测试离线报告是否混入旧结果、是否正确转义日志文本和折叠详细区。
class ReportTest(unittest.TestCase):
    # 验证仅提取模式页面没有陈旧新算法结果，调试信息默认关闭。
    def test_extract_page_is_offline_and_details_closed(self):
        # 隔离测试临时输入和输出，退出时自动清理，不污染工程目录。
        with tempfile.TemporaryDirectory() as tmp:
            # 创建临时场次目录，所有报告文件均在测试后自动清理。
            out=Path(tmp);bundle=out/'match_013';bundle.mkdir()
            # 写入一个障碍和一个未知格的最小地图样本。
            np.save(bundle/'map.npy',np.array([[245,128]],dtype=np.uint8))
            # 给第二格设置未观测，用来验证缺失数据独立表示。
            np.save(bundle/'observed.npy',np.array([[True,False]]))
            # 写一个最小点云CSV，覆盖报告读取单行数组的情况。
            np.savetxt(bundle/'scan.csv',[[1,2,0,0]],delimiter=',',header='x,y,ox,oy',comments='')
            # 模拟同目录曾运行匹配；提取模式不可误展示陈旧的新算法结果。
            # 保存当前核心完整结果，作为默认报告的数据来源。
            (bundle/'result.json').write_text('{}')
            # 元数据里故意包含脚本结束标签，测试报告嵌入JSON时正确转义。
            meta={'warnings':['</script><script>invalid()</script>'],'prediction':[0,0,0]}
            # 使用真实报告生成函数，result=None明确表示只提取。
            report(out,[(bundle,meta,None)],'log')
            # 读取实际生成的汇总HTML，而非测试手写模板。
            page=(out/'index.html').read_text()
            # 从固定JSON脚本标签中提取数据，核对页面嵌入内容。
            payload=re.search(r'<script id="report-data" type="application/json">(.*?)</script>',page,re.S).group(1)
            # 解码第一场数据以检查报告语义。
            item=json.loads(payload)['cases'][0]
            # 转义前后元数据语义必须一致，不能为了安全丢失原日志内容。
            self.assertEqual(item['meta'],meta)
            # 提取模式不能伪造新算法结果对象。
            self.assertIsNone(item['result'])
            # 即使磁盘存在陈旧result.json，也不能出现在当前详细文件链接中。
            self.assertFalse(any(f[0]=='result.json' for f in item['files']))
            # 原日志中的script片段不应成为可执行页面标签。
            self.assertNotIn('<script>invalid',page)
            # 所有details初始都不能带open属性，保持详细信息默认折叠。
            self.assertNotRegex(page,r'<details[^>]*\bopen\b')
            # 同时检查单场页面，而不只检查总览页。
            single=(bundle/'index.html').read_text()
            # 提取单场页内嵌数据，核对相对文件链接约定。
            single_payload=re.search(r'<script id="report-data" type="application/json">(.*?)</script>',single,re.S).group(1)
            # 单场文件就在当前目录，因此prefix必须为空。
            self.assertEqual(json.loads(single_payload)['cases'][0]['prefix'],'')

# 只有直接启动脚本时进入main/测试运行器，导入模块时不自动执行。
if __name__=='__main__':unittest.main()
