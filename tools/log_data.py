"""串口日志解析；只复用旧工具确认的字段约定，不导入旧工具/算法。

注意：日志不是完整的现场状态快照。每条关联保留行号和警告，缺损不猜补。
sp: x_cm y_cm theta_centiradian range_cm；range 已含前置外参（旧工具约定）。
地图使用最后一次 dump；早期匹配只可用于离线评估，不能声称复现现场得分。
"""
# 用Path统一日志和输出路径操作，避免手写平台路径分隔符。
from pathlib import Path
# 二分查找匹配记录之前最近的扫描段，避免每次从头遍历。
import bisect
# 写带列名的原始地图CSV，方便逐格核对来源。
import csv
# 保存结构化元数据、警告和损坏行记录。
import json
# 角度换算与旋转参数计算，内部角度统一弧度。
import math
# 用正则提取日志中明确完整的记录，不跨损坏行猜测拼接。
import re
# 用数组处理点云、地图、布尔掩码和批量刚体变换。
import numpy as np

# 支持有正负号的整数或小数，用于匹配日志中的角度字段。
NUM = r'[-+]?\d+(?:\.\d+)?'
# 提取i、j、mapdata三项；边界校验在解析成功后执行。
MAP = re.compile(r'\bi\s+(\d+)\s+j\s+(\d+)\s+mapdata\s+(\d+)\b')
# 只接受以四个整数结束的完整sp行，避免把交错日志误当测距记录。
SP = re.compile(r'\*#sp\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*$')
# 提取高频位姿代理记录，不能把它等同于固件精确匹配输入。
POSE = re.compile(r'g_pose_20ms\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\b')
# 识别每次匹配尝试的成功/失败、得分和阈值，作为场次索引来源。
MATCH = re.compile(r'\[MATCH_RES\] res=(success|failed) score=([\d.]+) th=([\d.]+)')
# 提取该次尝试的候选修正：x/y整数格、theta弧度，不混用最终累计修正。
BEST = re.compile(r'\[t\] match_(?:success|failed)2?.*?score:([\d.]+)\s+\((-?\d+),(-?\d+),('+NUM+r')\)')
# 提取旧算法任务编号和搜索格数范围，仅供报告查看。
RANGE = re.compile(r'\[SLAM_BOW\].*?task=(\d+).*?range=(\d+)\.\.(\d+)')


# 局部端点转世界地图坐标：行向量写法对应先旋转再平移。
def transform(points, pose):
    """行向量二维刚体变换；输入 Nx2，返回地图坐标。"""
    # 用位姿角度计算旋转系数；pose前两项是米制世界平移。
    c, s = math.cos(pose[2]), math.sin(pose[2])
    # 输入统一转数组，右乘适合行向量的旋转矩阵，再广播加平移。
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


# 世界坐标转参考机器人坐标，与transform互为逆变换。
def inverse_transform(points, pose):
    # 用位姿角度计算旋转系数；pose前两项是米制世界平移。
    c, s = math.cos(pose[2]), math.sin(pose[2])
    # 先减世界平移，再乘逆旋转矩阵，不要把运算顺序颠倒。
    return (np.asarray(points) - pose[:2]) @ np.array([[c, -s], [s, c]])


# 一次日志解析对象：保存原始行号、末次地图、扫描片段和匹配尝试的关联。
class LogData:
    # 读取整份日志并建立索引；此阶段不运行匹配、不修改原日志。
    def __init__(self, path):
        # 保存绝对源路径，后续导出元数据可追溯到原文件。
        self.path = Path(path).resolve()
        # splitlines 的逻辑行号会把 CR 也作为分隔，保持与旧工具场次索引一致。
        # 容忍串口中的非法编码并拆分逻辑行；记录的行号从1开始。
        self.lines = self.path.read_text(errors='replace').splitlines()
        # 分别保存位姿、测距、扫描边界及匹配尝试列表。
        self.poses, self.samples, self.boundaries, self.matches = [], [], [], []
        # 用(i,j)映射占据值和来源行，避免将缺失格误填成已观测空闲。
        self.map_cells, self.map_lines = {}, {}
        # 尚未看到地图dump起始标志。
        self.map_start = None
        # 收集疑似地图但格式不完整的原始文本，保留人工核对依据。
        self.map_fragments = []
        # 计数可解析但索引或占据值越界的地图行。
        self.invalid_map_rows = 0
        # 计数同次地图dump里重复出现的格子记录。
        self.map_duplicates = 0
        # 保存日志给出的地图边界提示，没有则保留未知。
        self.pborder = None
        # 统计含sp标记但无法完整解析的测距行。
        self.bad_sp = 0
        # 按源顺序扫描日志，同时保留从1开始的逻辑行号。
        for line_no, line in enumerate(self.lines, 1):
            # 遇到新地图dump时丢弃之前地图快照，只保留最后一份用于离线评估。
            if 'start print map data' in line:
                # 记录当前地图快照开始于哪一行。
                self.map_start = line_no
                # 新快照的占据值、行号和损坏记录从空开始，不能跨快照补缺。
                self.map_cells, self.map_lines, self.map_fragments = {}, {}, []
                # 重置当前地图dump的异常统计。
                self.invalid_map_rows = self.map_duplicates = 0
            # 尝试提取本行地图坐标和值。
            m = MAP.search(line)
            # 正则确实匹配到完整字段后才读取捕获组。
            if m:
                # 把文本坐标和值转成整数，准备校验。
                i, j, value = map(int, m.groups())
                # 索引必须在256×256地图内，值必须能装入uint8。
                if 0 <= i < 256 and 0 <= j < 256 and 0 <= value <= 255:
                    # 同一个格子再次出现时记录重复，而不是悄悄忽略异常。
                    if (i, j) in self.map_cells:
                        # 重复计数加1，下面仍采用该格最后一次记录。
                        self.map_duplicates += 1
                    # 保存当前格占据值；物理数组转置在导出阶段明确处理。
                    self.map_cells[i, j] = value
                    # 保存该值的来源行号，便于追溯原日志。
                    self.map_lines[i, j] = line_no
                # 不满足上方条件时走替代分支，避免把非法或缺失记录作为正常数据。
                else:
                    # 记录地图越界/非法值，不写入地图字典。
                    self.invalid_map_rows += 1
            # 只对地图dump中的疑似损坏地图行留存文本，不尝试跨日志片段拼接。
            elif ('mapdata' in line or re.search(r'\bi\s+\d+\s+j\b', line)) and self.map_start:
                # 日志线程交错时不跨任意文本拼接，防止创造一条看似正确的记录。
                # 保留损坏行的编号和完整文本，用于详细模式查看。
                self.map_fragments.append({'line': line_no, 'text': line})
            # 独立解析地图边界提示，不影响格子值的完整性判定。
            m = re.search(r'pborder minx (\d+) maxx (\d+) miny (\d+) maxy (\d+)', line)
            # 正则确实匹配到完整字段后才读取捕获组。
            if m:
                # 将四个边界索引转整数并保存。
                self.pborder = list(map(int, m.groups()))
            # 尝试解析本行完整单束ToF采样记录。
            m = SP.search(line)
            # 正则确实匹配到完整字段后才读取捕获组。
            if m:
                # 依次获得机器人位置cm、角度百分之一弧度、测距cm。
                x, y, theta, distance = map(int, m.groups())
                # 保存行号并将位置/距离除100变成米、角度除100变成弧度。
                self.samples.append((line_no, x/100, y/100, theta/100, distance/100))
            # 出现测距标记但字段不完整时只计异常，不构造假点。
            elif re.search(r'\*#sp\s', line):
                # 损坏测距行计数加1。
                self.bad_sp += 1
            # 显式扫描边界标志用于分段，避免仅靠时间间隔猜测一圈。
            if re.search(r'\*#sp_g\b', line):
                # 记下边界逻辑行号，随后与测距记录按顺序关联。
                self.boundaries.append(line_no)
            # 解析高频预测位姿记录。
            m = POSE.search(line)
            # 正则确实匹配到完整字段后才读取捕获组。
            if m:
                # 保存位姿行号和统一为米/弧度的三个分量。
                self.poses.append((line_no, *(int(x)/100 for x in m.groups())))
            # 解析一次匹配尝试的状态行；重试会单独计为一个场次。
            m = MATCH.search(line)
            # 正则确实匹配到完整字段后才读取捕获组。
            if m:
                # 按出现顺序生成场次编号，同时保存日志分数、阈值及来源行。
                self.matches.append({'index': len(self.matches), 'line': line_no,
                                     'logged_status': m[1], 'logged_score': float(m[2]),
                                     'logged_threshold': float(m[3])})
        # 使用显式 sp_g 分段，绝不把上一段缺失后的多圈数据静默拼接。
        # 开始建立按sp_g边界划分的扫描片段列表。
        self.spins = []
        # 暂存当前扫描段内的测距记录。
        current = []
        # 指向尚未消费的下一个扫描边界。
        boundary_i = 0
        # 记录当前段是否有明确起始边界，首段可能是不完整截取。
        previous_boundary = None
        # 按原顺序逐条采样分段，不根据匹配成功与否挑点。
        for row in self.samples:
            # 当采样已越过一个或多个边界时，逐个关闭之前片段。
            while boundary_i < len(self.boundaries) and row[0] > self.boundaries[boundary_i]:
                # 只有真正包含测距的片段才加入列表，不生成空扫描段。
                if current:
                    # 发布该段采样及起止边界是否完整的标志，不承诺角度一定覆盖一圈。
                    self.spins.append({'samples': current, 'closed': True,
                                       'has_start_boundary': previous_boundary is not None})
                # 暂存当前扫描段内的测距记录。
                current = []
                # 记录刚刚跨过的边界，用来判断下一段是否有起始标记。
                previous_boundary = self.boundaries[boundary_i]
                # 向后移动边界游标，避免同一边界重复处理。
                boundary_i += 1
            # 把当前采样放入所属扫描片段。
            current.append(row)
        # 只有真正包含测距的片段才加入列表，不生成空扫描段。
        if current:
            # 发布该段采样及起止边界是否完整的标志，不承诺角度一定覆盖一圈。
            self.spins.append({'samples': current,
                               'closed': boundary_i < len(self.boundaries),
                               'has_start_boundary': previous_boundary is not None})
        # 取每个扫描段最后一个采样的行号，供二分关联匹配尝试。
        ends = [s['samples'][-1][0] for s in self.spins]
        # 逐次匹配尝试寻找对应的扫描片段及近邻日志细节。
        for i, match in enumerate(self.matches):
            # 寻找结束行不晚于MATCH_RES的最后一段，减1转换成数组索引。
            pos = bisect.bisect_right(ends, match['line']) - 1
            # 候选段必须存在且距匹配行不超过3000行，否则标记无法可靠关联。
            match['spin_index'] = pos if pos >= 0 and match['line']-ends[pos] <= 3000 else None
            # 仅关联本条 MATCH_RES 后、下条 MATCH_RES 前的细节，不将最终重试
            # 的 m_final 错配给前面的每一次尝试。
            # 细节搜索最多看后30行且不跨下一次MATCH_RES，防止重试之间串结果。
            end = min(self.matches[i+1]['line']-1 if i+1<len(self.matches) else len(self.lines),
                      match['line']+30)
            # 源行号从1开始，列表从0开始；切片从MATCH_RES之后第一行开始。
            for line in self.lines[match['line']:end]:
                # 寻找这次尝试的详细最佳修正记录。
                m = BEST.search(line)
                # 只取限定窗口内第一个细节记录，后续记录不能覆盖此次结果。
                if m and 'logged_best' not in match:
                    # 保存原日志候选分数与格数/弧度修正，后续报告再换算米。
                    match['logged_best'] = {'score': float(m[1]), 'dx_cells': int(m[2]),
                                            'dy_cells': int(m[3]), 'dtheta_rad': float(m[4])}
                # 读取同一尝试附近的旧搜索范围。
                m = RANGE.search(line)
                # 搜索范围同样只取首个匹配记录，避免后面的任务覆盖。
                if m and 'logged_search' not in match:
                    # 把任务号及最小/最大格数保存为可读元信息。
                    match['logged_search'] = {'task': int(m[1]), 'min_cells': int(m[2]), 'max_cells': int(m[3])}

    # 惰性产生每次尝试的摘要，供--list使用，不导出大数组。
    def listing(self):
        # 按场次出现顺序输出摘要。
        for m in self.matches:
            # 关联不存在时保持None，而不是错误访问第0段。
            spin = self.spins[m['spin_index']] if m['spin_index'] is not None else None
            # 合并匹配元数据与关联段原始采样数，没有段则点数为0。
            yield {**m, 'samples': len(spin['samples']) if spin else 0}

    # 导出一个场次的地图、点云、元数据和C场景；默认前置外参0.142m、最大测距12m。
    def export(self, index, out, sensor_offset=.142, max_range=12.0):
        # 先校验场次索引，错误消息带合法范围。
        if index < 0 or index >= len(self.matches):
            # 记录不足或关联不可靠时明确失败，不制造一份看似完整的仿真输入。
            raise ValueError(f'场次 {index} 不存在；范围 0..{len(self.matches)-1}')
        # 取得待导出的匹配尝试元信息。
        match = self.matches[index]
        # 没有关联扫描段就无法构造点云，必须拒绝导出。
        if match['spin_index'] is None:
            # 记录不足或关联不可靠时明确失败，不制造一份看似完整的仿真输入。
            raise ValueError(f'场次 {index} 无可关联的 sp 段')
        # 完全没有可解析地图时不能凭空构造全未知场景。
        if not self.map_cells:
            # 记录不足或关联不可靠时明确失败，不制造一份看似完整的仿真输入。
            raise ValueError('没有完整可解析的地图格子')
        # 取关联扫描段，后续只使用这段已有测距。
        spin = self.spins[match['spin_index']]
        # 把采样列表转成二维浮点数组，列仍含行号、机器人位姿和距离。
        rows = np.asarray(spin['samples'], dtype=float)
        # 扫描最后一条测距行号作为寻找预测位姿的时间上限。
        end = int(rows[-1, 0])
        # 只用扫描结束前的位姿，避免混入匹配修正后的未来位姿。
        # 只选扫描结束前800行内的位姿记录，排除匹配后的未来修正。
        poses = [p for p in self.poses if p[0] <= end and end-p[0] <= 800]
        # 没有合适的预测代理时停止导出，不能使用默认零位姿冒充。
        if not poses:
            # 记录不足或关联不可靠时明确失败，不制造一份看似完整的仿真输入。
            raise ValueError(f'场次 {index} 无扫描结束前 800 行内的预测位姿代理')
        # 选最接近扫描结束时刻的过去位姿。
        pred_record = poses[-1]
        # 取该记录的x/y/theta组成米/弧度预测向量。
        prediction = np.array(pred_record[1:], dtype=float)
        # 只剔除不大于外参/最短距离或超过最大量程的采样，不按匹配分数筛点。
        keep = (rows[:, 4] > max(sensor_offset, 0.05)) & (rows[:, 4] <= max_range)
        # 用有效量程布尔掩码取点，保持原顺序。
        filtered = rows[keep]
        # 少于3个有效测距不足以形成此工具的回放输入，直接报错。
        if len(filtered) < 3:
            # 记录不足或关联不可靠时明确失败，不制造一份看似完整的仿真输入。
            raise ValueError(f'场次 {index} 有效测距不足 3 个')
        # 用每次采样自己的机器人角度生成单位射线方向，处理一圈中机器人转动。
        directions = np.column_stack([np.cos(filtered[:, 3]), np.sin(filtered[:, 3])])
        # 端点为采样机器人位置加方向乘日志距离；日志距离按已有外参约定解释。
        world_points = filtered[:, 1:3] + directions*filtered[:, 4:5]
        # 原日志 range 已含前置外参；只调整射线原点，不再次给端点加外参。
        # 射线起点为采样机器人位置加安装偏移，不再给端点重复增加外参。
        world_origins = filtered[:, 1:3] + directions*sensor_offset
        # 把所有世界端点统一到选定的预测参考系，供C核心进行刚体匹配。
        local = inverse_transform(world_points, prediction)
        # 射线起点也必须转换到同一参考系，才能与端点一致变换。
        origins = inverse_transform(world_origins, prediction)
        # 组成四列场景数据：端点x/y、起点x/y，单位米。
        cloud = np.column_stack([local, origins])
        # 缺失格默认128未知；数组索引为[y,x]，与连续C地图布局一致。
        grid = np.full((256, 256), 128, dtype=np.uint8)  # y,x，与 C 一致
        # 单独保存是否真的打印过的掩码，区分真实未知与日志缺失。
        observed = np.zeros_like(grid, dtype=bool)
        # 把日志的i=x、j=y逐项填入行优先数组。
        for (i, j), value in self.map_cells.items():
            # 明确交换数组索引顺序，防止可视化地图转置。
            grid[j, i] = value
            # 这个格子有日志证据，即使值为128也不是缺失记录。
            observed[j, i] = True
        # 消除角度跨±π或2π时的跳变，估算该扫描段的实际角度跨度。
        angles = np.unwrap(filtered[:, 3])
        # 用最大角减最小角估计跨度，并转成普通float便于JSON序列化。
        span = float(np.ptp(angles))
        # 把日志代理、地图时刻和外参等限制写入元数据，报告默认只在详细区展示。
        warnings = [
            '使用日志末尾地图，不等于现场匹配时的地图；日志成功/失败也不是定位真值。',
            '预测位姿为扫描结束前 g_pose_20ms 代理；并非固件精确 initial_pose。',
            'sp 字段单位、已含外参的距离解释来自旧工具，需要人工核对。',
            'sp 段不保证完整一圈；未做额外去噪、补点或利用匹配结果筛点。',
            '射线原点使用固定前置外参假设，冲突比例只作诊断。',
        ]
        # 角度跨度小于300°时额外提醒不能把它叫完整一圈。
        if span < math.radians(300):
            # 增加该场次特有的数据完整性说明，避免隐藏输入限制。
            warnings.append(f'该段角度跨度仅 {math.degrees(span):.1f}°，不应称为完整一圈。')
        # 起始或结束标志缺失时，明确提示扫描边界不完整。
        if not spin['closed'] or not spin['has_start_boundary']:
            # 增加该场次特有的数据完整性说明，避免隐藏输入限制。
            warnings.append('扫描段的起始或结束边界不完整。')
        # 只对实际观测格统计原始占据值分布，缺失填充值不混入统计。
        values, counts = np.unique(grid[observed], return_counts=True)
        # 组织可追溯元数据：日志场次、预测来源、点云数量、地图完整性、警告；ground_truth为空。
        meta = {'schema': 1, 'source_log': str(self.path), 'match': match,
                'prediction': prediction.tolist(), 'prediction_source_line': pred_record[0],
                'prediction_gap_lines': end-pred_record[0],
                'scan': {'source_start_line': int(rows[0, 0]), 'source_end_line': end,
                         'raw_count': len(rows), 'count': len(cloud), 'invalid_range_count': int((~keep).sum()),
                         'angle_span_deg': math.degrees(span), 'sensor_offset_m': sensor_offset,
                         'frame': 'reference_robot', 'columns': ['x_m','y_m','origin_x_m','origin_y_m']},
                'map': {'width':256, 'height':256, 'resolution_m':.12, 'origin_center_m':[-15.36,-15.36],
                        'dump_start_line':self.map_start, 'pborder':self.pborder,
                        'parsed_cells':len(self.map_cells), 'missing_cells':int((~observed).sum()),
                        'damaged_fragments':len(self.map_fragments), 'duplicate_rows':self.map_duplicates,
                        'invalid_rows':self.invalid_map_rows,
                        'value_histogram':{str(int(v)):int(n) for v,n in zip(values,counts)}},
                'warnings':warnings, 'ground_truth': None}
        # 创建当前场次输出目录，已存在时允许覆盖同名生成文件。
        out = Path(out); out.mkdir(parents=True, exist_ok=True)
        # 以UTF-8可读JSON保存元信息，末尾换行便于文本工具查看。
        (out/'bundle.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2)+'\n')
        # 保存原始占据数组，供绘图和交互报告加载。
        np.save(out/'map.npy', grid)
        # 保存观测掩码，用于把缺失格显示成独立颜色。
        np.save(out/'observed.npy', observed)
        # 导出统一参考系的四列点云，不给CSV表头添加numpy默认#前缀。
        np.savetxt(out/'scan.csv', cloud, delimiter=',', header='x_m,y_m,origin_x_m,origin_y_m', comments='')
        # 保留原始扫描段的采样及来源行，便于核查被量程过滤的点。
        np.savetxt(out/'raw_sp.csv', rows, delimiter=',', header='line,x_m,y_m,theta_rad,range_m', comments='')
        # 使用上下文管理器写逐格CSV，退出时自动关闭文件。
        with (out/'map_cells.csv').open('w', newline='') as f:
            # 创建CSV写入器并写列名，line列关联回原日志。
            w=csv.writer(f);w.writerow(['i','j','value','line'])
            # 按坐标排序输出，方便比较两个导出文件。
            for (i,j),value in sorted(self.map_cells.items()):
                # 写一个格子的坐标、原始占据值和对应日志行号。
                w.writerow([i,j,value,self.map_lines[i,j]])
        # 另存损坏地图片段，缺损只记录不猜补。
        (out/'damaged_map_rows.json').write_text(json.dumps(self.map_fragments,ensure_ascii=False,indent=2)+'\n')
        # 生成C命令行程序能读取的独立文本输入。
        write_scene(out/'scene.txt', grid, cloud, prediction)
        # 返回元数据供上层画图和生成汇总报告。
        return meta


# 写SM_SCENE_V1协议；原图和点云数据只序列化，不运行算法。
def write_scene(path, grid, cloud, prediction, resolution=.12, origin=(-15.36,-15.36)):
    """简单文本边界协议，方便 review；核心本身不依赖此格式。
    格子顺序 y 外层、x 内层；数值使用足够精度，保持坐标逆变换可复核。
    """
    # 打开并自动管理场景文件，写入失败会向上抛异常。
    with Path(path).open('w') as f:
        # 第一行标记版本，第二行给宽高、每格米数及(0,0)中心世界坐标。
        f.write(f'SM_SCENE_V1\n{grid.shape[1]} {grid.shape[0]} {resolution:.12g} {origin[0]:.12g} {origin[1]:.12g}\n')
        # 下一行写预测x/y/theta和点数，与C端fscanf顺序一致。
        f.write(' '.join(f'{x:.12g}' for x in prediction)+f' {len(cloud)}\n')
        # 按y外层、x内层写出地图整数值。
        np.savetxt(f,grid,fmt='%d')
        # 逐点写四个米制浮点数，12位有效数字用于复核坐标变换。
        np.savetxt(f,cloud,fmt='%.12g')
