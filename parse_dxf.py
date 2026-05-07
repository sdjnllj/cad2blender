"""
parse_dxf.py — 解析 CAD DXF 图纸，输出结构化 JSON 供 Blender 建模使用。

支持图层:
  Plan_Walls   — 墙体平面 (LINE 双线 + MTEXT 标签)
  Plan_Columns — 柱子平面 (LWPOLYLINE 闭合矩形)
  Elev_<编号>  — 立面图 (LINE 外轮廓 + LWPOLYLINE 洞口)

用法: python parse_dxf.py <输入.dxf> [输出.json]
"""

import json
import math
import sys
from collections import defaultdict
from itertools import combinations

import ezdxf
from ezdxf.math import area as polygon_area


# ---------------------------------------------------------------------------
# 几何工具
# ---------------------------------------------------------------------------

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def line_direction(start, end, angle_tol_deg=5):
    """返回线段方向: 'H' (水平), 'V' (垂直), 或 'A' (斜)"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return 'P'  # point
    angle = abs(math.degrees(math.atan2(dy, dx)))
    # 水平: 接近 0° 或 180°
    if angle < angle_tol_deg or angle > 180 - angle_tol_deg:
        return 'H'
    # 垂直: 接近 90°
    if 90 - angle_tol_deg < angle < 90 + angle_tol_deg:
        return 'V'
    return 'A'


def line_perp_coord(start, end):
    """
    对于水平线返回 y 坐标, 对于垂直线返回 x 坐标。
    用于按"垂直于线段方向"的坐标排序。
    """
    direction = line_direction(start, end)
    if direction == 'H':
        return (start[1] + end[1]) / 2
    elif direction == 'V':
        return (start[0] + end[0]) / 2
    else:
        return 0


def line_parallel_coord(start, end):
    """
    对于水平线返回 x 范围 (min, max), 垂直线返回 y 范围。
    用于判断两条平行线是否"共线投影重叠"。
    """
    direction = line_direction(start, end)
    if direction == 'H':
        return (min(start[0], end[0]), max(start[0], end[0]))
    elif direction == 'V':
        return (min(start[1], end[1]), max(start[1], end[1]))
    else:
        return (0, 0)


def overlap_ratio(a, b):
    """两个区间 [a_min,a_max] [b_min,b_max] 的重叠比例"""
    lo = max(a[0], b[0])
    hi = min(a[1], b[1])
    if lo >= hi:
        return 0
    overlap = hi - lo
    span_a = a[1] - a[0] if a[1] > a[0] else 1e-9
    span_b = b[1] - b[0] if b[1] > b[0] else 1e-9
    return max(overlap / span_a, overlap / span_b)


# ---------------------------------------------------------------------------
# DXF 提取
# ---------------------------------------------------------------------------

def extract_plan_walls(msp):
    """从 Plan_Walls 层提取所有线段和标签。"""
    lines = []
    labels = []
    for e in msp:
        if e.dxf.layer != 'Plan_Walls':
            continue
        if e.dxftype() == 'LINE':
            s = (e.dxf.start.x, e.dxf.start.y, e.dxf.start.z)
            end = (e.dxf.end.x, e.dxf.end.y, e.dxf.end.z)
            if dist(s, end) > 1e-3:
                lines.append((s, end))
        elif e.dxftype() == 'MTEXT':
            ins = e.dxf.insert
            labels.append({
                'text': e.text.strip(),
                'pos': (ins.x, ins.y),
            })
    return lines, labels


def extract_elevation(msp, layer_name):
    """从指定 Elev_* 层提取外轮廓(LINE)和洞口(LWPOLYLINE)。"""
    outer_lines = []
    holes = []
    for e in msp:
        if e.dxf.layer != layer_name:
            continue
        if e.dxftype() == 'LINE':
            s = (e.dxf.start.x, e.dxf.start.y)
            ee = (e.dxf.end.x, e.dxf.end.y)
            if dist(s, ee) > 1e-3:
                outer_lines.append((s, ee))
        elif e.dxftype() == 'LWPOLYLINE':
            pts_raw = list(e.vertices())
            pts_2d = [(p[0] if isinstance(p, tuple) else p.x,
                       p[1] if isinstance(p, tuple) else p.y)
                      for p in pts_raw]
            if len(pts_2d) < 3:
                continue
            # 判断是否为闭合多段线
            if dist(pts_2d[0], pts_2d[-1]) < 1e-3:
                pts_2d = pts_2d[:-1]  # 去重闭合点
            holes.append(pts_2d)
    return outer_lines, holes


def extract_columns(msp):
    """从 Plan_Columns 层提取柱子轮廓。"""
    columns = []
    for e in msp:
        if e.dxf.layer != 'Plan_Columns':
            continue
        if e.dxftype() == 'LWPOLYLINE':
            pts_raw = list(e.vertices())
            pts_2d = [(p[0] if isinstance(p, tuple) else p.x,
                       p[1] if isinstance(p, tuple) else p.y)
                      for p in pts_raw]
            if len(pts_2d) < 3:
                continue
            if dist(pts_2d[0], pts_2d[-1]) < 1e-3:
                pts_2d = pts_2d[:-1]
            columns.append(pts_2d)
    return columns


# ---------------------------------------------------------------------------
# 墙体线段配对 (双线 → 墙体中心线 + 厚度)
# ---------------------------------------------------------------------------

def build_wall_segments(lines, labels, wall_thickness_range=(80, 800)):
    """
    将 Plan_Walls 中的 LINE 按方向分组, 配对平行且投影重叠的线段,
    识别为墙体段。每段墙由两条平行线(双线)定义。
    """
    # 按方向分组
    h_lines = []  # (start, end, perp_y, parallel_x_range)
    v_lines = []  # (start, end, perp_x, parallel_y_range)

    for s, e in lines:
        d = line_direction(s, e)
        if d == 'H':
            h_lines.append((s, e, line_perp_coord(s, e), line_parallel_coord(s, e)))
        elif d == 'V':
            v_lines.append((s, e, line_perp_coord(s, e), line_parallel_coord(s, e)))
        # 忽略斜线段

    t_min, t_max = wall_thickness_range

    def pair_lines(line_list):
        """在平行线中配对 (双线 = 一面墙)"""
        # 按垂距排序
        sorted_lines = sorted(line_list, key=lambda x: x[2])
        pairs = []
        used = [False] * len(sorted_lines)
        for i in range(len(sorted_lines)):
            if used[i]:
                continue
            for j in range(i + 1, len(sorted_lines)):
                if used[j]:
                    continue
                gap = abs(sorted_lines[i][2] - sorted_lines[j][2])
                if t_min <= gap <= t_max:
                    overlap = overlap_ratio(sorted_lines[i][3], sorted_lines[j][3])
                    if overlap > 0.5:
                        pairs.append((sorted_lines[i], sorted_lines[j]))
                        used[i] = used[j] = True
                        break
        # 未配对的单线 — 可能是一侧外墙, 作为厚度未知的墙段
        singles = [sorted_lines[i] for i in range(len(sorted_lines)) if not used[i]]
        return pairs, singles

    h_pairs, h_singles = pair_lines(h_lines)
    v_pairs, v_singles = pair_lines(v_lines)

    # 构建墙体段列表
    segments = []

    def add_segment(line_a, line_b, direction):
        """由配对双线生成墙体段, 保留原始端点用于外轮廓定位。"""
        a_start, a_end = line_a[0], line_a[1]
        b_start, b_end = line_b[0], line_b[1]
        # 统一线段方向 (从 perp_coord 小的端点起)
        thickness = round(abs(line_a[2] - line_b[2]), 1)
        seg = {
            'line_a': ((a_start[0], a_start[1]), (a_end[0], a_end[1])),
            'line_b': ((b_start[0], b_start[1]), (b_end[0], b_end[1])),
            'thickness': thickness,
            'direction': direction,
        }
        segments.append(seg)

    for a, b in h_pairs:
        add_segment(a, b, 'H')
    for a, b in v_pairs:
        add_segment(a, b, 'V')

    # 计算建筑重心 (用于判断哪侧是外立面)
    all_pts = []
    for seg in segments:
        for lp in (seg['line_a'], seg['line_b']):
            all_pts.extend(lp)
    cx = sum(p[0] for p in all_pts) / len(all_pts) if all_pts else 0
    cy = sum(p[1] for p in all_pts) / len(all_pts) if all_pts else 0
    centroid = (cx, cy)

    def _line_midpoint(ln):
        return ((ln[0][0] + ln[1][0]) / 2, (ln[0][1] + ln[1][1]) / 2)

    # 关联标签 → 最近的墙体段 (用外轮廓线中点算距离)
    for lbl in labels:
        best_seg = None
        best_dist = float('inf')
        px, py = lbl['pos']
        for seg in segments:
            # 用两条线的中点平均值做匹配
            ma = _line_midpoint(seg['line_a'])
            mb = _line_midpoint(seg['line_b'])
            mx = (ma[0] + mb[0]) / 2
            my = (ma[1] + mb[1]) / 2
            d = math.hypot(px - mx, py - my)
            if d < best_dist:
                best_dist = d
                best_seg = seg
        if best_seg is not None:
            best_seg['label'] = lbl['text']

    # 为每段墙确定外轮廓线 (离重心更远的那条线)
    for seg in segments:
        ma = _line_midpoint(seg['line_a'])
        mb = _line_midpoint(seg['line_b'])
        da = math.hypot(ma[0] - cx, ma[1] - cy)
        db = math.hypot(mb[0] - cx, mb[1] - cy)
        exterior = seg['line_a'] if da >= db else seg['line_b']
        seg['exterior_start'] = (round(exterior[0][0], 2), round(exterior[0][1], 2))
        seg['exterior_end'] = (round(exterior[1][0], 2), round(exterior[1][1], 2))

    return segments


# ---------------------------------------------------------------------------
# 立面图处理
# ---------------------------------------------------------------------------

def lines_to_polygon(lines):
    """将未排序的 LINE 集合拼接为有序多边形顶点列表。"""
    if not lines:
        return []
    # 构建端点邻接表
    endpoints = []
    for s, e in lines:
        endpoints.append((s, e))
    # 简单贪心拼接
    chain = [endpoints[0][0], endpoints[0][1]]
    remaining = set(range(1, len(endpoints)))
    while remaining:
        found = False
        tail = chain[-1]
        for idx in list(remaining):
            s, e = endpoints[idx]
            if dist(tail, s) < 1e-3:
                chain.append(e)
                remaining.remove(idx)
                found = True
                break
            elif dist(tail, e) < 1e-3:
                chain.append(s)
                remaining.remove(idx)
                found = True
                break
        if not found:
            break
    # 去除重复的闭合点
    if len(chain) > 1 and dist(chain[0], chain[-1]) < 1e-3:
        chain = chain[:-1]
    return chain


def process_elevation(outer_lines, holes):
    """
    处理立面图:
      outer_lines → 排序为外轮廓多边形
      holes      → 内部洞口多边形列表
    返回 (outer_poly, holes_polys, width_mm, height_mm)
    """
    outer_poly = lines_to_polygon(outer_lines)
    if not outer_poly:
        return None, [], 0, 0

    # 计算立面尺寸
    xs = [p[0] for p in outer_poly]
    ys = [p[1] for p in outer_poly]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)

    # 以左下角为原点重新归一化 (方便后续 3D 映射)
    ox, oy = min(xs), min(ys)
    outer_norm = [(p[0] - ox, p[1] - oy) for p in outer_poly]
    holes_norm = [[(p[0] - ox, p[1] - oy) for p in h] for h in holes]

    return outer_norm, holes_norm, round(width, 1), round(height, 1)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def parse_dxf(dxf_path):
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()

    # 1. 提取平面图数据
    plan_lines, labels = extract_plan_walls(msp)
    print(f"[parse] Plan_Walls: {len(plan_lines)} LINEs, {len(labels)} labels")

    # 2. 识别墙体段
    wall_segments = build_wall_segments(plan_lines, labels)
    print(f"[parse] wall segments: {len(wall_segments)}")
    for seg in wall_segments:
        lbl = seg.get('label', '?')
        thk = seg['thickness']
        d = seg['direction']
        es = seg['exterior_start']
        ee = seg['exterior_end']
        length = math.hypot(ee[0] - es[0], ee[1] - es[1])
        print(f"  {lbl}: {d} 厚={thk}mm 外轮廓 ({es[0]:.0f},{es[1]:.0f})→({ee[0]:.0f},{ee[1]:.0f}) len={length:.0f}")

    # 3. 提取柱子
    columns = extract_columns(msp)
    print(f"[parse] Plan_Columns: {len(columns)} columns")

    # 4. 提取立面图
    elevations = {}
    all_layers = {l.dxf.name for l in doc.layers}
    elev_layers = sorted([ln for ln in all_layers if ln.startswith('Elev_')])
    print(f"[parse] Elev layers: {elev_layers}")

    for layer_name in elev_layers:
        outer_lines, holes = extract_elevation(msp, layer_name)
        if not outer_lines and not holes:
            print(f"  {layer_name}: empty, skipping")
            continue
        outer_poly, holes_polys, w, h = process_elevation(outer_lines, holes)
        wall_id = layer_name.replace('Elev_', '')
        elevations[wall_id] = {
            'outer_contour': outer_poly,
            'openings': holes_polys,
            'width_mm': w,
            'height_mm': h,
        }
        print(f"  {layer_name}: {w}×{h} mm, {len(holes_polys)} openings")

    # 5. 组装输出
    walls_output = []
    for seg in wall_segments:
        lbl = seg.get('label', '')
        wall_entry = {
            'label': lbl,
            'exterior_start': list(seg['exterior_start']),
            'exterior_end': list(seg['exterior_end']),
            'thickness_mm': seg['thickness'],
            'direction': seg['direction'],
        }
        if lbl and lbl in elevations:
            wall_entry['elevation'] = elevations[lbl]
        walls_output.append(wall_entry)

    columns_output = []
    for pts in columns:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        columns_output.append({
            'center': [round(sum(xs) / len(xs), 2), round(sum(ys) / len(ys), 2)],
            'width_x': round(max(xs) - min(xs), 2),
            'width_y': round(max(ys) - min(ys), 2),
            'vertices': [[round(p[0], 2), round(p[1], 2)] for p in pts],
        })

    # 6. 计算默认高度 (取所有立面中最大的高度)
    max_elev_h = max((e['height_mm'] for e in elevations.values()), default=3000)

    result = {
        'meta': {
            'file': dxf_path,
            'unit': 'mm',
            'default_height_mm': max_elev_h,
        },
        'walls': walls_output,
        'columns': columns_output,
    }

    return result


def main():
    if len(sys.argv) < 2:
        print("用法: python parse_dxf.py <input.dxf> [output.json]")
        sys.exit(1)

    dxf_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else dxf_path.rsplit('.', 1)[0] + '.json'

    result = parse_dxf(dxf_path)

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    print(f"\n[output] {out_path}")
    print(f"  Walls: {len(result['walls'])}")
    print(f"  Columns: {len(result['columns'])}")
    print(f"  Default height: {result['meta']['default_height_mm']} mm")


if __name__ == '__main__':
    main()
