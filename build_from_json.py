"""
build_from_json.py — 在 Blender 中读取 JSON，自动构建建筑外立面 3D 模型。

由 parse_dxf.py 生成的 JSON 驱动，使用纯四边形网格 (无布尔运算)。

用法 (在 Blender Python 控制台或通过 MCP):
    import build_from_json
    build_from_json.build("path/to/data.json")
    build_from_json.export_fbx("output.fbx")
"""

import json
import math
from collections import Counter

import bpy
from mathutils import Vector


# =============================================================================
#  网格生成 — 带矩形洞口的四边形网格 (grid-cut 法)
# =============================================================================

E_BOTTOM, E_RIGHT, E_TOP, E_LEFT = 0, 1, 2, 3


def build_wall_mesh_with_openings(outer_w, outer_h, openings, thickness):
    """在局部坐标生成带洞口墙体网格。

    局部坐标系: X=沿墙, Y=高度, Z=厚度 (0=外立面, -thickness=内立面)
    openings: [(x1,y1, x2,y2), ...] 洞口矩形包围盒 (mm)

    返回 (vertices, faces, boundary_edge_list):
      boundary_edge_list: [(va, vb, world_outward_dir_code), ...]
        用于后续在 world space 中计算正确法线
    """
    x_cuts = [0.0, outer_w]
    y_cuts = [0.0, outer_h]
    for x1, y1, x2, y2 in openings:
        x_cuts.extend([x1, x2])
        y_cuts.extend([y1, y2])
    x_cuts = sorted(set(x_cuts))
    y_cuts = sorted(set(y_cuts))

    gw = len(x_cuts)
    gh = len(y_cuts)

    # 共享格点顶点
    vert_index = {}
    front_verts = []
    for iy, y in enumerate(y_cuts):
        for ix, x in enumerate(x_cuts):
            vert_index[(ix, iy)] = len(front_verts)
            front_verts.append((x, y, 0.0))

    bo = len(front_verts)
    # local Z: 0=外立面, +thickness=内立面 (向内法线方向)
    back_verts = [(x, y, thickness) for x, y, _ in front_verts]

    def cell_in_opening(cx, cy):
        cx1, cx2 = x_cuts[cx], x_cuts[cx + 1]
        cy1, cy2 = y_cuts[cy], y_cuts[cy + 1]
        for ox1, oy1, ox2, oy2 in openings:
            if (cx1 >= ox1 - 1e-6 and cx2 <= ox2 + 1e-6 and
                cy1 >= oy1 - 1e-6 and cy2 <= oy2 + 1e-6):
                return True
        return False

    front_faces = []
    back_faces = []
    edge_entries = []

    for cy in range(gh - 1):
        for cx in range(gw - 1):
            if cell_in_opening(cx, cy):
                continue

            v00 = vert_index[(cx, cy)]
            v10 = vert_index[(cx + 1, cy)]
            v11 = vert_index[(cx + 1, cy + 1)]
            v01 = vert_index[(cx, cy + 1)]

            front_faces.append((v00, v10, v11, v01))
            back_faces.append((v01 + bo, v11 + bo, v10 + bo, v00 + bo))

            edge_entries.append((v00, v10, E_BOTTOM))
            edge_entries.append((v10, v11, E_RIGHT))
            edge_entries.append((v11, v01, E_TOP))
            edge_entries.append((v01, v00, E_LEFT))

    # 边界检测
    edge_keys = [(min(a, b), max(a, b)) for a, b, _ in edge_entries]
    edge_count = Counter(edge_keys)
    boundary_set = {k for k, c in edge_count.items() if c == 1}

    # 侧面 — 用统一绕组，法线修正交给 fix_normals()
    side_faces = []
    boundary_edges = []  # 输出给 fix_normals 的边界信息
    for va, vb, code in edge_entries:
        k = (min(va, vb), max(va, vb))
        if k not in boundary_set:
            continue
        # 统一: (va, vb, vb+bo, va+bo) — 部分面法线后续修正
        side_faces.append((va, vb, vb + bo, va + bo))
        boundary_edges.append((len(side_faces) - 1, code, k))

    all_verts = front_verts + back_verts
    all_faces = front_faces + back_faces + side_faces
    face_groups = {
        'front': list(range(len(front_faces))),
        'back': list(range(len(front_faces), len(front_faces) + len(back_faces))),
        'side': list(range(len(front_faces) + len(back_faces), len(all_faces))),
        'side_meta': boundary_edges,  # [(side_face_local_idx, edge_type_code, sorted_edge_key)]
    }

    return all_verts, all_faces, face_groups


# =============================================================================
#  简单墙体 / 柱子网格 (无洞口)
# =============================================================================

def build_simple_wall_mesh(exterior_start, exterior_end, height_mm, thickness_mm,
                           inward_normal):
    """生成无洞口矩形墙体网格 (mm 单位)。

    exterior_start/end: 外立面轮廓线端点 (平面图原始 LINE)
    inward_normal: 从外立面向内的方向 (nx, ny)
    返回 (vertices, faces): 世界坐标 (m) 的顶点 + 面索引
    """
    dx = exterior_end[0] - exterior_start[0]
    dy = exterior_end[1] - exterior_start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return [], []

    wdx = dx / length
    wdy = dy / length

    in_len = math.hypot(inward_normal[0], inward_normal[1])
    inx = inward_normal[0] / in_len
    iny = inward_normal[1] / in_len

    h = height_mm
    t = thickness_mm
    sx, sy = exterior_start

    # 外立面 (exterior face)
    # 内立面 = 外立面 + inward_normal * thickness
    verts_mm = [
        (sx,                       sy,                       0),   # 0 底-外-始
        (sx + wdx * length,        sy + wdy * length,        0),   # 1 底-外-终
        (sx + wdx * length + inx * t, sy + wdy * length + iny * t, 0),   # 2 底-内-终
        (sx + inx * t,             sy + iny * t,             0),   # 3 底-内-始
        (sx,                       sy,                       h),   # 4 顶-外-始
        (sx + wdx * length,        sy + wdy * length,        h),   # 5 顶-外-终
        (sx + wdx * length + inx * t, sy + wdy * length + iny * t, h),   # 6 顶-内-终
        (sx + inx * t,             sy + iny * t,             h),   # 7 顶-内-始
    ]

    faces = [
        (0, 1, 2, 3),
        (7, 6, 5, 4),
        (0, 4, 5, 1),
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 4, 0),
    ]

    all_verts = [(x / 1000.0, y / 1000.0, z / 1000.0) for x, y, z in verts_mm]
    return all_verts, faces


def build_column_mesh(center, wx, wy, height_mm):
    """生成矩形柱子网格 (世界坐标)。"""
    cx, cy = center
    hw, hd = wx / 2.0, wy / 2.0
    h = height_mm

    verts_mm = [
        (cx - hw, cy - hd, 0),
        (cx + hw, cy - hd, 0),
        (cx + hw, cy + hd, 0),
        (cx - hw, cy + hd, 0),
        (cx - hw, cy - hd, h),
        (cx + hw, cy - hd, h),
        (cx + hw, cy + hd, h),
        (cx - hw, cy + hd, h),
    ]
    faces = [
        (3, 2, 1, 0),
        (4, 5, 6, 7),
        (1, 5, 4, 0),
        (2, 6, 5, 1),
        (3, 7, 6, 2),
        (0, 4, 7, 3),
    ]
    all_verts = [(x / 1000.0, y / 1000.0, z / 1000.0) for x, y, z in verts_mm]
    return all_verts, faces


# =============================================================================
#  窗框网格生成
# =============================================================================

def _extract_window_bars(lines_local, tol=2.0, max_bar_thick=200):
    """用格栅法从窗框线段识别框料矩形区域。

    以所有线端点坐标为切割线建格；
    格子的最短边 ≤ max_bar_thick，且该短边两侧均有窗线部分覆盖 → 框料。
    """
    h_segs = []  # (y, x_min, x_max)
    v_segs = []  # (x, y_min, y_max)

    xs_set, ys_set = set(), set()
    for seg in lines_local:
        (x1, y1), (x2, y2) = seg[0], seg[1]
        xs_set.update([round(x1), round(x2)])
        ys_set.update([round(y1), round(y2)])
        if abs(y2 - y1) < tol:
            h_segs.append(((y1 + y2) / 2, min(x1, x2), max(x1, x2)))
        elif abs(x2 - x1) < tol:
            v_segs.append(((x1 + x2) / 2, min(y1, y2), max(y1, y2)))

    xs = sorted(xs_set)
    ys = sorted(ys_set)

    def h_partial(y, cx1, cx2):
        """y 处是否有 H 线部分覆盖 [cx1,cx2]"""
        for (sy, xa, xb) in h_segs:
            if abs(sy - y) < tol and xa < cx2 - tol and xb > cx1 + tol:
                return True
        return False

    def v_partial(x, cy1, cy2):
        """x 处是否有 V 线部分覆盖 [cy1,cy2]"""
        for (sx, ya, yb) in v_segs:
            if abs(sx - x) < tol and ya < cy2 - tol and yb > cy1 + tol:
                return True
        return False

    bars = []
    for ix in range(len(xs) - 1):
        for iy in range(len(ys) - 1):
            cx1, cx2 = xs[ix], xs[ix + 1]
            cy1, cy2 = ys[iy], ys[iy + 1]
            dx, dy = cx2 - cx1, cy2 - cy1
            solid = False
            # 薄竖条: 两侧 V 线都部分覆盖
            if dx <= max_bar_thick:
                if v_partial(cx1, cy1, cy2) and v_partial(cx2, cy1, cy2):
                    solid = True
            # 薄横条: 上下 H 线都部分覆盖
            if dy <= max_bar_thick:
                if h_partial(cy1, cx1, cx2) and h_partial(cy2, cx1, cx2):
                    solid = True
            if solid:
                bars.append((cx1, cy1, cx2, cy2))

    if not bars:
        return bars

    # --- 后处理 1: 合并同轴、间距 ≤ max_bar_thick 的相邻条 ---
    # (修复外框竖条被横档 Y 切割导致的间断)
    changed = True
    while changed:
        changed = False
        result = []
        used = [False] * len(bars)
        for i, (ax1, ay1, ax2, ay2) in enumerate(bars):
            if used[i]:
                continue
            for j in range(i + 1, len(bars)):
                if used[j]:
                    continue
                bx1, by1, bx2, by2 = bars[j]
                # 同 X 范围 → 尝试 Y 方向合并
                if abs(ax1 - bx1) < 1 and abs(ax2 - bx2) < 1:
                    gap = max(ay1, by1) - min(ay2, by2)
                    if gap <= max_bar_thick:
                        bars[i] = (ax1, min(ay1, by1), ax2, max(ay2, by2))
                        ax1, ay1, ax2, ay2 = bars[i]
                        used[j] = True
                        changed = True
                # 同 Y 范围 → 尝试 X 方向合并
                elif abs(ay1 - by1) < 1 and abs(ay2 - by2) < 1:
                    gap = max(ax1, bx1) - min(ax2, bx2)
                    if gap <= max_bar_thick:
                        bars[i] = (min(ax1, bx1), ay1, max(ax2, bx2), ay2)
                        ax1, ay1, ax2, ay2 = bars[i]
                        used[j] = True
                        changed = True
        bars = [b for k, b in enumerate(bars) if not used[k]]

    # --- 后处理 2: 接触外边界的条延伸至整个外框范围 (填满四角) ---
    ox1, ox2 = min(xs), max(xs)
    oy1, oy2 = min(ys), max(ys)
    extended = []
    for (bx1, by1, bx2, by2) in bars:
        orig_bx1, orig_by1, orig_bx2, orig_by2 = bx1, by1, bx2, by2
        # 竖条接触左/右外边 → 延伸到全高
        if abs(orig_bx1 - ox1) < 1 or abs(orig_bx2 - ox2) < 1:
            by1, by2 = min(by1, oy1), max(by2, oy2)
        # 横条接触底/顶外边 (用原始 Y 判断，不受上面 Y 延伸影响) → 延伸到全宽
        if abs(orig_by1 - oy1) < 1 or abs(orig_by2 - oy2) < 1:
            bx1, bx2 = min(bx1, ox1), max(bx2, ox2)
        extended.append((bx1, by1, bx2, by2))

    # --- 后处理 3: 竖框裁剪到横框之间，消除四角重叠面 ---
    # 横框占全宽、竖框让位给横框（标准窗框做法）
    inner_oy1 = ys[1]   # 底横框上边 Y
    inner_oy2 = ys[-2]  # 顶横框下边 Y
    final = []
    for (bx1, by1, bx2, by2) in extended:
        dx, dy = bx2 - bx1, by2 - by1
        is_stile = (abs(bx1 - ox1) < 1 or abs(bx2 - ox2) < 1) and dy > dx
        if is_stile:
            by1 = max(by1, inner_oy1)
            by2 = min(by2, inner_oy2)
            if by2 <= by1:
                continue
        final.append((bx1, by1, bx2, by2))


    return final





def build_window_frame_meshes(win_lines_local, exterior_start, exterior_end,
                              inward_normal, frame_offset_mm, frame_depth_mm):
    """将窗框线段转换为 Blender 世界坐标的网格列表。

    win_lines_local: [[[lx1,ly1],[lx2,ly2]], ...]  立面局部坐标 (mm)
    frame_offset_mm: 窗框前脸距外立面偏移
    frame_depth_mm:  窗框进深
    返回 list of (verts, faces)  世界坐标 (m)
    """
    bars = _extract_window_bars(win_lines_local)
    if not bars:
        return []

    sx, sy = exterior_start
    ex, ey = exterior_end
    wx, wy = ex - sx, ey - sy
    dir_len = math.hypot(wx, wy)
    if dir_len < 1e-6:
        return []
    wdx, wdy = wx / dir_len, wy / dir_len

    in_len = math.hypot(inward_normal[0], inward_normal[1])
    inx, iny = inward_normal[0] / in_len, inward_normal[1] / in_len

    z0 = frame_offset_mm                   # 前脸 (距外立面 offset)
    z1 = frame_offset_mm + frame_depth_mm  # 后脸

    results = []
    for (lx1, ly1, lx2, ly2) in bars:
        def to_world(lx, ly, lz):
            gx = sx + lx * wdx + lz * inx
            gy = sy + lx * wdy + lz * iny
            gz = ly
            return (gx / 1000.0, gy / 1000.0, gz / 1000.0)

        verts = [
            to_world(lx1, ly1, z0),  # 0 前-左下
            to_world(lx2, ly1, z0),  # 1 前-右下
            to_world(lx2, ly2, z0),  # 2 前-右上
            to_world(lx1, ly2, z0),  # 3 前-左上
            to_world(lx1, ly1, z1),  # 4 后-左下
            to_world(lx2, ly1, z1),  # 5 后-右下
            to_world(lx2, ly2, z1),  # 6 后-右上
            to_world(lx1, ly2, z1),  # 7 后-左上
        ]
        faces = [
            (0, 3, 2, 1),  # 前脸 (朝外)
            (4, 5, 6, 7),  # 后脸 (朝内)
            (0, 1, 5, 4),  # 底面
            (2, 3, 7, 6),  # 顶面
            (0, 4, 7, 3),  # 左面
            (1, 2, 6, 5),  # 右面
        ]
        results.append((list(verts), list(faces)))

    return results


# =============================================================================
#  外立面法线计算
# =============================================================================


def compute_inward_normal(exterior_start, exterior_end, centroid):
    """返回墙体内法线方向 (nx, ny)，指向建筑重心那一侧。"""
    wx = exterior_end[0] - exterior_start[0]
    wy = exterior_end[1] - exterior_start[1]
    n1 = (-wy, wx)
    n2 = (wy, -wx)
    mx = (exterior_start[0] + exterior_end[0]) / 2.0
    my = (exterior_start[1] + exterior_end[1]) / 2.0
    tc = (centroid[0] - mx, centroid[1] - my)
    d1 = n1[0] * tc[0] + n1[1] * tc[1]
    # 选指向重心的一侧 (内法线)
    return n1 if d1 > 0 else n2


# =============================================================================
#  局部 → 世界 坐标映射
# =============================================================================

def place_elevation_in_3d(outer_contour, openings, exterior_start, exterior_end,
                           thickness, inward_normal):
    """将立面图局部网格变换到 Blender 世界坐标 (m)。

    局部: X=沿墙, Y=高度, Z=厚度 (0=外立面, -t=内立面)
    世界: XY=地面, Z=高度

    exterior_start/end: 平面图外轮廓线端点, 直接取用, 无需计算/延伸。
    返回 (vertices, faces, face_groups)
    """
    sx, sy = exterior_start
    ex, ey = exterior_end
    wx = ex - sx
    wy = ey - sy
    dir_len = math.hypot(wx, wy)
    if dir_len < 1e-6:
        return [], [], None

    elev_w = max(p[0] for p in outer_contour) - min(p[0] for p in outer_contour)
    elev_h = max(p[1] for p in outer_contour) - min(p[1] for p in outer_contour)
    if elev_w < 1e-6 or elev_h < 1e-6:
        return [], [], None

    openings_rect = []
    for hole in openings:
        xs = [p[0] for p in hole]
        ys = [p[1] for p in hole]
        openings_rect.append((min(xs), min(ys), max(xs), max(ys)))

    local_verts, local_faces, face_groups = build_wall_mesh_with_openings(
        elev_w, elev_h, openings_rect, thickness
    )

    # 沿墙方向
    wdx = wx / dir_len
    wdy = wy / dir_len
    # 向内法线
    in_len = math.hypot(inward_normal[0], inward_normal[1])
    inx = inward_normal[0] / in_len
    iny = inward_normal[1] / in_len

    all_verts = []
    for lx, ly, lz in local_verts:
        # local X → 沿墙方向; local Y → 世界 Z
        # local Z (0=外立面) → 外立面线位置; lz<0 → 向内延伸
        gx = sx + lx * wdx + lz * inx
        gy = sy + lx * wdy + lz * iny
        gz = ly
        all_verts.append((gx / 1000.0, gy / 1000.0, gz / 1000.0))

    return all_verts, local_faces, face_groups


# =============================================================================
#  法线修正 — 确保所有面法线朝外 (远离墙材质)
# =============================================================================

def fix_normals(mesh, verts, faces, face_groups, inward_normal):
    """检查并修正网格中所有面的法线方向。

    face_groups: {'front':[...], 'back':[...], 'side':[...], 'side_meta':[...]}
    inward_normal: (nx, ny) 从外立面向内的方向

    侧面法线使用 side_meta 中的 edge type 做精确判断。
    """
    n_flipped = 0

    in_len = math.hypot(inward_normal[0], inward_normal[1])
    inx, iny = inward_normal[0] / in_len, inward_normal[1] / in_len

    def compute_face_normal(f_idx):
        f = faces[f_idx]
        v0 = Vector(verts[f[0]])
        v1 = Vector(verts[f[1]])
        v2 = Vector(verts[f[2]])
        n = (v1 - v0).cross(v2 - v0)
        return n.normalized() if n.length > 1e-12 else Vector((0, 0, 0))

    # 墙体方向: 由 XY 平面内最远顶点对确定 (墙长远大于壁厚)
    max_d2 = 0.0
    pi, pj = 0, 0
    for i in range(len(verts)):
        for j in range(i + 1, len(verts)):
            d2 = (verts[j][0] - verts[i][0]) ** 2 + (verts[j][1] - verts[i][1]) ** 2
            if d2 > max_d2:
                max_d2 = d2
                pi, pj = i, j
    wdx = verts[pj][0] - verts[pi][0]
    wdy = verts[pj][1] - verts[pi][1]
    wlen = math.hypot(wdx, wdy)
    if wlen > 1e-6:
        wdx /= wlen
        wdy /= wlen

    # 构建 side_face global_index → edge_type 的映射
    side_list = face_groups['side']
    edge_type_map = {}
    for local_idx, etype, _ekey in face_groups.get('side_meta', []):
        edge_type_map[side_list[local_idx]] = etype

    front_set = set(face_groups['front'])
    back_set = set(face_groups['back'])

    # --- 前后面 ---
    # 前面法线应向外 (-inward), 背面法线应向心 (inward)
    for f_idx in list(front_set):
        n = compute_face_normal(f_idx)
        if n.x * inx + n.y * iny > 0:
            faces[f_idx] = tuple(reversed(faces[f_idx]))
            n_flipped += 1

    for f_idx in list(back_set):
        n = compute_face_normal(f_idx)
        if n.x * inx + n.y * iny < 0:
            faces[f_idx] = tuple(reversed(faces[f_idx]))
            n_flipped += 1

    # --- 侧面: 使用 edge type 做精确判断 ---
    # 统一绕组 (va, vb, vb+BO, va+BO) 在局部坐标下:
    #   E_BOTTOM → +Y (世界 +Z), 应该 -Z → flip
    #   E_TOP    → -Y (世界 -Z), 应该 +Z → flip
    #   E_RIGHT  → -X (local),   应该沿墙方向 → flip
    #   E_LEFT   → +X (local),   应该逆墙方向 → flip
    for side_idx in side_list:
        n = compute_face_normal(side_idx)
        etype = edge_type_map.get(side_idx)

        if etype == E_BOTTOM:
            if n.z > 0:
                faces[side_idx] = tuple(reversed(faces[side_idx]))
                n_flipped += 1
        elif etype == E_TOP:
            if n.z < 0:
                faces[side_idx] = tuple(reversed(faces[side_idx]))
                n_flipped += 1
        elif etype == E_RIGHT:
            if n.x * wdx + n.y * wdy < 0:
                faces[side_idx] = tuple(reversed(faces[side_idx]))
                n_flipped += 1
        elif etype == E_LEFT:
            if n.x * (-wdx) + n.y * (-wdy) < 0:
                faces[side_idx] = tuple(reversed(faces[side_idx]))
                n_flipped += 1

    return n_flipped


# =============================================================================
#  Blender 对象创建
# =============================================================================

def get_or_create_material(name, color_rgba=(0.8, 0.8, 0.8, 1.0)):
    """返回已有材质或新建一个带基础颜色的材质。"""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    if bsdf:
        bsdf.inputs['Base Color'].default_value = color_rgba
    return mat


def create_mesh_object(verts, faces, name, parent=None, material=None):
    """从顶点+面创建 Blender Mesh 对象，并可选地赋予材质。"""
    if not verts:
        return None
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    if parent:
        obj.parent = parent
    if material:
        if len(obj.data.materials) == 0:
            obj.data.materials.append(material)
        else:
            obj.data.materials[0] = material
    return obj



# =============================================================================
#  主构建流程
# =============================================================================

def build(json_path):
    """读取 JSON 并在 Blender 中构建完整建筑模型。"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    meta = data['meta']
    walls = data['walls']
    columns = data.get('columns', [])
    default_h = meta['default_height_mm']

    # 窗框参数 (可在 meta.window_defaults 中覆盖)
    win_defaults = meta.get('window_defaults', {})
    FRAME_OFFSET = win_defaults.get('frame_offset_mm', 100)
    FRAME_DEPTH  = win_defaults.get('frame_depth_mm',  60)

    print(f"[build] {len(walls)} walls, {len(columns)} columns, "
          f"default height: {default_h} mm")

    # --- 建筑重心 (用外轮廓线端点计算) ---
    all_cx, all_cy, n_pts = 0.0, 0.0, 0
    for w in walls:
        all_cx += w['exterior_start'][0] + w['exterior_end'][0]
        all_cy += w['exterior_start'][1] + w['exterior_end'][1]
        n_pts += 2
    centroid = ((all_cx / n_pts, all_cy / n_pts)
                if n_pts > 0 else (0.0, 0.0))

    # --- 预计算内法线 (从外立面向内) ---
    wall_inward = {}
    for w in walls:
        es = (w['exterior_start'][0], w['exterior_start'][1])
        ee = (w['exterior_end'][0], w['exterior_end'][1])
        wall_inward[w.get('label', '')] = compute_inward_normal(es, ee, centroid)

    # --- 创建父级 ---
    parent = bpy.data.objects.new("Building", None)
    bpy.context.collection.objects.link(parent)

    # --- 预建材质 ---
    mat_wall   = get_or_create_material('Mat_Wall',   (0.75, 0.75, 0.72, 1.0))  # 浅灰
    mat_window = get_or_create_material('Mat_Window', (0.55, 0.65, 0.75, 1.0))  # 浅蓝灰
    print(f"[build] 材质: {mat_wall.name}, {mat_window.name}")


    # --- 墙体 ---
    for i, w in enumerate(walls):
        label = w.get('label', str(i))
        name = f"Wall_{label}"
        thick = w['thickness_mm']
        es = (w['exterior_start'][0], w['exterior_start'][1])
        ee = (w['exterior_end'][0], w['exterior_end'][1])
        inward = wall_inward.get(label, (0.0, 1.0))

        if 'elevation' in w and w['elevation']:
            elev = w['elevation']
            verts, faces, fg = place_elevation_in_3d(
                elev['outer_contour'], elev['openings'],
                es, ee, thick, inward
            )
            n_flip = 0
            if fg:
                n_flip = fix_normals(None, verts, faces, fg, inward)
            print(f"  {name}: elev {elev['width_mm']}x{elev['height_mm']}mm, "
                  f"{len(elev['openings'])} openings → {len(verts)}v {len(faces)}f "
                  f"({n_flip} flipped)")
        else:
            verts, faces = build_simple_wall_mesh(
                es, ee, default_h, thick, inward
            )
            n_flip = fix_simple_normals(verts, faces)
            print(f"  {name}: plain, h={default_h}mm → {len(verts)}v {len(faces)}f "
                  f"({n_flip} flipped)")

        create_mesh_object(verts, faces, name, parent, material=mat_wall)


        # --- 窗框 ---
        if 'elevation' in w and w['elevation']:
            elev = w['elevation']
            for win_entry in elev.get('windows', []):
                oi = win_entry['opening_index']
                win_lines = win_entry['lines']
                frame_meshes = build_window_frame_meshes(
                    win_lines, es, ee, inward,
                    FRAME_OFFSET, FRAME_DEPTH
                )
                for bi, (wv, wf) in enumerate(frame_meshes):
                    fix_simple_normals(wv, wf)
                    wname = f"Win_{label}_o{oi}_b{bi}"
                    create_mesh_object(wv, wf, wname, parent, material=mat_window)

            if elev.get('windows'):
                n_win = sum(len(build_window_frame_meshes(
                    e['lines'], es, ee, inward, FRAME_OFFSET, FRAME_DEPTH
                )) for e in elev['windows'])
                print(f"    窗框: {len(elev['windows'])} 扇, {n_win} 个框料条")

    # --- 柱子 ---
    for i, c in enumerate(columns):
        name = f"Column_{i + 1}"
        cx, cy = c['center']
        wx, wy = c['width_x'], c['width_y']
        verts, faces = build_column_mesh((cx, cy), wx, wy, default_h)
        n_flip = fix_simple_normals(verts, faces)
        print(f"  {name}: {wx}x{wy}x{default_h}mm → {len(verts)}v {len(faces)}f "
              f"({n_flip} flipped)")
        create_mesh_object(verts, faces, name, parent, material=mat_wall)


    print("[build] done")

    bpy.ops.object.select_all(action='DESELECT')
    parent.select_set(True)
    bpy.context.view_layer.objects.active = parent
    return parent


def fix_simple_normals(verts, faces):
    """对简单 box 网格做质心法线修正。"""
    if not verts:
        return 0
    from mathutils import Vector
    cx = sum(v[0] for v in verts) / len(verts)
    cy = sum(v[1] for v in verts) / len(verts)
    cz = sum(v[2] for v in verts) / len(verts)
    center = Vector((cx, cy, cz))

    flipped = 0
    for i, f in enumerate(faces):
        v0 = Vector(verts[f[0]])
        v1 = Vector(verts[f[1]])
        v2 = Vector(verts[f[2]])
        n = (v1 - v0).cross(v2 - v0)
        if n.length < 1e-12:
            continue
        n.normalize()
        fc = Vector((
            sum(verts[vi][0] for vi in f) / len(f),
            sum(verts[vi][1] for vi in f) / len(f),
            sum(verts[vi][2] for vi in f) / len(f),
        ))
        if n.dot(fc - center) < 0:
            # normal points toward center → flip
            faces[i] = tuple(reversed(f))
            flipped += 1
    return flipped


# =============================================================================
#  FBX 导出
# =============================================================================

def export_fbx(filepath, building_parent="Building"):
    """导出建筑模型为 FBX，参数针对 3ds Max 优化。"""
    parent = bpy.data.objects.get(building_parent)
    if not parent:
        print(f"[export] '{building_parent}' not found, run build() first")
        return

    bpy.ops.object.select_all(action='DESELECT')
    parent.select_set(True)
    bpy.context.view_layer.objects.active = parent
    for child in parent.children_recursive:
        child.select_set(True)

    bpy.ops.export_scene.fbx(
        filepath=filepath,
        use_selection=True,
        object_types={'MESH', 'EMPTY'},
        axis_forward='-Y',
        axis_up='Z',
        apply_scale_options='FBX_SCALE_UNITS',
        apply_unit_scale=True,
        use_mesh_modifiers=True,
        mesh_smooth_type='FACE',
        use_tspace=False,
        use_custom_props=False,
        use_mesh_edges=False,
        use_triangles=False,
        bake_anim=False,
        add_leaf_bones=False,
        path_mode='AUTO',
        embed_textures=False,
        batch_mode='OFF',
    )

    n = len(parent.children_recursive) + 1
    print(f"[export] {filepath}  ({n} objects)")


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        build(sys.argv[1])
    else:
        print("Usage: build_from_json.build('path/to/data.json')")
