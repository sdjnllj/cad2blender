# CAD 到 Blender 自动化建模工作流

## 概述

用户使用 AutoCAD 绘制建筑平面图和立面图（DXF 格式），通过 Python 脚本自动在 Blender 中生成 3D 建筑外立面模型。

## 工作流

```
CAD 绘图 (.dxf)  →  parse_dxf.py  →  data.json  →  build_from_json.py  →  Blender 3D  →  export_fbx.py  →  .fbx
```

### 1. 用户操作
- 在 AutoCAD 2021 中按规范绘图
- 导出 DXF 文件（AutoCAD 2010+ 格式）
- 运行解析脚本生成 JSON

### 2. 解析
```bash
python parse_dxf.py <图纸.dxf> [输出.json]
```

### 3. 建模（通过 MCP Blender 工具）
在 Blender 中执行：
```python
import build_from_json
build_from_json.build("路径/data.json")
```

### 4. FBX 导出
在 Blender 中执行：
```python
import export_fbx
export_fbx.export("输出路径.fbx")                      # 默认导出 "Building" 父对象
export_fbx.export("输出.fbx", building_name="Building") # 或指定父对象名
```
导出参数针对 3ds Max 优化（Z-up、单位缩放、四边形网格）。

## CAD 图层规范

| 图层名 | 用途 | 图元类型 | 说明 |
|--------|------|---------|------|
| `Plan_Walls` | 墙体平面定位 | LINE（双线） | 每对平行线 = 一面墙，壁厚 = 双线间距 |
| `Plan_Walls` | 墙编号标签 | MTEXT | 在墙体旁标注编号（W1, W2, ...），自动关联到最近墙体 |
| `Plan_Columns` | 柱子平面 | LWPOLYLINE（闭合） | 矩形/异形柱轮廓 |
| `Elev_<编号>` | 立面图 | LINE 外轮廓 + LWPOLYLINE 洞口 | 如 `Elev_W6` 对应编号 W6 的墙体 |

### 立面图关键规则
- **最大闭合环** = 墙体外轮廓
- **内部闭合 LWPOLYLINE** = 门窗洞口
- 立面图可覆盖多层（一楼到三楼一次画完）
- 立面的 X 轴 = 沿墙方向，Y 轴 = 高度方向
- 立面图在 CAD 中可放在任意位置（不要求和平面图重叠）

## 建模逻辑

### 墙体定位（核心原则）
- **平面图外轮廓 LINE 端点 = 墙体精确位置**，不做中心线计算或转角延伸
- 每对平行线中，离建筑重心更远的那条即为外立面线
- 外立面线端点直接用于 3D 定位，长度即墙体实际长度
- 壁厚 = 双线间距（从 LINE 坐标直接读取）

### 有立面图的墙体
- 使用 grid-cut 算法生成纯四边形网格（无布尔运算）
- 立面外轮廓 → 墙体外面，洞口 → 窗洞切穿
- 外立面定位在外轮廓线上，墙体向内（重心方向）延伸

### 无立面图的墙体
- 自动继承所有立面的最大高度作为默认高度
- 生成为简单矩形盒体（无洞口），同样用外轮廓线定位

### 柱子
- 按平面轮廓挤出，高度继承默认高度

### 坐标映射
- 单位：毫米（mm）→ Blender 中自动转换为米（÷1000）
- 墙体外面方向：自动判断（远离建筑重心的一侧）

## 设计决策

- **纯四边形网格**：不使用布尔运算，避免伪影
- **各墙独立建模**：转角处自然重叠，不处理斜街（用于外立面效果图）
- **不需要楼层概念**：用户直接在立面图中画好总高度和各层洞口
- **两步流水线**：解析和建模分离，方便单独调试 JSON 中间数据

## 依赖

- `ezdxf` — Python DXF 解析库
- Blender Python API (bpy, bmesh)

## 当前状态

测试文件：`test03平面加立面加柱子+多层立面1.dxf`
- 6 面墙（W1-W6），2 面带立面（W1 1洞口, W6 8洞口）
- 5 根柱子
- 默认高度 12m
