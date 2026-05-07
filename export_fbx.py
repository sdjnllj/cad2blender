"""
export_fbx.py — 将 Blender 中的建筑模型导出为 FBX，参数针对 3ds Max 优化。

用法 (在 Blender Python 控制台或通过 MCP):
    import export_fbx
    export_fbx.export("output.fbx")

    # 或指定父对象名
    export_fbx.export("output.fbx", building_name="Building")
"""

import bpy
import os


def export(filepath, building_name="Building"):
    """导出建筑模型为 FBX。

    Args:
        filepath: 输出 .fbx 文件路径
        building_name: 场景中建筑父对象名称
    """
    parent = bpy.data.objects.get(building_name)
    if not parent:
        print(f"[export] 未找到 '{building_name}'，请先运行 build_from_json.build()")
        return

    # 确保输出目录存在
    out_dir = os.path.dirname(filepath)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # 选中建筑及其所有子对象
    bpy.ops.object.select_all(action='DESELECT')
    parent.select_set(True)
    bpy.context.view_layer.objects.active = parent
    for child in parent.children_recursive:
        child.select_set(True)

    child_count = len(parent.children_recursive)

    bpy.ops.export_scene.fbx(
        filepath=filepath,
        use_selection=True,
        object_types={'MESH', 'EMPTY'},
        # 坐标轴: Blender Z-up → 3ds Max Z-up
        axis_forward='-Y',
        axis_up='Z',
        # 单位: 米 → 3ds Max 系统单位
        apply_scale_options='FBX_SCALE_UNITS',
        apply_unit_scale=True,
        # 网格
        use_mesh_modifiers=True,
        mesh_smooth_type='FACE',
        use_tspace=False,
        # 不导出无关数据
        use_custom_props=False,
        use_mesh_edges=False,
        use_triangles=False,
        bake_anim=False,
        add_leaf_bones=False,
        # 路径
        path_mode='AUTO',
        embed_textures=False,
        batch_mode='OFF',
    )

    file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
    print(f"[export] {filepath}")
    print(f"[export] {child_count + 1} 个对象, {file_size_mb:.1f} MB")


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        export(sys.argv[1])
    else:
        # 默认路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_path = os.path.join(script_dir, "building_3dsmax.fbx")
        export(default_path)
