import bpy
from mathutils import Vector

def inspect_blend():
    bpy.ops.wm.open_mainfile(filepath="robot.blend")
    scene = bpy.context.scene

    print("=" * 88)
    print(f"ПРОВЕРКА ФАЙЛА robot.blend В HEADLESS BLENDER ({bpy.app.version_string})")
    print("=" * 88)
    print(f"Система единиц: {scene.unit_settings.system}")
    print(f"Единица длины : {scene.unit_settings.length_unit}")
    print(f"Масштаб длины : {scene.unit_settings.scale_length} (1 ед. = 1 мм)")
    print(f"Количество объектов: {len(scene.objects)}")
    print("-" * 88)
    print(f"| {'Имя объекта':<26} | {'Габариты X x Y x Z (мм)':<28} | {'Позиция Z_min':<12} | {'Вершин':<8} |")
    print("-" * 88)

    for obj in sorted(scene.objects, key=lambda x: x.name):
        if obj.type == 'MESH':
            bbox = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
            min_z = min(p.z for p in bbox)
            dims = obj.dimensions
            dim_str = f"{dims.x:6.2f} x {dims.y:6.2f} x {dims.z:6.2f}"
            print(f"| {obj.name:<26} | {dim_str:<28} | {min_z:8.2f} мм   | {len(obj.data.vertices):<8} |")

    print("=" * 88)

if __name__ == "__main__":
    inspect_blend()
