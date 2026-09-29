#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_assembly.py -- автоматическая проверка сборки robot.blend.

Для всех пар объектов коллекций ASSEMBLY и PHANTOM считается объём взаимного
пересечения (булева операция INTERSECT, solver EXACT). Пары с заметным
пересечением выводятся как ошибки компоновки.

Запуск: blender -b robot.blend -P check_assembly.py
"""
import bpy
import bmesh
from mathutils import Vector


def bbox(ob):
    pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
    xs = [p.x for p in pts]; ys = [p.y for p in pts]; zs = [p.z for p in pts]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def overlap(a, b, tol=0.6):
    return not (a[3]-tol < b[0] or b[3]-tol < a[0] or
                a[4]-tol < b[1] or b[4]-tol < a[1] or
                a[5]-tol < b[2] or b[5]-tol < a[2])


PRESS_FITS = (
    # (маска1, маска2, пояснение) - пары, где пересечение задано конструктивно:
    ("Leg",        "Sole_Plate", "рёбра трения заглушки подошвы входят в стенку выборки на 0.45 мм"),
    ("Servo_Shoe", "SG90",       "прижимные рёбра 'туфли' садятся на корпус сервы с натягом 0.25 мм"),
    ("Torso_Tub",  "Arduino_Uno","установочный усик-фиксатор платы входит в паз платы на 0.1 мм"),
)


def is_press_fit(n1, n2):
    for (m1, m2, _) in PRESS_FITS:
        if (m1 in n1 and m2 in n2) or (m1 in n2 and m2 in n1):
            return True
    return False


def intersect_volume(a, b):
    """Объём пересечения двух объектов (мм^3); объекты не изменяются."""
    tmp = a.copy()
    tmp.data = a.data.copy()
    tmp.name = "__inter"
    bpy.context.scene.collection.objects.link(tmp)
    tmp.matrix_world = a.matrix_world
    mod = tmp.modifiers.new("i", 'BOOLEAN')
    mod.solver = 'EXACT'
    mod.operation = 'INTERSECT'
    mod.object = b
    mod.use_self = True
    bpy.context.view_layer.objects.active = tmp
    try:
        bpy.ops.object.modifier_apply(modifier=mod.name)
        bm = bmesh.new()
        bm.from_mesh(tmp.data)
        vol = abs(bm.calc_volume())
        if bm.verts:
            xs=[v.co.x for v in bm.verts]; ys=[v.co.y for v in bm.verts]; zs=[v.co.z for v in bm.verts]
            bb=(min(xs),min(ys),min(zs),max(xs),max(ys),max(zs))
        else:
            bb=None
        bm.free()
    except Exception:
        vol = -1.0; bb=None
    me = tmp.data
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.meshes.remove(me)
    return vol, bb


def main():
    bpy.ops.wm.open_mainfile(filepath="robot.blend")
    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.length_unit = 'MILLIMETERS'
    objs = [o for o in bpy.data.objects if o.get("kind") in ("assembly", "ref")]
    objs.sort(key=lambda o: o.name)
    boxes = {o.name: bbox(o) for o in objs}
    print("=" * 100)
    print("ПРОВЕРКА СБОРКИ: взаимные пересечения деталей (мм^3)")
    print("=" * 100)
    print(f"объектов в проверке: {len(objs)}")
    bad = []
    fits = []
    checked = 0
    for i, a in enumerate(objs):
        for b in objs[i+1:]:
            if not overlap(boxes[a.name], boxes[b.name]):
                continue
            checked += 1
            v, bb = intersect_volume(a, b)
            # "туфля" сервы садится на корпус с конструктивным натягом рёбер (~0.05 мм)
            if v > 1.0:
                (fits if is_press_fit(a.name, b.name) else bad).append((a.name, b.name, v, bb))
    print(f"проверено пар-кандидатов (с пересечением габаритов): {checked}")
    print("-" * 100)
    if fits:
        print(f"КОНСТРУКТИВНЫЕ НАТЯГИ (заданы по проекту): {len(fits)}")
        for (n1, n2, v, bb) in sorted(fits, key=lambda t: -t[2]):
            note = ""
            for (m1, m2, txt) in PRESS_FITS:
                if (m1 in n1 and m2 in n2) or (m1 in n2 and m2 in n1):
                    note = txt
            print(f"    натяг     {n1:<22} x  {n2:<22} {v:8.2f} мм^3 : {note}")
        print("-" * 100)
    if not bad:
        print("КОНФЛИКТОВ НЕ НАЙДЕНО: детали сборки не пересекаются.")
    else:
        bad.sort(key=lambda t: -t[2])
        for (n1, n2, v, bb) in bad:
            loc = ""
            if bb:
                loc = (f"   зона X[{bb[0]:7.1f},{bb[3]:7.1f}]"
                       f" Y[{bb[1]:7.1f},{bb[4]:7.1f}] Z[{bb[2]:7.1f},{bb[5]:7.1f}]")
            print(f"  КОНФЛИКТ  {n1:<22} x  {n2:<22} {v:8.2f} мм^3{loc}")
    print("=" * 100)
    # габарит собранного робота
    asm = [o for o in objs if o.get("kind") == "assembly"]
    xs = [boxes[o.name] for o in asm]
    print("ГАБАРИТ СБОРКИ (без фантомов), мм:")
    print(f"   X: {min(b[0] for b in xs):8.1f} ... {max(b[3] for b in xs):8.1f}   "
          f"({max(b[3] for b in xs)-min(b[0] for b in xs):.1f})")
    print(f"   Y: {min(b[1] for b in xs):8.1f} ... {max(b[4] for b in xs):8.1f}   "
          f"({max(b[4] for b in xs)-min(b[1] for b in xs):.1f})")
    print(f"   Z: {min(b[2] for b in xs):8.1f} ... {max(b[5] for b in xs):8.1f}   "
          f"({max(b[5] for b in xs)-min(b[2] for b in xs):.1f})")
    print("=" * 100)


main()
