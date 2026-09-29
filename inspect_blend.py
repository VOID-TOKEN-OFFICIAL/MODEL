#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspect_blend.py -- headless-проверка файла robot.blend:
выводит единицы сцены, состав коллекций и габариты объектов в миллиметрах.
Запуск: blender -b robot.blend -P inspect_blend.py     (или через pip-пакет bpy)
"""
import bpy
import bmesh
from mathutils import Vector

bpy.ops.wm.open_mainfile(filepath="robot.blend")
sc = bpy.context.scene
us = sc.unit_settings
print("=" * 96)
print("ФАЙЛ: robot.blend")
print(f"ЕДИНИЦЫ: system={us.system}  length={us.length_unit}  scale_length={us.scale_length}  "
      f"(1 единица = {us.scale_length*1000:.0f} мм)")
print(f"ДВИЖОК РЕНДЕРА: {sc.render.engine}; камера сцены: {sc.camera.name if sc.camera else '-'}")
print("=" * 96)


def bbox(ob):
    pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
    xs = [p.x for p in pts]; ys = [p.y for p in pts]; zs = [p.z for p in pts]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


for cname in ("PRINT_BED", "ASSEMBLY", "PHANTOM"):
    col = bpy.data.collections.get(cname)
    if col is None:
        continue
    objs = sorted(col.objects, key=lambda o: o.name)
    print()
    print(f"--- КОЛЛЕКЦИЯ {cname}: {len(objs)} объектов " +
          ("(печатные детали на столе печати)" if cname == "PRINT_BED" else
           "(те же детали в сборе)" if cname == "ASSEMBLY" else "(габаритные фантомы)"))
    print(f"{'объект':<22}{'X, мм':>9}{'Y, мм':>9}{'Z, мм':>9}    {'положение (мм)':<26}{'объём, см3':>11}")
    print("-" * 96)
    for ob in objs:
        bb = bbox(ob)
        vol = 0.0
        if ob.type == 'MESH':
            bm = bmesh.new(); bm.from_mesh(ob.data)
            vol = abs(bm.calc_volume()) / 1000.0
            bm.free()
        pos = f"({(bb[0]+bb[3])/2:7.1f},{(bb[1]+bb[4])/2:7.1f},{(bb[2]+bb[5])/2:7.1f})"
        print(f"{ob.name:<22}{bb[3]-bb[0]:>9.1f}{bb[4]-bb[1]:>9.1f}{bb[5]-bb[2]:>9.1f}"
              f"    {pos:<26}{vol:>11.2f}")
print("=" * 96)

asm = sorted(bpy.data.collections["ASSEMBLY"].objects, key=lambda o: o.name)
xs = [bbox(o) for o in asm]
print("ГАБАРИТ СОБРАННОГО РОБОТА (коллекция ASSEMBLY), мм:")
print(f"   ширина X = {max(b[3] for b in xs)-min(b[0] for b in xs):.1f}"
      f"   глубина Y = {max(b[4] for b in xs)-min(b[1] for b in xs):.1f}"
      f"   высота Z = {max(b[5] for b in xs)-min(b[2] for b in xs):.1f}")
bed = sorted(bpy.data.collections["PRINT_BED"].objects, key=lambda o: o.name)
xs = [bbox(o) for o in bed]
print("РАСКЛАДКА ПЕЧАТНЫХ ДЕТАЛЕЙ НА СТОЛЕ, мм:")
print(f"   ширина X = {max(b[3] for b in xs)-min(b[0] for b in xs):.1f}"
      f"   глубина Y = {max(b[4] for b in xs)-min(b[1] for b in xs):.1f}"
      f"   все детали стоят на Z = 0")
print("=" * 96)
