#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_check.py -- рендеры сборки, узлов и раскладки (Cycles, CPU).
Запуск: python render_check.py [samples]
"""
import bpy, math, sys

SAMPLES = 24
for a in sys.argv:
    if a.isdigit():
        SAMPLES = int(a)

bpy.ops.wm.open_mainfile(filepath="/home/user/MODEL/robot.blend")
sc = bpy.context.scene
sc.render.engine = 'CYCLES'
sc.cycles.samples = SAMPLES
sc.cycles.use_denoising = False
sc.cycles.max_bounces = 4
OUT = "/home/user/MODEL/"

col_bed = bpy.data.collections["PRINT_BED"]


def shoot(name, loc, rot, lens=62.0, rx=900, ry=800, bed=False):
    col_bed.hide_render = not bed
    cam = bpy.data.objects["CAM_Assembly"]
    cam.data.lens = lens
    cam.location = loc
    cam.rotation_euler = rot
    sc.camera = cam
    sc.render.resolution_x, sc.render.resolution_y = rx, ry
    sc.render.filepath = OUT + name
    bpy.ops.render.render(write_still=True)
    print("rendered", name)


D = math.radians
shoot("r_front.png", (0, -1150, 125), (D(88), 0, 0))
shoot("r_three.png", (700, -900, 250), (D(78), 0, D(38)))
shoot("r_side.png", (1150, 0, 125), (D(88), 0, D(90)))
shoot("r_head.png", (330, -700, 250), (D(78), 0, D(25)), 85, 900, 700)
shoot("r_bed.png", (0, -430, 900), (D(14), 0, 0), 45, 1000, 900, bed=True)
