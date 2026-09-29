#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_robot.py  --  генератор настольного гуманоидного робота "MODEL-1" (FDM-детали).

Запуск:  blender -b -P gen_robot.py
    или: python -c "exec(open('gen_robot.py').read())"  (в окружении с pip-пакетом bpy)

Результат: robot.blend
  * единицы: METRIC / MILLIMETERS / scale_length = 0.001   (1 единица = 1 мм);
  * PRINT_BED - печатные детали, разложенные на плоскости XY как на столе печати,
                каждая в ориентации печати, низ детали на Z=0;
  * ASSEMBLY  - те же детали (те же меш-данные) в положении сборки;
  * PHANTOM   - габаритные "фантомы" 5 x SG90, 5 качалок и Arduino Uno в сборе.

==========================================================================================
КОНСТРУКЦИЯ (форма, стилистика и компоновка выбраны автором)
==========================================================================================
Робот 272.5 мм высотой: приземистый игрушечный андроид - широкий короб-торс, шлем с
визором, руки на круглых ступицах, полые ноги-утяжелители. 12 уникальных печатных
деталей, 17 напечатанных экземпляров. Сборка без клея и без винтов (кроме штатного
винтика качалки из комплекта сервы): пазы, шипы, штифты, язычковые защёлки,
направляющие, пробки-штифты из обрезка филамента.

  1  TORSO_TUB   короб-ванна (пол + 2 боковые и передняя стенки; верх и зад открыты).
                 Внутри: направляющие Arduino Uno (2 полки + центральное ребро + упор-губа),
                 2 кармана плечевых серв (выборка под ушки + окно корпуса + 2 штифта
                 в отверстиях ушек + прорезь под кабель), окно USB/питания, 2 окна
                 кабельных трасс в руки, 4 глухих отверстия под пробки задней крышки,
                 отверстие Ø12 под провода в дне со стойками-стяжками.
  2  NECK_PLATE  плита-шея: закрывает верх, несёт серву поворота головы (вал вверх,
                 ушки опёрты в выборке, штифты в отверстиях ушек), отверстие Ø10 в голову.
  3  BACK_COVER  задняя крышка: контурный борт-пробка, 4 отверстия под пробки-штифты,
                 прорезь-захват для пальца, вентиляционные прорези. Снимается рукой.
  4  HEAD_SHELL  шлем: полость под электронику с двускатным потолком (печать без
                 поддержек), визор с глазницами, боковые подушки, антенна, снизу -
                 гнездо под качалку сервы шеи и отверстие под провода.
  5/6   UPPER_ARM_R/L  плечевое звено: ступица с карманом качалки, пустотелое звено,
                 на дальнем конце - клетка локтевой сервы (то же посадочное решение).
  7/8   FOREARM_R/L    предплечье: ступица с карманом качалки, звено, клешня-трёхзуб.
  9/10  LEG_R/L        полая нога: гнездо шипа торса с 2 язычковыми защёлками, полость
                 Ø17 под утяжелитель, выборка под заглушку подошвы.
  11/12 SOLE_PLATE     съёмная заглушка подошвы (посадка с натягом, прорезь под ноготь).
  13-17 SERVO_SHOE     печатная "туфля": чашка, надеваемая на корпус сервы изнутри,
                 прижимает ушки к выборке стенки. 5 шт (2 плеча, 2 локтя, 1 шея).

КИНЕМАТИКА - 5 сервоприводов SG90 (по ТЗ):
  плечо  - вал наружу через боковую стенку торса; локоть - вал из первого звена руки
  во второе; шея - вал вертикально вверх из тела.

ТРАССЫ ПРОВОДОВ (пучок Ø8-10 мм, одиночный провод Ø1.5 мм):
  низ -> тело:   через отверстие Ø12 в дне торса (стойки-стяжки под хомут), вверх мимо
                 центрального ребра к разъёмам платы;
  тело -> голова: по свободной передней зоне торса (Y < -6) через отверстие Ø10 в плите
                 шеи и Ø9 в основании головы;
  тело -> руки:  через окна 12x8 мм в боковых стенках торса (в зоне Z 62...70, скрыты за
                 звеньями рук), далее по полости звена вверх к серве локтя; кабель
                 плечевой сервы выходит из её корпуса вверх прямо в полость торса.

ПРИНЯТЫЕ АВТОРСКИЕ ДОПУЩЕНИЯ (в ТЗ не заданы; помечены в коде [ASSUME]):
  A1. Вылет шпильки SG90 - 6.2 мм; качалка: бобышка Ø9 x 3.5 мм, перо 2.6 мм толщиной,
      8.0 мм шириной, вылет пера ±16.3 мм (крестовая или одинарная качалка).
  A2. Кабель сервы выходит из боковой грани корпуса (23 x 22.7) у плоскости вала;
      внутри шарнира кабель не проходит - прокладывается по полостям деталей.
  A3. Электроника головы: плата 45 x 18 мм (класс Nano) + 2 светодиода на полках.
  A4. Утяжелители: стальной пруток Ø16 x 52 мм (или набор гаек М16) в каждую ногу.
  A5. Ориентация деталей на столе подобрана под печать без поддержек; мелкие мосты
      (потолки карманов) FDM печатает штатно.
  A6. Пробки задней крышки - отрезки филамента Ø1.75 x 10 мм, 4 шт (посадка с натягом).
==========================================================================================
"""

import bpy
import bmesh
import math
from mathutils import Matrix, Vector

# =========================================================================================
# 1. ПАРАМЕТРЫ (мм)
# =========================================================================================

SCALE_LENGTH = 0.001            # 1 единица Blender = 1 мм

# --- общие допуски FDM
WALL         = 2.80             # толщина стенок корпусных деталей
FIT          = 0.35             # зазор на сторону: съёмные / подвижные посадки
PRESS        = 0.15             # натяг на сторону: лёгкая запрессовка

# --- SG90 (по ТЗ)
SG_L         = 23.0             # корпус: длина вдоль оси ушек
SG_W         = 12.2             # корпус: толщина
SG_H         = 22.7             # корпус: высота от плоскости вала до задней стенки
SG_FIT       = 0.30             # зазор сервы в посадочном окне (на сторону)
SG_SHAFT_D   = 5.0              # вал (по ТЗ)
SG_SHAFT_L   = 6.2              # [ASSUME A1] вылет шпильки от плоскости вала
SG_EAR_L     = 32.0             # ушки суммарно (по ТЗ)
SG_EAR_T     = 2.0              # толщина ушка (по ТЗ)
SG_EAR_SET   = 1.2              # [ASSUME A1] верх ушка ниже плоскости вала
SG_EAR_HOLE  = 2.2              # Ø отверстия в ушке (по ТЗ)
SG_EAR_PITCH = 25.0             # межцентровое отверстий (по ТЗ)

# --- качалка и ступица (единая норма для всех шарниров)
HORN_T       = 2.60             # [ASSUME A1] толщина пера качалки
HORN_BOSS_D  = 9.00             # [ASSUME A1] бобышка под шпильку
HORN_BOSS_H  = 3.50             # [ASSUME A1] высота бобышки
HORN_ARM_W   = 8.00             # [ASSUME A1] ширина пера
HORN_REACH   = 16.30            # [ASSUME A1] вылет пера от оси
HUB_T        = 12.00            # толщина ступицы
HUB_CB_D     = 10.00            # расточка под бобышку качалки
HUB_CB_END   = HORN_BOSS_H + 0.40            # торец расточки от плоскости вала
HUB_SLOT_W   = HORN_ARM_W + 2*0.30           # 8.6 паз под перо (0.3 на сторону)
HUB_SLOT_END = HORN_BOSS_H + HORN_T + 0.50   # потолок паза от плоскости вала
HUB_SCR_D    = 6.50             # карман под головку штатного винтика качалки
HUB_SCR_END  = HUB_SLOT_END + 3.10

# --- корпус (ТЗ: Ш 80-100, Г 50-70, В 90-130)
TORSO_W      = 94.0
TORSO_D      = 62.0
TORSO_H      = 121.0
TUB_D        = TORSO_D - 3.0    # 59 - глубина самой ванны (3 мм - задняя крышка)
TUB_TOP      = 118.0            # верх стенок ванны
NECK_T       = 3.0              # толщина плиты шеи
COVER_T      = 3.0              # толщина задней крышки
TUB_BACK_Y   = 28.0             # плоскость стыка ванны и крышки
CAV_X        = 44.2             # внутренняя плоскость боковой стенки
CAV_YF       = -28.2            # внутренняя плоскость передней стенки
RIM_W        = 3.0              # ширина борта-пробки
RIM_H        = 6.0              # высота борта-пробки
PLUG_D       = 1.85             # [ASSUME A6] Ø отверстия под пробку-филамент
PLUG_LEN     = 2.20             # глубина глухого отверстия в стенке

# --- Arduino Uno (по ТЗ)
UNO_L, UNO_W, UNO_T, UNO_H = 68.6, 53.4, 1.6, 12.0
BOARD_Z      = 76.0             # низ платы
RAIL_W       = 2.5              # полка-направляющая
RAIL_H       = 3.0              # высота полки
FIN_W        = 2.4              # центральное ребро-опора платы
LIP_H        = 2.4              # упор-губа по переднему краю платы
UNO_WIN_W    = 26.0             # окно USB-B / питания в правой стенке
UNO_WIN_H    = 14.0
WIRE_WIN_W   = 12.0             # окно кабельной трассы в руку
WIRE_WIN_H   = 8.0
WIRE_WIN_Z   = 66.0             # центр окна по высоте

# --- оси шарниров (система торса, Z=0 - низ торса; низ робота = -LEG_H)
SH_AXIS_Z    = 101.0            # ось плеча
EL_AXIS_Z    = 39.0             # ось локтя
NECK_FACE_Z  = 121.0            # плоскость вала сервы шеи (верх плиты шеи)

# --- звенья рук
HUB_D_SH     = 42.0             # Ø ступицы плеча
HUB_D_EL     = 38.0             # Ø ступицы локтя
ARM_LIMB_W   = 34.0             # толщина плечевого звена по X (47...81)
ARM_LIMB_D   = 26.0
ARM_LIMB_Z0  = 20.0             # низ плечевого звена
FA_LIMB_W    = 28.0             # толщина предплечья (81...109)
FA_LIMB_D    = 24.0
FA_WRIST_Z   = -6.0
HAND_Z0      = -34.0            # низ клешни
HAND_FING_W  = 9.0

# --- посадочное место сервы (единое: плечо, локоть, шея)
REB_L        = SG_EAR_L + 1.0   # 33.0 длина выборки под ушки (0.5 на сторону)
REB_W        = SG_W + 0.6       # 12.8 ширина выборки под ушки
WIN_L        = SG_L + 2*SG_FIT  # 23.6 окно корпуса по оси ушек
WIN_W        = SG_W + 2*SG_FIT  # 12.8 окно корпуса по толщине
PIN_D        = 2.0              # штифт в отверстие ушка Ø2.2
PIN_LEN      = SG_EAR_T - 0.05     # штифт заполняет отверстие ушка, не выступая наружу
CABLE_NOTCH  = 2.4              # прорезь под кабель на кромке окна

# --- "туфля" сервы
SHOE_T       = 2.80             # толщина стенок чашки
SHOE_H       = SG_W*0.5 + 0.40  # полвысоты чашки
SHOE_W_IN    = SG_L*0.5 + 0.20  # внутренний габарит по корпусу
SHOE_W_OUT   = SG_EAR_L*0.5 + 0.30

# --- ноги и подошва
LEG_H        = 70.0             # низ торса над полом
LEG_X        = 25.0             # ось ноги
LEG_TOP_W, LEG_TOP_D = 46.0, 34.0
LEG_BOT_W, LEG_BOT_D = 36.0, 28.0
FOOT_W, FOOT_D = 48.0, 44.0
SPIGOT_W, SPIGOT_D, SPIGOT_R = 36.0, 28.0, 4.0     # шип на верху ноги
TENON_H      = 9.0              # высота шипа (входит в стакан торса)
TENON_NOTCH  = 1.3              # глубина паза под защёлку в шипе
CUP_T        = 2.6              # толщина стенок стакана в дне торса (по X)
CUP_TY       = 2.0              # толщина стенок стакана по Y
CUP_H        = 9.5              # высота стакана над полом торса
CUP_TAB_Z0, CUP_TAB_Z1 = 1.2, 8.1   # вырезка, освобождающая язычок (система торса)
TAB_BUMP_Z0, TAB_BUMP_Z1 = 3.2, 7.1 # защёлка-клин язычка (система торса)
WEIGHT_D     = 17.0             # полость под утяжелитель Ø16
WEIGHT_TOP   = -14.0            # потолок полости от верха ноги
SOLE_REB_W, SOLE_REB_D, SOLE_REB_T = 44.0, 36.0, 3.5
SOLE_PL_W, SOLE_PL_D, SOLE_PL_T = 43.4, 35.4, 3.4
SOLE_RIB     = 0.45             # выступ рёбер трения заглушки
LEGTAB_T     = 1.20             # толщина язычка защёлки ноги
LEGTAB_W     = 6.0              # ширина язычка
LEGTAB_BUMP  = 0.50             # выступ защёлки в паз шипа
LEGTAB_RELIEF = 3.0             # глубина разгрузочной выборки снаружи

# --- голова
HEAD_BASE_W, HEAD_BASE_D, HEAD_BASE_T = 72.0, 56.0, 12.0
HEAD_W, HEAD_D = 78.0, 62.0
HEAD_R       = 10.0
HEAD_TOP_Z   = 189.5            # верх шлема (система торса)
HEAD_BASE_Z  = NECK_FACE_Z + 0.5   # низ основания головы (зазор вращения 0.5)
HEAD_CAV_W, HEAD_CAV_D = 60.0, 46.0
HEAD_VISOR   = (52.0, 24.0, 3.0)
HEAD_EYE_D   = 13.0
HEAD_EAR_D   = 18.0
HEAD_ANT_H   = 13.0

# --- задняя крышка
COVER_HOLE_Z = (20.0, 60.0)     # высоты пробок крышки
COVER_HOLE_Y = 25.4             # координата отверстий по Y

# --- раскладка на столе печати
LAYOUT_X0, LAYOUT_Y0 = -160.0, -150.0
LAYOUT_GAP = 15.0
LAYOUT_W   = 330.0

TOTAL_HEIGHT = LEG_H + HEAD_TOP_Z + HEAD_ANT_H


# =========================================================================================
# 2. ПРИМИТИВЫ
# =========================================================================================

def rect_pts(w, h, cx=0.0, cy=0.0):
    hw, hh = w*0.5, h*0.5
    return [(cx-hw, cy-hh), (cx+hw, cy-hh), (cx+hw, cy+hh), (cx-hw, cy+hh)]


def rrect_pts(w, h, r, cx=0.0, cy=0.0, seg=8):
    """Прямоугольник со скруглёнными углами, обход CCW."""
    r = max(0.0, min(r, w*0.5-1e-6, h*0.5-1e-6))
    hw, hh = w*0.5-r, h*0.5-r
    corners = [(hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh)]
    a0 = [0.0, 90.0, 180.0, 270.0]
    pts = []
    for (cx0, cy0), a in zip(corners, a0):
        for i in range(seg+1):
            t = math.radians(a + 90.0*i/seg)
            pts.append((cx+cx0+r*math.cos(t), cy+cy0+r*math.sin(t)))
    return pts


def circle_pts(d, cx=0.0, cy=0.0, seg=40):
    r = d*0.5
    return [(cx+r*math.cos(2*math.pi*i/seg), cy+r*math.sin(2*math.pi*i/seg)) for i in range(seg)]


class Shell:
    """Набор замкнутых оболочек-примитивов (позитив либо негатив детали).

    Отображение профиля (u,v) и оси выдавливания w:
        'X': (x,y,z) = (w, u+ou, v+ov)
        'Y': (x,y,z) = (u+ou, w, v+ov)
        'Z': (x,y,z) = (u+ou, v+ov, w)
    mirror=True инвертирует X (левая деталь).
    """

    def __init__(self, mirror=False, name="shell"):
        self.prims = []
        self.mirror = mirror
        self.name = name

    def _emit(self, verts, faces):
        if self.mirror:
            verts = [(-v[0], v[1], v[2]) for v in verts]
            faces = [tuple(reversed(f)) for f in faces]
        self.prims.append((verts, faces))

    @staticmethod
    def _map(u, v, w, axis, off):
        if axis == 'X':
            return (w, u+off[0], v+off[1])
        if axis == 'Y':
            return (u+off[0], w, v+off[1])
        if axis == 'Z':
            return (u+off[0], v+off[1], w)
        raise ValueError(axis)

    def prism(self, pts, axis, w0, w1, off=(0.0, 0.0)):
        n = len(pts)
        verts = [self._map(u, v, w, axis, off) for w in (w0, w1) for (u, v) in pts]
        faces = [(i, (i+1) % n, n+(i+1) % n, n+i) for i in range(n)]
        faces.append(tuple(range(n)))
        faces.append(tuple(range(2*n-1, n-1, -1)))
        self._emit(verts, faces)

    def loft(self, ptsA, wA, ptsB, wB, axis, off=(0.0, 0.0)):
        n = len(ptsA)
        assert len(ptsB) == n, "loft: у профилей должно быть равное число точек"
        verts = [self._map(u, v, wA, axis, off) for (u, v) in ptsA]
        verts += [self._map(u, v, wB, axis, off) for (u, v) in ptsB]
        faces = [(i, (i+1) % n, n+(i+1) % n, n+i) for i in range(n)]
        faces.append(tuple(range(n)))
        faces.append(tuple(range(2*n-1, n-1, -1)))
        self._emit(verts, faces)

    # --- удобные обёртки (a,b - координаты в плоскости профиля)
    def box(self, x0, y0, z0, x1, y1, z1):
        self.prism(rect_pts(abs(x1-x0), abs(y1-y0), (x0+x1)*0.5, (y0+y1)*0.5),
                   'Z', min(z0, z1), max(z0, z1))

    def slab(self, axis, w0, w1, a0, a1, b0, b1):
        self.prism(rect_pts(abs(a1-a0), abs(b1-b0), (a0+a1)*0.5, (b0+b1)*0.5),
                   axis, min(w0, w1), max(w0, w1))

    def cyl(self, axis, c, d, w0, w1, seg=40):
        off = {'X': (c[1], c[2]), 'Y': (c[0], c[2]), 'Z': (c[0], c[1])}[axis]
        self.prism(circle_pts(d, seg=seg), axis, min(w0, w1), max(w0, w1), off)

    def cone(self, axis, c, d0, d1, w0, w1, seg=40):
        off = {'X': (c[1], c[2]), 'Y': (c[0], c[2]), 'Z': (c[0], c[1])}[axis]
        self.loft(circle_pts(d0, seg=seg), w0, circle_pts(d1, seg=seg), w1, axis, off)

    def to_object(self, name):
        me = bpy.data.meshes.new(name)
        bm = bmesh.new()
        for (verts, faces) in self.prims:
            bv = [bm.verts.new(v) for v in verts]
            for f in faces:
                try:
                    bm.faces.new([bv[i] for i in f])
                except ValueError:
                    pass
        bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.001)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new(name, me)
        bpy.context.scene.collection.objects.link(ob)
        return ob


# =========================================================================================
# 3. БУЛЕВЫ (solver EXACT, применяются) И СБОРКА ДЕТАЛИ
# =========================================================================================

def _bool_collection(target, operands, operation):
    col = bpy.data.collections.new("TMP_BOOL")
    bpy.context.scene.collection.children.link(col)
    for ob in operands:
        bpy.context.scene.collection.objects.unlink(ob)
        col.objects.link(ob)
    mod = target.modifiers.new("bool", 'BOOLEAN')
    mod.solver = 'EXACT'
    mod.operation = operation
    mod.operand_type = 'COLLECTION'
    mod.collection = col
    mod.use_self = True
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    for ob in list(col.objects):
        me = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.meshes.remove(me)
    bpy.data.collections.remove(col)


def build_part(name, pos, neg, material):
    """(объединение позитивов) - (объединение негативов); нормали наружу."""
    objs = []
    for i, (v, f) in enumerate(pos.prims):
        s = Shell(name=f"{name}_p{i}")
        s.prims = [(v, f)]
        objs.append(s.to_object(f"{name}_p{i}"))
    if not objs:
        raise RuntimeError(f"{name}: пустая позитивная геометрия")
    target = objs[0]
    if len(objs) > 1:
        _bool_collection(target, objs[1:], 'UNION')
    negs = []
    for i, (v, f) in enumerate(neg.prims):
        s = Shell(name=f"{name}_n{i}")
        s.prims = [(v, f)]
        negs.append(s.to_object(f"{name}_n{i}"))
    if negs:
        _bool_collection(target, negs, 'DIFFERENCE')
    target.name = name
    target.data.name = name + "_mesh"
    bm = bmesh.new()
    bm.from_mesh(target.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(target.data)
    bm.free()
    target.data.validate()
    target.data.materials.clear()
    target.data.materials.append(material)
    return target


def mesh_stats(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    nonman = sum(1 for e in bm.edges if not e.is_manifold)
    vol = abs(bm.calc_volume())
    bm.free()
    tris = sum(len(p.vertices)-2 for p in ob.data.polygons)
    return tris, nonman, vol


def bbox(ob):
    pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
    xs = [p.x for p in pts]; ys = [p.y for p in pts]; zs = [p.z for p in pts]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


# =========================================================================================
# 4. МАТЕРИАЛЫ
# =========================================================================================

MAT = {}


def make_mat(name, color, rough=0.55, alpha=1.0, metallic=0.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = color
        bsdf.inputs["Roughness"].default_value = rough
        bsdf.inputs["Metallic"].default_value = metallic
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = alpha
    m.diffuse_color = color
    if alpha < 1.0 and hasattr(m, "blend_method"):
        try:
            m.blend_method = 'BLEND'
        except Exception:
            pass
    return m


def init_materials():
    MAT['main']  = make_mat("PLA_Cobalt",   (0.055, 0.160, 0.420, 1.0), 0.45)
    MAT['frame'] = make_mat("PLA_Graphite", (0.045, 0.050, 0.060, 1.0), 0.60)
    MAT['limb']  = make_mat("PLA_Ivory",    (0.860, 0.850, 0.800, 1.0), 0.50)
    MAT['visor'] = make_mat("PLA_Amber",    (0.950, 0.550, 0.080, 1.0), 0.25)
    MAT['ghost'] = make_mat("REF_Ghost",    (0.850, 0.150, 0.550, 0.35), 0.40, alpha=0.35)
    MAT['pcb']   = make_mat("REF_PCB",      (0.050, 0.350, 0.180, 0.85), 0.60, alpha=0.85)
    MAT['metal'] = make_mat("REF_Metal",    (0.720, 0.720, 0.750, 1.0), 0.30, metallic=0.9)


# =========================================================================================
# 5. ОБЩИЕ УЗЛЫ
# =========================================================================================

def hub_pocket(N, axis, sf, side, mouth, c2=(0.0, 0.0)):
    """Карман под качалку в ступице (единая норма для всех шарниров).

    axis/side - направление вала; sf - плоскость вала сервы; mouth - плоскость устья
    кармана (внутренняя плоскость ступицы); c2 - центр вала в двух других осях.
    Привязано к плоскости вала:
      * расточка Ø10 под бобышку качалки (3.5 от плоскости вала);
      * крестовой паз 8.6 под перо качалки, потолок 6.5 от плоскости вала -
        он же осевой фиксатор ступицы на качалке (осевой люфт HUB_PLAY);
      * карман Ø6.5 под головку штатного винтика качалки.
    """
    u, v = c2
    if axis == 'X':
        c3 = (0.0, u, v)
    elif axis == 'Y':
        c3 = (u, 0.0, v)
    else:
        c3 = (u, v, 0.0)
    d1 = sf + side*HORN_BOSS_H
    d2 = sf + side*HUB_SLOT_END
    d3 = sf + side*HUB_SCR_END
    N.cyl(axis, c3, HUB_CB_D, mouth, d1)
    w = HUB_SLOT_W*0.5
    A = HORN_REACH
    if axis == 'X':
        N.slab('X', d1, d2, u-w, u+w, v-A, v+A)
        N.slab('X', d1, d2, u-A, u+A, v-w, v+w)
    elif axis == 'Y':
        N.slab('Y', d1, d2, u-w, u+w, v-A, v+A)
        N.slab('Y', d1, d2, u-A, u+A, v-w, v+w)
    else:
        N.slab('Z', d1, d2, u-w, u+w, v-A, v+A)
        N.slab('Z', d1, d2, u-A, u+A, v-w, v+w)
    N.cyl(axis, c3, HUB_SCR_D, d2, d3)


def servo_seat(P, N, axis, wall_out, side, c2, wall_t=WALL, ear_along='u',
               notch_side=+1):
    """Посадочное место SG90 в стенке + осевой упор + штифты + прорезь под кабель.

    wall_out - координата НАРУЖНОЙ поверхности стенки вдоль оси вала;
    side     - направление наружу (+1/-1), корпус сервы уходит внутрь;
    c2       - центр вала в двух других осях;
    ear_along- вдоль какой координаты профиля лежат ушки ('u' или 'v').

    Плоскость вала утоплена на SG_EAR_SET=1.2 мм внутрь от наружной поверхности.
    Ушки (32 мм) длиннее окна корпуса (23.6 мм), поэтому:
      * они входят в выборку в стенке и работают как осевой упор сервы;
      * снаружи ступица закрывает посадочное место;
      * изнутри "туфля" прижимает ушки к выборке.
    Возвращает описания штифтов (они уже добавлены в позитив).
    """
    u, v = c2
    face    = wall_out - side*SG_EAR_SET             # плоскость вала сервы
    ear_out = face - side*SG_EAR_SET                 # наружная плоскость ушек
    ear_in  = face - side*(SG_EAR_SET + SG_EAR_T)    # внутренняя плоскость ушек
    wall_in = wall_out - side*wall_t                 # внутренняя плоскость стенки
    rb0, rb1 = min(wall_in, ear_out + side*0.10), max(wall_in, ear_out + side*0.10)
    wn0, wn1 = min(wall_in - side*1.2, wall_out + side*1.2), \
               max(wall_in - side*1.2, wall_out + side*1.2)
    if ear_along == 'u':
        N.slab(axis, rb0, rb1, u-REB_L*0.5, u+REB_L*0.5, v-REB_W*0.5, v+REB_W*0.5)
        N.slab(axis, wn0, wn1, u-WIN_L*0.5, u+WIN_L*0.5, v-WIN_W*0.5, v+WIN_W*0.5)
        n0 = v + notch_side*(WIN_W*0.5-0.4)
        n1 = v + notch_side*(WIN_W*0.5+CABLE_NOTCH)
        N.slab(axis, wn0, wn1, u-CABLE_NOTCH, u+CABLE_NOTCH, min(n0, n1), max(n0, n1))
    else:
        N.slab(axis, rb0, rb1, u-REB_W*0.5, u+REB_W*0.5, v-REB_L*0.5, v+REB_L*0.5)
        N.slab(axis, wn0, wn1, u-WIN_W*0.5, u+WIN_W*0.5, v-WIN_L*0.5, v+WIN_L*0.5)
        n0 = u + notch_side*(WIN_W*0.5-0.4)
        n1 = u + notch_side*(WIN_W*0.5+CABLE_NOTCH)
        N.slab(axis, wn0, wn1, min(n0, n1), max(n0, n1), v-CABLE_NOTCH, v+CABLE_NOTCH)
    pins = []
    for s2 in (+1, -1):
        a = (u if ear_along == 'u' else v) + s2*SG_EAR_PITCH*0.5
        b = v if ear_along == 'u' else u
        if axis == 'X':
            c3 = (0.0, a if ear_along == 'u' else u, v if ear_along == 'u' else a)
        elif axis == 'Y':
            c3 = (a if ear_along == 'u' else u, 0.0, v if ear_along == 'u' else a)
        else:
            c3 = (a if ear_along == 'u' else u, v if ear_along == 'u' else a, 0.0)
        w0, w1 = ear_out, ear_out - side*PIN_LEN
        P.cyl(axis, c3, PIN_D - 0.10, w0, w1, seg=24)
        pins.append((c3, w0, w1))
    return pins, dict(face=face, ear_out=ear_out, ear_in=ear_in)


def blind_hole(N, axis, face, side, c2, d=PLUG_D, depth=PLUG_LEN):
    """Глухое отверстие под пробку-штифт."""
    N.cyl(axis, (0.0, 0.0, 0.0), d, face, face - side*depth) if False else None
    if axis == 'X':
        N.cyl('X', (0.0, c2[0], c2[1]), d, face, face - side*depth)
    elif axis == 'Y':
        N.cyl('Y', (c2[0], 0.0, c2[1]), d, face, face - side*depth)
    else:
        N.cyl('Z', (c2[0], c2[1], 0.0), d, face, face - side*depth)


# =========================================================================================
# 6. ПЕЧАТНЫЕ ДЕТАЛИ
# =========================================================================================

# ----------------------------------------------------------------- 6.1 короб-ванна
def build_torso():
    P = Shell(name="torso_pos")
    N = Shell(name="torso_neg")
    yc = -1.5                                     # центр ванны по Y (диапазон Y: -31...28)

    # оболочка с фаской по низу
    P.loft(rrect_pts(TORSO_W-3.0, TUB_D-3.0, 3.0, 0.0, yc), 0.0,
           rrect_pts(TORSO_W, TUB_D, 6.0, 0.0, yc), 1.5, 'Z')
    P.prism(rrect_pts(TORSO_W, TUB_D, 6.0, 0.0, yc), 'Z', 1.5, TUB_TOP)

    # полость (открыта вверх и назад)
    N.box(-CAV_X, CAV_YF, WALL, CAV_X, 60.0, TUB_TOP+8.0)

    # --- направляющие Arduino Uno: 2 полки со скосом, центральное ребро, упор-губа
    y_rail_end = TUB_BACK_Y - RIM_H - 0.5
    for sx in (+1, -1):
        x0 = sx*(CAV_X - RAIL_W)
        P.slab('Z', BOARD_Z-RAIL_H, BOARD_Z, min(x0, sx*CAV_X), max(x0, sx*CAV_X),
               CAV_YF, y_rail_end)
        P.loft(rect_pts(0.9, y_rail_end-CAV_YF, sx*(CAV_X-RAIL_W+0.45), (CAV_YF+y_rail_end)*0.5),
               BOARD_Z-RAIL_H,
               rect_pts(RAIL_W, y_rail_end-CAV_YF, sx*(CAV_X-RAIL_W*0.5), (CAV_YF+y_rail_end)*0.5),
               BOARD_Z-RAIL_H+RAIL_W, 'Z')
    P.slab('Z', WALL, BOARD_Z, -FIN_W*0.5, FIN_W*0.5, CAV_YF, y_rail_end)
    P.slab('Z', BOARD_Z, BOARD_Z+LIP_H, -UNO_L*0.5-0.5, UNO_L*0.5+0.5, CAV_YF, CAV_YF+1.2)

    # --- плечевые сервы (обе стороны)
    for sx in (+1, -1):
        servo_seat(P, N, 'X', sx*TORSO_W*0.5, sx, (0.0, SH_AXIS_Z), ear_along='u')
        # окно кабельной трассы в руку (скрыто за звеном руки)
        N.slab('X', CAV_X-1.0, TORSO_W*0.5+1.0, -WIRE_WIN_W*0.5, WIRE_WIN_W*0.5,
               WIRE_WIN_Z-WIRE_WIN_H*0.5, WIRE_WIN_Z+WIRE_WIN_H*0.5)

    # --- окно доступа к USB-B и разъёму питания (правая стенка)
    N.slab('X', CAV_X-2.0, TORSO_W*0.5+1.5, -UNO_WIN_W*0.5, UNO_WIN_W*0.5,
           BOARD_Z+0.5, BOARD_Z+0.5+UNO_WIN_H)

    # --- отверстие под провода в дне + стойки-стяжки
    N.cyl('Z', (20.0, 22.0, 0.0), 10.0, -1.0, WALL+1.0)
    for dx in (-6.5, 6.5):
        P.cyl('Z', (20.0+dx, 22.0, 0.0), 5.0, WALL, WALL+8.0, seg=20)

    # --- крепление ног: окно в полу + стакан с двумя язычковыми защёлками
    #     (решение выбрано так, чтобы наружное дно торса осталось плоским - печать
    #      без опор и с полным прилеганием первого слоя)
    cup_w, cup_d = SPIGOT_W + 2*FIT, SPIGOT_D + 2*FIT        # 36.7 x 28.7 - гнездо шипа
    cup_ow, cup_od = cup_w + 2*CUP_T, cup_d + 2*CUP_TY       # наружный габарит стакана
    for sx in (+1, -1):
        cx = sx*LEG_X
        N.prism(rrect_pts(cup_w, cup_d, SPIGOT_R, cx, 0.0), 'Z', -1.0, WALL+1.0)
        P.prism(rrect_pts(cup_ow, cup_od, SPIGOT_R+CUP_T, cx, 0.0), 'Z', WALL, WALL+CUP_H)
        N.prism(rrect_pts(cup_w, cup_d, SPIGOT_R, cx, 0.0), 'Z', WALL-0.01, WALL+CUP_H+1.0)
        for s2 in (+1, -1):
            x_in = cx + s2*cup_w*0.5                         # рабочая плоскость язычка
            x_out = cx + s2*cup_ow*0.5
            # разгрузочная выборка снаружи: оставляет язычок толщиной LEGTAB_T
            N.slab('X', x_in+s2*LEGTAB_T, x_out+s2*1.5,
                   -LEGTAB_W*0.5, LEGTAB_W*0.5, CUP_TAB_Z0, CUP_TAB_Z1)
            # боковые прорези, освобождающие язычок
            for sgn in (+1, -1):
                a0, a1 = sgn*LEGTAB_W*0.5, sgn*(LEGTAB_W*0.5 + 2.2)
                N.slab('X', x_in-0.5, x_out+1.5, min(a0, a1), max(a0, a1),
                       CUP_TAB_Z0, CUP_TAB_Z1)
            # защёлка-клин: вход 43 град. снизу, упор 90 град. сверху
            P.prism([(x_in, TAB_BUMP_Z0), (x_in - s2*LEGTAB_BUMP, TAB_BUMP_Z1), (x_in, TAB_BUMP_Z1)],
                    'Y', -LEGTAB_W*0.5, LEGTAB_W*0.5)

    # --- 4 глухих отверстия под пробки задней крышки
    for sx in (+1, -1):
        for zc in COVER_HOLE_Z:
            N.cyl('X', (0.0, COVER_HOLE_Y, zc), PLUG_D, CAV_X, CAV_X+PLUG_LEN, seg=20)

    # --- декор: нагрудная панель и вентиляционные прорези
    N.prism(rrect_pts(52.0, 34.0, 6.0, 0.0, 0.0), 'Y', -31.0, -29.0, off=(0.0, 96.0))
    for i in range(3):
        N.prism(rrect_pts(40.0, 3.6, 1.6, 0.0, 0.0), 'Y', -29.5, -27.0, off=(0.0, 86.0+i*8.0))
    return P, N


# ----------------------------------------------------------------- 6.2 задняя крышка
def build_back_cover():
    P = Shell(name="cover_pos")
    N = Shell(name="cover_neg")
    y0 = TUB_BACK_Y                                 # 28 - внутренняя плоскость плиты
    y1 = TUB_BACK_Y + COVER_T                       # 31 - наружная плоскость
    z0, z1 = WALL, TUB_TOP - WALL
    P.prism(rrect_pts(TORSO_W, TUB_TOP, 6.0, 0.0, TUB_TOP*0.5), 'Y', y0, y1)
    # контурный борт-пробка
    for sx in (+1, -1):
        P.slab('Y', y0-RIM_H, y0, sx*(CAV_X-FIT-RIM_W), sx*(CAV_X-FIT), z0, z1)
    P.slab('Y', y0-RIM_H, y0, -(CAV_X-FIT), CAV_X-FIT, z0, z0+RIM_W)
    P.slab('Y', y0-RIM_H, y0, -(CAV_X-FIT), CAV_X-FIT, z1-RIM_W, z1)
    # 4 отверстия под пробки-штифты
    for sx in (+1, -1):
        for zc in COVER_HOLE_Z:
            N.cyl('X', (0.0, COVER_HOLE_Y, zc), PLUG_D, sx*(CAV_X-FIT-RIM_W)-sx*1.0,
                  sx*(CAV_X-FIT)+sx*1.0, seg=20)
    # прорезь-захват для пальца и вентиляционные прорези
    N.prism(rrect_pts(26.0, 11.0, 5.0), 'Y', y0-0.5, y1+1.0, off=(0.0, 9.0))
    for zc in (98.0, 105.0, 112.0):
        N.prism(rrect_pts(30.0, 4.0, 1.8), 'Y', y0-0.5, y1+1.0, off=(0.0, zc))
    return P, N


# ----------------------------------------------------------------- 6.3 плита шеи
def build_neck_plate():
    P = Shell(name="neck_pos")
    N = Shell(name="neck_neg")
    z0, z1 = TUB_TOP, TUB_TOP + NECK_T
    P.prism(rrect_pts(TORSO_W, TORSO_D, 6.0, 0.0, -1.5), 'Z', z0, z1)
    yb = 20.0
    for sx in (+1, -1):
        P.slab('Z', z0-RIM_H, z0, sx*(CAV_X-FIT-RIM_W), sx*(CAV_X-FIT), CAV_YF+FIT, yb)
    P.slab('Z', z0-RIM_H, z0, -(CAV_X-FIT), CAV_X-FIT, CAV_YF+FIT, CAV_YF+FIT+RIM_W)
    P.slab('Z', z0-RIM_H, z0, -(CAV_X-FIT), CAV_X-FIT, yb-RIM_W, yb)
    # фиксирующие рёбра борта (посадка с лёгким натягом)
    for sx in (+1, -1):
        P.slab('X', sx*(CAV_X-FIT), sx*(CAV_X-FIT+0.22), CAV_YF+FIT, yb, z0-RIM_H+1.0, z0-1.0)
    # посадочное место сервы шеи (вал вверх, корпус уходит вниз)
    servo_seat(P, N, 'Z', z1, +1, (0.0, 0.0), wall_t=NECK_T, ear_along='u')
    N.cyl('Z', (22.0, 0.0, 0.0), 10.0, z0-1.0, z1+1.0)
    return P, N


# ----------------------------------------------------------------- 6.4 голова
def build_head():
    P = Shell(name="head_pos")
    N = Shell(name="head_neg")
    zb, zt = HEAD_BASE_Z, HEAD_TOP_Z
    P.prism(rrect_pts(HEAD_BASE_W, HEAD_BASE_D, 8.0), 'Z', zb, zb+HEAD_BASE_T)
    P.loft(rrect_pts(HEAD_BASE_W, HEAD_BASE_D, 8.0), zb+HEAD_BASE_T,
           rrect_pts(HEAD_W, HEAD_D, HEAD_R), zb+HEAD_BASE_T+7.5, 'Z')
    P.prism(rrect_pts(HEAD_W, HEAD_D, HEAD_R), 'Z', zb+HEAD_BASE_T+7.5, zt-11.0)
    P.loft(rrect_pts(HEAD_W, HEAD_D, HEAD_R), zt-11.0,
           rrect_pts(HEAD_W-20.0, HEAD_D-20.0, HEAD_R), zt, 'Z')
    P.loft(rrect_pts(HEAD_W-20.0, HEAD_D-20.0, HEAD_R), zt,
           rrect_pts(HEAD_W-34.0, HEAD_D-34.0, 8.0), zt+3.0, 'Z')
    for sx in (+1, -1):
        P.cyl('X', (0.0, 0.0, zb+37.0), HEAD_EAR_D, sx*(HEAD_W*0.5-6.0), sx*(HEAD_W*0.5+4.5))
        N.cyl('X', (0.0, 0.0, zb+37.0), 9.0, sx*(HEAD_W*0.5+2.6), sx*(HEAD_W*0.5+5.5))
    P.cyl('Z', (0.0, 18.0, 0.0), 9.0, zt-4.0, zt+9.0)
    P.cone('Z', (0.0, 18.0, 0.0), 9.0, 4.0, zt+9.0, zt+HEAD_ANT_H)
    # полость под электронику, потолок - двускатный
    cwid, cdep = HEAD_CAV_W, HEAD_CAV_D
    zc1 = zt - 19.0
    N.box(-cwid*0.5, -cdep*0.5, zb+HEAD_BASE_T+1.0, cwid*0.5, cdep*0.5, zc1)
    N.loft(rect_pts(cwid, cdep), zc1, rect_pts(cwid-30.0, cdep-30.0), zt-10.0, 'Z')
    for sx in (+1, -1):
        P.slab('Z', zc1-14.0, zc1-11.0, sx*(cwid*0.5-6.0), sx*cwid*0.5, -9.0, 9.0)
    P.slab('Z', zc1-11.0, zc1-9.6, -(cwid*0.5-6.0), cwid*0.5-6.0, -9.0, -7.4)
    N.cyl('Z', (22.0, 0.0, 0.0), 10.0, zb-1.0, zb+HEAD_BASE_T+1.5)
    # визор с глазницами
    vw, vh, vd = HEAD_VISOR
    yf = -HEAD_D*0.5
    zv = zb + HEAD_BASE_T + 27.0
    N.prism(rrect_pts(vw, vh, 7.0), 'Y', yf-1.0, yf+vd, off=(0.0, zv))
    for sx in (+1, -1):
        N.cyl('Y', (sx*11.0, 0.0, zv), HEAD_EYE_D, yf+vd-0.6, yf+vd+1.8)
    # карман качалки сервы шеи (вал снизу)
    hub_pocket(N, 'Z', NECK_FACE_Z - SG_EAR_SET, +1, HEAD_BASE_Z, (0.0, 0.0))
    return P, N


# ----------------------------------------------------------------- 6.5 плечевое звено
def build_upper_arm(side):
    # геометрия всегда строится для правой детали, левая получается зеркалированием
    sx = +1
    P = Shell(mirror=(side < 0), name="ua_pos")
    N = Shell(mirror=(side < 0), name="ua_neg")
    x_face = sx*TORSO_W*0.5                     # 47 - плоскость вала плеча
    x_out = x_face + sx*ARM_LIMB_W              # 81 - плоскость вала локтя
    lim_cx = (x_face + x_out)*0.5
    P.cyl('X', (0.0, 0.0, SH_AXIS_Z), HUB_D_SH, x_face, x_face+sx*HUB_T)
    hub_pocket(N, 'X', x_face - sx*SG_EAR_SET, sx, x_face, (0.0, SH_AXIS_Z))
    P.prism(rrect_pts(ARM_LIMB_W, ARM_LIMB_D, 7.0, lim_cx, 0.0), 'Z', ARM_LIMB_Z0, SH_AXIS_Z)
    N.prism(rrect_pts(ARM_LIMB_W-2*WALL, ARM_LIMB_D-2*WALL, 4.0, lim_cx, 0.0), 'Z',
            ARM_LIMB_Z0+2.4, SH_AXIS_Z-8.0)
    # клетка локтевой сервы
    servo_seat(P, N, 'X', x_out, sx, (0.0, EL_AXIS_Z), ear_along='v', notch_side=-1)
    # окно кабельной трассы в стенку торса
    N.slab('X', x_face-1.5, x_face+WALL+1.0, -WIRE_WIN_W*0.5, WIRE_WIN_W*0.5,
           WIRE_WIN_Z-WIRE_WIN_H*0.5, WIRE_WIN_Z+WIRE_WIN_H*0.5)
    # декор: по 2 окна на боковых сторонах звена
    for sy in (+1, -1):
        for zc in (72.0, 57.0):
            N.prism(rrect_pts(8.0, 12.0, 3.0), 'Y', sy*ARM_LIMB_D*0.5,
                    sy*(ARM_LIMB_D*0.5-1.2), off=(lim_cx, zc))
    return P, N


# ----------------------------------------------------------------- 6.6 предплечье
def build_forearm(side):
    sx = +1
    P = Shell(mirror=(side < 0), name="fa_pos")
    N = Shell(mirror=(side < 0), name="fa_neg")
    x_face = sx*(TORSO_W*0.5 + ARM_LIMB_W)      # 81 - плоскость вала локтя
    x_out = x_face + sx*FA_LIMB_W               # 109
    lim_cx = (x_face + x_out)*0.5
    P.cyl('X', (0.0, 0.0, EL_AXIS_Z), HUB_D_EL, x_face, x_face+sx*HUB_T)
    hub_pocket(N, 'X', x_face - sx*SG_EAR_SET, sx, x_face, (0.0, EL_AXIS_Z))
    P.prism(rrect_pts(FA_LIMB_W, FA_LIMB_D, 6.0, lim_cx, 0.0), 'Z', FA_WRIST_Z, EL_AXIS_Z)
    N.prism(rrect_pts(FA_LIMB_W-2*WALL, FA_LIMB_D-2*WALL, 4.0, lim_cx, 0.0), 'Z',
            FA_WRIST_Z+1.2, EL_AXIS_Z-8.0)
    P.prism(rrect_pts(FA_LIMB_W, FA_LIMB_D, 6.0, lim_cx, 0.0), 'Z', -14.0, FA_WRIST_Z)
    for dy, zlo in ((7.0, HAND_Z0), (-7.0, HAND_Z0), (0.0, HAND_Z0+6.0)):
        P.loft(rrect_pts(HAND_FING_W, 11.0, 3.0, lim_cx, dy), -14.0,
               rrect_pts(HAND_FING_W-2.6, 7.4, 2.4, lim_cx, dy), zlo, 'Z')
    return P, N


# ----------------------------------------------------------------- 6.7 нога
def build_leg(side):
    sx = +1
    P = Shell(mirror=(side < 0), name="leg_pos")
    N = Shell(mirror=(side < 0), name="leg_neg")
    cx = sx*LEG_X
    zt = 0.0                                    # верх ноги (= низ торса)
    P.prism(rrect_pts(LEG_TOP_W, LEG_TOP_D, 6.0, cx, 0.0), 'Z', zt-42.0, zt)
    P.loft(rrect_pts(LEG_TOP_W, LEG_TOP_D, 6.0, cx, 0.0), zt-42.0,
           rrect_pts(LEG_BOT_W, LEG_BOT_D, 6.0, cx, 0.0), zt-54.0, 'Z')
    P.loft(rrect_pts(LEG_BOT_W, LEG_BOT_D, 6.0, cx, 0.0), zt-54.0,
           rrect_pts(FOOT_W, FOOT_D, 10.0, cx, 0.0), zt-63.0, 'Z')
    P.prism(rrect_pts(FOOT_W, FOOT_D, 10.0, cx, 0.0), 'Z', zt-LEG_H+1.6, zt-63.0)
    P.loft(rrect_pts(FOOT_W, FOOT_D, 10.0, cx, 0.0), zt-LEG_H+1.6,
           rrect_pts(FOOT_W-4.0, FOOT_D-4.0, 8.0, cx, 0.0), zt-LEG_H, 'Z')
    # шип крепления к торсу (входит в стакан в дне торса, где язычковые защёлки)
    P.prism(rrect_pts(SPIGOT_W, SPIGOT_D, SPIGOT_R, cx, 0.0), 'Z', zt-1.0, zt+TENON_H)
    for s2 in (+1, -1):
        x_out = cx + s2*SPIGOT_W*0.5
        x_in = cx + s2*(SPIGOT_W*0.5 - TENON_NOTCH)
        N.slab('X', min(x_out, x_in), max(x_out, x_in), -6.0, 6.0,
               TAB_BUMP_Z0, TAB_BUMP_Z1)
    # полость под утяжелитель с коническим потолком
    N.cyl('Z', (cx, 0.0, 0.0), WEIGHT_D, zt-LEG_H, zt+WEIGHT_TOP-4.0)
    N.cone('Z', (cx, 0.0, 0.0), WEIGHT_D, 6.0, zt+WEIGHT_TOP-4.0, zt+WEIGHT_TOP)
    N.prism(rrect_pts(SOLE_REB_W, SOLE_REB_D, 8.0, cx, 0.0), 'Z',
            zt-LEG_H, zt-LEG_H+SOLE_REB_T)
    return P, N


# ----------------------------------------------------------------- 6.8 заглушка подошвы
def build_sole_plate():
    P = Shell(name="sole_pos")
    N = Shell(name="sole_neg")
    P.prism(rrect_pts(SOLE_PL_W, SOLE_PL_D, 8.0), 'Z', 0.0, SOLE_PL_T)
    for sx in (+1, -1):
        P.slab('X', sx*SOLE_PL_W*0.5, sx*(SOLE_PL_W*0.5+SOLE_RIB), -8.0, 8.0, 0.6, 2.8)
    N.prism(rrect_pts(12.0, 3.0, 1.2), 'Z', -1.0, 1.3, off=(0.0, -SOLE_PL_D*0.5+0.9))
    return P, N


# ----------------------------------------------------------------- 6.9 "туфля" сервы
def build_servo_shoe():
    """Локально: плоскость прижима ушек X=0, корпус сервы уходит в -X."""
    P = Shell(name="shoe_pos")
    N = Shell(name="shoe_neg")
    back = SG_H + 0.3
    for sy in (+1, -1):
        P.slab('X', -back-SHOE_T, 0.0, sy*SHOE_W_IN, sy*SHOE_W_OUT, -SHOE_H, SHOE_H)
        # поперечное ребро у устья (лёгкий натяг по корпусу сервы)
        P.slab('X', -4.2, -0.6, sy*(SHOE_W_IN-0.25), sy*SHOE_W_IN, -3.0, 3.0)
        # ребро у задней стенки
        P.slab('X', -back+0.8, -back+3.0, sy*(SHOE_W_IN-0.20), sy*SHOE_W_IN, -2.0, 2.0)
    P.slab('X', -back-SHOE_T, -back, -SHOE_W_OUT, SHOE_W_OUT, -SHOE_H, SHOE_H)
    # фаска устья чашки
    for sy in (+1, -1):
        N.slab('X', -1.6, 0.4, sy*SHOE_W_OUT, sy*(SHOE_W_OUT+1.0), -SHOE_H-1.0, SHOE_H+1.0)
    return P, N


# ----------------------------------------------------------------- 6.10 фантомы (REF)
def _ref_finish(ob, mat):
    ob.data.materials.clear()
    ob.data.materials.append(mat)
    ob["kind"] = "ref"
    return ob


def ref_servo(name):
    """Фантом SG90: локально вал по +X, ушки вдоль Y, толщина корпуса по Z."""
    P = Shell(name=name)
    P.slab('X', -SG_H, 0.0, -SG_L*0.5, SG_L*0.5, -SG_W*0.5, SG_W*0.5)
    P.slab('X', -SG_EAR_SET-SG_EAR_T, -SG_EAR_SET, -SG_EAR_L*0.5, SG_EAR_L*0.5,
           -SG_W*0.5, SG_W*0.5)
    P.cyl('X', (0.0, 0.0, 0.0), SG_SHAFT_D, 0.0, SG_SHAFT_L, seg=32)
    P.slab('Z', -5.0, 5.0, -13.0, -3.0, SG_W*0.5, SG_W*0.5+1.6)     # жгут проводов
    ob = P.to_object(name)
    N = Shell(name=name + "_n")
    for s2 in (+1, -1):                                            # отверстия ушек Ø2.2
        N.cyl('X', (0.0, s2*SG_EAR_PITCH*0.5, 0.0), SG_EAR_HOLE,
              -(SG_H + 1.0), -0.2, seg=16)
    neg = N.to_object(name + "_n")
    _bool_collection(ob, [neg], 'DIFFERENCE')
    return _ref_finish(ob, MAT['ghost'])


def ref_horn(name):
    P = Shell(name=name)
    P.cyl('X', (0.0, 0.0, 0.0), HORN_BOSS_D, 0.0, HORN_BOSS_H, seg=24)
    P.slab('X', HORN_BOSS_H, HORN_BOSS_H+HORN_T, -HORN_ARM_W*0.5, HORN_ARM_W*0.5,
           -HORN_REACH, HORN_REACH)
    P.slab('X', HORN_BOSS_H, HORN_BOSS_H+HORN_T, -HORN_REACH, HORN_REACH,
           -HORN_ARM_W*0.5, HORN_ARM_W*0.5)
    P.cyl('X', (0.0, 0.0, 0.0), 3.6, HORN_BOSS_H+HORN_T, HORN_BOSS_H+HORN_T+1.2, seg=16)
    ob = P.to_object(name)
    N = Shell(name=name + "_n")                                    # отверстие под шпильку
    N.cyl('X', (0.0, 0.0, 0.0), SG_SHAFT_D + 0.6, -0.5,
          HORN_BOSS_H + HORN_T + 0.2, seg=32)
    neg = N.to_object(name + "_n")
    _bool_collection(ob, [neg], 'DIFFERENCE')
    return _ref_finish(ob, MAT['metal'])


def ref_uno(name):
    P = Shell(name=name)
    P.box(-UNO_L*0.5, -UNO_W*0.5, -UNO_T*0.5, UNO_L*0.5, UNO_W*0.5, UNO_T*0.5)
    for sy in (+1, -1):
        P.slab('Z', UNO_T*0.5, UNO_T*0.5+11.0, -22.0, 32.0, sy*(UNO_W*0.5-2.5), sy*UNO_W*0.5)
    P.slab('Z', UNO_T*0.5, UNO_T*0.5+11.0, UNO_L*0.5-4.0, UNO_L*0.5+8.0, -6.0, 6.0)
    P.slab('Z', UNO_T*0.5, UNO_T*0.5+11.0, UNO_L*0.5-2.0, UNO_L*0.5+10.0, 10.0, 20.0)
    for dx in (-18.0, -7.0):
        P.cyl('Z', (dx, 12.0, 0.0), 8.0, UNO_T*0.5, UNO_T*0.5+11.0, seg=20)
    P.slab('Z', UNO_T*0.5, UNO_T*0.5+3.0, -6.0, 6.0, -4.0, 4.0)
    return _ref_finish(P.to_object(name), MAT['pcb'])


# =========================================================================================
# 7. СЦЕНА, РАСКЛАДКА, ОТЧЁТ
# =========================================================================================

def set_units():
    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.length_unit = 'MILLIMETERS'
    sc.unit_settings.scale_length = SCALE_LENGTH
    # рендер сцены по умолчанию (вид по F12 - собранный робот)
    sc.render.engine = 'CYCLES'
    sc.cycles.samples = 64
    sc.cycles.use_denoising = False
    sc.render.resolution_x, sc.render.resolution_y = 1280, 1024
    sc.render.image_settings.file_format = 'PNG'


def rot_matrix(rot_deg):
    rx, ry, rz = [math.radians(a) for a in rot_deg]
    return (Matrix.Rotation(rz, 4, 'Z') @ Matrix.Rotation(ry, 4, 'Y') @ Matrix.Rotation(rx, 4, 'X'))


def place_for_bed(ob, rot_deg, pos_xy):
    """Ориентация печати + сдвиг: низ детали на Z=0, центр габарита в pos_xy."""
    ob.matrix_world = rot_matrix(rot_deg)
    bb = bbox(ob)
    ob.matrix_world = (Matrix.Translation((-(bb[0]+bb[3])*0.5, -(bb[1]+bb[4])*0.5, -bb[2]))
                       @ ob.matrix_world)
    bb = bbox(ob)
    w, d = bb[3]-bb[0], bb[4]-bb[1]
    ob.matrix_world = (Matrix.Translation((pos_xy[0]+w*0.5, pos_xy[1]+d*0.5, 0.0))
                       @ ob.matrix_world)
    return w, d


def setup_scene_extras():
    w = bpy.data.worlds.new("World")
    w.use_nodes = True
    bg = w.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = (0.045, 0.05, 0.055, 1.0)
        bg.inputs[1].default_value = 1.0
    bpy.context.scene.world = w

    def light(name, loc, energy, size, rot):
        d = bpy.data.lights.new(name, 'AREA')
        d.energy = energy
        d.size = size
        ob = bpy.data.objects.new(name, d)
        ob.location = loc
        ob.rotation_euler = rot
        bpy.context.scene.collection.objects.link(ob)
        ob["kind"] = "scene"

    light("LIGHT_KEY",  (-330, -380, 540), 1.1e7, 420.0, (math.radians(46), 0, math.radians(-38)))
    light("LIGHT_FILL", ( 400, -280, 320), 3.5e6, 380.0, (math.radians(62), 0, math.radians(52)))
    light("LIGHT_RIM",  ( 150,  420, 400), 4.5e6, 320.0, (math.radians(118), 0, math.radians(160)))

    cam_d = bpy.data.cameras.new("CAM_Assembly")
    cam_d.lens = 80.0
    cam_d.clip_start = 2.0          # 2 мм (сцена в мм!)
    cam_d.clip_end = 100000.0       # 100 м
    cam = bpy.data.objects.new("CAM_Assembly", cam_d)
    cam.location = (0.0, -900.0, 300.0)
    cam.rotation_euler = (math.radians(80), 0.0, 0.0)
    bpy.context.scene.collection.objects.link(cam)
    bpy.context.scene.camera = cam
    cam["kind"] = "scene"

    cam2_d = bpy.data.cameras.new("CAM_PrintBed")
    cam2_d.lens = 50.0
    cam2_d.clip_start = 2.0
    cam2_d.clip_end = 100000.0
    cam2 = bpy.data.objects.new("CAM_PrintBed", cam2_d)
    cam2.location = (0.0, -330.0, 760.0)
    cam2.rotation_euler = (math.radians(16), 0.0, 0.0)
    bpy.context.scene.collection.objects.link(cam2)
    cam2["kind"] = "scene"


# ------------------------------------------------------------------ описание сборки
def asm_matrix(name):
    """Положение детали в сборе (система торса, Z=0 - низ торса)."""
    if name.startswith("Leg"):
        return Matrix.Identity(4)      # нога строится от своего верха: верх = низ торса
    if name.startswith("Sole_Plate"):
        sx = +1 if name.endswith("_1") else -1
        return Matrix.Translation((sx*LEG_X, 0.0, -LEG_H))
    return Matrix.Identity(4)


def shoe_assembly(idx):
    Tz = lambda x, y, z: Matrix.Translation((x, y, z))
    Rz = lambda a: Matrix.Rotation(math.radians(a), 4, 'Z')
    Rx = lambda a: Matrix.Rotation(math.radians(a), 4, 'X')
    M_neck = Matrix(((0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 0), (0, 0, 0, 1)))
    ears_sh = TORSO_W*0.5 - 2*SG_EAR_SET - SG_EAR_T                 # 42.6
    ears_el = TORSO_W*0.5 + ARM_LIMB_W - 2*SG_EAR_SET - SG_EAR_T    # 76.6
    ears_nk = NECK_FACE_Z - 2*SG_EAR_SET - SG_EAR_T                 # 116.6
    return {
        1: Tz(+ears_sh, 0.0, SH_AXIS_Z),
        2: Tz(-ears_sh, 0.0, SH_AXIS_Z) @ Rz(180),
        3: Tz(+ears_el, 0.0, EL_AXIS_Z) @ Rx(90),
        4: Tz(-ears_el, 0.0, EL_AXIS_Z) @ Rz(180) @ Rx(90),
        5: Tz(0.0, 0.0, ears_nk) @ M_neck,
    }[idx]


def part_specs():
    """(имя, функция, материал, ориентация печати)"""
    return [
        ("Torso_Tub",    build_torso,                 'main',  (0, 0, 0)),
        ("Back_Cover",   build_back_cover,            'frame', (-90, 0, 0)),
        ("Neck_Plate",   build_neck_plate,            'frame', (180, 0, 0)),
        ("Head_Shell",   build_head,                  'main',  (0, 0, 0)),
        ("Upper_Arm_R",  lambda: build_upper_arm(+1), 'limb',  (0, -90, 0)),
        ("Upper_Arm_L",  lambda: build_upper_arm(-1), 'limb',  (0, -90, 0)),
        ("Forearm_R",    lambda: build_forearm(+1),   'limb',  (0, -90, 0)),
        ("Forearm_L",    lambda: build_forearm(-1),   'limb',  (0, -90, 0)),
        ("Leg_R",        lambda: build_leg(+1),       'limb',  (0, 0, 0)),
        ("Leg_L",        lambda: build_leg(-1),       'limb',  (0, 0, 0)),
        ("Sole_Plate_1", build_sole_plate,            'frame', (0, 0, 0)),
        ("Sole_Plate_2", build_sole_plate,            'frame', (0, 0, 0)),
        ("Servo_Shoe_1", build_servo_shoe,            'frame', (0, -90, 0)),
        ("Servo_Shoe_2", build_servo_shoe,            'frame', (0, -90, 0)),
        ("Servo_Shoe_3", build_servo_shoe,            'frame', (0, -90, 0)),
        ("Servo_Shoe_4", build_servo_shoe,            'frame', (0, -90, 0)),
        ("Servo_Shoe_5", build_servo_shoe,            'frame', (0, -90, 0)),
    ]


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    set_units()
    init_materials()

    col_bed = bpy.data.collections.new("PRINT_BED")
    col_asm = bpy.data.collections.new("ASSEMBLY")
    col_ref = bpy.data.collections.new("PHANTOM")
    for c in (col_bed, col_asm, col_ref):
        bpy.context.scene.collection.children.link(c)

    lx, ly, row_h = LAYOUT_X0, LAYOUT_Y0, 0.0
    printed = []

    for (name, fn, mat_key, rot) in part_specs():
        P, N = fn()
        ob = build_part(name, P, N, MAT[mat_key])
        ob["kind"] = "src"

        w, d = place_for_bed(ob, rot, (0.0, 0.0))
        if lx > LAYOUT_X0 and lx + w > LAYOUT_X0 + LAYOUT_W:
            lx = LAYOUT_X0
            ly -= row_h + LAYOUT_GAP
            row_h = 0.0
        place_for_bed(ob, rot, (lx, ly))
        bb = bbox(ob)
        lx += (bb[3]-bb[0]) + LAYOUT_GAP
        row_h = max(row_h, bb[4]-bb[1])

        ob.name = name
        ob.data.name = name + "_mesh"
        ob["kind"] = "printed"
        bpy.context.scene.collection.objects.unlink(ob)
        col_bed.objects.link(ob)
        printed.append(ob)

        # копия в сборе (те же меш-данные)
        if name.startswith("Servo_Shoe"):
            M = shoe_assembly(int(name.split("_")[-1]))
        else:
            M = asm_matrix(name)
        asm = ob.copy()
        asm.data = ob.data
        asm.name = "ASM_" + name
        asm["kind"] = "assembly"
        bpy.context.scene.collection.objects.link(asm)
        bpy.context.scene.collection.objects.unlink(asm)
        col_asm.objects.link(asm)
        asm.matrix_world = M

    # ---------------- фантомы в сборе ----------------
    Rz = lambda a: Matrix.Rotation(math.radians(a), 4, 'Z')
    Rx = lambda a: Matrix.Rotation(math.radians(a), 4, 'X')
    Tz = lambda x, y, z: Matrix.Translation((x, y, z))
    M_neck = Matrix(((0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 0), (0, 0, 0, 1)))
    x_el = TORSO_W*0.5 + ARM_LIMB_W
    refs = [
        (ref_servo("REF_SG90_Shoulder_R"), Tz(TORSO_W*0.5 - SG_EAR_SET, 0, SH_AXIS_Z)),
        (ref_servo("REF_SG90_Shoulder_L"), Tz(-(TORSO_W*0.5 - SG_EAR_SET), 0, SH_AXIS_Z) @ Rz(180)),
        (ref_servo("REF_SG90_Elbow_R"),    Tz(x_el - SG_EAR_SET, 0, EL_AXIS_Z) @ Rx(90)),
        (ref_servo("REF_SG90_Elbow_L"),    Tz(-(x_el - SG_EAR_SET), 0, EL_AXIS_Z) @ Rz(180) @ Rx(90)),
        (ref_servo("REF_SG90_Neck"),       Tz(0, 0, NECK_FACE_Z - SG_EAR_SET) @ M_neck),
        (ref_horn("REF_Horn_Shoulder_R"),  Tz(TORSO_W*0.5 - SG_EAR_SET, 0, SH_AXIS_Z)),
        (ref_horn("REF_Horn_Shoulder_L"),  Tz(-(TORSO_W*0.5 - SG_EAR_SET), 0, SH_AXIS_Z) @ Rz(180)),
        (ref_horn("REF_Horn_Elbow_R"),     Tz(x_el - SG_EAR_SET, 0, EL_AXIS_Z) @ Rx(90)),
        (ref_horn("REF_Horn_Elbow_L"),     Tz(-(x_el - SG_EAR_SET), 0, EL_AXIS_Z) @ Rz(180) @ Rx(90)),
        (ref_horn("REF_Horn_Neck"),        Tz(0, 0, NECK_FACE_Z - SG_EAR_SET) @ M_neck),
        (ref_uno("REF_Arduino_Uno"),       Tz(0.0, -27.0 + UNO_W*0.5 + FIT, BOARD_Z + UNO_T*0.5)),
    ]
    for ob, M in refs:
        bpy.context.scene.collection.objects.unlink(ob)
        col_ref.objects.link(ob)
        ob.matrix_world = M

    setup_scene_extras()
    # раскладка печатных деталей скрыта из рендера, чтобы вид по умолчанию (F12)
    # показывал собранного робота; в окне Blender раскладка видна всегда
    col_bed.hide_render = True
    bpy.ops.wm.save_as_mainfile(filepath="robot.blend")
    report()


def report():
    print("")
    print("=" * 114)
    print("ROBOT.BLEND -- объекты и габариты (мм).  Единицы сцены: METRIC / MILLIMETERS / scale 0.001")
    print("=" * 114)
    printed = sorted([o for o in bpy.data.objects if o.get("kind") == "printed"],
                     key=lambda o: o.name)
    tot_nm = 0
    print(f"{'печатная деталь':<18}{'X, мм':>9}{'Y, мм':>9}{'Z, мм':>9}{'V, см3':>10}{'треуг.':>9}{'не-ман.':>9}")
    print("-" * 114)
    for ob in printed:
        bb = bbox(ob)
        tris, nm, vol = mesh_stats(ob)
        tot_nm += nm
        print(f"{ob.name:<18}{bb[3]-bb[0]:>9.1f}{bb[4]-bb[1]:>9.1f}{bb[5]-bb[2]:>9.1f}"
              f"{vol/1000:>10.2f}{tris:>9}{nm:>9}")
    print("-" * 114)
    print(f"печатных экземпляров: {len(printed)}   суммарно не-манифолдных рёбер: {tot_nm}")
    allbb = [bbox(o) for o in printed]
    print(f"габарит раскладки на столе: {max(b[3] for b in allbb)-min(b[0] for b in allbb):.0f} x "
          f"{max(b[4] for b in allbb)-min(b[1] for b in allbb):.0f} мм")
    print("-" * 114)
    print("ФАНТОМЫ (не печатаются, показаны в сборе):")
    for ob in sorted([o for o in bpy.data.objects if o.get("kind") == "ref"], key=lambda o: o.name):
        bb = bbox(ob)
        print(f"   {ob.name:<24}{bb[3]-bb[0]:>8.1f}{bb[4]-bb[1]:>8.1f}{bb[5]-bb[2]:>8.1f}")
    print("=" * 114)
    print("ГАБАРИТЫ РОБОТА В СБОРЕ:")
    print(f"   полная высота ................. {TOTAL_HEIGHT:.1f} мм   (ТЗ 250...300)")
    print(f"   тело  Ш x Г x В ............... {TORSO_W:.1f} x {TORSO_D:.1f} x {TORSO_H:.1f} мм"
          f"   (ТЗ 80-100 / 50-70 / 90-130)")
    print(f"   ширина с руками ............... {2*(TORSO_W*0.5+ARM_LIMB_W+FA_LIMB_W):.1f} мм")
    print(f"   стойка ног .................... {2*(LEG_X+FOOT_W*0.5):.1f} x {FOOT_D:.1f} мм")
    print(f"   стенки {WALL} мм; зазоры FDM: съёмные {FIT}, посадка сервы {SG_FIT}, натяг пробок {PRESS}")
    print("=" * 114)


if __name__ == "__main__":
    main()
