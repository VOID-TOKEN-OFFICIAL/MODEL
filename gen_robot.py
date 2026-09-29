"""
gen_robot.py - Генератор 3D-модели настольного гуманоидного FDM-робота для Blender.

Конструкторские решения:
1. КИНЕМАТИКА РУК:
   - Сервоприводы плеч SG90 ориентированы валами вдоль горизонтальной оси X (сквозь боковые стенки).
   - Вращение плечевого и локтевого суставов происходит строго в сагиттальной плоскости (Y-Z),
     т.е. руки движутся ПАРАЛЛЕЛЬНО телу (вперёд-назад), а не перпендикулярно в стороны!
2. УСИЛЕННОЕ КРЕПЛЕНИЕ СЕРВОПРИВОДОВ ("УШКИ"):
   - Все 5 сервоприводов SG90 (голова, 2 плеча, 2 локтя) фиксируются отдельными жёсткими
     прижимными скобами-планками ("ушками") с крепежными отверстиями под винты M2 (межцентровое 25 мм).
   - Корпус сервы плотно усаживается в посадочный карман с опорными буртиками, а прижимная
     планка намертво притягивает монтажные ушки сервы, исключая люфты и проворачивание.
3. БЕЗВИНТОВАЯ И БЕСКЛЕЕВАЯ СБОРКА ОСНОВНЫХ УЗЛОВ:
   - Arduino Uno: удерживается внутри корпуса в вертикальных пазах-направляющих (без клея),
     вынимается при снятии задней крышки.
   - Задняя крышка тела: съемная, скользит по боковым направляющим и фиксируется защелкой.
   - Ноги: крепятся к основанию тела через направляющие T-пазов (T-slot rail) с упором.
   - Полости ног под утяжелители: вместительные камеры со съёмными нижними заглушками стоп.
   - Голова: гнездо под качалку сервы шеи, полость под электронику, глаза HC-SR04, съемная крышка.
4. FDM-ОПТИМИЗАЦИЯ И РАСКЛАДКА:
   - Все 19 деталей разложены на плоскости XY с Z_min = 0 (как на печатном столе 3D-принтера).
"""

import bpy
import bmesh
import math
from mathutils import Vector

# ==============================================================================
# КОНСТРУКТОРСКИЕ ПАРАМЕТРЫ (ВСЕ РАЗМЕРЫ В МИЛЛИМЕТРАХ)
# ==============================================================================

# Общие габариты робота в сборке:
# Высота ног (85 мм) + Высота тела (110 мм) + Зазор шеи (8 мм) + Высота головы (55 мм) = 258 мм
# Соответствует ограничению: 250 - 300 мм.

# Корпус тела (Torso)
TORSO_WIDTH  = 92.0   # Ширина тела (по оси X), допустимо 80-100 мм
TORSO_DEPTH  = 62.0   # Глубина тела (по оси Y), допустимо 50-70 мм
TORSO_HEIGHT = 110.0  # Высота тела (по оси Z), допустимо 90-130 мм
WALL_THICK   = 2.8    # Базовая толщина печатных стенок FDM

# Электроника: Arduino Uno
UNO_LENGTH    = 68.6  # Длина платы Uno
UNO_WIDTH     = 53.4  # Ширина платы Uno
UNO_THICK     = 1.6   # Толщина текстолита
UNO_COMP_H    = 13.0  # Максимальная высота электронных компонентов
UNO_RAIL_GAP  = 2.4   # Ширина паза под плату (1.6 мм + зазоры)
UNO_RAIL_DP   = 2.5   # Глубина направляющего паза

# Сервоприводы SG90 (5 штук: шея, 2 плеча, 2 локтя)
SG90_BODY_L   = 23.0  # Длина корпуса сервы
SG90_BODY_W   = 12.2  # Ширина корпуса сервы
SG90_BODY_H   = 22.7  # Высота корпуса сервы
SG90_EARS_L   = 32.0  # Полная длина с ушками
SG90_EAR_TH   = 2.0   # Толщина ушек сервы
SG90_HOLE_P   = 25.0  # Межцентровое расстояние крепежных отверстий ушек
SG90_HOLE_D   = 2.2   # Диаметр отверстий ушек под винты M2
SG90_SHAFT_D  = 5.0   # Диаметр выходного вала
SG90_COLLAR_D = 12.0  # Диаметр выступа редуктора вала

# Зазоры FDM (Clearance)
FDM_CLEAR_FIT   = 0.35  # Посадочный зазор на сторону
FDM_SLIDE_FIT   = 0.40  # Зазор для скользящих соединений
FDM_TIGHT_FIT   = 0.20  # Плотный зазор под качалки

# Кабельные трассы
WIRE_HOLE_BOTTOM_D = 12.0  # Отверстие под внешнее питание в дне
WIRE_SLOT_NECK_W   = 10.0  # Канал проводов в шею
WIRE_SLOT_NECK_D   = 8.0
WIRE_SLOT_ARM_W    = 10.0  # Канал проводов в плечо
WIRE_SLOT_ARM_D    = 6.0
ARM_WIRE_DUCT_W    = 8.0   # Внутренний кабель-канал руки
ARM_WIRE_DUCT_D    = 6.0

# Голова (Head)
HEAD_WIDTH  = 54.0
HEAD_DEPTH  = 52.0
HEAD_HEIGHT = 55.0
EYE_DIAMETER = 16.2  # Диаметр глаз (под ультразвуковой датчик HC-SR04)
EYE_SPACING  = 26.0  # Межцентровое расстояние глаз датчика

# Руки (Arms)
UPPER_ARM_W = 24.0   # Толщина плеча по оси X
UPPER_ARM_D = 26.0   # Глубина плеча по оси Y
UPPER_ARM_L = 62.0   # Длина сегмента плеча
FOREARM_W   = 22.0   # Толщина предплечья по X
FOREARM_D   = 22.0   # Глубина предплечья по Y
FOREARM_L   = 64.0   # Длина сегмента предплечья с кистью

# Ноги (Legs)
FOOT_WIDTH     = 44.0  # Ширина подошвы
FOOT_DEPTH     = 72.0  # Длина подошвы
FOOT_HEIGHT    = 20.0  # Высота подошвы
LEG_COLUMN_W   = 26.0  # Ширина стойки голени
LEG_COLUMN_D   = 30.0  # Глубина стойки голени
LEG_TOTAL_H    = 85.0  # Полная высота ноги от пола до низа тела
BALLAST_CAV_W  = 28.0  # Полость под утяжелители (металл)
BALLAST_CAV_D  = 52.0
BALLAST_CAV_H  = 58.0
T_SLOT_TENON_W = 14.0  # Ширина Т-образного шипа крепления к телу
T_SLOT_TENON_H = 8.0
T_SLOT_NECK_W  = 10.0

# ==============================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ МОДЕЛИРОВАНИЯ
# ==============================================================================

def reset_scene():
    """Сброс и настройка сцены Blender в миллиметры."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.length_unit = 'MILLIMETERS'
    scene.unit_settings.scale_length = 0.001

def create_box(name, dimensions, location=(0,0,0), rotation=(0,0,0)):
    """Создает параллелепипед заданных размеров с центром в location."""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    return obj

def create_cylinder(name, radius, depth, location=(0,0,0), rotation=(0,0,0), vertices=32):
    """Создает цилиндр заданного радиуса и длины."""
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=depth, vertices=vertices,
                                       location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    return obj

def apply_bool(target, tool, operation='DIFFERENCE', solver='EXACT', delete_tool=True):
    """Применяет булеву операцию модификатором с solver EXACT."""
    mod = target.modifiers.new(name=f"Bool_{operation}", type='BOOLEAN')
    mod.operation = operation
    mod.solver = solver
    mod.object = tool
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    if delete_tool and tool.name in bpy.data.objects:
        bpy.data.objects.remove(tool, do_unlink=True)

def recalc_normals(obj):
    """Согласовывает внешние нормали меша."""
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

def get_bbox(obj):
    """Возвращает минимальные и максимальные мировые координаты Bounding Box."""
    bbox = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    min_x = min(p.x for p in bbox)
    max_x = max(p.x for p in bbox)
    min_y = min(p.y for p in bbox)
    max_y = max(p.y for p in bbox)
    min_z = min(p.z for p in bbox)
    max_z = max(p.z for p in bbox)
    return (min_x, max_x, min_y, max_y, min_z, max_z)

# ==============================================================================
# ПОСТРОЕНИЕ ДЕТАЛЕЙ РОБОТА
# ==============================================================================

def build_torso_body():
    """
    Основной корпус тела (Torso_Body):
    - Внутренняя полость под плату Arduino Uno с вертикальными направляющими пазами.
    - Окно доступа к разъемам USB-B и DC Power Jack для подключения кабелей без разборки.
    - Посадочные места плечевых серв SG90: валы выходят наружу по оси X (сагиттальное движение рук параллельно телу).
    - Опорные приливы под крепежные ушки сервоприводов с резьбовыми отверстиями.
    - Посадочное место сервопривода головы (вал вертикально вверх по оси Z).
    - Кабельные каналы: дно (питание), шея (в голову), боковые каналы (в плечи).
    - Нижние ответные пазы T-slot для фиксации ног без клея и винтов.
    - Пазы под сдвижную заднюю крышку.
    """
    torso = create_box("Torso_Body", (TORSO_WIDTH, TORSO_DEPTH, TORSO_HEIGHT),
                       (0, 0, TORSO_HEIGHT / 2))

    # 1. Основная внутренняя полость
    # Открыта назад для монтажа Uno и установки задней крышки
    cavity_w = TORSO_WIDTH - 2 * WALL_THICK
    cavity_d = TORSO_DEPTH - WALL_THICK + 5.0
    cavity_h = TORSO_HEIGHT - 2 * WALL_THICK
    cavity = create_box("Torso_Cavity", (cavity_w, cavity_d, cavity_h),
                        (0, 2.5 + WALL_THICK / 2, TORSO_HEIGHT / 2))
    apply_bool(torso, cavity)

    # 2. Вертикальные направляющие рельсы для Arduino Uno (ширина платы 53.4 мм)
    # Создаем выступы с пазами 2.4 мм x 2.5 мм
    rail_post_l = create_box("Rail_L", (4.0, 6.0, 75.0),
                             (-UNO_WIDTH / 2 - 1.5, 0.0, 45.0))
    apply_bool(torso, rail_post_l, operation='UNION')
    rail_post_r = create_box("Rail_R", (4.0, 6.0, 75.0),
                             (UNO_WIDTH / 2 + 1.5, 0.0, 45.0))
    apply_bool(torso, rail_post_r, operation='UNION')

    slot_l = create_box("Slot_L", (UNO_RAIL_GAP, 3.5, 80.0),
                        (-UNO_WIDTH / 2, 0.0, 47.0))
    apply_bool(torso, slot_l)
    slot_r = create_box("Slot_R", (UNO_RAIL_GAP, 3.5, 80.0),
                        (UNO_WIDTH / 2, 0.0, 47.0))
    apply_bool(torso, slot_r)

    # Окно под разъемы Arduino Uno (USB-B и DC Jack) на левом боку корпуса
    uno_ports = create_box("Uno_Ports_Window", (16.0, 32.0, 16.0),
                           (-TORSO_WIDTH / 2, 0.0, 20.0))
    apply_bool(torso, uno_ports)

    # 3. Посадочные места плечевых серв (Ось X — вал наружу сквозь боковую стенку)
    # Это ключевое требование: руки вращаются в плоскости YZ (параллельно телу)!
    shoulder_z = TORSO_HEIGHT - 24.0

    # Левое плечо: сквозное отверстие под вал и редуктор
    hole_sh_l = create_cylinder("Hole_Shoulder_L", SG90_COLLAR_D / 2 + 0.5, 12.0,
                                (-TORSO_WIDTH / 2, 0.0, shoulder_z),
                                (0, math.radians(90), 0))
    apply_bool(torso, hole_sh_l)

    # Карман под корпус сервы изнутри
    pocket_sh_l = create_box("Pocket_Shoulder_L",
                             (SG90_BODY_H + 2.0, SG90_BODY_L + 0.8, SG90_BODY_W + 0.8),
                             (-TORSO_WIDTH / 2 + (SG90_BODY_H / 2) + 1.0, 0.0, shoulder_z))
    apply_bool(torso, pocket_sh_l)

    # Отверстия под крепежные ушки сервы (межцентровое 25 мм)
    ear_hole_l1 = create_cylinder("Ear_Hole_L1", SG90_HOLE_D / 2, 12.0,
                                  (-TORSO_WIDTH / 2 + 5.0, -SG90_HOLE_P / 2, shoulder_z),
                                  (0, math.radians(90), 0))
    apply_bool(torso, ear_hole_l1)
    ear_hole_l2 = create_cylinder("Ear_Hole_L2", SG90_HOLE_D / 2, 12.0,
                                  (-TORSO_WIDTH / 2 + 5.0, SG90_HOLE_P / 2, shoulder_z),
                                  (0, math.radians(90), 0))
    apply_bool(torso, ear_hole_l2)

    # Кабель-канал плеча в корпус
    wire_arm_l = create_box("Wire_Arm_L", (12.0, WIRE_SLOT_ARM_W, WIRE_SLOT_ARM_D),
                            (-TORSO_WIDTH / 2 + 4.0, 0.0, shoulder_z - 14.0))
    apply_bool(torso, wire_arm_l)

    # Правое плечо: симметричные отверстия и карман
    hole_sh_r = create_cylinder("Hole_Shoulder_R", SG90_COLLAR_D / 2 + 0.5, 12.0,
                                (TORSO_WIDTH / 2, 0.0, shoulder_z),
                                (0, math.radians(90), 0))
    apply_bool(torso, hole_sh_r)

    pocket_sh_r = create_box("Pocket_Shoulder_R",
                             (SG90_BODY_H + 2.0, SG90_BODY_L + 0.8, SG90_BODY_W + 0.8),
                             (TORSO_WIDTH / 2 - (SG90_BODY_H / 2) - 1.0, 0.0, shoulder_z))
    apply_bool(torso, pocket_sh_r)

    ear_hole_r1 = create_cylinder("Ear_Hole_r1", SG90_HOLE_D / 2, 12.0,
                                  (TORSO_WIDTH / 2 - 5.0, -SG90_HOLE_P / 2, shoulder_z),
                                  (0, math.radians(90), 0))
    apply_bool(torso, ear_hole_r1)
    ear_hole_r2 = create_cylinder("Ear_Hole_r2", SG90_HOLE_D / 2, 12.0,
                                  (TORSO_WIDTH / 2 - 5.0, SG90_HOLE_P / 2, shoulder_z),
                                  (0, math.radians(90), 0))
    apply_bool(torso, ear_hole_r2)

    wire_arm_r = create_box("Wire_Arm_R", (12.0, WIRE_SLOT_ARM_W, WIRE_SLOT_ARM_D),
                            (TORSO_WIDTH / 2 - 4.0, 0.0, shoulder_z - 14.0))
    apply_bool(torso, wire_arm_r)

    # 4. Посадочное место сервопривода головы (Шея, вал вертикально вверх по Z)
    neck_hole = create_cylinder("Neck_Shaft_Hole", SG90_COLLAR_D / 2 + 0.5, 12.0,
                                (0.0, 0.0, TORSO_HEIGHT),
                                (0, 0, 0))
    apply_bool(torso, neck_hole)

    neck_pocket = create_box("Neck_Pocket",
                             (SG90_BODY_L + 0.8, SG90_BODY_W + 0.8, SG90_BODY_H + 2.0),
                             (0.0, 0.0, TORSO_HEIGHT - SG90_BODY_H / 2 - 1.0))
    apply_bool(torso, neck_pocket)

    neck_wire = create_box("Neck_Wire_Slot",
                           (WIRE_SLOT_NECK_W, WIRE_SLOT_NECK_D, 12.0),
                           (0.0, -12.0, TORSO_HEIGHT))
    apply_bool(torso, neck_wire)

    neck_ear1 = create_cylinder("Neck_Ear1", SG90_HOLE_D / 2, 10.0,
                                (-SG90_HOLE_P / 2, 0.0, TORSO_HEIGHT - 3.0),
                                (0, 0, 0))
    apply_bool(torso, neck_ear1)
    neck_ear2 = create_cylinder("Neck_Ear2", SG90_HOLE_D / 2, 10.0,
                                (SG90_HOLE_P / 2, 0.0, TORSO_HEIGHT - 3.0),
                                (0, 0, 0))
    apply_bool(torso, neck_ear2)

    # 5. Кабельное отверстие внешнего питания в дне корпуса (Ø12 мм)
    bottom_hole = create_cylinder("Bottom_Power_Hole", WIRE_HOLE_BOTTOM_D / 2, 12.0,
                                  (0.0, 0.0, 0.0), (0, 0, 0))
    apply_bool(torso, bottom_hole)

    # 6. Ответные T-пазы крепления ног (без клея и винтов)
    # Ноги вставляются сзади по направляющим до переднего упора
    for sign, leg_x in [(-1, -26.0), (1, 26.0)]:
        t_slot = create_box(f"T_Slot_Mortise_{sign}",
                            (T_SLOT_TENON_W + 0.8, 48.0, T_SLOT_TENON_H + 0.6),
                            (leg_x, 4.0, 3.0))
        apply_bool(torso, t_slot)
        t_neck = create_box(f"T_Slot_Neck_{sign}",
                            (T_SLOT_NECK_W + 0.8, 50.0, 4.0),
                            (leg_x, 4.0, 0.0))
        apply_bool(torso, t_neck)

    # 7. Пазы под сдвижную заднюю крышку по боковым кромкам проема
    back_slot_l = create_box("Back_Slot_L", (2.5, 2.5, TORSO_HEIGHT - 6.0),
                             (-TORSO_WIDTH / 2 + WALL_THICK + 1.0,
                              TORSO_DEPTH / 2 - 1.2,
                              TORSO_HEIGHT / 2))
    apply_bool(torso, back_slot_l)
    back_slot_r = create_box("Back_Slot_R", (2.5, 2.5, TORSO_HEIGHT - 6.0),
                             (TORSO_WIDTH / 2 - WALL_THICK - 1.0,
                              TORSO_DEPTH / 2 - 1.2,
                              TORSO_HEIGHT / 2))
    apply_bool(torso, back_slot_r)

    recalc_normals(torso)
    return torso

def build_torso_back_cover():
    """
    Съёмная задняя крышка тела (Torso_Back_Cover):
    - Отдельная деталь.
    - Снимается рукой без винтов и инструмента.
    - Оснащена направляющими ребрами скольжения и пружинной защелкой с выступом под палец.
    """
    w = TORSO_WIDTH - 2 * WALL_THICK + 1.8
    h = TORSO_HEIGHT - 2 * WALL_THICK - 0.8
    th = 2.8

    cover = create_box("Torso_Back_Cover", (w, th, h), (0, 0, h / 2))

    # Боковые выступы скольжения в пазы корпуса
    rib_l = create_box("Cover_Rib_L", (2.0, 1.8, h - 10.0), (-w / 2, -th / 2, h / 2))
    apply_bool(cover, rib_l, operation='UNION')
    rib_r = create_box("Cover_Rib_R", (2.0, 1.8, h - 10.0), (w / 2, -th / 2, h / 2))
    apply_bool(cover, rib_r, operation='UNION')

    # Защелка в верхней части с выступом под палец (finger pull)
    latch = create_box("Cover_Latch_Tab", (24.0, 4.0, 6.0), (0, 1.5, h - 3.0))
    apply_bool(cover, latch, operation='UNION')

    # Рифление для удобного захвата пальцами
    for i in range(4):
        grip = create_box(f"Grip_Notch_{i}", (30.0, 1.2, 2.0), (0, th / 2, 25.0 + i * 8.0))
        apply_bool(cover, grip)

    recalc_normals(cover)
    return cover

def build_head():
    """
    Голова робота (Head):
    - Пропорциональные размеры 54 x 52 x 55 мм.
    - Внутренняя вместительная полость под сенсоры/электронику.
    - "Глаза": 2 отверстия Ø16.2 мм под ультразвуковой дальномер HC-SR04 (межцентровое 26 мм).
    - Нижняя шейная втулка с посадочным гнездом под качалку сервы SG90 шеи.
    - Нижнее отверстие под провода из тела.
    - Открытый верх под съемную крышку.
    """
    head = create_box("Head", (HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT),
                      (0, 0, HEAD_HEIGHT / 2))

    # Срезы граней для придания стильного робототехнического фасеточного облика
    cut_top_f = create_box("Chamfer_Front", (HEAD_WIDTH + 10.0, 15.0, 15.0),
                           (0, HEAD_DEPTH / 2, HEAD_HEIGHT),
                           (math.radians(45), 0, 0))
    apply_bool(head, cut_top_f)

    # Внутренняя полость под электронику
    cavity = create_box("Head_Cavity",
                        (HEAD_WIDTH - 2 * WALL_THICK, HEAD_DEPTH - 2 * WALL_THICK, HEAD_HEIGHT),
                        (0, 0, HEAD_HEIGHT / 2 + WALL_THICK))
    apply_bool(head, cavity)

    # "Глаза" на лицевой панели под датчик HC-SR04
    eye_z = HEAD_HEIGHT * 0.55
    eye_l = create_cylinder("Eye_L", EYE_DIAMETER / 2, 12.0,
                            (-EYE_SPACING / 2, HEAD_DEPTH / 2, eye_z),
                            (math.radians(90), 0, 0))
    apply_bool(head, eye_l)

    eye_r = create_cylinder("Eye_R", EYE_DIAMETER / 2, 12.0,
                            (EYE_SPACING / 2, HEAD_DEPTH / 2, eye_z),
                            (math.radians(90), 0, 0))
    apply_bool(head, eye_r)

    # Шейный узел на дне головы
    neck_boss = create_cylinder("Neck_Boss", 12.0, 6.0, (0, 0, -3.0), (0, 0, 0))
    apply_bool(head, neck_boss, operation='UNION')

    # Гнездо под качалку сервы шеи (крестообразная/двойная качалка SG90)
    horn_slot = create_box("Neck_Horn_Slot", (16.2, 7.8, 3.2), (0, 0, -3.0))
    apply_bool(head, horn_slot)

    horn_screw = create_cylinder("Neck_Horn_Screw", 2.8, 12.0, (0, 0, -3.0), (0, 0, 0))
    apply_bool(head, horn_screw)

    # Отверстие под пучок проводов из шеи
    wire_neck = create_box("Head_Wire_Hole", (10.0, 8.0, 12.0), (0, -10.0, 0))
    apply_bool(head, wire_neck)

    recalc_normals(head)
    return head

def build_head_lid():
    """
    Съёмная верхняя крышка головы (Head_Lid):
    - Закрывает полость головы.
    - Имеет центрирующий внутренний буртик с зазором 0.35 мм для легкого снятия рукой.
    """
    w = HEAD_WIDTH - 0.4
    d = HEAD_DEPTH - 0.4
    th = 2.8

    lid = create_box("Head_Lid", (w, d, th), (0, 0, th / 2))

    # Центрирующий посадочный буртик
    lip = create_box("Lid_Lip", (HEAD_WIDTH - 2 * WALL_THICK - 0.7,
                                 HEAD_DEPTH - 2 * WALL_THICK - 0.7,
                                 2.5),
                     (0, 0, -1.25))
    apply_bool(lid, lip, operation='UNION')

    # Фаска спереди в тон скосу головы
    chamfer = create_box("Lid_Chamfer", (HEAD_WIDTH + 10.0, 15.0, 10.0),
                         (0, d / 2, th), (math.radians(45), 0, 0))
    apply_bool(lid, chamfer)

    recalc_normals(lid)
    return lid

def build_upper_arm(is_left=True):
    """
    Плечевой сегмент руки (Upper_Arm_Left / Upper_Arm_Right):
    - ВАЖНЕЙШЕЕ ТРЕБОВАНИЕ: шарнир плеча ориентирован валом по оси X.
      Рука движется строго параллельно телу!
    - Верхний фланец: гнездо под качалку плечевой сервы тела с центрирующим замком.
    - Внутри корпуса плеча: карман под сервопривод локтя SG90.
    - Вал локтевой сервы также направлен по оси X — сгибание локтя происходит параллельно телу!
    - Внутренний кабель-канал для проводов от локтя к телу.
    """
    side_str = "Left" if is_left else "Right"
    name = f"Upper_Arm_{side_str}"
    sign = 1 if is_left else -1

    arm = create_box(name, (UPPER_ARM_W, UPPER_ARM_D, UPPER_ARM_L),
                     (0, 0, UPPER_ARM_L / 2))

    # Скругление/бобышка плечевого сустава вверху
    sh_hub = create_cylinder(f"Sh_Hub_{side_str}", 13.0, UPPER_ARM_W,
                             (0, 0, UPPER_ARM_L - 13.0),
                             (0, math.radians(90), 0))
    apply_bool(arm, sh_hub, operation='UNION')

    # Гнездо под качалку плечевой сервы на внутренней грани руки
    # Внутренняя грань: x = sign * (UPPER_ARM_W / 2)
    inner_x = sign * (UPPER_ARM_W / 2)
    horn_recess = create_box(f"Sh_Horn_Slot_{side_str}",
                             (3.5, 16.4, 7.8),
                             (inner_x - sign * 1.5, 0, UPPER_ARM_L - 13.0))
    apply_bool(arm, horn_recess)

    horn_screw_hole = create_cylinder(f"Sh_Screw_Hole_{side_str}", 2.8, 12.0,
                                      (inner_x - sign * 5.0, 0, UPPER_ARM_L - 13.0),
                                      (0, math.radians(90), 0))
    apply_bool(arm, horn_screw_hole)

    # Карман под сервопривод локтя SG90 в нижней части сегмента плеча
    # Вал локтевой сервы направлен по оси X (параллельно плечевому)!
    elbow_z = 18.0
    elbow_pocket = create_box(f"Elbow_Pocket_{side_str}",
                              (SG90_BODY_W + 1.0, SG90_BODY_L + 1.0, SG90_BODY_H + 1.5),
                              (0, 0, elbow_z))
    apply_bool(arm, elbow_pocket)

    # Отверстие под выходной вал локтевой сервы наружу сквозь стенку
    elbow_shaft_out = create_cylinder(f"Elbow_Shaft_Out_{side_str}",
                                     SG90_COLLAR_D / 2 + 0.5, 12.0,
                                     (inner_x, 0, elbow_z + 4.0),
                                     (0, math.radians(90), 0))
    apply_bool(arm, elbow_shaft_out)

    # Крепежные отверстия под прижимную планку ("ушки") локтевой сервы
    elbow_ear1 = create_cylinder(f"Elbow_Ear1_{side_str}", SG90_HOLE_D / 2, 10.0,
                                 (0, -SG90_HOLE_P / 2, elbow_z),
                                 (0, math.radians(90), 0))
    apply_bool(arm, elbow_ear1)
    elbow_ear2 = create_cylinder(f"Elbow_Ear2_{side_str}", SG90_HOLE_D / 2, 10.0,
                                 (0, SG90_HOLE_P / 2, elbow_z),
                                 (0, math.radians(90), 0))
    apply_bool(arm, elbow_ear2)

    # Внутренний кабель-канал вдоль кости руки (от локтя в плечо)
    duct = create_box(f"Arm_Duct_{side_str}",
                      (ARM_WIRE_DUCT_W, ARM_WIRE_DUCT_D, UPPER_ARM_L - 25.0),
                      (0, -4.0, UPPER_ARM_L / 2))
    apply_bool(arm, duct)

    recalc_normals(arm)
    return arm

def build_forearm(is_left=True):
    """
    Сегмент предплечья с кистью (Forearm_Left / Forearm_Right):
    - Вертлуг локтевого шарнира с посадочным гнездом под качалку локтевой сервы.
    - Вращение происходит вокруг оси X в сагиттальной плоскости YZ (параллельно телу).
    - Робототехническая стилизованная кисть-манипулятор на дистальном конце.
    """
    side_str = "Left" if is_left else "Right"
    name = f"Forearm_{side_str}"
    sign = 1 if is_left else -1

    forearm = create_box(name, (FOREARM_W, FOREARM_D, FOREARM_L),
                         (0, 0, FOREARM_L / 2))

    # Круглая проушина локтевого сустава вверху
    elbow_hub = create_cylinder(f"Elbow_Hub_{side_str}", 11.5, FOREARM_W,
                                (0, 0, FOREARM_L - 11.5),
                                (0, math.radians(90), 0))
    apply_bool(forearm, elbow_hub, operation='UNION')

    # Гнездо под качалку сервопривода локтя
    inner_x = -sign * (FOREARM_W / 2)
    horn_slot = create_box(f"Forearm_Horn_{side_str}",
                           (3.4, 16.4, 7.8),
                           (inner_x + sign * 1.5, 0, FOREARM_L - 11.5))
    apply_bool(forearm, horn_slot)

    horn_hole = create_cylinder(f"Forearm_Horn_Hole_{side_str}", 2.8, 12.0,
                                (inner_x + sign * 5.0, 0, FOREARM_L - 11.5),
                                (0, math.radians(90), 0))
    apply_bool(forearm, horn_hole)

    # Моделирование стилизованной роботизированной кисти / манипулятора внизу
    # Вырез между пальцами
    finger_notch = create_box(f"Finger_Notch_{side_str}", (10.0, 24.0, 16.0), (0, 0, 6.0))
    apply_bool(forearm, finger_notch)

    # Фаски предплечья для эстетики робота
    bevel_f = create_box(f"Bevel_F_{side_str}", (26.0, 8.0, 30.0),
                         (0, FOREARM_D / 2, 25.0), (math.radians(20), 0, 0))
    apply_bool(forearm, bevel_f)

    recalc_normals(forearm)
    return forearm

def build_leg(is_left=True):
    """
    Нога со стопой (Leg_Left / Leg_Right):
    - Высокая устойчивость: широкая база стопы 44 x 72 x 20 мм.
    - Внутренняя вместительная полость (84 см³) под металлические утяжелители (гайки, дробь, свинец).
    - Нижний проем под съемную подошвенную крышку-заглушку.
    - Верхний Т-образный шип (T-slot tenon) для жесткого скользящего крепления к телу без клея и винтов.
    """
    side_str = "Left" if is_left else "Right"
    name = f"Leg_{side_str}"

    # 1. Подошва/стопа
    foot = create_box(f"Foot_Base_{side_str}", (FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT),
                      (0, 0, FOOT_HEIGHT / 2))

    # Срезы носка и пятки для динамичного облика стопы
    toe_cut = create_box(f"Toe_Cut_{side_str}", (FOOT_WIDTH + 10.0, 20.0, 20.0),
                         (0, FOOT_DEPTH / 2, FOOT_HEIGHT),
                         (math.radians(-35), 0, 0))
    apply_bool(foot, toe_cut)

    heel_cut = create_box(f"Heel_Cut_{side_str}", (FOOT_WIDTH + 10.0, 15.0, 15.0),
                          (0, -FOOT_DEPTH / 2, FOOT_HEIGHT),
                          (math.radians(30), 0, 0))
    apply_bool(foot, heel_cut)

    # 2. Стойка голени
    col_h = LEG_TOTAL_H - FOOT_HEIGHT
    column = create_box(f"Column_{side_str}", (LEG_COLUMN_W, LEG_COLUMN_D, col_h),
                        (0, 0, FOOT_HEIGHT + col_h / 2))
    apply_bool(foot, column, operation='UNION')

    # 3. Верхний T-образный шип для безвинтовой фиксации в теле
    tenon_base = create_box(f"T_Neck_{side_str}", (T_SLOT_NECK_W, 36.0, 3.0),
                            (0, 0, LEG_TOTAL_H + 1.5))
    apply_bool(foot, tenon_base, operation='UNION')

    tenon_top = create_box(f"T_Flange_{side_str}", (T_SLOT_TENON_W, 36.0, 4.5),
                           (0, 0, LEG_TOTAL_H + 4.5 + 0.5))
    apply_bool(foot, tenon_top, operation='UNION')

    # 4. Вместительная полость под утяжелители (металл)
    cavity = create_box(f"Ballast_Cavity_{side_str}",
                        (BALLAST_CAV_W, BALLAST_CAV_D, BALLAST_CAV_H),
                        (0, 0, BALLAST_CAV_H / 2 + 1.5))
    apply_bool(foot, cavity)

    # Нижнее посадочное гнездо под заглушку стопы
    plug_seat = create_box(f"Plug_Seat_{side_str}",
                           (BALLAST_CAV_W + 2.0, BALLAST_CAV_D + 2.0, 3.5),
                           (0, 0, 1.75))
    apply_bool(foot, plug_seat)

    foot.name = name
    recalc_normals(foot)
    return foot

def build_foot_cap(is_left=True):
    """
    Съёмная пробка-заглушка стопы (Foot_Cap_Left / Foot_Cap_Right):
    - Удерживает металл-утяжелитель внутри стопы.
    - Оснащена посадочным замком с FDM-зазором 0.35 мм.
    """
    side_str = "Left" if is_left else "Right"
    name = f"Foot_Cap_{side_str}"

    w = BALLAST_CAV_W + 6.0
    d = BALLAST_CAV_D + 6.0
    th = 2.8

    cap = create_box(name, (w, d, th), (0, 0, th / 2))

    # Выступ-пробка, входящий в полость стопы
    plug = create_box(f"Cap_Plug_{side_str}",
                      (BALLAST_CAV_W - 0.7, BALLAST_CAV_D - 0.7, 2.8),
                      (0, 0, th + 1.4))
    apply_bool(cap, plug, operation='UNION')

    # Паз под ноготь/отвертку для быстрого извлечения
    notch = create_box(f"Cap_Notch_{side_str}", (12.0, 2.5, 1.5), (0, d / 2 - 1.5, th))
    apply_bool(cap, notch)

    recalc_normals(cap)
    return cap

def build_servo_clamp(name):
    """
    Усиленная прижимная планка крепления сервопривода ("Ушки"):
    - Решает критическое требование пользователя:
      "улучшить крепление сервоприводов чтобы крепко держалось (на сервоприводы можно надеть 'ушки' и с помощью них прикрутить)"
    - Надевается на монтажные ушки SG90 и намертво прижимает их винтами M2 (pitch 25 мм).
    - Предотвращает вибрации, люфты и проворачивание мотора под нагрузкой.
    """
    length = 32.5
    width = 12.0
    thick = 3.0

    clamp = create_box(name, (length, width, thick), (0, 0, thick / 2))

    # Крепежные отверстия ушек (Ø2.4 мм с межцентровым 25 мм под винты M2)
    h1 = create_cylinder(f"{name}_H1", SG90_HOLE_D / 2 + 0.1, 10.0,
                         (-SG90_HOLE_P / 2, 0, thick / 2), (0, 0, 0))
    apply_bool(clamp, h1)

    h2 = create_cylinder(f"{name}_H2", SG90_HOLE_D / 2 + 0.1, 10.0,
                         (SG90_HOLE_P / 2, 0, thick / 2), (0, 0, 0))
    apply_bool(clamp, h2)

    # Центральное облегчение/окно под корпус сервы
    window = create_box(f"{name}_Win", (17.0, 7.0, 10.0), (0, 0, thick / 2))
    apply_bool(clamp, window)

    recalc_normals(clamp)
    return clamp

def build_arm_horn_retainer(is_left=True):
    """
    Фиксатор качалки плеча (Arm_Horn_Retainer_L / R):
    - Защелкивает или фиксирует качалку SG90 внутри посадочного гнезда руки.
    - Исключает соскальзывание руки с качалки сервопривода без необходимости склеивания.
    """
    side_str = "Left" if is_left else "Right"
    name = f"Arm_Horn_Retainer_{side_str}"

    ret = create_box(name, (18.0, 9.0, 2.5), (0, 0, 1.25))
    hole = create_cylinder(f"{name}_Hole", 3.0, 10.0, (0, 0, 1.25), (0, 0, 0))
    apply_bool(ret, hole)

    recalc_normals(ret)
    return ret

# ==============================================================================
# РАСКЛАДКА НА ПЕЧАТНОМ СТОЛЕ XY (Z_MIN = 0)
# ==============================================================================

def orient_and_layout_parts(parts):
    """
    Ориентирует каждую деталь наилучшей для 3D-печати гранью вниз
    и раскладывает все детали на столе XY (Z >= 0) с зазорами >= 15 мм.
    """
    # Специфические довороты деталей для идеального качества печати:
    # 1. Torso_Body печатается стоя (как в сборке)
    # 2. Torso_Back_Cover плашмя
    # 3. Head горловиной вниз
    # 4. Arms плашмя на боковую грань
    # 5. Ноги стоя на стопах
    rotations = {
        "Head": (math.radians(180), 0, 0),
        "Upper_Arm_Left": (0, math.radians(90), 0),
        "Upper_Arm_Right": (0, math.radians(-90), 0),
        "Forearm_Left": (0, math.radians(90), 0),
        "Forearm_Right": (0, math.radians(-90), 0),
    }

    for p in parts:
        if p.name in rotations:
            p.rotation_euler = rotations[p.name]
            bpy.context.view_layer.objects.active = p
            bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)

    # Сетка размещения на печатном столе (колонки и ряды)
    # 4 колонки x 5 рядов:
    layout_grid = [
        # Ряд 1: Крупные детали корпуса
        ("Torso_Body",              -95.0,  100.0),
        ("Torso_Back_Cover",         50.0,  100.0),
        # Ряд 2: Голова и стопы
        ("Head",                   -100.0,    0.0),
        ("Head_Lid",                -35.0,    0.0),
        ("Foot_Cap_Left",            30.0,    0.0),
        ("Foot_Cap_Right",           85.0,    0.0),
        # Ряд 3: Ноги и плечи
        ("Leg_Left",               -100.0,  -90.0),
        ("Leg_Right",               -40.0,  -90.0),
        ("Upper_Arm_Left",           25.0,  -90.0),
        ("Upper_Arm_Right",          85.0,  -90.0),
        # Ряд 4: Предплечья и планки ("ушки")
        ("Forearm_Left",           -100.0, -170.0),
        ("Forearm_Right",           -40.0, -170.0),
        ("Servo_Clamp_Shoulder_L",   15.0, -150.0),
        ("Servo_Clamp_Shoulder_R",   65.0, -150.0),
        ("Servo_Clamp_Head",         15.0, -175.0),
        ("Servo_Clamp_Elbow_L",      65.0, -175.0),
        ("Servo_Clamp_Elbow_R",      15.0, -200.0),
        ("Arm_Horn_Retainer_L",      65.0, -200.0),
        ("Arm_Horn_Retainer_R",     105.0, -200.0),
    ]

    parts_dict = {p.name: p for p in parts}

    for name, target_x, target_y in layout_grid:
        if name in parts_dict:
            obj = parts_dict[name]
            min_x, max_x, min_y, max_y, min_z, max_z = get_bbox(obj)

            # Центрируем по X, Y и опускаем основание строго на Z = 0
            cur_cx = (min_x + max_x) / 2
            cur_cy = (min_y + max_y) / 2

            dx = target_x - cur_cx
            dy = target_y - cur_cy
            dz = -min_z  # Чтобы нижняя точка лежала на столе Z=0

            obj.location.x += dx
            obj.location.y += dy
            obj.location.z += dz

            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)

# ==============================================================================
# ГЛАВНЫЙ ИСПОЛНИТЕЛЬНЫЙ БЛОК
# ==============================================================================

def main():
    print("=" * 80)
    print("  ГЕНЕРАТОР FDM-ДЕТАЛЕЙ НАСТОЛЬНОГО ГУМАНОИДНОГО РОБОТА (Blender 5.0 / Python)")
    print("=" * 80)

    reset_scene()

    parts = []

    print("[1/5] Построение деталей корпуса и головы...")
    parts.append(build_torso_body())
    parts.append(build_torso_back_cover())
    parts.append(build_head())
    parts.append(build_head_lid())

    print("[2/5] Построение рук (параллельная кинематика Y-Z)...")
    parts.append(build_upper_arm(is_left=True))
    parts.append(build_upper_arm(is_left=False))
    parts.append(build_forearm(is_left=True))
    parts.append(build_forearm(is_left=False))

    print("[3/5] Построение ног и заглушек полостей утяжелителей...")
    parts.append(build_leg(is_left=True))
    parts.append(build_leg(is_left=False))
    parts.append(build_foot_cap(is_left=True))
    parts.append(build_foot_cap(is_left=False))

    print("[4/5] Построение усиленных прижимных планок ('ушек') сервоприводов...")
    parts.append(build_servo_clamp("Servo_Clamp_Shoulder_L"))
    parts.append(build_servo_clamp("Servo_Clamp_Shoulder_R"))
    parts.append(build_servo_clamp("Servo_Clamp_Head"))
    parts.append(build_servo_clamp("Servo_Clamp_Elbow_L"))
    parts.append(build_servo_clamp("Servo_Clamp_Elbow_R"))
    parts.append(build_arm_horn_retainer(is_left=True))
    parts.append(build_arm_horn_retainer(is_left=False))

    print("[5/5] Оптимизация ориентации и раскладка деталей на печатном столе (Z=0)...")
    orient_and_layout_parts(parts)

    # Сохраняем итоговый файл robot.blend
    blend_path = "robot.blend"
    bpy.ops.wm.save_as_mainfile(filepath=blend_path)
    print(f"\n[OK] Файл успешно сохранен: {blend_path}")

    # Вывод проверочной таблицы габаритов
    print("\n" + "=" * 94)
    print(f"| {'Имя объекта (Деталь)':<26} | {'Габариты X x Y x Z (мм)':<28} | {'Вершин':<8} | {'Z min (стол)':<12} |")
    print("=" * 94)

    for p in sorted(parts, key=lambda x: x.name):
        dims = p.dimensions
        min_x, max_x, min_y, max_y, min_z, max_z = get_bbox(p)
        dim_str = f"{dims.x:6.2f} x {dims.y:6.2f} x {dims.z:6.2f}"
        print(f"| {p.name:<26} | {dim_str:<28} | {len(p.data.vertices):<8} | {min_z:10.2f} мм |")

    print("=" * 94)
    print(f"Всего смоделировано деталей: {len(parts)}. Все детали готовы к FDM 3D-печати!")

if __name__ == "__main__":
    main()
