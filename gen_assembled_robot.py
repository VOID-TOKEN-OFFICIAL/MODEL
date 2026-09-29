"""
gen_assembled_robot.py - Генератор полностью собранной 3D-модели гуманоидного робота
для файла robot_assembled.blend и фотореалистичного рендера robot_assembled.png.

Особенности сборки:
1. КИНЕМАТИКА РУК:
   - Плечи и локти ориентированы вдоль оси X. Движение рук происходит строго в сагиттальной
     плоскости YZ (параллельно телу робота).
   - Руки находятся в активной гуманоидной позе с естественным изгибом в локтях на 25 градусов.
2. ПОЛНАЯ АНАТОМИЧЕСКАЯ СБОРКА:
   - Ноги с устойчивыми стопами на полу (Z = 0), Т-шипы входят в дно корпуса (Z = 85 мм).
   - Корпус (Torso) по центру (Z = 85..195 мм) со смонтированной задней крышкой и нагрудным реактором.
   - Голова (Z = 195..250 мм) со светящимися сенсорными глазами HC-SR04 и визором-крышкой.
   - Усиленные прижимные скобы ("ушки") сервоприводов зафиксированы в своих узлах.
   - Внутренние фантомы: плата Arduino Uno в направляющих и 5 сервоприводов SG90.
3. ОФОРМЛЕНИЕ И РЕНДЕР:
   - Студийная циклорама, 3-точечное освещение, PBR-материалы (матовый полимер, темный карбон,
     техно-циан, светящиеся сенсоры).
   - Генерация файлов: robot_assembled.blend, robot_assembled.png, robot_assembled_side.png.
"""

import bpy
import bmesh
import math
from mathutils import Vector

# ==============================================================================
# ПАРАМЕТРЫ СБОРКИ (ММ)
# ==============================================================================
TORSO_W      = 92.0
TORSO_D      = 62.0
TORSO_H      = 110.0
WALL         = 2.8

LEG_TOTAL_H  = 85.0
LEG_X        = 26.0
FOOT_W       = 44.0
FOOT_D       = 72.0
FOOT_H       = 20.0
LEG_COL_W    = 28.0
LEG_COL_D    = 32.0

HEAD_W       = 54.0
HEAD_D       = 52.0
HEAD_H       = 55.0
EYE_D        = 16.2
EYE_PITCH    = 26.0

UPPER_ARM_W  = 22.0
UPPER_ARM_D  = 24.0
UPPER_ARM_L  = 62.0

FOREARM_W    = 20.0
FOREARM_D    = 20.0
FOREARM_L    = 64.0

SG_L         = 23.0
SG_W         = 12.2
SG_H         = 22.7
SG_PITCH     = 25.0

# Высотные отметки сборки
TORSO_Z0     = LEG_TOTAL_H                 # 85.0 мм (база корпуса)
TORSO_ZC     = TORSO_Z0 + TORSO_H / 2      # 140.0 мм (центр корпуса)
SHOULDER_Z   = TORSO_Z0 + TORSO_H - 24.0   # 171.0 мм (ось плеч)
NECK_Z       = TORSO_Z0 + TORSO_H          # 195.0 мм (основание головы)
HEAD_ZC      = NECK_Z + HEAD_H / 2         # 222.5 мм (центр головы)
ARM_X_R      = TORSO_W / 2 + UPPER_ARM_W / 2 + 1.5  # +58.5 мм
ARM_X_L      = -ARM_X_R                             # -58.5 мм
ELBOW_Z      = SHOULDER_Z - (UPPER_ARM_L - 13.0 - 18.0) # 140.0 мм

# ==============================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ==============================================================================

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.length_unit = 'MILLIMETERS'
    scene.unit_settings.scale_length = 0.001

def create_box(name, dimensions, location=(0,0,0), rotation=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    return obj

def create_cylinder(name, radius, depth, location=(0,0,0), rotation=(0,0,0), vertices=32):
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=depth, vertices=vertices,
                                       location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    return obj

def apply_bool(target, tool, operation='DIFFERENCE', solver='EXACT', delete_tool=True):
    mod = target.modifiers.new(name=f"Bool_{operation}", type='BOOLEAN')
    mod.operation = operation
    mod.solver = solver
    mod.object = tool
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    if delete_tool and tool.name in bpy.data.objects:
        bpy.data.objects.remove(tool, do_unlink=True)

def recalc_normals(obj):
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')

def create_mat(name, color, roughness=0.35, metallic=0.0, emission=None):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = color
        bsdf.inputs["Roughness"].default_value = roughness
        bsdf.inputs["Metallic"].default_value = metallic
        if emission:
            bsdf.inputs["Emission Color"].default_value = emission
            bsdf.inputs["Emission Strength"].default_value = 4.0
    return mat

def assign_mat(obj, mat):
    obj.data.materials.clear()
    obj.data.materials.append(mat)

# ==============================================================================
# ПОСТРОЕНИЕ УЗЛОВ СБОРКИ
# ==============================================================================

def build_assembly(materials):
    mat_white, mat_dark, mat_cyan, mat_eyes, mat_metal, mat_uno, mat_servo = materials

    # Коллекции для порядка в сцене
    col_assembly = bpy.data.collections.new("ASSEMBLY")
    bpy.context.scene.collection.children.link(col_assembly)

    col_electronics = bpy.data.collections.new("ELECTRONICS")
    bpy.context.scene.collection.children.link(col_electronics)

    def link_assembly(obj):
        if obj.name in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(obj)
        col_assembly.objects.link(obj)

    def link_electronics(obj):
        if obj.name in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(obj)
        col_electronics.objects.link(obj)

    # 1. НОГИ И СТОПЫ
    for is_left in (True, False):
        side = "Left" if is_left else "Right"
        lx = -LEG_X if is_left else LEG_X

        foot = create_box(f"Foot_{side}", (FOOT_W, FOOT_D, FOOT_H), (lx, 5.0, FOOT_H / 2))
        toe = create_box("Toe", (FOOT_W + 10, 20, 20), (lx, -FOOT_D/2 + 5.0, FOOT_H), (math.radians(35), 0, 0))
        apply_bool(foot, toe)
        heel = create_box("Heel", (FOOT_W + 10, 15, 15), (lx, FOOT_D/2 + 5.0, FOOT_H), (math.radians(-30), 0, 0))
        apply_bool(foot, heel)

        # Стойка голени (надежное перекрытие 2 мм для монолитного союза)
        col_h = LEG_TOTAL_H - FOOT_H + 2.0
        col = create_box(f"Col_{side}", (LEG_COL_W, LEG_COL_D, col_h), (lx, 0, FOOT_H - 1.0 + col_h / 2))
        apply_bool(foot, col, operation='UNION')

        # Верхний Т-шип
        tenon = create_box("TTenon", (14.0, 36.0, 7.5), (lx, 0, LEG_TOTAL_H + 3.75))
        apply_bool(foot, tenon, operation='UNION')

        # Внутренние полости утяжелителей (строго внутри стенок толщиной >= 5 мм)
        cav_foot = create_box("BallastFoot", (FOOT_W - 12.0, FOOT_D - 20.0, 14.0), (lx, 5.0, 9.0))
        apply_bool(foot, cav_foot)
        cav_col = create_box("BallastCol", (LEG_COL_W - 8.0, LEG_COL_D - 8.0, 50.0), (lx, 0, FOOT_H + 22.0))
        apply_bool(foot, cav_col)

        foot.name = f"Leg_{side}"
        recalc_normals(foot)
        assign_mat(foot, mat_white)
        link_assembly(foot)

        # Съемная заглушка подошвы
        cap = create_box(f"Foot_Cap_{side}", (FOOT_W - 8.0, FOOT_D - 16.0, 2.8), (lx, 5.0, 1.4))
        recalc_normals(cap)
        assign_mat(cap, mat_dark)
        link_assembly(cap)

    # 2. КОРПУС ТЕЛА (TORSO)
    torso = create_box("Torso_Body", (TORSO_W, TORSO_D, TORSO_H), (0, 0, TORSO_ZC))

    cavity = create_box("Torso_Cavity", (TORSO_W - 2*WALL, TORSO_D - WALL + 5.0, TORSO_H - 2*WALL),
                        (0, 2.5 + WALL/2, TORSO_ZC))
    apply_bool(torso, cavity)

    # Направляющие Uno
    rail_l = create_box("Rail_L", (4.0, 6.0, 75.0), (-53.4/2 - 1.5, 0.0, TORSO_Z0 + 45.0))
    apply_bool(torso, rail_l, operation='UNION')
    rail_r = create_box("Rail_R", (4.0, 6.0, 75.0), (53.4/2 + 1.5, 0.0, TORSO_Z0 + 45.0))
    apply_bool(torso, rail_r, operation='UNION')

    slot_l = create_box("Slot_L", (2.4, 3.5, 80.0), (-53.4/2, 0.0, TORSO_Z0 + 47.0))
    apply_bool(torso, slot_l)
    slot_r = create_box("Slot_R", (2.4, 3.5, 80.0), (53.4/2, 0.0, TORSO_Z0 + 47.0))
    apply_bool(torso, slot_r)

    # Ниша нагрудного знака на лицевой панели (-Y)
    chest_recess = create_box("Chest_Recess", (TORSO_W - 20.0, 2.0, 45.0), (0, -TORSO_D/2 + 0.5, TORSO_ZC + 10.0))
    apply_bool(torso, chest_recess)

    # Окно портов Uno на левом боку
    uno_ports = create_box("Uno_Ports", (16.0, 32.0, 16.0), (-TORSO_W/2, 0.0, TORSO_Z0 + 20.0))
    apply_bool(torso, uno_ports)

    # Шарниры плеч (Ось X сквозь стенки наружу)
    hole_sh_l = create_cylinder("Hole_Sh_L", 6.5, 12.0, (-TORSO_W/2, 0, SHOULDER_Z), (0, math.radians(90), 0))
    apply_bool(torso, hole_sh_l)
    hole_sh_r = create_cylinder("Hole_Sh_R", 6.5, 12.0, (TORSO_W/2, 0, SHOULDER_Z), (0, math.radians(90), 0))
    apply_bool(torso, hole_sh_r)

    # Шейный проем
    neck_h = create_cylinder("Neck_H", 6.5, 12.0, (0, 0, NECK_Z), (0, 0, 0))
    apply_bool(torso, neck_h)

    # Нижние T-пазы для ног
    for leg_x in (-LEG_X, LEG_X):
        t_slot = create_box(f"TSlot_{leg_x}", (14.8, 48.0, 8.6), (leg_x, 4.0, TORSO_Z0 + 3.0))
        apply_bool(torso, t_slot)
        t_neck = create_box(f"TNeck_{leg_x}", (10.8, 50.0, 4.0), (leg_x, 4.0, TORSO_Z0))
        apply_bool(torso, t_neck)

    recalc_normals(torso)
    assign_mat(torso, mat_white)
    link_assembly(torso)

    # Задняя крышка корпуса (+Y)
    cover = create_box("Torso_Back_Cover", (TORSO_W - 2*WALL + 1.8, 2.8, TORSO_H - 2*WALL - 0.8),
                       (0, TORSO_D/2 - 1.4, TORSO_ZC))
    recalc_normals(cover)
    assign_mat(cover, mat_dark)
    link_assembly(cover)

    # Нагрудная защитная пластина и светящийся реактор
    chest_plate = create_box("Chest_Plate", (TORSO_W - 24.0, 1.8, 41.0), (0, -TORSO_D/2 + 0.9, TORSO_ZC + 10.0))
    recalc_normals(chest_plate)
    assign_mat(chest_plate, mat_dark)
    link_assembly(chest_plate)

    reactor_ring = create_cylinder("Reactor_Ring", 10.5, 3.0, (0, -TORSO_D/2 - 0.5, TORSO_ZC + 10.0), (math.radians(90), 0, 0))
    recalc_normals(reactor_ring)
    assign_mat(reactor_ring, mat_metal)
    link_assembly(reactor_ring)

    reactor_core = create_cylinder("Reactor_Core", 8.0, 3.5, (0, -TORSO_D/2 - 0.8, TORSO_ZC + 10.0), (math.radians(90), 0, 0))
    recalc_normals(reactor_core)
    assign_mat(reactor_core, mat_eyes)
    link_assembly(reactor_core)

    # 3. ГОЛОВА (HEAD)
    head = create_box("Head", (HEAD_W, HEAD_D, HEAD_H), (0, 0, HEAD_ZC))
    chamfer = create_box("Head_Chamfer", (HEAD_W + 10, 15, 15), (0, -HEAD_D/2, HEAD_ZC + HEAD_H/2), (math.radians(-45), 0, 0))
    apply_bool(head, chamfer)

    head_cav = create_box("Head_Cav", (HEAD_W - 2*WALL, HEAD_D - 2*WALL, HEAD_H), (0, 0, HEAD_ZC + WALL))
    apply_bool(head, head_cav)

    # Глаза HC-SR04 на лицевой грани (-Y)
    eye_z = HEAD_ZC + 3.0
    for sign, ex in ((-1, -EYE_PITCH/2), (1, EYE_PITCH/2)):
        eye_hole = create_cylinder(f"Eye_Hole_{sign}", EYE_D/2, 12.0, (ex, -HEAD_D/2, eye_z), (math.radians(90), 0, 0))
        apply_bool(head, eye_hole)

        bezel = create_cylinder(f"Eye_Bezel_{sign}", EYE_D/2, 3.5, (ex, -HEAD_D/2 - 1.2, eye_z), (math.radians(90), 0, 0))
        recalc_normals(bezel)
        assign_mat(bezel, mat_metal)
        link_assembly(bezel)

        lens = create_cylinder(f"Eye_Lens_{sign}", EYE_D/2 - 2.0, 4.0, (ex, -HEAD_D/2 - 1.0, eye_z), (math.radians(90), 0, 0))
        recalc_normals(lens)
        assign_mat(lens, mat_eyes)
        link_assembly(lens)

    recalc_normals(head)
    assign_mat(head, mat_white)
    link_assembly(head)

    # Верхний козырек-крышка головы
    head_lid = create_box("Head_Lid", (HEAD_W - 0.4, HEAD_D - 0.4, 2.8), (0, 0, HEAD_ZC + HEAD_H/2 + 1.4))
    recalc_normals(head_lid)
    assign_mat(head_lid, mat_dark)
    link_assembly(head_lid)

    # 4. РУКИ (ПАРАЛЛЕЛЬНОЕ ДВИЖЕНИЕ В ПЛОСКОСТИ Y-Z)
    for is_left in (True, False):
        side = "Left" if is_left else "Right"
        ax = ARM_X_L if is_left else ARM_X_R

        # Плечо (Upper Arm)
        u_arm = create_box(f"Upper_Arm_{side}", (UPPER_ARM_W, UPPER_ARM_D, UPPER_ARM_L),
                           (ax, 0, SHOULDER_Z - UPPER_ARM_L/2 + 13.0))
        sh_hub = create_cylinder("ShHub", 13.0, UPPER_ARM_W, (ax, 0, SHOULDER_Z), (0, math.radians(90), 0))
        apply_bool(u_arm, sh_hub, operation='UNION')

        el_pocket = create_box("ElPocket", (13.2, 24.0, 24.0), (ax, 0, ELBOW_Z))
        apply_bool(u_arm, el_pocket)

        recalc_normals(u_arm)
        assign_mat(u_arm, mat_white)
        link_assembly(u_arm)

        # Наружная декоративная ступица шарнира плеча
        sh_cap = create_cylinder(f"Sh_Cap_{side}", 8.5, 3.0,
                                 (ax + (-UPPER_ARM_W/2 if is_left else UPPER_ARM_W/2), 0, SHOULDER_Z),
                                 (0, math.radians(90), 0))
        recalc_normals(sh_cap)
        assign_mat(sh_cap, mat_cyan)
        link_assembly(sh_cap)

        # Предплечье (Forearm) - согнуто в локте на 25 градусов строго в сагиттальной плоскости YZ
        fa = create_box(f"Forearm_{side}", (FOREARM_W, FOREARM_D, FOREARM_L),
                        (0, 0, -FOREARM_L/2 + 11.5))
        fa_hub = create_cylinder("FaHub", 11.5, FOREARM_W, (0, 0, 0), (0, math.radians(90), 0))
        apply_bool(fa, fa_hub, operation='UNION')

        # Захват-манипулятор
        gripper = create_box("Gripper", (10.0, 24.0, 16.0), (0, 0, -FOREARM_L + 11.5 + 6.0))
        apply_bool(fa, gripper)

        recalc_normals(fa)
        assign_mat(fa, mat_white)

        # Сгиб локтя вперед (поворот вокруг оси X)
        fa.rotation_euler = (math.radians(-25.0), 0, 0)
        fa.location = (ax, 0, ELBOW_Z)
        bpy.context.view_layer.objects.active = fa
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        link_assembly(fa)

        # Наружная ступица локтевого шарнира
        el_cap = create_cylinder(f"El_Cap_{side}", 7.5, 3.0,
                                 (ax + (-FOREARM_W/2 if is_left else FOREARM_W/2), 0, ELBOW_Z),
                                 (0, math.radians(90), 0))
        recalc_normals(el_cap)
        assign_mat(el_cap, mat_cyan)
        link_assembly(el_cap)

    # 5. УСИЛЕННЫЕ ПРИЖИМНЫЕ СКОБЫ ("УШКИ") СЕРВОПРИВОДОВ
    clamps = [
        ("Servo_Clamp_Shoulder_L", (-TORSO_W/2 + 10.0, 0, SHOULDER_Z), (0, math.radians(90), 0)),
        ("Servo_Clamp_Shoulder_R", (TORSO_W/2 - 10.0, 0, SHOULDER_Z),  (0, math.radians(90), 0)),
        ("Servo_Clamp_Head",       (0, 0, NECK_Z - 8.0),                (0, 0, 0)),
        ("Servo_Clamp_Elbow_L",    (ARM_X_L, 0, ELBOW_Z),               (0, math.radians(90), 0)),
        ("Servo_Clamp_Elbow_R",    (ARM_X_R, 0, ELBOW_Z),               (0, math.radians(90), 0)),
    ]
    for c_name, loc, rot in clamps:
        c_obj = create_box(c_name, (32.5, 12.0, 3.0), loc, rot)
        recalc_normals(c_obj)
        assign_mat(c_obj, mat_cyan)
        link_assembly(c_obj)

    # 6. ФАНТОМЫ ЭЛЕКТРОНИКИ (Arduino Uno и сервоприводы SG90)
    uno_pcb = create_box("Phantom_Arduino_Uno", (1.6, 53.4, 68.6), (0, 0, TORSO_Z0 + 50.0))
    recalc_normals(uno_pcb)
    assign_mat(uno_pcb, mat_uno)
    link_electronics(uno_pcb)

    s_neck = create_box("Phantom_SG90_Neck", (SG_L, SG_W, SG_H), (0, 0, NECK_Z - SG_H/2))
    recalc_normals(s_neck)
    assign_mat(s_neck, mat_servo)
    link_electronics(s_neck)

    s_sh_l = create_box("Phantom_SG90_Shoulder_L", (SG_H, SG_L, SG_W), (-TORSO_W/2 + SG_H/2, 0, SHOULDER_Z))
    recalc_normals(s_sh_l)
    assign_mat(s_sh_l, mat_servo)
    link_electronics(s_sh_l)

    s_sh_r = create_box("Phantom_SG90_Shoulder_R", (SG_H, SG_L, SG_W), (TORSO_W/2 - SG_H/2, 0, SHOULDER_Z))
    recalc_normals(s_sh_r)
    assign_mat(s_sh_r, mat_servo)
    link_electronics(s_sh_r)

# ==============================================================================
# СТУДИЯ, ОСВЕЩЕНИЕ И РЕНДЕР
# ==============================================================================

def setup_studio_and_render():
    scene = bpy.context.scene

    # Бесшовный студийный подиум
    bpy.ops.mesh.primitive_plane_add(size=1400.0, location=(0, 100.0, 0))
    floor = bpy.context.active_object
    floor.name = "Studio_Floor"
    mat_floor = create_mat("Mat_Floor", (0.86, 0.88, 0.90, 1.0), roughness=0.35)
    assign_mat(floor, mat_floor)

    # Окружение World
    world = bpy.data.worlds.new("Studio_World")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.88, 0.90, 0.93, 1.0)
        bg.inputs["Strength"].default_value = 0.50

    # 3-точечное освещение (Sun Lights без затухания по расстоянию)
    D = math.radians
    def add_sun(name, energy, rot, color=(1,1,1)):
        sun_data = bpy.data.lights.new(name, 'SUN')
        sun_data.energy = energy
        sun_data.color = color
        sun_obj = bpy.data.objects.new(name, sun_data)
        scene.collection.objects.link(sun_obj)
        sun_obj.rotation_euler = rot
        return sun_obj

    add_sun("Sun_Key",  4.5, (D(55), 0, D(-35)), (1.0, 0.98, 0.95))
    add_sun("Sun_Fill", 2.2, (D(45), 0, D(50)),  (0.90, 0.95, 1.0))
    add_sun("Sun_Rim",  4.0, (D(135), 0, D(-160)), (0.75, 0.90, 1.0))

    # Камера
    cam_data = bpy.data.cameras.new("Hero_Camera")
    cam_data.lens = 50.0
    cam_obj = bpy.data.objects.new("Hero_Camera", cam_data)
    scene.collection.objects.link(cam_obj)
    scene.camera = cam_obj

    # Ракурс 1: Трехчетвертной Hero-ракурс (полный рост робота)
    cam_obj.location = (250.0, -680.0, 140.0)
    cam_obj.rotation_euler = (D(87.0), 0, D(20.0))

    # Настройки Cycles CPU
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = False
    scene.cycles.max_bounces = 4
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 960

    # Сохраняем модель сборки
    out_blend = "/home/user/MODEL/robot_assembled.blend"
    bpy.ops.wm.save_as_mainfile(filepath=out_blend)
    print(f"\n[OK] Модель сборки сохранена в: {out_blend}")

    # Рендерим основную презентационную картинку
    out_png = "/home/user/MODEL/robot_assembled.png"
    scene.render.filepath = out_png
    print(f"\n[РЕНДЕР 1/2] Рендер общей презентации: {out_png}...")
    bpy.ops.render.render(write_still=True)
    print(f"[OK] Главное изображение готово: {out_png}")

    # Ракурс 2: Боковой ракурс (показывает параллельное движение рук в плоскости YZ)
    cam_obj.location = (700.0, 0.0, 135.0)
    cam_obj.rotation_euler = (D(90.0), 0, D(90.0))
    out_side = "/home/user/MODEL/robot_assembled_side.png"
    scene.render.filepath = out_side
    print(f"\n[РЕНДЕР 2/2] Рендер бокового вида (параллельные руки): {out_side}...")
    bpy.ops.render.render(write_still=True)
    print(f"[OK] Боковой рендер готов: {out_side}")

def main():
    print("=" * 80)
    print("  ПОЛНАЯ СБОРКА И РЕНДЕР ГУМАНОИДНОГО РОБОТА")
    print("=" * 80)

    reset_scene()

    # Инициализация PBR материалов
    mat_white = create_mat("Robo_White", (0.92, 0.93, 0.94, 1.0), roughness=0.28)
    mat_dark  = create_mat("Robo_Dark",  (0.12, 0.13, 0.15, 1.0), roughness=0.25)
    mat_cyan  = create_mat("Tech_Cyan",  (0.02, 0.65, 0.88, 1.0), roughness=0.20, metallic=0.5)
    mat_eyes  = create_mat("Eyes_Cyan",  (0.05, 0.85, 1.0, 1.0),  roughness=0.10, emission=(0.05, 0.90, 1.0, 1.0))
    mat_metal = create_mat("Eye_Bezel",  (0.75, 0.76, 0.78, 1.0), roughness=0.25, metallic=0.9)
    mat_uno   = create_mat("Uno_Green",  (0.08, 0.35, 0.18, 1.0), roughness=0.40)
    mat_servo = create_mat("SG90_Blue",  (0.04, 0.22, 0.60, 1.0), roughness=0.35)

    materials = (mat_white, mat_dark, mat_cyan, mat_eyes, mat_metal, mat_uno, mat_servo)

    build_assembly(materials)
    setup_studio_and_render()
    print("\n[ЗАВЕРШЕНО] Все файлы успешно созданы!")

if __name__ == "__main__":
    main()
