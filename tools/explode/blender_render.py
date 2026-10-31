"""Рендер ролика разборки/сборки в Blender из GLB, выгруженного вьювером (страница ?exportglb=1.15).

blender -b --factory-startup -P tools/explode/blender_render.py -- <scene.glb> <папка кадров> [кадр_с кадр_по] [процент_размера]
Кадры PNG потом склеить ffmpeg. Свет: студийный HDRI (только отражения) + ключевой, заполняющий и контровой свет; фон — тёмный градиент.
"""
import bpy, math, os, sys
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
GLB, OUT = argv[0], argv[1]
F0, F1 = (int(argv[2]), int(argv[3])) if len(argv) >= 4 else (None, None)
PCT = int(argv[4]) if len(argv) >= 5 else 100
FPS, SPEED = 30, 1.15
HOLD_A, T, HOLD_B = 1.0, 6.2 / SPEED, 2.6                 # как в exportGLB (app.js)

sc = bpy.context.scene
for o in list(bpy.data.objects): bpy.data.objects.remove(o, do_unlink=True)
sc.render.fps = FPS
bpy.ops.import_scene.gltf(filepath=GLB)
meshes = [o for o in sc.objects if o.type == 'MESH']
n_frames = max(int(a.frame_range[1]) for a in bpy.data.actions) if bpy.data.actions else 420
sc.frame_start, sc.frame_end = 1, n_frames

# материалы: пластик матовее, металл болтов как есть
for m in bpy.data.materials:
    b = m.node_tree.nodes.get('Principled BSDF') if m.use_nodes else None
    if not b: continue
    if b.inputs['Metallic'].default_value < 0.5:
        b.inputs['Roughness'].default_value = max(0.55, b.inputs['Roughness'].default_value)
        if 'Specular IOR Level' in b.inputs: b.inputs['Specular IOR Level'].default_value = 0.45


def bounds(frame):
    sc.frame_set(frame)
    dg = bpy.context.evaluated_depsgraph_get()
    lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for o in meshes:
        ob = o.evaluated_get(dg)
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w)); hi = Vector(map(max, hi, w))
    return (lo + hi) / 2, (hi - lo).length / 2


cA, rA = bounds(1)
cB, rB = bounds(int((HOLD_A + T + 1.0) * FPS))
sc.frame_set(1)

# камера: облёт спереди-справа, сверху; от собранной к разобранной и обратно
cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
cam.data.lens = 35; cam.data.sensor_width = 36
fov_v = 2 * math.atan(36 * 9 / 16 / 2 / 35)
dA, dB = rA / math.sin(fov_v / 2) * 0.85, rB / math.sin(fov_v / 2) * 0.56
ease = lambda k: 4 * k ** 3 if k < .5 else 1 - (-2 * k + 2) ** 3 / 2
for f in range(1, n_frames + 1):
    s = (f - 1) / FPS
    if s < HOLD_A: tt = 0
    elif s < HOLD_A + T: tt = (s - HOLD_A) / T
    elif s < HOLD_A + T + HOLD_B: tt = 1
    else: tt = max(0, 1 - (s - HOLD_A - T - HOLD_B) / T)
    k = ease(min(1, tt))
    c = cA.lerp(cB, k); d = dA + (dB - dA) * k
    az, el = -0.75 + s * 0.05, 0.40
    cam.location = c + Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))) * d
    cam.rotation_euler = (c - cam.location).to_track_quat('-Z', 'Y').to_euler()
    cam.keyframe_insert('location', frame=f); cam.keyframe_insert('rotation_euler', frame=f)

# свет
def light(name, kind, energy, color, loc, size=1.0):
    L = bpy.data.lights.new(name, kind); L.energy = energy; L.color = color
    if kind == 'AREA': L.size = size
    o = bpy.data.objects.new(name, L); sc.collection.objects.link(o)
    o.location = Vector(loc) + cB; o.rotation_euler = (cB - o.location).to_track_quat('-Z', 'Y').to_euler()
    return o
light('key', 'AREA', 120, (1.0, 0.93, 0.85), (0.9, -1.1, 1.4), 1.2)
light('fill', 'AREA', 45, (0.8, 0.86, 1.0), (-1.2, -0.6, 0.5), 1.5)
light('rim', 'AREA', 140, (0.75, 0.85, 1.0), (-0.6, 1.3, 1.0), 1.0)

# мир: студийный HDRI для отражений, в кадре — тёмный градиент
w = sc.world or bpy.data.worlds.new('w'); sc.world = w; w.use_nodes = True
nt = w.node_tree; nt.nodes.clear()
out = nt.nodes.new('ShaderNodeOutputWorld')
env = nt.nodes.new('ShaderNodeTexEnvironment')
hdr = os.path.join(bpy.utils.system_resource('DATAFILES', path='studiolights/world'), 'studio.exr')
if os.path.exists(hdr): env.image = bpy.data.images.load(hdr)
bg_env = nt.nodes.new('ShaderNodeBackground'); bg_env.inputs['Strength'].default_value = 0.35
nt.links.new(env.outputs['Color'], bg_env.inputs['Color'])
grad_tc = nt.nodes.new('ShaderNodeTexCoord'); sep = nt.nodes.new('ShaderNodeSeparateXYZ')
ramp = nt.nodes.new('ShaderNodeValToRGB')
ramp.color_ramp.elements[0].position = 0.35; ramp.color_ramp.elements[0].color = (0.012, 0.013, 0.016, 1)
ramp.color_ramp.elements[1].position = 0.75; ramp.color_ramp.elements[1].color = (0.05, 0.053, 0.06, 1)
nt.links.new(grad_tc.outputs['Window'], sep.inputs[0]); nt.links.new(sep.outputs['Y'], ramp.inputs['Fac'])
bg_cam = nt.nodes.new('ShaderNodeBackground'); nt.links.new(ramp.outputs['Color'], bg_cam.inputs['Color'])
lp = nt.nodes.new('ShaderNodeLightPath'); mix = nt.nodes.new('ShaderNodeMixShader')
nt.links.new(lp.outputs['Is Camera Ray'], mix.inputs['Fac'])
nt.links.new(bg_env.outputs['Background'], mix.inputs[1]); nt.links.new(bg_cam.outputs['Background'], mix.inputs[2])
nt.links.new(mix.outputs['Shader'], out.inputs['Surface'])

# рендер
r = sc.render
r.engine = 'BLENDER_EEVEE' if 'BLENDER_EEVEE' in {e.identifier for e in bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items} else 'BLENDER_EEVEE_NEXT'
ee = sc.eevee
for k, v in (('taa_render_samples', 48), ('use_raytracing', True), ('use_shadows', True), ('use_gtao', True)):
    if hasattr(ee, k): setattr(ee, k, v)
r.resolution_x, r.resolution_y, r.resolution_percentage = 1920, 1080, PCT
r.film_transparent = False
vts = {e.identifier for e in sc.view_settings.bl_rna.properties['view_transform'].enum_items}
sc.view_settings.view_transform = 'Khronos PBR Neutral' if 'Khronos PBR Neutral' in vts else 'Standard'   # AgX выбеливает жёлтый пластик
r.image_settings.file_format = 'PNG'
os.makedirs(OUT, exist_ok=True)
frames = range(F0 or 1, (F1 or n_frames) + 1)
for f in frames:
    sc.frame_set(f); r.filepath = os.path.join(OUT, f'f{f:05d}.png')
    bpy.ops.render.render(write_still=True)
print('RENDER_DONE', len(frames))
