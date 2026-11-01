"""Build a character: a MakeHuman (MPFB) soldier skinned to the SOMA skeleton, exported as a glTF binary.

Run with Blender, with the MPFB extension and the MakeHuman CC0 system assets installed:

    blender --background --python make_soldier.py -- <soma_character.glb> <out.glb> <soldier|enemy>

`soldier` wears olive fatigues and boots; `enemy` a dark navy jacket and trousers, boots and a helmet.

The character is fitted to SOMA's bind pose joint by joint, takes its skin weights from the SOMA mesh, and is bound
to the SOMA armature, so the game drives it exactly like the SOMA mesh.
"""

import importlib
import sys

import bmesh
import bpy
import numpy as np


importlib.import_module("bl_ext.blender_org.mpfb")
from bl_ext.blender_org.mpfb.services import HumanService  # noqa: E402


soma_path, out_path, preset_name = sys.argv[sys.argv.index("--") + 1 :]

# Clothes are dyed: each outfit's texture keeps its shading and takes the given color (sRGB, 0 to 1).
PRESETS = {
    "soldier": {
        "hair": "short02/short02.mhclo",
        "clothes": {"male_casualsuit01": (0.36, 0.38, 0.22), "shoes03": None},
        "helmet": None,
    },
    "enemy": {
        "hair": "short02/short02.mhclo",
        "clothes": {"male_casualsuit05": (0.06, 0.08, 0.16), "shoes03": None},
        "helmet": (0.07, 0.08, 0.10),
    },
}
preset = PRESETS[preset_name]

for obj in list(bpy.data.objects):
    bpy.data.objects.remove(obj, do_unlink=True)

# The character.
info = HumanService._create_default_human_info_dict()
info["phenotype"].update(
    {
        "gender": 1.0,
        "age": 0.5,
        "muscle": 0.65,
        "weight": 0.5,
        "height": 0.5,
        "proportions": 0.5,
        "race": {"asian": 0.0, "caucasian": 1.0, "african": 0.0},
    }
)
info.update(
    {
        "rig": "default",
        "eyes": "low-poly/low-poly.mhclo",
        "eyebrows": "eyebrow001/eyebrow001.mhclo",
        "eyelashes": "eyelashes01/eyelashes01.mhclo",
        "hair": preset["hair"] or "",
        "clothes": [f"{name}/{name}.mhclo" for name in preset["clothes"]],
        "skin_mhmat": "young_caucasian_male/young_caucasian_male.mhmat",
        "skin_material_type": "MAKESKIN",
        "eyes_material_type": "MAKESKIN",
    }
)
settings = HumanService.get_default_deserialization_settings()
settings["subdiv_levels"] = 0
HumanService.deserialize_from_dict(info, settings)
rig = next(o for o in bpy.data.objects if o.type == "ARMATURE")
meshes = [o for o in bpy.data.objects if o.type == "MESH"]

# The SOMA mesh and armature, in the same space (meters, Z up, facing -Y).
bpy.ops.import_scene.gltf(filepath=soma_path)
soma_armature = next(o for o in bpy.data.objects if o.type == "ARMATURE" and o != rig)
soma_mesh = next(o for o in bpy.data.objects if o.type == "MESH" and o.parent == soma_armature)
soma = {b.name: soma_armature.matrix_world @ b.head_local for b in soma_armature.data.bones}


# Fit the character to SOMA's bind pose: move him so his hip joints sit on SOMA's, lean his torso so his shoulders
# line up with SOMA's, then put each limb segment on SOMA's joint and point it where SOMA's does. Parents first.
def mid(a, b):
    return (a + b) / 2


bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode="EDIT")
for bone in rig.data.edit_bones:
    bone.use_connect = False
bpy.ops.object.mode_set(mode="POSE")


def head(name):
    bpy.context.view_layer.update()
    return rig.matrix_world @ rig.pose.bones[name].head


rig.location += mid(soma["LeftLeg"], soma["RightLeg"]) - mid(head("upperleg01.L"), head("upperleg01.R"))


def place(name, direction_from, direction_to, target_from, target_to, at=None):
    bpy.context.view_layer.update()
    bone = rig.pose.bones[name]
    turn = (direction_to - direction_from).rotation_difference(target_to - target_from)
    world = rig.matrix_world @ bone.matrix
    placed = turn.to_matrix().to_4x4() @ world
    placed.translation = world.translation if at is None else at
    bone.matrix = rig.matrix_world.inverted() @ placed


hips, shoulders = mid(soma["LeftLeg"], soma["RightLeg"]), mid(soma["LeftArm"], soma["RightArm"])
place(
    "spine05",
    mid(head("upperleg01.L"), head("upperleg01.R")),
    mid(head("upperarm01.L"), head("upperarm01.R")),
    hips,
    shoulders,
)
place("neck01", head("neck01"), head("head"), soma["Neck1"], soma["Head"], at=soma["Neck1"])
place(
    "head",
    head("head"),
    rig.matrix_world @ rig.pose.bones["head"].tail,
    soma["Head"],
    soma["HeadEnd"],
    at=soma["Head"],
)
LIMBS = []
for side, soma_side in (("L", "Left"), ("R", "Right")):
    LIMBS += [
        (f"upperarm01.{side}", f"lowerarm01.{side}", f"{soma_side}Arm", f"{soma_side}ForeArm"),
        (f"lowerarm01.{side}", f"wrist.{side}", f"{soma_side}ForeArm", f"{soma_side}Hand"),
        (f"wrist.{side}", f"finger3-1.{side}", f"{soma_side}Hand", f"{soma_side}HandMiddle1"),
        (f"upperleg01.{side}", f"lowerleg01.{side}", f"{soma_side}Leg", f"{soma_side}Shin"),
        (f"lowerleg01.{side}", f"foot.{side}", f"{soma_side}Shin", f"{soma_side}Foot"),
        (f"foot.{side}", f"toe3-1.{side}", f"{soma_side}Foot", f"{soma_side}ToeBase"),
    ]
for side, soma_side in (("L", "Left"), ("R", "Right")):
    # The shoulder moves with the arm so the arm doesn't shear away from it.
    clavicle = rig.pose.bones[f"clavicle.{side}"]
    bpy.context.view_layer.update()
    world = rig.matrix_world @ clavicle.matrix
    world.translation += soma[f"{soma_side}Arm"] - head(f"upperarm01.{side}")
    clavicle.matrix = rig.matrix_world.inverted() @ world
for start, end, soma_from, soma_to in LIMBS:
    place(start, head(start), head(end), soma[soma_from], soma[soma_to], at=soma[soma_from])
bpy.ops.object.mode_set(mode="OBJECT")

# Bake the pose and the hidden parts into the meshes, then take SOMA's weights and armature.
for obj in meshes:
    bpy.context.view_layer.objects.active = obj
    # The body's shape is in shape keys; keep their current mix.
    if obj.data.shape_keys:
        bpy.ops.object.shape_key_remove(all=True, apply_mix=True)
    for modifier in list(obj.modifiers):
        if modifier.type in ("ARMATURE", "MASK"):
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        else:
            obj.modifiers.remove(modifier)
    world = obj.matrix_world.copy()
    obj.parent = None
    obj.matrix_world = world
    obj.vertex_groups.clear()
    transfer = obj.modifiers.new("weights", "DATA_TRANSFER")
    transfer.object = soma_mesh
    transfer.use_vert_data = True
    transfer.data_types_verts = {"VGROUP_WEIGHTS"}
    transfer.vert_mapping = "POLYINTERP_NEAREST"
    transfer.layers_vgroup_select_src = "ALL"
    transfer.layers_vgroup_select_dst = "NAME"
    bpy.ops.object.datalayout_transfer(modifier=transfer.name)
    bpy.ops.object.modifier_apply(modifier=transfer.name)
    obj.parent = soma_armature
    obj.matrix_parent_inverse = soma_armature.matrix_world.inverted()
    obj.modifiers.new("skin", "ARMATURE").object = soma_armature
bpy.data.objects.remove(rig, do_unlink=True)
bpy.data.objects.remove(soma_mesh, do_unlink=True)


# The helmet: a dome a little larger than the head, open below the brow, bound to the head joint.
def add_helmet(color):
    neck = soma["Neck1"].z
    points = [
        tuple(obj.matrix_world @ v.co)
        for obj in meshes
        if obj.name == "Human" or "short02" in obj.name
        for v in obj.data.vertices
    ]
    head = np.array([p for p in points if p[2] > neck + 0.06])
    low, high = head.min(axis=0), head.max(axis=0)
    radius = np.array([(high[0] - low[0]) / 2 + 0.02, (high[1] - low[1]) / 2 + 0.015, 0.0])
    radius[2] = radius[0]
    center = np.array([0.0, (low[1] + high[1]) / 2, high[2] + 0.015 - radius[2]])
    eyes = next(o for o in meshes if "low-poly" in o.name)
    front = 1.0 if np.mean([(eyes.matrix_world @ v.co).y for v in eyes.data.vertices]) > center[1] else -1.0

    mesh = bpy.data.meshes.new("helmet")
    shell = bmesh.new()
    bmesh.ops.create_uvsphere(shell, u_segments=32, v_segments=16, radius=1.0)
    # Open above the brow at the front, lower over the back of the head.
    cut = [
        v for v in shell.verts if v.co.z < -0.25 + 0.4 * max(0.0, front * v.co.y) - 0.15 * max(0.0, -front * v.co.y)
    ]
    bmesh.ops.delete(shell, geom=cut, context="VERTS")
    # The rim flares out a little.
    rim = {v for v in shell.verts if v.is_boundary}
    for v in shell.verts:
        flare = 1.1 if v in rim else 1.0
        v.co = (
            v.co.x * radius[0] * flare + center[0],
            v.co.y * radius[1] * flare + center[1],
            v.co.z * radius[2] + center[2],
        )
    bmesh.ops.solidify(shell, geom=shell.faces[:], thickness=0.012)
    shell.to_mesh(mesh)
    shell.free()
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    helmet = bpy.data.objects.new("helmet", mesh)
    bpy.context.collection.objects.link(helmet)
    material = bpy.data.materials.new("helmet")
    material.use_nodes = True
    bsdf = next(n for n in material.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*(c**2.2 for c in color), 1.0)
    bsdf.inputs["Roughness"].default_value = 0.55
    mesh.materials.append(material)
    helmet.vertex_groups.new(name="Head").add(range(len(mesh.vertices)), 1.0, "REPLACE")
    helmet.parent = soma_armature
    helmet.matrix_parent_inverse = soma_armature.matrix_world.inverted()
    helmet.modifiers.new("skin", "ARMATURE").object = soma_armature
    return helmet


def dye(image, color):
    pixels = np.array(image.pixels[:], dtype=np.float32).reshape(-1, 4)
    shade = pixels[:, :3] @ np.array([0.3, 0.59, 0.11], dtype=np.float32)
    used = pixels[:, 3] > 0.5
    shade = np.clip(shade / np.median(shade[used]), 0.0, 1.8)
    pixels[:, :3] = np.clip(shade[:, None] * np.array(color, dtype=np.float32), 0.0, 1.0)
    image.pixels[:] = pixels.ravel()
    image.pack()


# Plain materials for glTF: the color texture straight into the shader (with its alpha for hair, eyebrows and
# eyelashes), and the normal map if there is one.
for obj in meshes:
    for material in obj.data.materials:
        nodes, links = material.node_tree.nodes, material.node_tree.links
        bsdf = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
        images = [n for n in nodes if n.type == "TEX_IMAGE" and n.image]
        color = next(
            (n for n in images if "normal" not in n.image.name.lower() and "_ao" not in n.image.name.lower()), None
        )
        normal = next((n for n in nodes if n.type == "NORMAL_MAP"), None)
        for node in list(nodes):
            if node.type in ("MIX_RGB", "MIX", "BUMP"):
                nodes.remove(node)
        if color:
            dye_color = next((c for name, c in preset["clothes"].items() if name in obj.name and c), None)
            if dye_color:
                dye(color.image, dye_color)
            links.new(color.outputs["Color"], bsdf.inputs["Base Color"])
            if any(k in obj.name for k in ("eyebrow", "eyelashes", "short")):
                links.new(color.outputs["Alpha"], bsdf.inputs["Alpha"])
                material.surface_render_method = "DITHERED"
        if normal:
            links.new(normal.outputs["Normal"], bsdf.inputs["Normal"])

if preset["helmet"]:
    meshes.append(add_helmet(preset["helmet"]))

bpy.ops.object.select_all(action="DESELECT")
soma_armature.select_set(True)
for obj in meshes:
    obj.select_set(True)
bpy.ops.export_scene.gltf(
    filepath=out_path,
    export_format="GLB",
    use_selection=True,
    export_skins=True,
    export_animations=False,
    export_yup=True,
    export_image_format="AUTO",
)
print(f"wrote {out_path}: {len(meshes)} meshes on {len(soma_armature.data.bones)} SOMA bones")
