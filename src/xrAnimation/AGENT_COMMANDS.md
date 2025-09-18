# AGENT_COMMANDS.md

## Bone Rest Pose Dump (Blender)
- Context: Needed the rest-pose transforms for the `stalker_hero_1.ogf` armature in Blender.
- Workflow:
  1. Retrieve the armature object by name and ensure it is an `ARMATURE`.
  2. Force rest pose by setting `armature.data.pose_position = 'REST'`.
  3. Iterate the `armature.data.bones`, decompose each `matrix_local`, convert rotations to XYZ Euler degrees, and print a table.
- Blender Python snippet:
```python
import bpy
from math import degrees

armature_name = "stalker_hero_1.ogf"
armature = bpy.data.objects.get(armature_name)
if armature is None or armature.type != 'ARMATURE':
    raise RuntimeError(f"Armature '{armature_name}' not found or not an armature object")

armature.data.pose_position = 'REST'

rows = []
for bone in armature.data.bones:
    matrix = bone.matrix_local
    loc, rot, _scale = matrix.decompose()
    euler = rot.to_euler('XYZ')
    rows.append({
        "name": bone.name,
        "pos": (loc.x, loc.y, loc.z),
        "rot": (degrees(euler.x), degrees(euler.y), degrees(euler.z))
    })

name_width = max(len("Bone"), *(len(row["name"]) for row in rows))
coord_headers = ["PosX", "PosY", "PosZ", "RotX", "RotY", "RotZ"]
col_widths = [max(len(header), 10) for header in coord_headers]

header = f"{ 'Bone'.ljust(name_width) }  " + "  ".join(h.ljust(w) for h, w in zip(coord_headers, col_widths))
separator = f"{'-' * name_width}  " + "  ".join('-' * w for w in col_widths)
print(header)
print(separator)
for row in rows:
    pos = row['pos']
    rot = row['rot']
    data = [f"{pos[0]: .6f}", f"{pos[1]: .6f}", f"{pos[2]: .6f}",
            f"{rot[0]: .3f}", f"{rot[1]: .3f}", f"{rot[2]: .3f}"]
    line = f"{row['name'].ljust(name_width)}  " + "  ".join(val.rjust(w) for val, w in zip(data, col_widths))
    print(line)
```

## Ozz Bind Pose Dump (ozz_animation_viewer)
- Context: Need bind-pose translations and Euler rotations directly from ozz runtime without legacy debug binaries.
- Workflow:
  1. Reconfigure debug build if needed: `cmake -B xray-16/build-debug -S xray-16 -DCMAKE_BUILD_TYPE=Debug`.
  2. Rebuild the viewer: `cmake --build xray-16/build-debug --target ozz_animation_viewer -j`.
  3. Run the viewer headless (ensure `LD_LIBRARY_PATH` points at the build bin dir):
     `LD_LIBRARY_PATH=xray-16/build-debug/bin/Debug xray-16/build-debug/bin/Debug/ozz_animation_viewer --skeleton=asset_tests/stalker_hero_1.ozz --render=false --max_idle_loops=1`.
  4. Capture the `=== OZZ BIND POSE TABLE ===` output for comparisons.

## Blender Bind Pose Dump (execute_blender_code)
- Context: Export Blender’s world-space bind pose transforms in the same format as the ozz viewer table.
- Workflow:
  1. Ensure `stalker_hero_1.ogf` is the active armature.
  2. Run `mcp__blender__execute_blender_code` with the Python snippet below, which sets rest pose, decomposes each bone’s world matrix, and prints a formatted table.
- Blender Python snippet:
```python
import bpy
from math import degrees

armature_name = "stalker_hero_1.ogf"
armature = bpy.data.objects.get(armature_name)
if armature is None or armature.type != 'ARMATURE':
    raise RuntimeError(f"Armature '{armature_name}' not found or not an armature object")

armature.data.pose_position = 'REST'

rows = []
for bone in armature.data.bones:
    matrix = armature.matrix_world @ bone.matrix_local
    loc, rot, _scale = matrix.decompose()
    euler = rot.to_euler('XYZ')
    rows.append({
        "name": bone.name,
        "pos": (loc.x, loc.y, loc.z),
        "rot": (degrees(euler.x), degrees(euler.y), degrees(euler.z))
    })

name_width = max(len("Bone"), *(len(row["name"]) for row in rows))
coord_headers = ["PosX", "PosY", "PosZ", "RotX", "RotY", "RotZ"]
col_widths = [max(len(header), 10) for header in coord_headers]

header = f"{ 'Bone'.ljust(name_width) }  " + "  ".join(h.ljust(w) for h, w in zip(coord_headers, col_widths))
separator = f"{'-' * name_width}  " + "  ".join('-' * w for w in col_widths)
print(header)
print(separator)
for row in rows:
    pos = row['pos']
    rot = row['rot']
    data = [f"{pos[0]: .6f}", f"{pos[1]: .6f}", f"{pos[2]: .6f}",
            f"{rot[0]: .3f}", f"{rot[1]: .3f}", f"{rot[2]: .3f}"]
    line = f"{row['name'].ljust(name_width)}  " + "  ".join(val.rjust(w) for val, w in zip(data, col_widths))
    print(line)
```
