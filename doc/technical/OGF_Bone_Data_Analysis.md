# OGF Bone Data Format Analysis

## Research Summary

Based on analysis of **xrSDK**, **blender-xray**, and **OGF-tool** reference implementations, the correct OGF bone data format has been determined.

## Key Findings

### 1. OGF_S_BONE_NAMES Chunk Format

This chunk contains the **primary bone transform data** as OBB (Oriented Bounding Box) data:

```cpp
uint32 bones_count;
for each bone:
    string bone_name;
    string parent_name;  
    float[9] rotation_matrix;     // 3x3 rotation matrix (row-major)
    float[3] translation;         // translation vector
    float[3] half_size;          // bounding box half-size (unused for skeleton)
```

**Source:** blender-xray `/io_scene_xray/formats/ogf/imp/bone.py:15-17`

### 2. OGF_S_IKDATA Chunk Format

This chunk contains **per-bone physics and bind pose override data**:

```cpp
for each bone (same order as bone names):
    uint32 version;              // Optional: only if read_ver=True
    string game_material;        // Physics material name
    uint16 shape_type;           // Physics shape type
    uint16 shape_flags;          // Shape flags
    
    // Box shape data (60 bytes)
    float[9] box_rotation;       // 3x3 rotation matrix  
    float[3] box_translation;    // box center
    float[3] box_half_size;      // box half dimensions
    
    // Sphere shape data (16 bytes)
    float[3] sphere_translation; // sphere center
    float sphere_radius;         // sphere radius
    
    // Cylinder shape data (28 bytes)  
    float[3] cylinder_translation;
    float[3] cylinder_direction;
    float cylinder_height;
    float cylinder_radius;
    
    // IK joint data (52 bytes)
    uint32 joint_type;
    float limit_x_min, limit_x_max;
    float limit_x_spring, limit_x_damping;
    float limit_y_min, limit_y_max; 
    float limit_y_spring, limit_y_damping;
    float limit_z_min, limit_z_max;
    float limit_z_spring, limit_z_damping;
    float joint_spring, joint_damping;
    uint32 ik_flags;
    float breakable_force, breakable_torque;
    float friction;              // Only if version > BONE_VERSION_0
    
    // BIND POSE OVERRIDE (24 bytes) ⭐ KEY DATA ⭐
    float[3] bind_rotation;      // Euler angles (radians)
    float[3] bind_translation;   // Translation vector
    
    // Mass data (16 bytes)
    float mass_value;
    float[3] mass_center;
```

**Source:** blender-xray `/io_scene_xray/formats/ogf/imp/ik.py:72-134`

## Critical Insights

### Transform Data Priority

1. **Primary:** Use `bind_rotation`/`bind_translation` from IK data if available
2. **Fallback:** Use `rotation_matrix`/`translation` from bone names (OBB data) if IK data missing/corrupted

### Coordinate System Conversion

From blender-xray implementation:
```python
# Convert from X-Ray to Blender coordinates
rotation = mathutils.Euler(
    (-bind_rotation[0], -bind_rotation[1], -bind_rotation[2]), 'YXZ'
).to_matrix().to_4x4()

translation = mathutils.Matrix.Translation(bind_translation)

# Apply coordinate conversion matrix
mat = utils.version.multiply(
    translation,
    rotation, 
    motions.const.MATRIX_BONE  # X-Ray to Blender coordinate conversion
)
```

### Issues with Our Current Implementation

1. **Wrong data structure:** We assumed linear stream format, but it's per-bone structured data
2. **Incorrect skip sizes:** We skipped wrong amount of physics shape data 
3. **Wrong order:** We tried to read bind pose before all the physics data
4. **Missing fallback:** No fallback to OBB data when IK data is corrupted

## Implementation Plan

1. **Fix IK data reading:** Parse the complete per-bone structure correctly
2. **Add corruption detection:** Check for astronomical/NaN values in bind pose
3. **Implement fallback:** Use OBB transform when IK bind pose is invalid
4. **Verify coordinate conversion:** Ensure X-Ray to ozz coordinate transform is correct

## Data Structure Size Reference

- Total IK data per bone: ~250+ bytes
- Critical bind pose data: 24 bytes (6 floats)
- OBB fallback data: 60 bytes (15 floats) 

## Reference Sources

- **xrSDK:** `/src/editors/ECore/Editor/ExportSkeleton.cpp:1370-1385`
- **blender-xray:** `/io_scene_xray/formats/ogf/imp/ik.py:72-134`
- **OGF-tool:** `/Tools/Converter/xr_bone.cxx:61-107`