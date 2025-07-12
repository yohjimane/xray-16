# OGF IK Data Parsing Solution

## Problem Summary
The ozz-animation skeleton converter was producing skeletons where all bones collapsed to position (0,0,0) when loaded in the ozz viewer. Despite successful file loading and conversion, the skeleton was invisible due to all bones being at the origin.

## Root Cause Analysis

### Discovery Process
1. Created custom debug version of ozz sample_playback to output bone positions
2. Found all bone positions were (0.00, 0.00, 0.00) for every frame
3. Investigated IK data parsing by cross-referencing:
   - xrSDK source code (ExportSkeleton.cpp)
   - blender-xray addon implementation
   - OGF-tool reference

### Key Finding
The issue was that `CBone::ExportOGF()` in xrSDK only writes IK data for bones where `shape.Valid()` returns true:

```cpp
bool CBone::ExportOGF(IWriter& F)
{
    // check valid
    if (!shape.Valid())
    {
        ELog.Msg(mtError, "Bone '%s' has invalid shape.", *Name());
        return false;
    }
    // ... IK data is only written after this check
}
```

### Shape Validity Rules (from xrSDK)
```cpp
bool SBoneShape::Valid() const
{
    switch (type)
    {
    case stBox:
        return !fis_zero(box.m_halfsize.x) && 
               !fis_zero(box.m_halfsize.y) && 
               !fis_zero(box.m_halfsize.z);
    case stSphere: 
        return !fis_zero(sphere.R);
    case stCylinder:
        return !fis_zero(cylinder.m_height) && 
               !fis_zero(cylinder.m_radius) &&
               !fis_zero(cylinder.m_direction.square_magnitude());
    default:
        return true;
    }
}
```

## Solution Implementation

### Modified IK Data Reading
The OGF converter now:
1. Reads the SBoneShape structure for each bone
2. Checks shape validity using the same logic as xrSDK
3. Only attempts to read IK data for bones with valid shapes
4. Uses OBB transform as fallback for bones without IK data

### Code Changes in OGFConverter.cpp
```cpp
// Read SBoneShape structure to check validity
struct {
    u16 type;     // EShapeType: stNone=0, stBox=1, stSphere=2, stCylinder=3
    u16 flags;
    Fobb box;     // 60 bytes (15 floats)
    Fsphere sphere; // 16 bytes (4 floats)
    Fcylinder cylinder; // 32 bytes (8 floats)
} shape;

// Read shape data
shape.type = ik_reader->r_u16();
shape.flags = ik_reader->r_u16();
ik_reader->r(&shape.box, sizeof(Fobb));
ik_reader->r(&shape.sphere, sizeof(Fsphere));
ik_reader->r(&shape.cylinder, sizeof(Fcylinder));

// Check shape validity using same logic as xrSDK
bool shape_valid = true;
switch (shape.type) {
    case 1: // stBox
        shape_valid = !fis_zero(shape.box.m_halfsize.x) && 
                     !fis_zero(shape.box.m_halfsize.y) && 
                     !fis_zero(shape.box.m_halfsize.z);
        break;
    case 2: // stSphere
        shape_valid = !fis_zero(shape.sphere.R);
        break;
    case 3: // stCylinder
        shape_valid = !fis_zero(shape.cylinder.m_height) && 
                     !fis_zero(shape.cylinder.m_radius) &&
                     !fis_zero(shape.cylinder.m_direction.square_magnitude());
        break;
    default: // stNone or other
        shape_valid = true;
        break;
}

if (!shape_valid) {
    // Bone has invalid shape, no IK data written - use OBB fallback
    bone.obb.xform_get(bone.bind_transform);
    continue; // Skip to next bone - no IK data for this bone
}
```

## Additional Discoveries

### IK Data Structure (from xrSDK)
The IK data export format in CBone::ExportOGF() is:
```cpp
F.w_u32(OGF_IKDATA_VERSION);
F.w_stringZ(game_mtl);
F.w(&shape, sizeof(SBoneShape));
IK_data.Export(F);  // 76 bytes of joint data
F.w_fvector3(rest_rotate);  // Euler rotation (local to parent)
F.w_fvector3(rest_offset);  // Translation (local to parent)
F.w_float(mass);
F.w_fvector3(center_of_mass);
```

### Bind Pose Transforms
- The `rest_rotate` and `rest_offset` are **local transforms relative to parent bone**
- These are the bind pose transforms that position bones in their rest state
- ozz-animation expects these local transforms in the skeleton's joint data

## Results
After implementing the shape validity check:
- All 47 bones properly parsed with valid bind poses
- Skeleton bounds: 1.107 units (vs previous astronomical values)
- Skeleton successfully visible in ozz viewer with correct bone positions
- Scene bounds: min(-0.23, -0.03, -0.84) max(0.26, 1.72, 0.73)

## Debug Playback Output
Created custom debug_playback tool that shows:
- All bone names and positions
- Scene bounds
- Support for skeleton-only viewing (no animation required)
- Bind pose visualization when no animation is loaded

## Future Considerations
1. Some OGF files may have bones without valid shapes - these need OBB fallback
2. The variable-length IK data format means careful reading is required
3. Always verify against xrSDK source code for format specifications