# Coordinate Systems and Bone Transformations Master Guide

## Overview

This document consolidates all coordinate system and bone transformation knowledge for X-Ray engine, Blender, and ozz-animation integration.

## Part 1: Coordinate Systems

### X-Ray Engine
- **Coordinate System**: Y-up, Left-handed
- **X-axis**: Right
- **Y-axis**: Up  
- **Z-axis**: Forward (into the screen)
- **Rotation Order**: HPB (Heading, Pitch, Bank) = Y, X, Z rotations

### Blender
- **Coordinate System**: Z-up, Right-handed
- **X-axis**: Right
- **Y-axis**: Forward
- **Z-axis**: Up
- **Note**: This is hardcoded and cannot be changed

### ozz-animation
- **Coordinate System**: Y-up, Right-handed (OpenGL standard)
- **X-axis**: Right
- **Y-axis**: Up
- **Z-axis**: Backward (out of screen)
- **Forward Vector**: Typically -Z or +Y depending on context

## Part 2: Coordinate System Conversions

### X-Ray to Blender (MATRIX_BONE)

The blender-xray addon uses this transformation matrix:

```python
MATRIX_BONE = mathutils.Matrix((
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, -1.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0)
)).freeze()
```

This transforms: **X-Ray(X,Y,Z) → Blender(X,-Z,Y)**

### X-Ray to ozz-animation (SOLVED ✅)

Both use Y-up, but different handedness:
- X-Ray: Y-up left-handed
- ozz: Y-up right-handed

**Solution: Quaternion Conjugation**

```cpp
ozz::math::Transform TransformConverter::XRayToOzz(const Fmatrix& xray_matrix) {
    ozz::math::Transform result;
    
    // Extract translation - remains unchanged for Y-up to Y-up
    result.translation = ozz::math::Float3(
        xray_matrix.c.x,
        xray_matrix.c.y,
        xray_matrix.c.z
    );
    
    // Extract rotation and conjugate for handedness conversion
    Fquaternion quat;
    quat.set(xray_matrix);
    // Conjugate: negate x, y, z components, keep w the same
    result.rotation = ozz::math::Quaternion(-quat.x, -quat.y, -quat.z, quat.w);
    
    // Extract scale
    result.scale = ozz::math::Float3(
        xray_matrix.i.magnitude(),
        xray_matrix.j.magnitude(),
        xray_matrix.k.magnitude()
    );
    
    return result;
}
```

#### Why Quaternion Conjugation Works
1. **Handedness affects rotation direction**: Left-handed uses clockwise positive rotations, right-handed uses counter-clockwise
2. **Quaternion conjugation reverses rotation**: (x,y,z,w) → (-x,-y,-z,w) reverses the rotation direction while preserving the axis
3. **Translation is unaffected**: Both systems use Y-up, so translation values transfer directly

## Part 3: Bone Transformation Pipeline

### X-Ray Bone Data Structure

Each bone contains:
- **Name**: Bone identifier
- **Parent Name**: Parent bone identifier (empty for root bones)
- **Bind Pose**: Rest position transformation
  - `rest_offset`: Translation in parent space
  - `rest_rotate`: Euler angles (XYZ order)
- **Motion Data**: Animation transformation
  - Local transforms relative to parent

### Transformation Matrices in X-Ray

1. **Bind Transform**: World-space rest pose
2. **Motion Transform**: Local animation transform
3. **World Transform**: Current world-space position
4. **Render Transform**: Final skinning matrix (world * inverse_bind)

### Critical Transform Order Discovery

**The order of coordinate transformation is critical for correct results!**

**WRONG Approach** (causes errors):
```cpp
// Calculate local transforms in X-Ray space
local_xray = parent_inv_xray * world_xray;
// Transform local to target space - WRONG!
local_ozz = TransformConverter::XRayToOzz(local_xray);
```

**CORRECT Approach** (matches blender-xray):
```cpp
// Transform world poses to target space FIRST
world_ozz = TransformConverter::XRayToOzz(world_xray);
// Calculate local transforms in target space
local_ozz = parent_inv_ozz * world_ozz;
```

### Why Transform Order Matters

When a parent bone has rotation, its children's local transforms are relative to that rotated coordinate system. Transforming local transforms directly causes incorrect results because the parent's rotation affects the child's coordinate space.

Example: If `bip01_pelvis` has 90° rotation, its children's "forward" direction is rotated 90° from world forward. Converting the local transform doesn't account for this rotated reference frame.

## Part 4: Implementation Guide

### Correct OGF to ozz Conversion

```cpp
// Step 1: Read bind poses and build world transforms
for (each bone in depth-first order) {
    if (has_parent) {
        world_pose[bone] = world_pose[parent] * local_bind_pose[bone];
    } else {
        world_pose[bone] = local_bind_pose[bone];
    }
}

// Step 2: Transform ALL world poses to ozz space
for (each bone) {
    world_pose_ozz[bone] = TransformConverter::XRayToOzz(world_pose[bone]);
}

// Step 3: Calculate local transforms in ozz space
for (each bone) {
    if (has_parent) {
        parent_inv = world_pose_ozz[parent].inverted();
        joint.transform = extract_transform(parent_inv * world_pose_ozz[bone]);
    } else {
        joint.transform = extract_transform(world_pose_ozz[bone]);
    }
}
```

### Animation Conversion

For OMF animations:
1. Decompress keyframes using proper formulas
2. Apply coordinate transformation (quaternion conjugation) to rotation keyframes
3. Build ozz animation tracks

## Part 5: Common Issues and Solutions

### Issue 1: Upside-Down Skeleton (SOLVED)
**Symptom**: Skeleton displays upside down in ozz viewer
**Cause**: Incorrect handedness conversion
**Solution**: Apply quaternion conjugation to handle left-handed to right-handed conversion

### Issue 2: All Bones at Origin
**Symptom**: All bones show at (0,0,0) during animation
**Cause**: Incorrect matrix column extraction in Float4x4ToMatrix
**Solution**: Extract translation from 4th column (cols[3]), not W components

### Issue 3: Accumulated Transform Errors
**Symptom**: Child bones increasingly misaligned down hierarchy
**Cause**: Transforming local transforms instead of world transforms
**Solution**: Transform world poses first, then calculate locals

### Issue 4: Left/Right Bone Swap
**Symptom**: Left bones appear on right side and vice versa
**Cause**: X-axis inversion during coordinate conversion
**Solution**: Quaternion conjugation handles this correctly

## Part 6: Matrix Operations Reference

### X-Ray Fmatrix Structure
```cpp
struct Fmatrix {
    Fvector i;  // X axis (right)
    Fvector j;  // Y axis (up in Y-up system)
    Fvector k;  // Z axis (forward)
    Fvector c;  // Translation
};
```

### Evidence from Source Code

**Camera Initialization** (Device_create.cpp):
```cpp
vCameraDirection.set(0, 0, 1);  // Z forward
vCameraTop.set(0, 1, 0);        // Y up
vCameraRight.set(1, 0, 0);      // X right
```

**Matrix Rotation Order** (_matrix.h):
```cpp
// HPB = Heading, Pitch, Bank
setHPB(float h, float p, float b);
// XYZ rotation is mapped as:
setXYZ(float x, float y, float z) { return setHPB(y, x, z); }
```

## Part 7: Verified Results ✅

After implementing quaternion conjugation:
- Feet at Y=0.133 (near ground) ✅
- Head at Y=1.73 (top of character) ✅
- Pelvis at Y=0.987 (mid-height) ✅
- Scene bounds: min(-0.87, -0.00, -0.04) max(0.87, 1.73, 0.17) ✅

## Key Discoveries Summary

1. **Transform Order**: Must transform world poses before calculating local transforms
2. **Handedness Conversion**: Use quaternion conjugation for left-handed to right-handed
3. **Matrix Extraction**: Use correct column for translation extraction
4. **Bone Count**: OGF OMF files animate ALL skeleton bones
5. **String Format**: Motion marks use \r\n termination in version 4

## References

- xrSDK source: `/mnt/f/modding/claude_sessions/xrSDK/`
- blender-xray addon: coordinate transformation implementation
- ozz-animation samples: coordinate system usage examples
- OpenXRay CKinematicsAnimated implementation