# X-Ray Engine OGF/OMF File Format Master Reference

## Overview
This document contains the authoritative technical specifications for X-Ray Engine's OGF (Object Geometry Format) and OMF (Object Motion Format) file formats.

## OGF (Object Geometry Format)

### File Type Identifiers
```cpp
enum MT {
    MT_NORMAL               = 0,    // Normal mesh
    MT_HIERRARHY           = 1,    // Hierarchical mesh with children
    MT_PROGRESSIVE         = 2,    // Progressive mesh (LOD)
    MT_SKELETON_ANIM       = 3,    // Animated skeleton
    MT_SKELETON_GEOMDEF_PM = 4,    // Skeleton with progressive mesh
    MT_SKELETON_GEOMDEF_ST = 5,    // Skeleton with static mesh
    MT_LOD                 = 6,    // Level of detail
    MT_TREE_ST             = 7,    // Static tree
    MT_PARTICLE_EFFECT     = 8,    // Particle effect
    MT_PARTICLE_GROUP      = 9,    // Particle group
    MT_SKELETON_RIGID      = 10,   // Rigid skeleton
    MT_TREE_PM             = 11,   // Progressive tree
};
```

### OGF Chunk IDs
```cpp
enum OGF_Chuncks : u32 {
    OGF_HEADER              = 1,    // File header
    OGF_TEXTURE             = 2,    // Texture information
    OGF_VERTICES            = 3,    // Vertex data
    OGF_INDICES             = 4,    // Index data
    OGF_P_MAP               = 5,    // Progressive map
    OGF_SWIDATA             = 6,    // Switch data
    OGF_VCONTAINER          = 7,    // Vertex container
    OGF_ICONTAINER          = 8,    // Index container
    OGF_CHILDREN            = 9,    // Child meshes
    OGF_CHILDREN_L          = 10,   // Child meshes (large)
    OGF_LODDEF2             = 11,   // LOD definition v2
    OGF_TREEDEF2            = 12,   // Tree definition v2
    OGF_S_BONE_NAMES        = 13,   // Bone hierarchy (skeletons only)
    OGF_S_MOTIONS           = 14,   // Motion data (skeletons only)
    OGF_S_SMPARAMS          = 15,   // Motion parameters (skeletons only)
    OGF_S_IKDATA            = 16,   // IK constraints (skeletons only)
    OGF_S_USERDATA          = 17,   // User data ini-file (skeletons only)
    OGF_S_DESC              = 18,   // Description (skeletons only)
    OGF_S_MOTION_REFS       = 19,   // Motion references (skeletons only)
    OGF_SWICONTAINER        = 20,   // Switch container
    OGF_GCONTAINER          = 21,   // Geometry container
    OGF_FASTPATH            = 22,   // Fast path data
    OGF_S_LODS              = 23,   // LODs (skeletons only)
    OGF_S_MOTION_REFS2      = 24,   // Motion references v2 (skeletons only)
};
```

### OGF Header Structure
```cpp
const u8 xrOGF_FormatVersion = 4;
const u16 xrOGF_SMParamsVersion = 4;

struct ogf_header {
    u8 format_version;      // = xrOGF_FormatVersion
    u8 type;               // MT type
    u16 shader_id;         // Should not be ZERO
    ogf_bbox bb;           // Bounding box
    ogf_bsphere bs;        // Bounding sphere
};
```

## Skeleton-Specific Chunks

### OGF_S_BONE_NAMES Format
```cpp
u32 bone_count;
for each bone {
    stringZ bone_name;      // Null-terminated string
    stringZ parent_name;    // Empty string if root bone
    Fobb obb;              // Oriented bounding box (60 bytes)
}
```

### OGF_S_IKDATA Format
```cpp
u32 OGF_IKDATA_VERSION;    // Currently 1
for each bone {
    stringZ game_material;  // Material name
    SBoneShape shape;       // Collision shape
    IK_data {               // IK constraints
        u32 type;           // Joint type (rigid, joint, wheel, slider, none)
        f32 limits[3];      // Joint limits
        f32 spring_factor;
        f32 damping_factor;
        u32 ik_flags;
    }
    Fvector3 rest_rotate;   // Rest rotation (Euler angles)
    Fvector3 rest_offset;   // Rest offset (local position)
    float mass;
    Fvector3 center_of_mass;
}
```

### OGF_S_SMPARAMS Format (Motion Parameters)
```cpp
u16 version;               // Versions: 0, 1, 2, 3, 4

// Bone parts
u16 part_count;
for each part {
    stringZ part_name;
    u16 bone_count;
    for each bone {
        if (version == 0 || version == 1) {
            u32 bone_id;
        } else if (version == 2) {
            stringZ bone_name;
        } else if (version == 3 || version == 4) {
            stringZ bone_name;
            u32 bone_id;
        }
    }
}

// Motion definitions
u16 motion_count;
for each motion {
    stringZ motion_name;
    u32 flags;             // ESMFlags
    u16 bone_or_part;      // Bone or part ID (0xFFFF = no specific part)
    u16 motion_id;         // Motion index
    float speed;
    float power;
    float accrue;
    float falloff;
    
    // Motion marks (version 4 only)
    if (version == 4) {
        u32 marks_count;
        for each mark {
            string_rn mark_name;    // IMPORTANT: String ending with \r\n, not null-terminated!
            u32 interval_count;
            for each interval {
                float first;    // Start time
                float second;   // End time
            }
        }
    }
}
```

**CRITICAL: Motion Mark String Format**
In version 4, motion mark names use a special format:
- Terminated by `\r\n` (0x0D 0x0A) instead of null byte
- Must read until `\n` (0x0A) is found
- Strip trailing `\r` (0x0D) if present

### OGF_S_MOTIONS Format (Motion Keys)
```cpp
// Sub-chunk 0: Motion count
u32 motion_count;

// Sub-chunks 1+: Motion data
for each motion (chunk_id = motion_index + 1) {
    stringZ motion_name;
    u32 length;            // Number of frames
    
    for each bone {
        u8 flags;          // Key flags
        
        // Rotation keys
        if (!(flags & flRKeyAbsent)) {
            u32 crc32;     // CRC of rotation data
            CKeyQR keys[length];
        }
        
        // Translation keys
        if (flags & flTKeyPresent) {
            u32 crc32;     // CRC of translation data
            if (flags & flTKey16IsBit) {
                CKeyQT16 keys[length];  // 16-bit compression
            } else {
                CKeyQT8 keys[length];   // 8-bit compression
            }
            Fvector3 size;     // Compression range
            Fvector3 init;     // Initial position
        } else {
            Fvector3 init;     // Initial position only
        }
    }
}
```

## Motion Compression

### Key Flags
```cpp
enum {
    flTKeyPresent  = (1 << 0),  // Translation keys present
    flRKeyAbsent   = (1 << 1),  // Rotation keys absent (single key)
    flTKey16IsBit  = (1 << 2),  // Use 16-bit translation compression
};
```

### Decompression Formulas

**Quaternion Decompression:**
```cpp
float scale = 1.0f / 32767.0f;
quat.x = packed.x * scale;
quat.y = packed.y * scale;
quat.z = packed.z * scale;
quat.w = packed.w * scale;
quat.normalize();
```

**Translation Decompression:**
```cpp
// Formula: translation = init + (compressed * scale * size)
// scale = 1.0f / 127.0f (8-bit) or 1.0f / 32767.0f (16-bit)
```

## OMF (Object Motion Format)

### OMF File Types

1. **Game OMF Files**: Use OGF chunk format
   - Contains OGF_S_MOTIONS (0x0E) and OGF_S_SMPARAMS (0x0F)
   - **NO bone count stored** - animates ALL skeleton bones

2. **SDK OMF Files**: Use OMF_CHUNK_* format (SDK only)
   - Different chunk structure
   - Includes bone count

### Reading OMF Files

**IMPORTANT**: When reading OMF for OGF skeletons:
- The motion data applies to ALL bones in the skeleton
- Do NOT look for bone count in the file
- Use the skeleton's bone count when creating animations

## Common Issues and Solutions

1. **Upside-Down Skeleton**: Coordinate system conversion needed
   - X-Ray uses Y-up left-handed
   - ozz uses Y-up right-handed
   - Need to handle handedness conversion

2. **All Bones at (0,0,0)**: Animation playback matrix extraction
   - Extract translation from 4th column of transformation matrix
   - NOT from W components of rotation columns

3. **Reader Overflow**: Motion marks parsing
   - Use special \r\n terminated string reader for version 4

4. **Data Reading Order**: For motion compression
   - Read compressed keyframes first
   - Then read t_size and t_init parameters

## References

- xrSDK source: ExportSkeleton.cpp, FMesh.hpp
- blender-xray addon: Motion import/export implementation
- OpenXRay: CKinematicsAnimated implementation