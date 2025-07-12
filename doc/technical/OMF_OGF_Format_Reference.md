# X-Ray Engine OGF/OMF File Format Reference

## Overview
This document contains comprehensive technical specifications for X-Ray Engine's OGF (Object Geometry Format) and OMF (Object Motion Format) file formats, compiled from xrSDK source analysis.

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
    MT_3DFLUIDVOLUME       = 12,   // 3D fluid volume
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
    OGF_COLLISION_VERTICES  = 25,   // Collision vertices
    OGF_COLLISION_INDICES   = 26,   // Collision indices
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
    Fvector3 rest_rotate;   // Rest rotation
    Fvector3 rest_offset;   // Rest offset
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

**CRITICAL: Motion Mark String Format (MAJOR BUG SOURCE)**
In version 4, motion mark names use a special format that causes reader overflow if handled incorrectly:
- Terminated by `\r\n` (0x0D 0x0A) instead of null byte
- Must read until `\n` (0x0A) is found
- Strip trailing `\r` (0x0D) if present
- Standard `r_stringZ()` WILL FAIL and cause reader overflow
- This is different from standard null-terminated strings used elsewhere
- Implementation:
```cpp
xr_string mark_name_str;
char ch;
while (!reader.eof()) {
    ch = reader.r_u8();
    if (ch == '\n') break;
    mark_name_str += ch;
}
if (!mark_name_str.empty() && mark_name_str.back() == '\r') {
    mark_name_str.pop_back();
}
```

### OGF_S_MOTIONS Format (Motion Keys)
```cpp
// Sub-chunk 0: Motion count
u32 motion_count;

// Sub-chunks 1+: Motion data
for each motion (chunk_id = motion_index + 1) {
    stringZ motion_name;
    u32 length;            // Number of frames
    
    for each bone {
        u8 flags;          // Key flags (see below)
        
        // Rotation keys
        if (!(flags & flRKeyAbsent)) {
            if (flags & flRKeyAbsent) {
                CKeyQR key;    // Single key
            } else {
                u32 crc32;     // CRC of rotation data
                CKeyQR keys[length];
            }
        }
        
        // Translation keys
        if (flags & flTKeyPresent) {
            u32 crc32;         // CRC of translation data
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

### Compressed Key Structures
```cpp
// Quantized quaternion rotation
struct CKeyQR {
    s16 x, y, z, w;    // Range: -32767 to 32767
};

// 8-bit compressed translation
struct CKeyQT8 {
    s8 x1, y1, z1;     // Range: -127 to 127
};

// 16-bit compressed translation  
struct CKeyQT16 {
    s16 x1, y1, z1;    // Range: -32767 to 32767
};
```

### Compression Constants
- Rotation quantization: `KEY_Quant = 32767`
- Translation 8-bit range: `127.0f`
- Translation 16-bit range: `32767.0f`
- 16-bit mode triggers when motion range > 1.5m

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

**Translation Decompression (8-bit):**
```cpp
float scale = 1.0f / 127.0f;
trans.x = init.x + (compressed.x * scale * size.x);
trans.y = init.y + (compressed.y * scale * size.y);
trans.z = init.z + (compressed.z * scale * size.z);
```

**Translation Decompression (16-bit):**
```cpp
float scale = 1.0f / 32767.0f;
trans.x = init.x + (compressed.x * scale * size.x);
trans.y = init.y + (compressed.y * scale * size.y);
trans.z = init.z + (compressed.z * scale * size.z);
```

## Motion Flags (ESMFlags)
```cpp
enum ESMFlags {
    esmFX           = (1 << 0),  // Effect motion
    esmStopAtEnd    = (1 << 1),  // Stop at end
    esmNoMix        = (1 << 2),  // No mixing
    esmSyncPart     = (1 << 3),  // Sync part
    esmUseFootSteps = (1 << 4),  // Use foot steps
    esmRootMover    = (1 << 5),  // Root mover
    esmIdle         = (1 << 6),  // Idle animation
    esmUseWeaponBone = (1 << 7), // Use weapon bone
};
```

## OMF (Object Motion Format)

OMF files are essentially motion-only exports containing:
- OGF_S_MOTIONS chunk (0x0E) - Motion key data
- OGF_S_SMPARAMS chunk (0x0F) - Motion parameters

Game OMF files use OGF chunk format directly, not the SDK's OMF_CHUNK_* format.

## File Reading Process

### 1. Identify Format
- Check for OGF_S_MOTIONS (0x0E) and OGF_S_SMPARAMS (0x0F) chunks
- If present, use OGF motion format reader
- Otherwise, check for OMF_CHUNK_* format (SDK only)

### 2. Read Motion Parameters (OGF_S_SMPARAMS)
- Version check (should be 4)
- Read bone parts mapping
- Read motion definitions with marks

### 3. Read Motion Data (OGF_S_MOTIONS)
- Sub-chunk 0: Motion count
- Sub-chunks 1+: Per-motion compressed data

### 4. Decompress Motion Keys
- Apply appropriate decompression based on flags
- Reconstruct full transformation data

## Common Issues and Solutions

1. **Reader Overflow**: Usually caused by incorrect version handling or motion marks parsing
   - Motion marks use `\r\n` terminated strings in version 4
   - Check params version before attempting to read motion marks
   
2. **Missing Parent Bones**: Parent names must be resolved to indices
   - Empty parent name means root bone
   - Parent relationships are string-based in OGF
   
3. **Compression Artifacts**: Ensure proper quantization constants are used
   - Rotation: divide by 32767.0f
   - Translation: scale by size/127.0f or size/32767.0f
   
4. **Motion Synchronization**: Check esmSyncPart flag for synchronized animations
   - Bone_or_part = 0xFFFF means no specific part

5. **String Encoding**: X-Ray uses Windows-1251 (cp1251) encoding for strings

## Coordinate System Conversion (X-Ray to ozz-animation)

**Critical Issue**: X-Ray uses Y-up coordinate system, ozz-animation/OpenGL uses Z-up.

**Solution**: Apply coordinate transformation matrix similar to blender-xray:
```cpp
// X-Ray: (X, Y, Z) → ozz: (X, Z, -Y)
Fmatrix converted_matrix;
converted_matrix.i.set(xray_matrix.i.x, xray_matrix.i.z, -xray_matrix.i.y);  // X-axis
converted_matrix.j.set(xray_matrix.j.x, xray_matrix.j.z, -xray_matrix.j.y);  // Y-axis  
converted_matrix.k.set(xray_matrix.k.x, xray_matrix.k.z, -xray_matrix.k.y);  // Z-axis
converted_matrix.c.set(xray_matrix.c.x, xray_matrix.c.z, -xray_matrix.c.y);  // Translation
```

**Animation Keyframe Conversion**:
```cpp
// Translation keys: X-Ray Y-up to ozz Z-up
key.value = ozz::math::Float3(t.x, t.z, -t.y);

// Rotation keys: Quaternion coordinate conversion
key.value = ozz::math::Quaternion(q.x, q.z, -q.y, q.w);
```

**Importance**: Without this conversion, skeletons appear in wrong orientation or are invisible in ozz viewers.

## Implementation Notes from blender-xray

1. **Motion Mark Reading (Version 4)**:
   ```cpp
   // Read string until \n, strip trailing \r
   void read_string_rn(IReader& reader, shared_str& str) {
       xr_string temp;
       char ch;
       while (!reader.eof()) {
           ch = reader.r_u8();
           if (ch == '\n') break;
           temp += ch;
       }
       // Remove trailing \r if present
       if (!temp.empty() && temp.back() == '\r') {
           temp.pop_back();
       }
       str = temp.c_str();
   }
   ```

2. **Bone Part Handling**:
   - Parts group bones for animation purposes
   - Motion can target entire part or specific bone
   - Part ID 0xFFFF indicates motion affects all bones

3. **Motion Storage**:
   - Each motion stored as sub-chunk in OGF_S_MOTIONS
   - Sub-chunk 0 contains motion count
   - Sub-chunks 1+ contain actual motion data

## Related File Extensions
- `.ogf` - Object Geometry Format (meshes with optional skeleton)
- `.omf` - Object Motion Format (motions only)
- `.skl` - Skeleton motion (individual motion)
- `.skls` - Skeleton motions (motion collection)
- `.anm` - Animation (object motion)
- `.anms` - Animations (motion collection)
- `.object` - SDK object file

## Version History
- OGF Format Version: 4 (current)
- OGF SM Params Version: 4 (current)
- OGF IK Data Version: 1 (current)