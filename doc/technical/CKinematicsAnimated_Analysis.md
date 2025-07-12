# CKinematicsAnimated Implementation Analysis

## Overview
CKinematicsAnimated is the concrete implementation of IKinematicsAnimated in the X-Ray engine. It manages skeletal animation playback, blending, and bone transformations.

## Class Architecture

### Inheritance Hierarchy
```cpp
CKinematicsAnimated : public CKinematics, public IKinematicsAnimated
```

### Key Data Members

#### Motion Storage
```cpp
struct SMotionsSlot {
    shared_motions motions;      // Shared motion definitions (cycles & fx)
    BoneMotionsVec bone_motions; // Per-bone motion data
};
MotionsSlotVec m_Motions;       // Vector of motion slots
```

#### Blend Management
```cpp
svector<CBlend, MAX_BLENDED_POOL> blend_pool;  // Fixed-size pool of blends
BlendSVec blend_cycles[MAX_PARTS];             // Active cycle blends per partition
BlendSVec blend_fx;                             // Active FX (one-shot) blends
CBlendInstance* blend_instances;                // Per-bone blend instances
```

#### Animation Channels
```cpp
animation::channels channels;  // Channel weights (4 channels, 0-1.0f each)
```

## Animation Update Pipeline

### 1. UpdateTracks() - Main Entry Point
```cpp
void CKinematicsAnimated::UpdateTracks() {
    // Check if already updated this frame
    if (Update_LastTime == Device.dwTimeGlobal) return;
    
    // Calculate delta time (capped at 66ms)
    u32 DT = Device.dwTimeGlobal - Update_LastTime;
    if (DT > 66) DT = 66;
    float dt = float(DT) / 1000.f;
    
    // Optional callback for custom update logic
    if (GetUpdateTracksCalback()) {
        if ((*GetUpdateTracksCalback())(dt, *this))
            Update_LastTime = Device.dwTimeGlobal;
        return;
    }
    
    Update_LastTime = Device.dwTimeGlobal;
    LL_UpdateTracks(dt, false, false);
}
```

### 2. LL_UpdateTracks() - Process All Blends
```cpp
void CKinematicsAnimated::LL_UpdateTracks(float dt, bool b_force, bool leave_blends) {
    // Update cycle blends for each partition
    for (u16 part = 0; part < MAX_PARTS; part++) {
        for (auto& blend : blend_cycles[part]) {
            if (blend->update(dt, blend->Callback) && !leave_blends) {
                DestroyCycle(*blend);
                // Remove from list
            }
        }
    }
    
    // Update FX blends
    LL_UpdateFxTracks(dt, b_force);
}
```

### 3. Bone Transform Calculation
```cpp
void CKinematicsAnimated::BuildBoneMatrix(
    const CBoneData* bd, 
    CBoneInstance& bi, 
    const Fmatrix* parent, 
    u8 channel_mask) 
{
    // Step 1: Gather animation keys from all active blends
    SKeyTable keys;
    LL_BuldBoneMatrixDequatize(bd, channel_mask, keys);
    
    // Step 2: Build final bone matrix
    LL_BoneMatrixBuild(bi, parent, keys);
    
    // Step 3: Apply additional transforms (procedural)
    CalculateBonesAdditionalTransforms(bd, bi, parent, channel_mask);
}
```

## Blend System

### CBlend Structure
- Represents an active animation instance
- Contains timing, weight, and state information
- Manages blend transitions (accrue, fixed, falloff)

### Blend States
1. **eFREE_SLOT** - Available for allocation
2. **eAccrue** - Blending in
3. **eFixed** - Fully blended, playing normally  
4. **eFalloff** - Blending out

### Blend Allocation
```cpp
CBlend* CKinematicsAnimated::IBlend_Create() {
    UpdateTracks(); // Ensure up-to-date
    
    // Find free slot in pool
    for (auto& blend : blend_pool) {
        if (blend.blend_state() == CBlend::eFREE_SLOT)
            return &blend;
    }
    
    FATAL("Too many blended motions requested");
}
```

## Motion System

### Motion Types
1. **Cycles** - Looping animations (walk, idle, etc.)
2. **FX** - One-shot animations (attacks, deaths, etc.)

### Motion ID System
```cpp
struct MotionID {
    u16 slot;  // Motion collection slot
    u16 idx;   // Index within slot
};
```

### Motion Lookup
```cpp
MotionID CKinematicsAnimated::ID_Cycle(LPCSTR name) {
    // Search through motion slots in reverse order (newest first)
    for (int k = m_Motions.size() - 1; k >= 0; --k) {
        auto& slot = m_Motions[k];
        auto it = slot.motions.cycle()->find(name);
        if (it != slot.motions.cycle()->end()) {
            return MotionID(k, it->second);
        }
    }
    // Assert if not found
}
```

## Partition System

### Purpose
- Allows different animations on different body parts
- Enables upper/lower body split (e.g., run + reload)

### Implementation
```cpp
CPartition* m_Partition;  // Defines bone groups

// Playing animation on specific partition
CBlend* PlayCycle(u16 partition, MotionID motion, ...);
```

## Key Algorithms

### 1. Bone Matrix Dequantization
- Extracts rotation/translation keys from compressed motion data
- Interpolates between keyframes based on current time
- Handles multiple blends per bone

### 2. Blend Weight Calculation
- Accrue phase: Weight increases over time
- Fixed phase: Weight remains constant
- Falloff phase: Weight decreases to zero

### 3. Channel System
- 4 independent animation channels
- Each channel has a global weight factor
- Allows layering of animations

## ozz Integration Points

### Replace
1. **Blend Pool** → ozz animation handles
2. **Motion Storage** → ozz::animation::Animation
3. **Key Interpolation** → ozz::animation::SamplingJob
4. **Blend Mixing** → ozz::animation::BlendingJob
5. **Transform Pipeline** → ozz::animation::LocalToModelJob

### Maintain
1. **IKinematicsAnimated Interface** - Full API compatibility
2. **MotionID System** - For game code compatibility
3. **Partition Support** - Body part animation
4. **Callback System** - Animation events
5. **Channel Weights** - Global animation factors

## Performance Characteristics

### Current System
- Per-bone key sampling
- Scalar quaternion interpolation  
- Deep call hierarchy
- Cache-unfriendly memory layout

### ozz Advantages
- SIMD processing (4 bones at once)
- Optimized memory layout (SoA)
- Fewer function calls
- Better cache utilization