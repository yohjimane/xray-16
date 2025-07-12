# IKinematicsAnimated Interface Analysis

## Overview
IKinematicsAnimated is the primary interface for skeletal animation in the X-Ray engine. It extends IKinematics and provides methods for animation playback, blending, and motion management.

## Interface Methods Documentation

### Motion Management

#### `MotionID ID_Cycle(LPCSTR N)` / `MotionID ID_Cycle(shared_str N)`
- **Purpose**: Get motion ID for a cycle animation by name
- **Returns**: MotionID (16-bit slot + 16-bit index)
- **ozz equivalent**: Need to map string names to ozz::animation::Animation*

#### `MotionID ID_Cycle_Safe(LPCSTR N)` / `MotionID ID_Cycle_Safe(shared_str N)`
- **Purpose**: Safe version that returns invalid ID if animation not found
- **ozz equivalent**: Same as above with error handling

#### `MotionID ID_FX(LPCSTR N)` / `MotionID ID_FX_Safe(LPCSTR N)`
- **Purpose**: Get motion ID for FX (one-shot) animations
- **ozz equivalent**: Same animation lookup, different playback mode

### Animation Playback

#### `CBlend* PlayCycle(...)` (multiple overloads)
- **Purpose**: Start playing a looping animation
- **Parameters**:
  - `partition`: Body part to animate (0 = whole body)
  - `motion`: Animation ID
  - `bMixIn`: Whether to blend with existing animations
  - `Callback`: Function called on animation events
  - `channel`: Animation channel (0-3)
- **ozz equivalent**: `ozz::animation::SamplingJob` with blend weight

#### `CBlend* PlayFX(...)` 
- **Purpose**: Play one-shot animation (no loop)
- **Parameters**: Similar to PlayCycle but with `power_scale`
- **ozz equivalent**: Same as PlayCycle but stop at end

#### `void LL_CloseCycle(u16 partition, u8 mask_channel)`
- **Purpose**: Stop animations on partition/channel
- **ozz equivalent**: Remove from active animation list

### Blend Management

#### `void LL_IterateBlends(IterateBlendsCallback& callback)`
- **Purpose**: Iterate over all active blends
- **ozz equivalent**: Iterate over active animation handles

#### `u32 LL_PartBlendsCount(u32 bone_part_id)`
- **Purpose**: Get number of active blends for body part
- **ozz equivalent**: Count animations affecting partition

#### `CBlend* LL_PartBlend(u32 bone_part_id, u32 n)`
- **Purpose**: Get nth blend for body part
- **ozz equivalent**: Access specific animation handle

### Update System

#### `void UpdateTracks()`
- **Purpose**: Main update function (called from game loop)
- **ozz equivalent**: Run ozz jobs pipeline

#### `void LL_UpdateTracks(float dt, bool b_force, bool leave_blends)`
- **Purpose**: Low-level update with timing control
- **ozz equivalent**: 
  - `ozz::animation::SamplingJob` for each animation
  - `ozz::animation::BlendingJob` for mixing
  - `ozz::animation::LocalToModelJob` for transforms

### Motion Data Access

#### `CMotionDef* LL_GetMotionDef(MotionID id)`
- **Purpose**: Get motion metadata (speed, power, etc.)
- **ozz equivalent**: Store metadata separately

#### `CMotion* LL_GetRootMotion(MotionID id)`
- **Purpose**: Get root bone motion data
- **ozz equivalent**: Access ozz::animation::Animation track 0

#### `CMotion* LL_GetMotion(MotionID id, u16 bone_id)`
- **Purpose**: Get motion data for specific bone
- **ozz equivalent**: Access specific track in ozz::animation::Animation

### Low-Level Bone Processing

#### `void LL_BuldBoneMatrixDequatize(const CBoneData* bd, u8 channel_mask, SKeyTable& keys)`
- **Purpose**: Build bone transformation from compressed data
- **ozz equivalent**: ozz handles this internally in SamplingJob

#### `void LL_BoneMatrixBuild(CBoneInstance& bi, const Fmatrix* parent, const SKeyTable& keys)`
- **Purpose**: Build final bone matrix with parent transform
- **ozz equivalent**: ozz::animation::LocalToModelJob

### Additional Features

#### `void LL_AddTransformToBone(KinematicsABT::additional_bone_transform& offset)`
- **Purpose**: Add procedural bone offset (e.g., for aiming)
- **ozz equivalent**: Post-process ozz output transforms

#### `void LL_SetChannelFactor(u16 channel, float factor)`
- **Purpose**: Set global weight for animation channel
- **ozz equivalent**: Multiply blend weights by factor

### Callbacks

#### `void SetBlendDestroyCallback(IBlendDestroyCallback* cb)`
- **Purpose**: Callback when blend is destroyed
- **ozz equivalent**: Track animation handle lifecycle

#### `void SetUpdateTracksCalback(IUpdateTracksCallback* callback)`
- **Purpose**: Custom update logic injection
- **ozz equivalent**: Pre/post update hooks

## Key Data Structures

### MotionID
```cpp
struct MotionID {
    u16 slot;  // Motion collection slot
    u16 idx;   // Index within slot
};
```

### CBlend
- Represents an active animation instance
- Contains timing, weight, and state info
- Manages blend in/out transitions

### CPartition
- Defines body part boundaries
- Allows independent animation of parts

## ozz Integration Strategy

1. **Motion ID Mapping**: Create bidirectional map between MotionID and ozz animations
2. **Blend Pool**: Replace CBlend with ozz-compatible animation handles
3. **Update Pipeline**: Replace with ozz job system
4. **Compatibility Layer**: Implement IKinematicsAnimated with ozz backend

## Critical Paths

1. **Animation Update**: UpdateTracks() → LL_UpdateTracks() → Per-bone processing
2. **Blend Management**: PlayCycle() → Allocate CBlend → Update weights → Apply to bones
3. **Transform Pipeline**: Sample keys → Blend → Build matrices → Apply to visual

## Next Steps

1. Map each method to ozz equivalent functionality
2. Design data structure conversions
3. Implement compatibility layer incrementally
4. Maintain API compatibility for game code