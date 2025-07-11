# X-Ray to ozz-animation Mapping Guide

## Critical Animation Paths

### 1. **Animation Update Path**
```
Game Loop
  ↓
IKinematicsAnimated::UpdateTracks()
  ↓
CKinematicsAnimated::LL_UpdateTracks(dt)
  ↓
For each active CBlend:
  - Update blend time/weight
  - Sample animation keys
  - Build transformation
  ↓
CKinematicsAnimated::CalculateBones()
  ↓
Apply to visual/physics
```

### 2. **Animation Start Path**
```
Game Code calls PlayCycle()
  ↓
Allocate CBlend from pool
  ↓
Set blend parameters (time, weight, callback)
  ↓
Find MotionID in m_Motions slots
  ↓
Add to active blends list
  ↓
Next UpdateTracks() will process it
```

### 3. **Bone Transform Path**
```
Sample compressed keys (CMotion)
  ↓
Dequantize rotation/translation
  ↓
Interpolate between keyframes
  ↓
Apply channel weights
  ↓
Blend multiple animations
  ↓
Build bone matrix hierarchy
  ↓
Convert to world space
```

## X-Ray to ozz Method Mapping

### Core Animation Types

| X-Ray Type | ozz Equivalent | Notes |
|------------|----------------|-------|
| `MotionID` | `size_t` animation index | Store in xr_unordered_map |
| `CMotion` | `ozz::animation::Animation` | Compressed animation data |
| `CBlend` | Animation handle struct | Track time, weight, state |
| `shared_motions` | `xr_vector<ozz::animation::Animation*>` | Animation collection |
| `CMotionDef` | Custom metadata struct | Store speed, power, accrue |

### Interface Method Mapping

#### Motion Management
```cpp
// X-Ray
MotionID ID_Cycle(LPCSTR name);

// ozz implementation
MotionID OzzKinematicsAnimated::ID_Cycle(LPCSTR name) {
    auto it = motion_name_map_.find(shared_str(name));
    if (it != motion_name_map_.end()) {
        return it->second;
    }
    
    // Load animation if not cached
    size_t anim_index = LoadAnimation(name);
    MotionID id;
    id.idx = (u16)anim_index;
    id.slot = 0; // Single slot for ozz
    
    motion_name_map_[shared_str(name)] = id;
    return id;
}
```

#### Animation Playback
```cpp
// X-Ray
CBlend* PlayCycle(MotionID motion, BOOL bMixIn, PlayCallback callback, LPVOID param);

// ozz implementation
CBlend* OzzKinematicsAnimated::PlayCycle(MotionID motion, BOOL bMixIn, 
                                         PlayCallback callback, LPVOID param) {
    // Allocate animation handle
    AnimationHandle handle;
    handle.animation_index = motion.idx;
    handle.weight = bMixIn ? 0.0f : 1.0f;  // Will blend in if mixing
    handle.time = 0.0f;
    handle.is_looping = true;
    handle.callback = callback;
    handle.callback_param = param;
    
    // Allocate CBlend for compatibility
    CBlend* blend = AllocateBlend();
    blend->motionID = motion;
    blend->timeCurrent = 0.0f;
    blend->timeTotal = animations_[motion.idx]->duration();
    blend->blendAmount = handle.weight;
    blend->Callback = callback;
    blend->CallbackParam = param;
    
    // Store mapping
    blend_to_handle_[blend] = active_handles_.size();
    active_handles_.push_back(handle);
    
    return blend;
}
```

#### Update System
```cpp
// X-Ray
void UpdateTracks();
void LL_UpdateTracks(float dt, bool b_force, bool leave_blends);

// ozz implementation
void OzzKinematicsAnimated::UpdateTracks() {
    LL_UpdateTracks(Device.fTimeDelta, false, false);
}

void OzzKinematicsAnimated::LL_UpdateTracks(float dt, bool b_force, bool leave_blends) {
    // 1. Update animation times and weights
    UpdateAnimationStates(dt);
    
    // 2. Sample all active animations
    xr_vector<ozz::math::SoaTransform> samples[MAX_CHANNELS];
    for (size_t i = 0; i < active_handles_.size(); ++i) {
        auto& handle = active_handles_[i];
        if (!handle.is_active) continue;
        
        SampleAnimation(handle, samples[handle.channel]);
    }
    
    // 3. Blend animations per channel
    BlendAnimations(samples);
    
    // 4. Convert to model space
    ozz::animation::LocalToModelJob ltm_job;
    ltm_job.skeleton = skeleton_.get();
    ltm_job.input = ozz::make_span(local_transforms_);
    ltm_job.output = ozz::make_span(model_transforms_);
    ltm_job.Run();
    
    // 5. Convert to X-Ray matrices
    UpdateBoneMatrices();
}
```

#### Bone Transform Pipeline
```cpp
// X-Ray
void LL_BuldBoneMatrixDequatize(const CBoneData* bd, u8 channel_mask, SKeyTable& keys);
void LL_BoneMatrixBuild(CBoneInstance& bi, const Fmatrix* parent, const SKeyTable& keys);

// ozz implementation - handled internally by ozz jobs
void OzzKinematicsAnimated::SampleAnimation(AnimationHandle& handle, 
                                           ozz::math::SoaTransform* output) {
    ozz::animation::SamplingJob sampling_job;
    sampling_job.animation = animations_[handle.animation_index].get();
    sampling_job.context = &sampling_contexts_[handle.animation_index];
    sampling_job.ratio = handle.time / handle.duration;
    sampling_job.output = ozz::make_span(output, skeleton_->num_soa_joints());
    
    if (!sampling_job.Run()) {
        Msg("! Animation sampling failed");
    }
}

void OzzKinematicsAnimated::BlendAnimations(
    const xr_vector<ozz::math::SoaTransform> samples[MAX_CHANNELS]) {
    
    ozz::animation::BlendingJob blending_job;
    blending_job.threshold = 0.1f;
    blending_job.rest_pose = skeleton_->joint_rest_poses();
    
    // Setup blend layers
    xr_vector<ozz::animation::BlendingJob::Layer> layers;
    for (const auto& handle : active_handles_) {
        if (handle.weight > 0.0f) {
            layers.emplace_back();
            auto& layer = layers.back();
            layer.transform = ozz::make_span(samples[handle.channel]);
            layer.weight = handle.weight * channel_factors_[handle.channel];
        }
    }
    
    blending_job.layers = ozz::make_span(layers);
    blending_job.output = ozz::make_span(local_transforms_);
    
    if (!blending_job.Run()) {
        Msg("! Animation blending failed");
    }
}
```

### Additional Features Mapping

#### Partitions
```cpp
// X-Ray uses partition masks to animate body parts
// ozz equivalent: Use joint masks in BlendingJob

struct PartitionMask {
    xr_vector<ozz::math::SimdInt4> joint_masks;  // SIMD masks for joints
    
    void SetupForPartition(u16 partition_id, const ozz::animation::Skeleton& skeleton);
};
```

#### Callbacks
```cpp
// X-Ray: CBlend tracks callbacks
// ozz: Track in AnimationHandle

struct AnimationHandle {
    PlayCallback callback;
    LPVOID callback_param;
    
    void CheckCallback() {
        if (callback && !is_looping && time >= duration) {
            // Create temporary CBlend for callback compatibility
            CBlend temp_blend;
            temp_blend.CallbackParam = callback_param;
            callback(&temp_blend);
        }
    }
};
```

#### Additional Bone Transforms (Procedural)
```cpp
// X-Ray: LL_AddTransformToBone()
// ozz: Post-process after LocalToModelJob

void ApplyAdditionalTransforms() {
    for (const auto& transform : additional_transforms_) {
        u16 bone_id = transform.bone_id;
        
        // Convert model transform back to local
        // Apply additional transform
        // Convert back to model space
        
        Fmatrix& bone_transform = bone_matrices_[bone_id];
        bone_transform.mulB_43(transform.transform);
    }
}
```

## Integration Strategy

### Phase 1: Core Wrapper
1. Implement OzzAnimationSystem as standalone class
2. Test with simple animations
3. Verify transform pipeline

### Phase 2: IKinematicsAnimated Implementation
1. Create OzzKinematicsAnimated implementing interface
2. Map all virtual methods to ozz operations
3. Maintain CBlend compatibility layer

### Phase 3: Game Integration
1. Replace CKinematicsAnimated with OzzKinematicsAnimated
2. Test with existing game code
3. Optimize performance

### Phase 4: Advanced Features
1. Implement partitions with joint masks
2. Add IK support using ozz IK jobs
3. Optimize memory usage

## Performance Considerations

### X-Ray Bottlenecks
- Per-bone key sampling
- Scalar quaternion interpolation
- Deep function call hierarchy

### ozz Advantages
- SIMD sampling (4 bones at once)
- Optimized blending pipeline
- Cache-friendly SoA layout
- Pre-computed sampling contexts

### Expected Improvements
- 25-40% faster animation updates
- Better CPU cache utilization
- Reduced memory bandwidth
- Scalable to more complex skeletons