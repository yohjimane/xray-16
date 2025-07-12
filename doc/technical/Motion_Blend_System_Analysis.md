# X-Ray Motion & Blend System Analysis

## CMotion - Compressed Animation Data

### Structure
```cpp
class CMotion {
    u32 _flags : 8;        // Motion flags
    u32 _count : 24;       // Number of keyframes
    
    ref_smem<CKeyQR> _keysR;      // Rotation keys (quaternion, 16-bit)
    ref_smem<CKeyQT8> _keysT8;    // Translation keys (8-bit)
    ref_smem<CKeyQT16> _keysT16; // Translation keys (16-bit)
    Fvector _initT;               // Initial translation
    Fvector _sizeT;               // Translation range for decompression
};
```

### Key Compression
- **Rotation**: Stored as 16-bit quantized quaternions (CKeyQR)
- **Translation**: 
  - 8-bit (CKeyQT8) for small movements
  - 16-bit (CKeyQT16) for larger movements
- **Flags indicate** which data is present (rotation/translation)

### Decompression Formula
```cpp
// Rotation decompression
Fquaternion Q;
Q.x = float(keyQR.x) * (1.f/32767.f);
Q.y = float(keyQR.y) * (1.f/32767.f);
Q.z = float(keyQR.z) * (1.f/32767.f);
Q.w = float(keyQR.w) * (1.f/32767.f);

// Translation decompression (8-bit)
Fvector T;
T.x = _initT.x + (float(keyQT8.x) / 127.f) * _sizeT.x;
T.y = _initT.y + (float(keyQT8.y) / 127.f) * _sizeT.y;
T.z = _initT.z + (float(keyQT8.z) / 127.f) * _sizeT.z;
```

## CBlend - Runtime Animation Instance

### Purpose
CBlend represents an active animation that is being played. It manages:
- Animation timing and playback speed
- Blend weight transitions
- Callbacks for animation events
- Channel assignment

### Core Properties
```cpp
class CBlend {
    // Timing
    float timeCurrent;      // Current playback time
    float timeTotal;        // Total animation length
    float speed;            // Playback speed multiplier
    
    // Blending
    float blendAmount;      // Current blend weight (0-1)
    float blendPower;       // Target blend weight
    float blendAccrue;      // Blend-in speed
    float blendFalloff;     // Blend-out speed
    
    // Identity
    MotionID motionID;      // Which animation to play
    u16 bone_or_part;       // Bone or partition affected
    u8 channel;             // Animation channel (0-3)
    
    // Control
    bool playing;           // Is animation active
    bool stop_at_end;       // Stop vs loop
    bool fall_at_end;       // Auto-blend out when done
    
    // Callback
    PlayCallback Callback;  // Animation event callback
    void* CallbackParam;    // User data for callback
};
```

### Blend States
```cpp
enum ECurvature {
    eFREE_SLOT = 0,  // Available for allocation
    eAccrue,         // Blending in (weight increasing)
    eFalloff         // Blending out (weight decreasing)
};
```

### State Transitions
```
[eFREE_SLOT] --allocate--> [eAccrue] --full weight--> [eAccrue/eFalloff]
                                                              |
                                                              v
[eFREE_SLOT] <--deallocate-- [eFalloff] <--blend out--------/
```

### Update Logic

#### Time Update
```cpp
bool CBlend::update_time(float dt) {
    if (!playing) return false;
    
    timeCurrent += dt * speed;
    
    if (!stop_at_end) {
        // Loop animation
        if (timeCurrent > timeTotal)
            timeCurrent -= timeTotal;
    } else {
        // Clamp at end
        if (timeCurrent > timeTotal) {
            timeCurrent = timeTotal;
            return true; // Animation finished
        }
    }
    return false;
}
```

#### Blend Weight Update
```cpp
void CBlend::update_play(float dt, PlayCallback _Callback) {
    // Increase blend weight during accrue phase
    blendAmount += dt * blendAccrue * blendPower;
    clamp(blendAmount, 0.f, blendPower);
    
    if (update_time(dt)) {
        // Animation reached end
        if (_Callback && stop_at_end_callback)
            _Callback(this);
            
        if (fall_at_end) {
            // Transition to falloff
            blend = eFalloff;
            blendFalloff = 2.f;
        }
    }
}
```

## Motion Storage System

### Hierarchy
```
m_Motions (MotionsSlotVec)
    └── SMotionsSlot[0]
        ├── shared_motions (cycles & fx)
        │   ├── cycles: map<name, index>
        │   └── fx: map<name, index>
        └── bone_motions[bone_id]
            └── vector<CMotion>
```

### Motion Types
1. **Cycles** - Looping animations (idle, walk, run)
2. **FX** - One-shot effects (attack, death, reload)

### MotionID System
```cpp
struct MotionID {
    u16 slot;  // Which motion collection
    u16 idx;   // Index within that collection
};
```

## Channel System

### Purpose
- Allows 4 independent animation layers
- Each channel has global weight factor
- Enables complex animation mixing

### Usage Example
```cpp
// Channel 0: Base movement
PlayCycle("walk", channel=0);

// Channel 1: Upper body override
PlayCycle("reload", channel=1);

// Channel 2: Facial animation
PlayCycle("talk", channel=2);

// Channel 3: Damage reactions
PlayFX("hit", channel=3);
```

## ozz-animation Mapping

### CMotion → ozz::animation::Animation
- Compressed keyframes → ozz optimized format
- Shared memory → ozz resource management

### CBlend → Animation Handle
```cpp
struct AnimationHandle {
    size_t animation_index;
    float time;
    float weight;
    float speed;
    bool is_looping;
    PlayCallback callback;
    void* callback_param;
};
```

### Blend Pool → Handle Pool
- Fixed-size pool → Dynamic vector
- State machine → Simple flags
- Manual update → ozz job system

### Update Pipeline Comparison

#### X-Ray Current
```
UpdateTracks()
  └── For each blend
      ├── Update time
      ├── Update weight
      └── Sample keys per bone
```

#### ozz Replacement
```
UpdateTracks()
  └── For each animation
      ├── Update handle state
      ├── SamplingJob (SIMD)
      └── BlendingJob (SIMD)
```

## Key Insights for ozz Integration

1. **Preserve CBlend Interface** - Game code expects this API
2. **Map Blend States** - Convert to simple handle states
3. **Channel Weights** - Apply as layer weights in BlendingJob
4. **Callbacks** - Trigger at same points (animation end, etc.)
5. **Motion IDs** - Keep for backward compatibility
6. **Compression** - Let ozz handle optimization

## Performance Opportunities

### Current Bottlenecks
- Per-bone key sampling
- Scalar decompression
- Multiple blend iterations
- Poor cache locality

### ozz Advantages
- SIMD key sampling (4x speedup)
- Optimized compression
- Single-pass blending
- Cache-friendly layout