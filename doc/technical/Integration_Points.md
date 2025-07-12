# X-Ray Animation System Integration Points

## Overview
This document identifies all the critical integration points where the animation system interfaces with other engine components. These points must be maintained for ozz-animation integration.

## 1. Rendering Integration

### Visual System
```cpp
// IRenderVisual is the base for all rendered objects
IKinematicsAnimated* animated = smart_cast<IKinematicsAnimated*>(Visual());
```

### Bone Matrices for Rendering
- **Location**: `CalculateBones()` → GPU skinning
- **Format**: Array of 4x4 matrices in world space
- **Usage**: Vertex shader bone palette

### HUD Integration
- First-person weapon/hands rendering
- Separate coordinate space
- Special update path for smooth interpolation

## 2. Physics Integration

### Ragdoll System
- **Activation**: On death, switches from animation to physics
- **Bone Mapping**: Physics bones mapped to animation bones
- **Transform Sync**: Last animation pose → initial physics pose

### Character Controller
- **Root Motion**: Extracted from animations for movement
- **Collision**: Bone positions for hit detection
- **IK Corrections**: Foot placement on terrain

## 3. Game Logic Integration

### Actor (Player) Animation
```cpp
// ActorAnimation.cpp usage pattern
void CActor::g_SetAnimation(u32 mstate_rl) {
    IKinematicsAnimated* K = smart_cast<IKinematicsAnimated*>(Visual());
    
    // Play different animations based on state
    if (mstate_rl & mcFwd)
        K->PlayCycle("walk_fwd");
    else if (mstate_rl & mcCrouch)
        K->PlayCycle("crouch_idle");
}
```

### Weapon Animation
- Synchronized with actor animations
- Blend between states (idle, aim, reload)
- Animation callbacks for:
  - Shell ejection
  - Magazine detach/attach
  - Sound triggers

### AI/Monster Animation
- State machine driven
- Multiple animation channels:
  - Movement (base layer)
  - Attack (override layer)
  - Damage (additive layer)

## 4. Script System Integration

### Lua Bindings
```lua
-- Script access to animation system
function play_animation(obj, anim_name)
    local visual = obj:get_visual()
    if visual then
        visual:play_cycle(anim_name)
    end
end
```

### Exposed Methods
- `play_cycle()` - Start looping animation
- `play_fx()` - Play one-shot animation
- `set_animation_speed()` - Modify playback rate
- `get_animation_time()` - Query current time

## 5. Callback System

### Animation Events
```cpp
// Callback signature
typedef void (*PlayCallback)(CBlend* B);

// Common callbacks:
void OnReloadEnd(CBlend* B) {
    CWeapon* W = (CWeapon*)B->CallbackParam;
    W->OnAnimationEnd(W->GetState());
}
```

### Callback Types
1. **Animation End** - Fired when animation completes
2. **Animation Mark** - Fired at specific keyframes
3. **Blend Destroy** - Fired when blend is freed

## 6. Bone Manipulation

### Procedural Bone Control
```cpp
// Aim adjustment, look-at, etc.
virtual void Bone_Calculate(CBoneData* bd, Fmatrix* parent) {
    // Base animation
    inherited::Bone_Calculate(bd, parent);
    
    // Procedural overlay
    if (bd == m_head_bone) {
        Fmatrix rotation;
        rotation.setHPB(look_angles);
        bd->mTransform.mulA_43(rotation);
    }
}
```

### Bone Callbacks
- Per-bone procedural animation
- IK solvers
- Physics constraints

## 7. Save/Load System

### Animation State Persistence
- Current animation time
- Active blend list
- Motion IDs
- Callback preservation

## 8. Network Synchronization

### Multiplayer Animation Sync
- Motion ID replication
- Time synchronization
- State interpolation
- Compression for bandwidth

## 9. Tool Integration

### X-Ray SDK/Editor
- Animation preview
- Blend tree editing
- Motion extraction
- Compression settings

## Critical Integration Patterns

### 1. Smart Pointer Usage
```cpp
// Common pattern throughout codebase
IKinematicsAnimated* K = smart_cast<IKinematicsAnimated*>(Visual());
if (K) {
    // Use animation interface
}
```

### 2. Motion ID System
```cpp
// Game code stores motion IDs
class CStateManager {
    MotionID m_idle_motion;
    MotionID m_walk_motion;
    MotionID m_run_motion;
};
```

### 3. Partition System
```cpp
// Body part animation
K->PlayCycle(LEGS_PARTITION, walk_motion);
K->PlayCycle(TORSO_PARTITION, reload_motion);
```

### 4. Channel System
```cpp
// Layered animations
K->PlayCycle(motion, TRUE, callback, param, channel);
```

## ozz Integration Requirements

### Must Preserve
1. **IKinematicsAnimated Interface** - Complete API
2. **Motion ID System** - For game state storage
3. **Callback Signatures** - For game logic
4. **Bone Name Mapping** - For physics/script access
5. **Partition Support** - For split-body animation
6. **Channel Weights** - For animation layering

### Can Replace
1. **Internal Blend Pool** - Use ozz handles
2. **Motion Storage** - Use ozz animations
3. **Key Sampling** - Use ozz jobs
4. **Matrix Building** - Use ozz transforms

### New Wrapper Needs
1. **CBlend Compatibility** - Wrapper for ozz handles
2. **Motion ID Mapping** - Lookup table
3. **Callback Manager** - Track and fire events
4. **State Preservation** - For save/load

## Testing Requirements

### Functional Tests
1. Actor movement animations
2. Weapon handling animations
3. AI/Monster behaviors
4. Ragdoll activation
5. Script-driven animations

### Integration Tests
1. Physics synchronization
2. Network replication
3. Save/Load persistence
4. Tool compatibility

### Performance Tests
1. Frame time comparison
2. Memory usage
3. Cache efficiency
4. SIMD utilization