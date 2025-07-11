# CLAUDE.md

## X-Ray to ozz-animation Integration Project

### Current Status (Phase 1, Week 2 - In Progress)
- ✅ xrAnimation module created and building successfully
- ✅ Dependencies resolved (ozz-animation, imgui, SDL2)
- ✅ Basic project structure established
- ✅ IKinematicsAnimated interface analyzed and documented
- ✅ Usage patterns traced in game code (ActorAnimation.cpp patterns)
- ✅ Critical paths identified
- ✅ X-Ray to ozz method mapping completed
- ✅ Basic ozz test harness created
- ✅ CKinematicsAnimated implementation analyzed
- ✅ Motion/Blend system (CMotion, CBlend) documented
- ✅ Integration points documented
- 🔄 OzzAnimationSystem core implementation started
- 🔄 Next: Complete OzzAnimationSystem and begin compatibility layer

### Project Overview
Integrating ozz-animation (modern, SIMD-optimized animation library) into OpenXRay to replace the legacy X-Ray animation system.

### Timeline: 10-12 Weeks

#### Phase 1: Foundation (Weeks 1-2) - CURRENT
- Week 1: Build system & initial study
- Week 2: Existing system analysis

#### Phase 2: Prototype (Weeks 3-5)
- Week 3: Basic ozz integration
- Week 4: X-Ray compatibility layer
- Week 5: Asset pipeline prototype

#### Phase 3: Core Implementation (Weeks 6-8)
- Week 6: Full animation system
- Week 7: Asset conversion tools
- Week 8: Game integration

#### Phase 4: Polish (Weeks 9-10)
- Week 9: Testing & optimization
- Week 10: Tools & documentation

### Key Technical Decisions

1. **Use X-Ray Types**: xr_vector, xr_unique_ptr, Fmatrix, Fvector
2. **Incremental Integration**: Keep old system working while building new
3. **Full API Compatibility**: Maintain IKinematicsAnimated interface
4. **Asset Conversion**: Build tools for OGF → ozz conversion

### Implementation Notes

#### Core Classes
- `OzzAnimationSystem`: Core ozz wrapper
- `OzzKinematicsAnimated`: IKinematicsAnimated implementation
- `XRayToOzzConverter`: Asset conversion framework

#### Critical Integration Points
1. Motion ID system
2. Blend management (CBlend compatibility)
3. Bone callbacks
4. Partition support
5. Physics integration
6. Script bindings

### Documentation Created
- `docs/IKinematicsAnimated_Analysis.md` - Complete interface documentation
- `docs/XRay_to_ozz_Mapping.md` - Method mapping and integration strategy

### Test Infrastructure
- `tests/test_ozz_basic.cpp` - Basic ozz functionality tests
- `xrAnimation.cpp/h` - Module initialization and test runner

### Next Immediate Steps (Week 2)
1. Analyze CKinematicsAnimated implementation
2. Study motion/blend system (CMotion, CBlend)
3. Document integration points
4. Begin OzzAnimationSystem core implementation

### Important Reminders
- Always use X-Ray file system (FS.r_open)
- Use Msg() for logging, not printf/cout
- Memory: xr_new/xr_delete, not new/delete
- Fquaternion/Fvector use .set(), not constructors
- Test continuously, maintain working build

### Key Implementation Insights (Week 2)

#### CKinematicsAnimated Architecture
- **Blend Pool**: Fixed-size pool `svector<CBlend, MAX_BLENDED_POOL>`
- **Motion Storage**: `MotionsSlotVec` with shared_motions and per-bone data
- **Update Pipeline**: UpdateTracks → LL_UpdateTracks → Per-blend update → Bone calculation
- **Partition System**: Allows independent animation of body parts (torso/legs)
- **Channel System**: 4 channels with global weight factors

#### Animation Usage Patterns (from ActorAnimation.cpp)
```cpp
// Common pattern for state-based animation
IKinematicsAnimated* K = smart_cast<IKinematicsAnimated*>(Visual());
if (mstate_rl & mcFwd)
    M_legs = AS->legs_fwd;
else if (mstate_rl & mcBack)
    M_legs = AS->legs_back;

// Torso override for weapons
STorsoWpn* TW = &ST->m_torso[weapon_slot];
M_torso = W->IsZoomed() ? TW->zoom : TW->moving[moving_idx];
```

#### Integration Requirements
1. **Smart pointer casting**: Game code uses `smart_cast<IKinematicsAnimated*>(Visual())`
2. **Motion IDs**: Stored in game state objects (SActorState, STorsoWpn)
3. **Callbacks**: Critical for state transitions (legs_play_callback)
4. **Bone callbacks**: Per-bone procedural animation (HeadCallback, SpinCallback)

### ozz-animation SIMD Access Patterns

ozz-animation uses SIMD types (SSE/NEON) for performance. You cannot access members directly:

**WRONG:**
```cpp
ozz::math::SimdFloat4 vec;
float x = vec.x;  // ERROR: 'x' is not a member of '__m128'

ozz::math::Float4x4 matrix;
float m11 = matrix.cols[0].x;  // ERROR: cols[0] is SimdFloat4
```

**CORRECT:**
```cpp
// For SimdFloat4 - use GetX/Y/Z/W functions
ozz::math::SimdFloat4 vec;
float x = ozz::math::GetX(vec);
float y = ozz::math::GetY(vec);
float z = ozz::math::GetZ(vec);
float w = ozz::math::GetW(vec);

// For Float4x4 matrices
ozz::math::Float4x4 matrix;
float m11 = ozz::math::GetX(matrix.cols[0]);
float m12 = ozz::math::GetX(matrix.cols[1]);
float m21 = ozz::math::GetY(matrix.cols[0]);

// For SoaTransform (4 transforms stored in SoA layout)
ozz::math::SoaTransform transforms;
// Access translation of transform 0
float tx0 = ozz::math::GetX(transforms.translation.x);
// Access translation of transform 1  
float tx1 = ozz::math::GetY(transforms.translation.x);
// Access translation of transform 2
float tx2 = ozz::math::GetZ(transforms.translation.x);
// Access translation of transform 3
float tx3 = ozz::math::GetW(transforms.translation.x);
```

**Key SIMD Types in ozz:**
- `SimdFloat4` - 4 floats in SIMD register
- `Float4x4` - 4x4 matrix with SimdFloat4 columns
- `SoaTransform` - 4 transforms in Structure-of-Arrays layout
- `SoaFloat3` - 4 Float3 vectors in SoA layout
- `SoaQuaternion` - 4 quaternions in SoA layout

### OzzAnimationSystem Implementation Progress

#### Core Features Implemented
- Basic skeleton/animation loading using X-Ray FS
- Animation handle system (replaces CBlend)
- Multi-animation blending with ozz jobs
- Transform conversion (ozz ↔ X-Ray matrices)
- Callback support for X-Ray compatibility

#### Extended Features Added (OzzAnimationSystem_Extensions.cpp)
- **Channel System**: SetChannelFactor() for 4-channel weights
- **Partition Support**: PlayAnimationOnPartition() with bone masks
- **Additional Transforms**: Per-bone procedural overlays
- **Root Motion**: Extraction for movement
- **X-Ray Compatibility**: MotionID mapping, CBlend callbacks

#### Key Implementation Patterns
```cpp
// Use X-Ray file system
IReader* reader = FS.r_open(path.c_str());
// ... read data
FS.r_close(reader);

// Animation handle replaces CBlend
struct AnimationHandle {
    size_t animation_index;
    float current_time;
    float weight;
    u16 partition_id;
    u8 channel;
    PlayCallback callback;
    void* callback_param;
};

// Partition masks for body part animation
SetPartitionMask(TORSO_PARTITION, torso_bones);
PlayAnimationOnPartition("reload", TORSO_PARTITION, 1.0f, false, channel);
```