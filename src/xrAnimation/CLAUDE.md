# CLAUDE.md

## X-Ray to ozz-animation Integration Project

### Current Status (Phase 1, Week 1)
- ✅ xrAnimation module created and building successfully
- ✅ Dependencies resolved (ozz-animation, imgui, SDL2)
- ✅ Basic project structure established
- ✅ IKinematicsAnimated interface analyzed and documented
- ✅ Usage patterns traced in game code
- ✅ Critical paths identified
- ✅ X-Ray to ozz method mapping completed
- ✅ Basic ozz test harness created
- 🔄 Next: Week 2 - CKinematicsAnimated implementation study

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