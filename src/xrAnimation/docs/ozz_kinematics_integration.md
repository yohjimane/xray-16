# Ozz Kinematics Integration Plan

## Legacy Responsibilities Snapshot
- `IKinematics` (see `xray-16/src/Include/xrRender/Kinematics.h`) defines the runtime contract that gameplay, physics, and renderer expect today: bone lookup by name/ID, transform access (`LL_GetTransform`, `LL_GetTransform_R`), visibility masks, bounding boxes, bone callbacks, wallmark helpers, and a `CalculateBones` entrypoint that lazily recomputes poses.
- `CKinematics` (see `xray-16/src/Layers/xrRender/SkeletonCustom.{h,cpp}` and `SkeletonRigid.cpp`) implements that contract on top of legacy `CBoneData` and `CBoneInstance` structures. It owns bone maps, manages update throttling (`UCalc_Time`/`UCalc_Interval`), propagates transforms down the hierarchy, and rebuilds render-space transforms and visibility volumes.
- `IKinematicsAnimated`/`CKinematicsAnimated` extend the base class with animation playback (`CBlend` graphs, motion partitions, FX channels) but ultimately still rely on `CKinematics::BuildBoneMatrix` to fill `CBoneInstance.mTransform` and `mRenderTransform` before render/physics consumers pull data.

## Observations from CKinematics
- Pose evaluation is recursive: `CalculateBones` → `Bone_Calculate` → `CLBone`/`BuildBoneMatrix`. The current solver expects per-bone callbacks and additional transforms (ABT offsets) to be injected during the traversal.
- `CBoneInstance` remains the currency across systems. Callers mutate `CBoneInstance::param[]`, register callbacks, and read back `mTransform`/`mRenderTransform` without touching Ozz directly.
- Visibility and bounding data are recomputed opportunistically after pose updates; many subsystems rely on `GetBox()`, `LL_GetBox()`, and `LL_GetBonesVisible()` to stay coherent with the transform cache.
- Physics features (ragdolls, bone shapes) still operate on `CBoneData` metadata stored with the visual, so any façade must preserve access to that structure even if Ozz owns the live pose buffers.

## Integration Strategy Overview
- Introduce an `OzzKinematics` façade inside `xray-16/src/xrAnimation/` that implements `IKinematics` (and, in a follow-up, `IKinematicsAnimated`) while delegating pose evaluation to ozz-animation jobs.
- Keep `CBoneInstance` arrays alive so legacy systems continue reading/writing transforms and callbacks. The façade will synchronize these instances with Ozz joint output after each evaluation pass.
- Treat the converted `.ozz` skeleton as the source of truth for hierarchy/poses, but retain the legacy `CBoneData` graph for metadata (names, physics shapes, wallmarks). Store a stable mapping (`bone_id → joint index`) compiled during asset import.

### Data Ownership & Layout
- Runtime members: `ozz::vector<SimdFloat4>` local transforms, `ozz::vector<Float4x4>` model-space matrices, and a scratch buffer for sampling/blending jobs mirroring Ozz samples (`SamplingJob::Context`, `LocalToModelJob::Context`).
- Legacy bridge: `xr_vector<CBoneInstance>` allocated with engine allocators (`xr_alloc`). `CBoneInstance.mTransform` stays authoritative for X-Ray callers; the façade writes into it after each Ozz update.
- Metadata: retain pointers/references to `CBoneData` arrays emitted by the converter so bone shapes, IK flags, and motion partitions keep working.

### Update Loop Mapping
1. `CalculateBones` checks the same throttle fields (`UCalc_Time`, `UCalc_Interval`) to preserve caller expectations.
2. When evaluation is required, run Ozz sampling (`SamplingJob`) with the current animation state (populated by the upcoming `OzzKinematicsAnimated`).
3. Run `LocalToModelJob` to obtain model-space matrices (ozz column-major float4x4).
4. Convert each transform to X-Ray `Fmatrix` (row-major, applying the `(X, Y, -Z)` swizzle per coordinate system reference) and write into the corresponding `CBoneInstance.mTransform`. Apply additional bone transforms (ABT) and callbacks afterwards to preserve legacy overrides.
5. Update `mRenderTransform` using precomputed `m2b_transform` exactly as `CKinematics::CLBone` does today so render skinning remains unchanged.
6. Recompute visibility bounding volumes when the throttle dictates, filling `vis.box` / `vis.sphere` to satisfy `GetBox()` semantics.

### Bone Visibility & Callbacks
- Mirror `LL_SetBoneVisible` behaviour by storing the legacy mask and zeroing `mTransform` when a bone is hidden. Skip Ozz sampling for fully hidden actors, but maintain per-bone visibility so partial hides still update dependent bones correctly.
- Callbacks: run legacy callbacks after writing Ozz output into the `CBoneInstance`. Respect `callback_overwrite()` by short-circuiting Ozz data when a gameplay system takes over a bone.

### Physics, Wallmarks, and Queries
- Physics shells call into `LL_GetBoneInstance`, `LL_GetBox`, and `GetBoneData`; all are satisfied by keeping legacy arrays untouched. Only the pose source changes.
- Wallmark helpers (`AddWallmark`, `CalculateWallmarks`) depend on `mRenderTransform`. Ensuring that buffer is refreshed from Ozz matrices maintains compatibility.
- `Bone_GetAnimPos` must perform a one-off Ozz sampling limited to the target bone chain. Initially we can reuse the latest cached pose and fall back to forcing a full evaluate when `ignore_callbacks` is requested.

## Implementation Roadmap
1. **Façade Skeleton**: add `OzzKinematics` class (header/impl) under `xrAnimation/` exposing an `IKinematics` interface. Stub out every virtual with assertions and document mapping decisions inline. Wire into `xrAnimation` CMake target.
2. **Data Bootstrap**: load `.ozz` skeleton + legacy `CBoneData`, build name/ID maps, allocate `CBoneInstance` storage via engine allocators, and expose getters (`LL_BoneID`, `LL_Bones`, etc.) using converter metadata.
3. **Pose Evaluation Core**: implement the `CalculateBones` path using Ozz sampling + local-to-model jobs. Populate `CBoneInstance` buffers, respect callbacks/visibility, and update bounding volumes.
4. **Compatibility Layer**: port remaining helpers (`Bone_GetAnimPos`, wallmarks, bounding boxes) and add unit hooks so downstream systems (physics/render) see unchanged behaviour. `COzzKinematicsVisual` now consumes `.ozzx` mesh payloads, CPU-skins surfaces each frame, and feeds them into the renderer via standard `ref_geom` buffers.
5. **Animated Extension**: layer an `OzzKinematicsAnimated` companion that implements `IKinematicsAnimated` on top of the façade, translating X-Ray blend graphs into Ozz sampling jobs.
6. **Validation**: compare viewer output against legacy `CKinematics` for representative actors (`stalker_hero`, weapon rigs) using existing regression scripts; document discrepancies in `logs/`.

### Future Enhancement
- Provide an optional startup flag that scans legacy `.ogf/.omf` assets (whether loose or inside packed `.db` archives), converts them to `.ozz/.ozzx` on first launch, and writes the results back to the matching paths under `gamedata`. This removes the need to ship huge preconverted bundles while keeping runtime loading identical.
- `COzzKinematicsVisual` stages skeletons, meshes, and CPU-skinned geometry from `.ozzx` bundles. Surfaces currently update on the main thread using the computed palette; once the animation job system lands we can parallelise per-surface palette/skin computation (each surface only depends on a subset of joints), and longer term bind the palette to GPU skinning so we can drop the CPU pass entirely.

## Risks & Open Questions
- We need authoritative `CBoneData` when running inside the engine. Confirm the converter stores/loadable structures or plan a serialized cache.
- Legacy systems occasionally mutate `CBoneInstance` directly between updates (e.g., IK solvers). We must detect and reconcile those overrides with the Ozz pose buffer each frame.
- Motion marks and scripted callbacks depend on `CKinematicsAnimated` internals; the façade must eventually expose equivalent hooks through `OzzKinematicsAnimated` to avoid regressions.
- Threading: Ozz jobs can run multi-threaded, but legacy code touches `CBoneInstance` from the main thread. Initial version will run synchronously; future work should integrate with `xr_parallel_for` once correctness is proven.

## Verification Status
- 2025-09-21: `OzzKinematicsParity.AnimationPoseMatchesLegacySkeleton` passes using the converted `stalker_hero_1.ozz` / `critical_hit_grup_1.ozz` pair. Run
  `ctest --output-on-failure --test-dir xray-16/ozz_utils/src/xrAnimation/tests -C Debug`
  after rebuilding to confirm the regression suite remains green.
- 2025-09-21: `COzzKinematicsVisual` can now hydrate `OzzKinematics` directly from `.ozzx` bundles; `OzzKinematicsBootstrap.InitializesFromOzzxBundleSkeleton`
  guards the loader by comparing bundle output against the existing `.ozz` reference skeleton.
