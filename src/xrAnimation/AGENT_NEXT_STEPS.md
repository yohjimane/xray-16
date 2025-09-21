# AGENT_NEXT_STEPS

## Current Status
- `.ogf/.omf → .ozz` pipeline is validated: skeletons, meshes, and animations round-trip and render correctly in the updated `ozz_animation_viewer`.
- Viewer UI now mirrors Ozz profiling (FPS/update/render graphs) and cleanly renders converted assets without depth conflicts.
- Skinning path matches the legacy runtime; palette generation and joint remaps are in sync with Blender/XR exporters.

## Immediate Focus – Ozz Kinematics Integration
1. **Design the façade**
   - Catalogue everything `CKinematics` / `IKinematics` expose today (pose queries, motion marks, callbacks, physics hooks).
   - Sketch an `OzzKinematics` replacement that owns the Ozz runtime objects (skeleton, playback controller, scratch buffers) while satisfying the same public interface.
2. **Prototype the update loop**
   - Port the pose builder to Ozz jobs (`SamplingJob`, optional blending, `LocalToModelJob`), keeping bone-transform accessors compatible with downstream systems.
   - Preserve metadata like motion marks and profiling counters so gameplay/render code continues to function.
3. **Pilot migration**
   - Swap a test actor over to the façade, verify animation events, IK hooks, and renderer constant-buffer updates.
   - Document integration changes (ragdoll binding, script exposure) ahead of a wider rollout.

## TDD Execution Plan – OzzKinematics vs Legacy Parity
1. **Test Harness Upgrade**
   - Wire GoogleTest into `src/xrAnimation/tests` and add an `ozz_kinematics_tests` executable alongside the existing stub target.
   - Provide helpers to load both legacy `.ogf/.omf` assets and the converted `.ozz` pair from `tests/baseline_cases`.
2. **Bootstrap Parity Test**
   - Instantiate `CKinematics` and `OzzKinematics` for the same asset, asserting bone counts, name lookups, and default visibility masks match.
   - Implement the minimal construction logic in `OzzKinematics` to satisfy this test.
3. **Pose Comparison Loop**
   - Add a test that plays a deterministic animation frame in both runtimes, compares every `CBoneInstance.mTransform/mRenderTransform`, and verifies bounding volumes.
   - Incrementally implement Ozz sampling and transform swizzles until the test turns green.
4. **Callback & Overrides**
   - Drive tests covering `CBoneInstance` callbacks, ABT offsets, and `LL_SetBoneVisible` behaviour; extend `OzzKinematics` only as required to pass.
5. **Continuous Regression Check**
   - Run `ctest --output-on-failure` after each change and capture any diff diagnostics in `logs/` for future debugging.

## Supporting Work
- Keep converter/viewer docs in sync as we refine UX or add validation scripts.
- Plan the threading story once the façade stabilises (reuse `xr_parallel_for`, reuse per-actor buffers, maintain Ozz `Record` telemetry).
- Track remaining OMF metadata work separately in `AGENT_PLANS.md`.
