# AGENT_NEXT_STEPS

## Current Status
- `.ogf/.omf → .ozz` pipeline is validated: skeletons, meshes, and animations round-trip and render correctly in the updated `ozz_animation_viewer`.
- Viewer UI now mirrors Ozz profiling (FPS/update/render graphs) and cleanly renders converted assets without depth conflicts.
- Skinning path matches the legacy runtime; palette generation and joint remaps are in sync with Blender/XR exporters.

## Immediate Focus – Ozz Runtime Path
1. **`.ozzx` Visual Integration**
   - Introduce an engine visual that loads bundle payloads, owns an `OzzKinematics`, and prepares bone palette buffers for renderer submission.
   - Register the visual in `CModelPool` with a distinct `MT_*` type so assets can be selected through the existing model pool workflows.
2. **Pilot Actor / HUD Harness**
   - Wire a dev-only actor (or HUD item) to the new visual path, play a converted clip, and verify callbacks/physics hooks fire as expected.
   - Capture quirks encountered during animation events, ragdoll bind, or script exposure for follow-up fixes.
3. **Runtime Telemetry & Docs**
   - Expose lightweight logging/profiling toggles to compare legacy vs. Ozz frame costs and update docs to reflect the façade + visual split.

## TDD Execution Plan – Visual & Actor Rollout
1. **Bundle & Visual Unit Tests**
   - Add tests covering `.ozzx` bundle hydration (skeleton + mesh payload sizes, failure modes) and visual creation to ensure assets load deterministically.
   - Mock renderer/model pool dependencies where possible so tests stay fast and headless.
2. **Actor Harness Tests**
   - Introduce fixtures that construct the pilot actor with an `OzzKinematics`, advance animation time, and assert bone palettes match the parity baselines.
   - Cover callbacks (`CBoneInstance::callback`), additional transforms, and visibility changes within the new path.
3. **Optimization & Regression Suite**
   - Extend converter tests to assert `--optimize` reduces file size and maintains pose parity for representative clips (NPC, weapon, arms).
   - Keep `ctest --output-on-failure` in the default workflow; capture failing diagnostics under `logs/` for quick diffs.

## Supporting Work
- Keep README/agent docs aligned with runtime capabilities after each milestone.
- Plan the threading story once the façade + visual stabilise (per-actor buffers, deterministic job scheduling, telemetry hooks).
- Trim build friction by disabling upstream ozz test targets that trigger DLL copy timeouts during CI or local runs.
