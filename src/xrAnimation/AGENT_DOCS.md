# AGENT_DOCS.md - AI Assistant Guide

## Latest Session Notes
- Runtime conversion bug traced to missing inverse basis: `ConvertOzzMatrixToXRay` now applies a direct Ozz→X-Ray change-of-basis (`kOzzToXray`), fixing flipped bind poses while keeping legacy palette math intact. The converter switched to the matching direct `kXrayToOzz` transform, dropping the confusing Blender intermediate.
- `OzzKinematics` now evaluates bind pose and sampled animation with visibility masks, callbacks, and additional transforms; parity tests compare results against the legacy runtime.
- Legacy motion conversion pulls `.omf` payloads through the virtual filesystem, so packaged `.db` archives work without unpacking assets when `OzzKinematics` converts legacy motions on demand.
- Legacy `CKinematics` taps the shared palette dump hook, so `debug_dump_ozz_palette` snapshots `.ogf` bone matrices alongside `.ozzx` output.
- Initialization guards prevent palette/skinning requests until `OzzKinematics` has populated bone instances, avoiding crashes during pooled visual swaps.
- `OzzKinematics` implements `IKinematicsAnimated`, so gameplay callers obtain a valid `IKinematicsAnimated*` from `.ozzx` visuals while animation playback is proxied through the runtime.
- README and docs updated to describe the façade-first strategy and highlight remaining work around `.ozzx` runtime integration.
- `.ozzx` visuals instantiate CPU-skinned surfaces that honour engine containers (`xr_vector`, `xr_unique_ptr`) and feed geometry through the existing renderer; traversal code now treats `MT_OZZ_BUNDLE` like legacy skeletons.
- Developer toggle `g_use_ozz_visuals` enables loading converted `.ozzx` bundles (e.g., `dev_stalker.ozzx`) through the standard model pool for in-engine smoke tests.
- `CModelPool::Create` preserves `.ozzx` identifiers via `NormalizeModelIdentifier`, so bundle requests hydrate `COzzKinematicsVisual`; `ModelNaming.NormalizesModelIdentifiers` guards the normalization logic.
- Console helpers `debug_dump_ozz_palette`, `debug_dump_ozz_palette_toggle`, and `g_dev_ozz_actor` snapshot Ozz bone palettes and hot-swap the actor to `dev_stalker.ozzx`, making in-engine parity checks reproducible.
- IK limbs and step manager paths now bail out cleanly when an Ozz visual is active, so missing `IKinematicsAnimated` no longer hard-asserts during experiments.
- `OzzBundleRuntime` regression test hydrates `OzzKinematics` and mesh payloads from a `.ozzx` bundle to guard bundle/runtime parity.
- Parity/unit tests require the generated `src/xrAnimation/tests/testdata/stalker_hero_1.ozz` and `.ozzx` fixtures; rerun `convert_assets.sh` if they are missing before executing the suites.

## Active Objective
- Integrate a runtime visual that consumes `.ozzx` bundles, owns an `OzzKinematics`, and pushes bone palettes into the renderer/model pool.
- Drive a dev-only actor (or HUD item) through that path to exercise animation/physics callbacks using converted `.ozz` clips.
- Maintain TDD discipline: expand regression tests before introducing new runtime wiring or threading changes.

## Project Overview & Philosophy
- Primary project: integrate ozz-animation into the OpenXRay engine while preserving legacy behaviour.
- Repositories:
  - `xray-16/` – main C++17 cross-platform engine using CMake.
  - `omp-engine/` – Windows-focused reference repository (Visual Studio projects).
- Principles: favour modular, reusable systems; prioritise performance in animation/AI; write clear, maintainable code; keep `IKinematicsAnimated` API stable.

## Build Systems & Quick Start
- Always prefer Debug or Mixed builds for iterative work.
- OpenXRay build (from repo root):
  - `cmake -S xray-16 -B xray-16/ozz_utils -DCMAKE_BUILD_TYPE=Debug`
  - `cmake --build xray-16/ozz_utils -j$(nproc)`
- Notable CMake options: `MEMORY_ALLOCATOR` (`mimalloc` default), `XRAY_USE_LUAJIT` (ON by default).
- Reference engine (`omp-engine/`) builds via Visual Studio 2019+ solution `src/engine.sln` (v142 toolset).

## Development Guidelines
- Prefer engine containers/types over STL equivalents (`xr_vector`, `xr_string`, `shared_str`, `xr_map`, `xr_unique_ptr`, `intrusive_ptr`).
- Use engine allocators (`xr_new`, `xr_delete`, `xr_malloc`, `xr_free`).
- Keep code readable; comment only when intent is non-obvious.
- Logging: use `Msg()`; file IO via engine FS helpers.
- Maintain backward compatibility for gameplay systems relying on animation runtime behaviour.

## Workflow Expectations for Assistants
- Always consult available tooling/documentation ("memory") before running commands.
- Begin each interaction by fetching relevant memory entries; refer to them as "memory".
- Track new user facts: identity, behaviours, preferences, goals, relationships; update memory graphs accordingly.
- Refresh memory from `xray-16/src/xrAnimation` docs when the user requests a refresh.
- After modifying source files, rebuild the impacted targets and rerun their tests immediately—do this proactively without waiting for user prompts.

## Core Tenets
1. Check the knowledge graph before executing commands or writing files.
2. Use Debug (or Mixed) builds for X-Ray engine development whenever possible.
3. Keep responses concise, professional, factual; avoid flattery.
4. Synchronise documentation with code changes immediately—never allow docs to drift.

## Animation Converter Status
- Skeleton, mesh, and animation exports are validated end-to-end against Blender/XR references; resulting `.ozz` assets drive the viewer without discrepancies.
- Tooling (`debug_playback`, JSON dumps, viewer graphs) is in place for regression checks when converter logic evolves.
- Remaining work: surface richer OMF metadata (marks, params) through `.ozz`/`.ozzx` and ensure the new runtime visual exposes it alongside `OzzKinematics` events.

## Coordinate System Reference
- X-Ray is Y-up; ozz/OpenGL is Z-up.
- Converter outputs skeletons/meshes already in ozz space using the direct basis flip `(X, Y, Z)_XRAY → (X, Y, -Z)_ozz`; the runtime consumes the inverse (`(X, Y, Z)_ozz → (X, Y, -Z)_XRAY`) so palettes and vertex data share the same frame.

## Tools & Scripts
- `convert_assets.sh` (workspace root): converts `res/testdata/npc/stalker_hero_1.ogf` and `res/testdata/npc/critical_hit_grup_1.omf` into `.ozz` assets under `asset_tests/`.
- `run_stalker_hero_conversion.sh`: writes NPC skeleton/animation/mesh outputs to `src/xrAnimation/tests/testdata/npc/` and launches the viewer with the critical hit clip.
- `run_arms_conversion.sh`: processes `res/testdata/arms` into `src/xrAnimation/tests/testdata/arms/` and previews the first animation in the viewer.
- `run_arms_gunsl_conversion.sh`: processes `res/testdata/arms_gunsl` into `src/xrAnimation/tests/testdata/arms_gunsl/` and previews in the viewer.
- `run_monster_conversion.sh`: processes `res/testdata/monster` into `src/xrAnimation/tests/testdata/monster/` and previews in the viewer.
- `run_weapon_conversion.sh`: processes `res/testdata/weapon` into `src/xrAnimation/tests/testdata/weapon/` and previews in the viewer.
- `run_weapon_gunsl_conversion.sh`: processes `res/testdata/weapon_gunsl` into `src/xrAnimation/tests/testdata/weapon_gunsl/` and previews in the viewer.
- CLI converter (`xray_to_ozz_converter`):
  - Skeleton: `xray_to_ozz_converter skeleton <input.ogf> <output_dir>`.
  - Animation: `xray_to_ozz_converter animation <skeleton.ogf> <input.omf> <output_dir> [-optimize]`.
  - Batch: `xray_to_ozz_converter batch <input_dir> <output_dir> <skeleton.ogf> [-optimize]`.
- Features: handles compressed OMF data (8-/16-bit), preserves motion params, outputs metadata, supports batch processing.
- Known limits: SDK `.skl/.skls` unsupported; large skeletons (>64 bones) need special handling; some IK features may require manual tuning.

## Additional Resources
- `/animation_refactor_plan_revised.md`
- `/detailed_technical_implementation_revised.md`
- `/coordinate_analysis_final.md`
- Bind pose / animation comparison logs under workspace root (e.g., `bind_pose_comparison.md`, `animation_comparison.md`).
