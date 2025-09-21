# AGENT_DOCS.md - AI Assistant Guide

## Latest Session Notes
- TransformConverter now applies a consistent +90° rotation about the X axis to both rotation columns and translation vectors (X, Y, Z) → (X, -Z, Y).
- Regenerate the `.ozz` skeleton and animation assets and compare them against the Blender reference to confirm bind pose translations and per-bone animation deltas.
- OMFReader consumes `OGF_S_SMPARAMS` bone remap data so animation tracks align with the target skeleton before basis conversion.

## Active Objective
- Align the OMF animation converter output with Blender-provided reference poses so characters no longer appear rotated or offset during playback.

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

## Core Tenets
1. Check the knowledge graph before executing commands or writing files.
2. Use Debug (or Mixed) builds for X-Ray engine development whenever possible.
3. Keep responses concise, professional, factual; avoid flattery.
4. Synchronise documentation with code changes immediately—never allow docs to drift.

## Animation Converter Status
- Skeleton conversion reads bone hierarchy, IK data, OBBs, and now exports transforms using the unified basis rotation.
- Animation conversion distributes per-bone tracks, respects motion mark formatting (`\r\n`), and stores motion parameters/metadata.
- Validation tooling (`debug_playback`, position comparison scripts) is available for bind pose and animation diffing.
- Outstanding verification: confirm orientation fixes eliminate 180° deltas and misplaced translations in exported `.ozz` files.

## Coordinate System Reference
- X-Ray is Y-up; ozz/OpenGL is Z-up.
- Current transformation: `(X, Y, Z)_XRAY → (X, -Z, Y)_ozz` (right-handed 90° rotation about +X).
- Apply the same rotation matrix to translation vectors and orientation columns to maintain consistency.

## Tools & Scripts
- `convert_assets.sh` (workspace root): converts `res/testdata/npc/stalker_hero_1.ogf` and `res/testdata/npc/critical_hit_grup_1.omf` into `.ozz` assets under `asset_tests/`.
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
