# AGENTS.md – Assistant Quick Guide

## Mission Snapshot
- Modernise OpenXRay's animation runtime by integrating ozz-animation while keeping legacy behaviour intact.
- xrAnimation module owns the façade around `OzzKinematics`, converter tools, and parity tests against the old pipeline.
- Converter CLI already produces `.ozz/.ozzx` assets from legacy `.ogf/.omf`; viewer tooling validates bind pose and animation parity.

## Active Priorities
1. Ship an engine visual that consumes `.ozzx` bundles, instantiates `OzzKinematics`, and feeds bone palettes into the renderer/model pool.
2. Drive a pilot actor or HUD item through the new visual, confirming callbacks, visibility toggles, and additional transforms behave like the legacy path.
3. Expose lightweight telemetry/docs comparing legacy vs. ozz frame costs and keep documentation in sync with code changes.

## Workflow Expectations
- Read existing memory (docs, notes) before running commands or changing files.
- Prefer Debug/Mixed builds for iteration; use engine containers (`xr_vector`, `shared_str`, etc.) and `Msg()` for logging.
- After every code or doc change: rebuild the affected targets and rerun their tests without waiting for a prompt.
- Keep responses concise, factual, and professional; update documentation immediately when behaviour changes.

## Build & Test Quickstart
- Configure (if needed): `cmake -S xray-16 -B xray-16/ozz_utils -DCMAKE_BUILD_TYPE=Debug`
- Build animation targets: `cmake --build xray-16/ozz_utils --target ozz_kinematics_tests xrAnimation_converter_tests -j`
- Run suites: `ctest --test-dir xray-16/ozz_utils --output-on-failure`
- Focused test loop: `xray-16/ozz_utils/bin/Debug/ozz_kinematics_tests --gtest_filter=OzzKinematicsParity.*`

## Handy Tools & Scripts
- `src/xrAnimation/scripts/run_*_conversion.sh` regenerate sample `.ozz/.ozzx` fixtures for NPCs, weapons, arms, and monsters.
- `xray_to_ozz_converter` CLI handles skeleton/animation/batch conversion.
- `ozz_animation_viewer` (Debug/Mixed build) can dump bind poses, JSON animation samples, and headless previews for parity checks.
- Blender snippets and viewer usage details live in `src/xrAnimation/README.md`.

## Where To Look Next
- Module docs & workflows: `src/xrAnimation/README.md`
- Historical context: `src/xrAnimation/CLAUDE.md`

