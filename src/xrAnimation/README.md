# xrAnimation Module

## MVP Milestone
- Converted `.ogf/.omf` assets now drive the `OzzKinematics` runtime and render correctly in-engine through `.ozzx` visuals.
- `ozz_animation_viewer` matches the in-engine pose output, and `OzzKinematicsParity.AnimationPoseMatchesLegacySkeleton` passes — confirming animation playback parity with the legacy pipeline.

## Module Overview
- Owns the façade around ozz-animation (`OzzKinematics`, `COzzKinematicsVisual`) and conversion tooling.
- Maintains parity tests against legacy kinematics and coordinates `.ozz/.ozzx` asset generation.
- Provides developer toggles and logging for palette inspection while we continue migrating runtime paths.

## Build & Test Quickstart
1. Configure (Mixed recommended for iteration):
   ```sh
   cmake -S . -B ozz_utils -DCMAKE_BUILD_TYPE=Mixed
   ```
2. Build animation targets:
   ```sh
   cmake --build ozz_utils --target ozz_kinematics_tests xrAnimation_converter_tests xray_to_ozz_converter -j
   ```
3. Run suites:
   ```sh
   ctest --test-dir ozz_utils --output-on-failure
   ```
   *Focused loop:* `ctest --test-dir ozz_utils -R "ozz_kinematics_tests|xrAnimation_converter_tests" --output-on-failure`

## Conversion & Viewer Tooling
- **CLI Converter** `xray_to_ozz_converter`
  ```sh
  # Skeleton
  xray_to_ozz_converter skeleton <input.ogf> <output_dir>

  # Animation (legacy .omf -> .ozz)
  xray_to_ozz_converter animation <skeleton.ogf> <input.omf> <output_dir> [-optimize]

  # Batch creation
  xray_to_ozz_converter batch <input_dir> <output_dir> <skeleton.ogf> [-optimize]
  ```
- **Viewer** `ozz_animation_viewer`
  ```sh
  cmake --build ozz_utils --target ozz_animation_viewer -j
  ozz_utils/bin/Mixed/ozz_animation_viewer --bundle=asset_tests/stalker_hero.ozzx \
      --animation=asset_tests/critical_hit_grup_1.ozz --render=false --max_idle_loops=1
  ```
  Use `--dump-animation-json=<path>` for frame-by-frame export or `--dump-bind-pose` (viewer default output) to diff against in-engine palettes.

## In-Engine Developer Toggles
- `g_use_ozz_visuals 1` – hydrate `.ozzx` bundles (falls back to `.ogf` when unavailable).
- `g_dev_ozz_actor 1` – swap the player model to the dev `.ozzx` actor for quick smoke tests.
- `g_dev_ozz_animation <name>` / `g_dev_ozz_animation_stop` – play/stop converted legacy motions.
- `debug_dump_ozz_palette` / `debug_dump_ozz_palette_toggle` – capture legacy vs. ozz bone palettes.

## Asset Regeneration Checklist
1. Rebuild tooling (see Build & Test above).
2. Use the helper scripts under `src/xrAnimation/scripts/` (e.g. `run_stalker_hero_conversion.sh`, `run_weapon_conversion.sh`) to regenerate `.ozz/.ozzx` fixtures. Each script writes outputs to `src/xrAnimation/tests/testdata` and can launch the viewer for quick validation.
3. Mirror bundles into the runtime search path when testing in-engine, e.g.:
   ```sh
   install -D src/xrAnimation/tests/testdata/stalker_hero.ozzx bin/x86_64/Mixed/gamedata/meshes/actors/dev_stalker.ozzx
   install -D src/xrAnimation/tests/testdata/critical_hit_grup_1.ozz bin/x86_64/Mixed/gamedata/anims/critical_hit_grup_1.ozz
   ```

### Conversion Scripts
- `src/xrAnimation/scripts/run_stalker_hero_conversion.sh` – NPC skeleton/animation sample used by parity tests.
- `run_weapon_conversion.sh`, `run_weapon_gunsl_conversion.sh` – first-person weapon assets.
- `run_arms_conversion.sh`, `run_arms_gunsl_conversion.sh` – player arms variations.
- `run_monster_conversion.sh` – creature pipeline smoke tests.
- Each script sources `common.sh`, ensures the build tree exists, invokes the converter, and optionally runs `ozz_animation_viewer`.

## Blender / Debug Utilities
- Rest pose dumps (Blender): set armature to REST pose and iterate `bone.matrix_local` to emit translation + Euler rotation tables.
- Viewer bind pose dumps: run `ozz_animation_viewer` headless (`--render=false`) to capture the bind pose table printed to stdout.

## Housekeeping
- Legacy agent docs have been consolidated into this README following the MVP milestone. Future doc updates should land here so the runtime, tooling, and test workflows stay in sync.
